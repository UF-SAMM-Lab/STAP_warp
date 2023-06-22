#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <inverse_kinematics/ik_seed_search.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <Eigen/Dense>

//INTRODUCTION TO THE MAIN CODE -------------------------------------------------------------------------------------------------------------------------



int main(int argc, char** argv) {

  //Creation of the node
  ros::init(argc, argv, "sw_get_angles");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //Initialization of some useful objects    
  std::string plan_group = "manipulator";
  if (!nh.getParam("/plan_group",plan_group))
  {
    ROS_WARN("plan_group is not defined");
  }
  std::string ctrl_ns = "manipulator";
  if (!nh.getParam("/ctrl_ns",ctrl_ns))
  {
    ROS_WARN("ctrl_ns is not defined");
  }  

  int test_num = 1; 
  if (!nh.getParam("/sequence_test/test_num",test_num))
  {
    ROS_WARN("/sequence_test/test_num is not set");
  }  

  std::vector<double> workcell_transform(7,0.0);
  workcell_transform[6] = 1.0;
  Eigen::Isometry3f transform_to_world = Eigen::Isometry3f::Identity();
  if (nh.getParam("/test_sequence/" + std::to_string(test_num) + "/workcell_transform", workcell_transform)) {
    transform_to_world.linear() = Eigen::Matrix3f(Eigen::Quaternionf(workcell_transform[3],workcell_transform[4],workcell_transform[5],workcell_transform[6]));
    transform_to_world.translation() = Eigen::Vector3f(workcell_transform[0],workcell_transform[1],workcell_transform[2]);
  }
  ROS_INFO_STREAM("\n"<<transform_to_world.matrix());

   
  int num_robot_poses = 0;
  if (!nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_positions/length", num_robot_poses))
  {
    ROS_ERROR_STREAM("/test_sequence/"<<test_num<<"/robot_positions/length is not defined in sequence.yaml");
  }


  moveit::planning_interface::MoveGroupInterface move_group(plan_group);
  const std::string descr = "robot_description";

  //Initialize and start a monitor to listen to the correct updated planning scene
  robot_model_loader::RobotModelLoaderPtr robot_model_loader = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader(descr));
  const robot_model::RobotModelPtr& model = robot_model_loader->getModel();
  planning_scene::PlanningScene scene(model);
  planning_scene_monitor::PlanningSceneMonitorPtr monitor;
  monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  if(monitor->getPlanningScene())
  {
    monitor->startSceneMonitor("/move_group/monitored_planning_scene");
    monitor->startWorldGeometryMonitor();
    monitor->startStateMonitor();
  }
  else
  {
    exit(EXIT_FAILURE);
  }  


  //Initialization of the plans that will be used to correct the robot motion according to the specific behavior
  bool plan_success;
  int t = 0;
  int valid_joints;
  std::vector<double> end_joints;
  tf2::Quaternion desired_quat;
  geometry_msgs::Quaternion desired_quat_msg;
  geometry_msgs::Pose desired_pose;
  robot_state::RobotState desired_state_pose(*move_group.getCurrentState());
  std::vector<std::vector<double>> all_ik_slns;
  const robot_state::JointModelGroup* joint_model_group_ = move_group.getCurrentState()->getJointModelGroup(plan_group);    

  moveit::planning_interface::MoveGroupInterface::Plan new_plan;           //will store the plan modified by the controller  

  std::vector<double> pose(6);
  std::vector<double> prev_joints = {1.5707963267949, -2.44346095279206, 2.44346095279206,-1.5707963267949, 1.5707963267949, 0};

  for (int i=0;i<num_robot_poses;i++) {
    if ((i==5)||(i==6)) continue;
    nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_positions/"+std::to_string(i), pose);
    plan_success = false;

    std::cout<<std::endl;
    Eigen::Quaternionf q(pose[3],pose[4],pose[5],pose[6]);
    q = Eigen::Quaternionf(transform_to_world.rotation())*q;

    Eigen::Vector3f p(pose[0],pose[1],pose[2]);
    p = transform_to_world*p;
    // desired_quat.setRPY(desired_cartesian[3],desired_cartesian[4],desired_cartesian[5]);
    // desired_quat_msg = tf2::toMsg(desired_quat);                                           //with this we convert from rpy to quaternion
    desired_pose.position.x = p[0];              //assign position to pose
    desired_pose.position.y = p[1];              //assign position to pose
    desired_pose.position.z = p[2];              //assign position to pose
    desired_pose.orientation.x = q.x();                                          //assign orientation to pose
    desired_pose.orientation.y = q.y();                                          //assign orientation to pose
    desired_pose.orientation.z = q.z();                                          //assign orientation to pose
    desired_pose.orientation.w = q.w();                                          //assign orientation to pose
    std::cout<<"after transform:"<<p[0]<<","<<p[1]<<","<<p[2]<<","<<q.w()<<","<<q.x()<<","<<q.y()<<","<<q.z()<<std::endl;
    all_ik_slns = ik_search::ik_search(desired_state_pose,plan_group, joint_model_group_, desired_pose); //find joint angle solutions to reach goal pose
    std::cout<<"ik solutions:"<<all_ik_slns.size()<<std::endl;
    //if more than 1 solution publish size of solutions and wait to receive which solution
    std::string output_string = std::to_string(all_ik_slns.size())+" inverse kinematics solutions";
    std::vector<double> start_joints;
    if (!all_ik_slns.empty()) {
      start_joints = ik_search::ik_closest_sln(prev_joints,all_ik_slns);
      desired_state_pose.setVariablePositions(start_joints);
      prev_joints = start_joints;
      std::cout<<"goal:"<<i<<",";
      for (int j=0;j<6;j++) std::cout<<start_joints[j]<<",";
      std::cout<<std::endl;
    } else {
      output_string = "Process of Inverse Kinematics has failed... Given goal pose may be not valid or out of reach";
    }    
    std::cout<<output_string<<std::endl;   
  }

  ros::spinOnce();

  return 0;
}