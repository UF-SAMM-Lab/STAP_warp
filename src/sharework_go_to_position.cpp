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

  std::vector<double> desired_xyzq;
  
  // std::string pose_str;
  // if (!nh.getParam("/sw_test_pos/pose",pose_str))
  // {
  //   ROS_ERROR("%ssw_test_pos/pose not defined", nh.getNamespace().c_str());
  //   throw std::invalid_argument("/sw_test_pos/pose is not defined");
  // }  
  // ROS_INFO_STREAM("pose string:"<<pose_str);
  // std::stringstream ss (pose_str);
  // std::string item;
  // while (getline(ss, item, ',')) {
  //     desired_xyzq.push_back(std::stod(item));
  // }


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

   
  int pos_num = 0;
  if (!nh.getParam("/sw_go_pos/pos_num", pos_num))
  {
    ROS_ERROR("/sw_go_pos/pos_num is not defined");
  }

  std::vector<double> start_joints;
  ROS_INFO_STREAM("getting /test_sequence/" + std::to_string(test_num) + "/robot_poses/"<<pos_num);
  if (!nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_poses/"+std::to_string(pos_num), start_joints))
  {
    ROS_ERROR_STREAM("/test_sequence/"<<test_num<<"/robot_poses/" << pos_num << " is not defined in sequence.yaml");
  }

  std::cout<<"go to:";
  for (int i=0;i<6;i++) std::cout<<start_joints[i]<<",";
  std::cout<<std::endl;
  //Initialization of the plans that will be used to correct the robot motion according to the specific behavior
  bool plan_success;
  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("BiTRRT");
  move_group.setStartStateToCurrentState();
  move_group.setMaxVelocityScalingFactor(1.0);            //time parametrization
  move_group.setMaxAccelerationScalingFactor(1.0);  
  move_group.setJointValueTarget(start_joints);
  moveit::planning_interface::MoveGroupInterface::Plan new_plan;  
  plan_success = (move_group.plan(new_plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (plan_success){
      move_group.execute(new_plan);
  } else {
    ROS_ERROR("plan failed");
  }
  ros::spinOnce();

  return 0;
}