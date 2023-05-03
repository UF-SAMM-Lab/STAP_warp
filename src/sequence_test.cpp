#include <stap_warp/utilities.h>
#include <stap_warp/stap_warp.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib_msgs/GoalID.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <stap_warp/human_prediction.h>


std::mutex mtx;
std::mutex goal_mtx;
std::string goal_id;
ros::Time start_tm;
double start_duration;


void goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg) {
  std::lock_guard<std::mutex> lck(goal_mtx);
  goal_id = msg->goal_id.id;
  ROS_INFO_STREAM("read"<<goal_id);
}

// void segment_thread(moveit::planning_interface::MoveGroupInterface::Plan plan) {
//     ROS_INFO_STREAM("plan size:"<<plans.trajectory_.joint_trajectory.points.size());
//     move_group.asyncExecute(plans);
//     ros::Time p_start = ros::Time::now();
//     ros::Duration(0.1).sleep();
//     while ((rec.joint_pos_vec-goal_vec).norm()>0.001) {
//         stap_warp.warp(model_->joint_seq,std::max((ros::Time::now()-p_start).toSec(),0.0),rec.joint_pos_vec,rec.get_current_joint_state());
//         ros::Duration(0.1).sleep();
//     }
// }

int main(int argc, char** argv) {
    ros::init(argc,argv,"test_plan");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

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
    ros::Publisher pub_cancel_traj = nh.advertise<actionlib_msgs::GoalID>( ctrl_ns+"/follow_joint_trajectory/cancel", 0,false);
    actionlib_msgs::GoalID goal_id_msg;
    ros::Subscriber sub_goal = nh.subscribe<control_msgs::FollowJointTrajectoryActionGoal>(ctrl_ns+"/follow_joint_trajectory/goal",1,goal_callback);
    //do a test with no obsctacles:
    std::vector<double> start_joint;
    std::vector<double> goal_joint;

    moveit::planning_interface::MoveGroupInterface move_group(plan_group);
    // actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> mg_action_client = move_group.getMoveGroupClient();
    const robot_state::JointModelGroup* joint_model_group_ = move_group.getCurrentState()->getJointModelGroup(plan_group);
    robot_model_loader::RobotModelLoaderPtr robot_model_loader = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
    const robot_model::RobotModelPtr& model = robot_model_loader->getModel();
    std::shared_ptr<planning_scene::PlanningScene> scene(new planning_scene::PlanningScene(model));
    planning_scene_monitor::PlanningSceneMonitorPtr monitor;
    monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    planning_scene::PlanningScenePtr ps_ptr = monitor->getPlanningScene();
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

    std::string resp = "y";
    clearObstacles();

    std::vector<std::vector<double>> human_sequences;


    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
    
    std::vector<std::vector<double>> robot_poses;
    int num_robot_poses = 0;
    if (!nh.getParam("/robot_poses/length", num_robot_poses))
    {
      ROS_ERROR("/robot_poses is not defined in sequence.yaml");
      throw std::invalid_argument("/robot_poses is not defined in sequence.yaml");
    }
    std::vector<double> pose;
    for (int i=0; i<num_robot_poses.size();i++) {
        nh.getParam("/robot_poses/i", pose)
        robot_poses.push_back(pose);
        for (int q=0;q<6;q++) std::cout<<robot_poses.back()[q]<<",";
        std::cout<<std::endl;
    }

    // nh.getParam("/robot_poses/"+std::to_string(test_num)+"/robot_goal", goal_joint);
    int seg_num=0;
    // while (ros::ok()) {

    // }

}