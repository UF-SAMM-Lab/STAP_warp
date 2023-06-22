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
#include <manipulation_msgs/JobExecution.h>

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

  ros::ServiceClient gripper_client = nh.serviceClient<manipulation_msgs::JobExecution>("/robotiq_gripper");
  gripper_client.waitForExistence();

   
  bool open_grip = false;
  if (!nh.getParam("/sw_test_grip/open", open_grip))
  {
    ROS_ERROR("/sw_test_grip/open is not defined");
  }
                  
  manipulation_msgs::JobExecution srv;

  srv.request.property_id= open_grip ? "open":"close";
  if (gripper_client.call(srv)) {
    if (srv.response.results>=0) {
      ROS_INFO_STREAM("success, gripper:"<<open_grip ? "open":"close");
    } else {
      ROS_WARN("unable to grasp");
    }
  }

  ros::spinOnce();

  return 0;
}