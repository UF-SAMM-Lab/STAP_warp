#include <ros/ros.h>
#include <manipulation_msgs/JobExecution.h>
#include <std_msgs/Bool.h>

//INTRODUCTION TO THE MAIN CODE -------------------------------------------------------------------------------------------------------------------------
ros::ServiceClient gripper_client;

void gripper_callback(const std_msgs::Bool::ConstPtr& msg) {
  manipulation_msgs::JobExecution srv;

  ROS_INFO_STREAM("attempting gripper action:"<<msg->data ? "open":"close");
  srv.request.property_id= msg->data ? "open":"close";
  if (gripper_client.call(srv)) {
    if (srv.response.results>=0) {
      ROS_INFO_STREAM("success, gripper:"<<msg->data ? "open":"close");
    } else {
      ROS_WARN("unable to grasp");
    }
  }
}

int main(int argc, char** argv) {

  //Creation of the node
  ros::init(argc, argv, "sw_gripper_interpretter");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  gripper_client = nh.serviceClient<manipulation_msgs::JobExecution>("/robotiq_gripper");
  gripper_client.waitForExistence();

  ros::Subscriber sub_grip = nh.subscribe<std_msgs::Bool>("/open_gripper", 1, gripper_callback);

  ROS_INFO("Ready to grip!");

  ros::waitForShutdown();

  return 0;
}