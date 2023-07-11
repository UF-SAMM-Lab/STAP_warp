#include <ros/ros.h>
#include <manipulation_msgs/JobExecution.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

//INTRODUCTION TO THE MAIN CODE -------------------------------------------------------------------------------------------------------------------------
ros::ServiceClient gripper_client;
double gripper_pct = 50;
double close_pct = 0;
ros::Publisher pub_grip_done;

void gripper_callback(const std_msgs::Bool::ConstPtr& msg) {
  manipulation_msgs::JobExecution srv;
  std::string action_string = msg->data ? "pos_" + std::to_string(std::round(gripper_pct/100*85)) + "_force_100_vel_0":"pos_0_force_100_vel_0";
  ROS_INFO_STREAM("attempting gripper action:"<<action_string);
  srv.request.property_id= action_string;
  if (gripper_client.call(srv)) {
    if (srv.response.results>=0) {
      ROS_INFO_STREAM("success, gripper:"<<action_string);
    } else {
      ROS_WARN("unable to grasp");
    }
  }
  std_msgs::Bool grip_done_msg;
  grip_done_msg.data = true;
  pub_grip_done.publish(grip_done_msg);
}

void gripper_pct_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  gripper_pct = msg->data[0];
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
  ros::Subscriber sub_grip_pct = nh.subscribe<std_msgs::Float32MultiArray>("/set_gripper_open_close", 1, gripper_pct_callback);
  pub_grip_done = nh.advertise<std_msgs::Bool>("/gripper_done", 0,false);

  ROS_INFO("Ready to grip!");

  ros::waitForShutdown();

  return 0;
}