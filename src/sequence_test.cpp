#include <stap_warp/utilities.h>
#include <stap_warp/stap_warp.h>
#include <stap_warp/sequence.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib_msgs/GoalID.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <stap_warp/human_prediction.h>
#include <memory>
#include <mutex>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>


std::mutex mtx;
std::mutex goal_mtx;
std::string goal_id;
std::mutex skel_mtx;
ros::Time start_tm;
double start_duration;
std::vector<float> human_quat_pose = {-1.737439036369323730e-01,9.085529446601867676e-01,-3.478607162833213806e-02,9.981507693313929064e-01,-5.998364698210649493e-02,9.849049592449109214e-03,0.000000000000000000e+00,9.580149214898834309e-01,1.740551826395725366e-01,-2.278424973507739981e-01,0.000000000000000000e+00,7.123834513833746662e-01,1.841394957996581871e-01,6.772019375945099728e-01,-0.000000000000000000e+00,2.844575261446070646e-01,1.394740396576789054e-01,-9.484887495807520219e-01,0.000000000000000000e+00,7.097945123379674204e-01,4.777340680356021441e-01,5.176503747637605235e-01,-0.000000000000000000e+00,2.843504163844787769e-01,-3.630602545215000920e-01,8.873173571438557339e-01,0.000000000000000000e+00,7.148535069955277432e-01,2.736450296645902003e-01,-6.435082449169243768e-01,0.000000000000000000e+00};


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

void skel_quats_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lck(skel_mtx);
    human_quat_pose = msg->data;
    // live_human_quats.clear();
    // for (int i=0;i<7;i++){
    //     live_human_quats.push_back(Eigen::Quaternionf(pose_elements[i*4+4],pose_elements[i*4+5],pose_elements[i*4+6],pose_elements[i*4+7]));
    // }
}

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
    ros::Subscriber sub_quats = nh.subscribe<std_msgs::Float32MultiArray>("/skeleton_quats",1,skel_quats_cb);
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
    std::vector<float> human_link_lengths;// = {0.569,0.194,0.328,0.285,0.357,0.285,0.45}; 
    std::vector<float> human_link_radii;// = {0.12,0.05,0.1,0.04,0.03,0.04,0.03}; 
    double min_dist = 0.0;
    if (!nh.getParam("/human_link_lengths", human_link_lengths)) ROS_ERROR("couldn't read human dimensions");
    if (!nh.getParam("/human_link_radii", human_link_radii)) ROS_ERROR("couldn't read human radii");
    if (!nh.getParam("/minimum_distance", min_dist)) ROS_ERROR("couldn't read minimum_distance");

    std::string resp = "y";
    clearObstacles();
    std::shared_ptr<ros::ServiceClient> predictor = std::make_shared<ros::ServiceClient>(nh.serviceClient<stap_warp::human_prediction>("predict_human"));
    while (!predictor->exists()) {
      ROS_INFO("waiting for predict_human service");
      ros::Duration(0.1).sleep();
    }
    std::shared_ptr<ros::Publisher> human_pub = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("predicted_human", 0,false));
    skel_mtx.lock();
    std::cout<<human_quat_pose.size()<<std::endl;
    stap_test::humans human_data(nh,human_quat_pose,predictor,human_pub);
    skel_mtx.unlock();
    stap_test::robot_sequence robot_data(nh);
    human_data.predicted_motion();
    human_data.show_predictions(human_link_lengths,human_link_radii);
    int seg_num=0;
    // while (ros::ok()) {

    // }

}