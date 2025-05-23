#pragma once
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64MultiArray.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <subscription_notifier/subscription_notifier.h>
#include <ros/ros.h>
#include <cmath>
#include <string>
#include <mutex>
#include <avoidance_intervals/avoidance_model.h>
// #include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosdyn_core/primitives.h>
#include <rosparam_utilities/rosparam_utilities.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include <human_probablistic_occupancy/human_probablistic_occupancy.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>
#include <exception>
#include <geometry_msgs/Point.h>
#include <eigen_conversions/eigen_msg.h>

void clearObstacles(void);
void pub_plan(ros::Publisher nom_plan_pub,moveit::planning_interface::MoveGroupInterface::Plan plan,robot_state::RobotStatePtr state, std::string plan_group, std::string last_link);
moveit_msgs::CollisionObject createCollisionBox(Eigen::Vector3f dims, Eigen::Vector3f position, Eigen::Quaternionf quat, std::string id);

class humanCollisionObjects {
    protected:
        ros::NodeHandle nh;
    private:
        moveit_msgs::PlanningScene planning_scene;
        moveit::planning_interface::PlanningSceneInterface psi;
        std::vector<Eigen::Vector3f> link_centroids;
        std::vector<Eigen::Vector3f> human_points;
        planning_scene::PlanningScenePtr planning_scene_;
        ros::Publisher planning_scene_diff_publisher;
        std::vector<moveit_msgs::CollisionObject> collisionObjects;
        std::vector<Eigen::Quaternionf> link_quats;
        std::vector<std::vector<float>> pose_sequence;
        void forward_kinematics(std::vector<float> pose_elements);
        void moveCollisionObject(moveit_msgs::CollisionObject &msg, Eigen::Vector3f pos, Eigen::Quaternionf quat);
        moveit_msgs::CollisionObject createCollisionObject(Eigen::Vector3f pos, Eigen::Quaternionf quat, double length, double radius, std::string id);
        std::vector<std::string> co_ids = {"torso","neck","l_upper", "l_fore","r_upper","r_fore"};
        double shoulder_len = 0.35;
        std::vector<double> link_lengths_ = {0.5,0.2,0.35,0.45,0.35,0.45};
        std::vector<double> link_radii_ = {0.17,0.1,0.1,0.07,0.1,0.07};
        std::vector<double> act_lengths;
        std::vector<double> act_radii;
        double dt = 0;
        Eigen::Isometry3f transform_to_world;
        ros::Time timer_start;
        double min_dist_;
        ros::Timer udpate_timer;
        double elapsed_time;
        void update_timer(const ros::TimerEvent& event);
        bool pause_live = false;
        moveit_msgs::CollisionObject createHeadCollisionObject(Eigen::Vector3f pos, double radius, std::string id);
    public:
        void setJointLocations(std::vector<Eigen::Vector3f> joints);
        humanCollisionObjects(ros::NodeHandle node_handle, const planning_scene::PlanningScenePtr &planning_scene_ptr, std::vector<double> lengths, std::vector<double> radii, double min_dist);
        void removeHumans(void);
        void read_human_task(int task_num, Eigen::Isometry3f transform);
        void updateCollisionObjects(double t);
        void liveCollisionObjectsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void start_live_obs(void);
        void inflate_obs(void);
        void resume_obs(void);
        void start_obs(double start_time=0.0);
        bool inflate = true;
        ros::Subscriber sub_live_quats;
        void stop_live_obs(void);
        void resume_live_obs(void);
        void inflate_live_obs(void);
        void pause_obs(void);
        std::vector<Eigen::Vector3f> getLinkData(double t);


};

class human_publisher {
  public:
  human_publisher(ros::NodeHandle nh, bool sim = false);
  ros::Publisher model_pub;
  ros::Publisher poses_pub;
  ros::Publisher skel_pub;
  ros::Publisher skel_quat_pub;
  ros::Time start_time;
  ros::Time pose_start_time;
  double show_human_elapsed_time;
  double pub_poses_elapsed_time;
  void show_human_thread(const ros::TimerEvent& event);
  void pub_poses_thread(const ros::TimerEvent& event);
  void empty_poses(void);
  void start_show_human(double start_time2=0.0);
  void stop_show_human(void);
  void resume_show_human(void);
  void start_poses(double start_time2=0.0);
  void stop_poses(void);
  void resume_poses(void);
  std::shared_ptr<avoidance_intervals::skeleton> skel;
  ros::Timer show_human_timer;
  ros::Timer pub_poses_timer;
  protected:
  ros::NodeHandle nh_;
};

class human_occupancy_helper {
  public:
  human_occupancy_helper(ros::NodeHandle nh);
  void set_occupancy(std::vector<Eigen::Vector3f> avoid_pts);
  std::shared_ptr<avoidance_intervals::skeleton> skel;
  protected:
  human_occupancy::OccupancyGridPtr grid_;
  ros::NodeHandle nh_;
  private:

  geometry_msgs::PoseArray pc_pose_array;
  ros::Publisher pc_pub;
  ros::Publisher centroids_pub;
  ros::Publisher pub_occ_vis;
  sensor_msgs::PointCloud pc;
  ros::Timer pub_timer;
  int occupancy_id=0;
  void set_occupancy_timer(const ros::TimerEvent& event);
};



class data_recorder {
  public:
  std::mutex jnt_pos_mtx;
  double avoid_plan_time;
  geometry_msgs::PoseArray pose_msg;
  int spd_scale=100;
  Eigen::VectorXd joint_pos_vec;
  data_recorder(ros::NodeHandle nh,std::string log_file_full_path, const planning_scene::PlanningScenePtr &planning_scene_ptr, std::shared_ptr<avoidance_intervals::skeleton> skel_ptr,moveit::core::RobotModelConstPtr robot_model,robot_state::RobotStatePtr state, std::vector<double> human_link_len, std::vector<double> human_link_radii, rosdyn::ChainPtr chain,std::string solvperflog_file_full_path);
  ~data_recorder();
  void start(void);
  void stop(void);
  std::shared_ptr<avoidance_intervals::skeleton> skel;
  robot_state::RobotStatePtr state_;
  double plan_time;
  double get_min_dist(void);
  bool ready(void) {
    return ready_;
  }
  std::vector<double> joint_positions;
  Eigen::VectorXd joint_vel_vec;
  sensor_msgs::JointState get_current_joint_state() {
    std::lock_guard<std::mutex> lock(js_mtx);
    // std::cout<<"cur_js\n";
    // std::cout<<cur_js<<std::endl;
    return cur_js;
  }
  Eigen::VectorXd get_jnt_pos_vec() {
    std::lock_guard<std::mutex> lock(jnt_pos_mtx);
    // std::cout<<"cur_js\n";
    // std::cout<<cur_js<<std::endl;
    return joint_pos_vec;
  }
  std::mutex skel_pts_mtx;
  std::vector<Eigen::Vector3f> live_human_points;
  double t = 0.0;
  protected:
  ros::NodeHandle nh_;
  int n_dof_=0;
  ros::Subscriber spd_sub;
  ros::Subscriber joint_states_sub;
  ros::Subscriber skeleton_sub;
  ros::Subscriber pose_sub;
  ros::Subscriber sub_live_quats;
  ros::Subscriber sub_dist;
  ros::Subscriber solver_perf_sub;
  ros::Subscriber human_dims_sub;
  ros::Publisher time_pub;
  ros::Time start_time;
  ros::Timer record_timer;
  std::ofstream logFile;
  std::ofstream solvperflogFile;
  std::ofstream jsLogFile;
  ros::Time log_start_time = ros::Time::now();
  std::vector<double> human_link_radii_;
  std::vector<double> human_link_len_;
  planning_scene::PlanningScenePtr ps_ptr;
  private:
  sensor_msgs::JointState cur_js;
  std::mutex js_mtx;
  bool ready_ = false;
  std::vector<Eigen::Quaternionf> live_human_quats;
  std::mutex mtx;
  std::mutex mtx2;
  std::mutex mtx3;
  std::mutex mtx5;
  std::mutex dims_mtx;
  double tangential_speed;
  std::vector<Eigen::Quaternionf> human_quats;
  geometry_msgs::PoseArray pred_human_pose;
  double min_distance;
  int log_lines=0;
  std::vector<float> camera_keypoints;
  std::vector<std::string> joint_names;
  std::vector<double> joint_velocities;
  std::vector<double> joint_effort;
  std::vector<double> joint_accelerations;
  ros::Time last_joint_state_time;
  bool joint_state_ready = false;
  std::vector<double> human_pose_pts;
  std::vector<std::string> tof_cameras;
  void spd_callback(const std_msgs::Int64::ConstPtr& msg);
  void jnt_state_callback(const sensor_msgs::JointState::ConstPtr& msg);
  void pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg);
  void skeleton_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void record_thread(const ros::TimerEvent& event);
  void dist_callback(const std_msgs::Float32::ConstPtr& msg);
  void skel_quats_cb(const std_msgs::Float32MultiArray::ConstPtr& msg);
  void perf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
  rosdyn::ChainPtr chain_;
  moveit::planning_interface::PlanningSceneInterface psi;
  std::vector<float> human_dims;
  void human_dims_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
  Eigen::Vector3f pelvis_loc;
};
