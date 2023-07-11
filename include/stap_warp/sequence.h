#pragma once
#include <ros/ros.h>
#include <string>
#include <memory>
#include <thread>
#include <mutex>
#include <moveit/move_group_interface/move_group_interface.h>
#include <stap_warp/human_prediction.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <stap_warp/joint_seq.h>
#include <stap_warp/joint_seq_elem.h>
#include <stap_warp/utilities.h>
#include <stap_warp/stap_warp.h>
#include <stap_warp/human_motion_done.h>
#include <stap_warp/human_motion_reset.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib_msgs/GoalID.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <unistd.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64.h>
#include <filesystem>

namespace fs = std::filesystem;

namespace stap_test {

jsk_rviz_plugins::OverlayText gen_overlay_text(std::string txt);
std::vector<float> transform_pose_to_UF(std::vector<float> input_pose, Eigen::Isometry3f transform_to_world_inv);
std::vector<float> transform_pose_to_SW(std::vector<float> input_pose, Eigen::Isometry3f transform_to_world);
std::vector<float> transform_point_to_SW(std::vector<float> p, Eigen::Isometry3f transform_to_world);
std::vector<float> transform_point_to_UF(std::vector<float> p, Eigen::Isometry3f transform_to_world_inv);

class human {
    public:
        human(ros::NodeHandle nh, int idx, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub, float prediction_dt, int test_num, Eigen::Isometry3f transform_to_world);
        void get_predicted_motion(std::vector<float> start_pose);
        void show_human(std::vector<float> link_len,std::vector<float> link_r);
        void get_last_pose(std::vector<float>& pose) {
            if (sequence.empty()) return;
            pose = std::get<2>(sequence.back()); 
        }
        float get_start_delay(void) {return start_delay;}
        int get_prior_robot_task(void) {return prior_robot_task;}
        std::tuple<float,std::vector<float>,std::vector<float>,Eigen::MatrixXd> get_seq(int i);
        std::vector<std::tuple<float,std::vector<float>,std::vector<float>,Eigen::MatrixXd>> get_nominal_seq() {return nom_sequence;}
        int get_seq_size(void);
        std::vector<float> get_tgt(void) {return reach_target;}
        bool arm_right = true;
        bool arm_left = false;
        bool both_arms = false;
        void set_dimensions(std::vector<float> link_len,std::vector<float> link_r) {
            link_lengths_ = link_len;
            radii = link_r;
        }
        void forward_kinematics(std::vector<float> pose_elements, std::vector<Eigen::Vector3f>& link_centroids, std::vector<Eigen::Quaternionf>& link_quats, Eigen::Matrix3Xd& human_points);
        std::vector<float> link_lengths_;
        std::vector<float> radii;
        std::string description = "";
        bool check_pos = false;
        std::vector<float> reach_target_left;
        std::vector<float> reach_target_right;
        std::vector<float> reach_target;
        float get_step_end_time(void) {return std::get<0>(nom_sequence.back());}
        void show_step(int step_num);
        void set_nominal_seq(void) {nom_sequence=sequence;}
        bool done = false;
    private:
        int id = 0;
        int prior_robot_task = -1;
        float start_delay = 0.0;
        float end_delay = 0.0;
        std::vector<std::tuple<float,std::vector<float>,std::vector<float>,Eigen::MatrixXd>> sequence;
        std::vector<float> end_pose;
        std::shared_ptr<ros::ServiceClient> predictor;
        std::shared_ptr<ros::Publisher> seq_pub;
        float prediction_dt = 0.1;
        int show_human_rate = 50;
        std::vector<std::tuple<float,std::vector<float>,std::vector<float>,Eigen::MatrixXd>> nom_sequence;
        Eigen::Isometry3f transform_to_world;
        Eigen::Isometry3f transform_to_world_inv;
        std::vector<float> transform_pose_to_UF(std::vector<float> input_pose);
        std::vector<float> transform_point_to_SW(std::vector<float> p);
        std::vector<float> transform_point_to_UF(std::vector<float> p);
        std::vector<float> transform_pose_to_SW(std::vector<float> input_pose);
};
class humans {
    public:
        humans(ros::NodeHandle nh, std::vector<float> cur_pose, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub,std::shared_ptr<avoidance_intervals::skeleton> skel, std::shared_ptr<ros::Publisher> pub_txt, int test_num,Eigen::Isometry3f transform_to_world);
        void predicted_motion(void);
        void show_predictions(std::vector<float> link_len,std::vector<float> link_r);
        float pub_model(int start_seq, int robot_step, float start_tm_in_robot_seq);
        void generate_full_sequence(int start_seq, int robot_step, float start_tm_in_robot_seq,bool skip_first_delay=false);
        float human_start_delay(int human) {
            if ((human<0)||(human>=data.size())) return 0.0;
            return data[human].get_start_delay();
        }
        int human_prior_robot_task(int human) {
            if ((human<0)||(human>=data.size())) return 0.0;
            return data[human].get_prior_robot_task();
        }
        int get_num_steps(void) {return data.size();}
        std::vector<std::pair<float,Eigen::MatrixXd>> full_joint_seq;
        std::vector<std::pair<float,std::vector<Eigen::Vector3f>>> full_joint_seq_f;
        std::vector<std::pair<float,std::vector<Eigen::Quaternionf>>> full_quat_seq;
        bool is_step_done(int step);
        void update_predictions(int cur_step, std::vector<float> cur_pose, int robot_step, double current_robot_time);
        std::mutex joint_seq_mtx;
        void reset_motion_done(void);
        void set_dimensions(std::vector<float> link_len,std::vector<float> link_r) {
            for (int i=0;i<data.size();i++) data[i].set_dimensions(link_len,link_r);
        }
        void show_sequence(void);
        std::vector<double> sim_switch_times;
        void save_full_seq(std::string file_name);
        float get_step_end_time(int step_num) {
            if ((step_num<0)||(step_num>data.size())) return 0.0;
            return data[step_num].get_step_end_time();
        }
        bool simulate_step(int step_num, double elapsed_time, std::vector<float>& current_pose);
        void show_reach_tgt(int step_num);
        void pub_descrition(int step_num);
    private:
        ros::Publisher human_model_pub;
        int num_steps = 0;
        std::vector<human> data;
        std::vector<float> start_pose;
        std::shared_ptr<ros::ServiceClient> predictor;
        std::shared_ptr<ros::Publisher> seq_pub;
        ros::NodeHandle nh;
        std::shared_ptr<avoidance_intervals::skeleton> skel;
        ros::ServiceClient human_done_srv;
        ros::ServiceClient human_reset_srv;
        float prediction_dt = 0.1;
        std::shared_ptr<ros::Publisher> pub_txt;
        ros::Publisher wrist_trace_pub;
        ros::Publisher reach_tgt_pub;
        Eigen::Isometry3f transform_to_world;
        Eigen::Isometry3f transform_to_world_inv;
};
class robot_segment {
    public:
        robot_segment(ros::NodeHandle nh, int idx, int test_num);
        double planned_time = 0.0;
        int get_goal_id(void){return goal_id;}
        int get_type(void){return type;}
        int get_prior_human_task(void){return prior_human_task;}
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        std::string pipeline;
        std::string planner;
        std::string description = "";
        double get_gripper_pct() {return gripper_pct;}
        bool continuously_replan = true;
    private:
        int id = 0;
        int type = 0;
        int prior_human_task = -1;
        double start_delay = 0.0;
        int goal_id;
        double gripper_pct=0.0;
};
class robot_sequence {
    public:
        robot_sequence(ros::NodeHandle nh, robot_state::RobotStatePtr state, robot_model::RobotModelPtr model, std::string plan_group, std::shared_ptr<stap_test::humans> human_data,std::shared_ptr<data_recorder> rec,const planning_scene::PlanningScenePtr &planning_scene_, std::shared_ptr<ros::Publisher> pub_txt);
        double plan_robot_segment(int seg_num, std::vector<double>& start_joint);
        int num_segments(void) {
            return data.size();
        }
        bool do_segment(int seg_num);
        bool is_segment_active(void) {return segment_active;}
        ~robot_sequence() {
            if (segment_thread.joinable()) segment_thread.join();
        }
        int get_prior_human_step(int seg_num);
        void set_gripper(bool open);
    private:
        robot_state::RobotStatePtr state;
        robot_model::RobotModelPtr model;
        moveit::planning_interface::MoveGroupInterface move_group;
        int num_steps = 0;
        std::vector<robot_segment> data;
        std::vector<std::vector<double>> goals_angles;
        ros::NodeHandle nh;
        ros::Subscriber planner_sub;
        ros::Subscriber sub_response;
        bool wait_gripper_response = false;
        ros::Publisher nom_plan_pub;
        ros::Publisher grip_pos_pub;
        ros::Publisher grip_pub;
        void perf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        double plan_time = 0.0;
        ros::Time last_perf_received;
        std::shared_ptr<stap_test::humans> human_data;
        std::shared_ptr<data_recorder> rec;
        void segment_thread_fn(int seg_num);
        stap::stap_warper stap_warper;
        std::thread segment_thread;
        bool segment_active = false;
        std::shared_ptr<ros::Publisher> pub_txt;
        std::mutex goal_mtx;
        std::string goal_id;
        ros::Subscriber sub_goal;
        ros::Publisher pub_cancel_traj;
        void goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg);
        actionlib_msgs::GoalID goal_id_msg;
        ros::Publisher centroids_pub;
        ros::Publisher pc_pub;
        human_occupancy::OccupancyGridPtr grid_;
        void set_occupancy(std::vector<Eigen::Vector3f> avoid_pts);
        std::string plan_group;
        std::vector<std::string> link_names;
        bool output_positions = false;
        std::string tip_link = "";
        double tip_offset = 0.05;
        int test_num = 1;
        bool use_warp = false;
        bool gripper_done = false;
        void grip_resp_callback(const std_msgs::Bool::ConstPtr& msg);
        double global_override = 0.5;
        void speed_ovr_callback(const std_msgs::Int64::ConstPtr& msg);
        ros::Subscriber sub_ovr;
        int path_set_num = 0;
};
}