#include <ros/ros.h>
#include <string>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <stap_warp/human_prediction.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>
#include <stap_warp/joint_seq.h>
#include <stap_warp/joint_seq_elem.h>

namespace stap_test {
class human {
    public:
        human(ros::NodeHandle nh, int idx, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub);
        void get_predicted_motion(std::vector<float> start_pose);
        void show_human(std::vector<float> link_len,std::vector<float> link_r);
        std::vector<float> get_last_pose(void) {return end_pose;}
        float get_start_delay(void) {return start_delay;}
        int get_prior_robot_task(void) {return prior_robot_task;}
        std::tuple<float,std::vector<float>,std::vector<float>> get_seq(int i) {return sequence[i];}
        int get_seq_size(void) {return sequence.size();}
    private:
        int id = 0;
        int prior_robot_task = -1;
        std::string description = "";
        float start_delay = 0.0;
        std::vector<float> reach_target;
        float end_delay = 0.0;
        bool arm_right = true;
        bool arm_left = false;
        std::vector<std::tuple<float,std::vector<float>,std::vector<float>>> sequence;
        std::vector<float> end_pose;
        std::shared_ptr<ros::ServiceClient> predictor;
        std::shared_ptr<ros::Publisher> seq_pub;
        void forward_kinematics(std::vector<float> pose_elements, std::vector<Eigen::Vector3f> &link_centroids, std::vector<Eigen::Quaternionf> &link_quats, std::vector<Eigen::Vector3f>& human_points);
        std::vector<float> link_lengths_;
        std::vector<float> radii;
};
class humans {
    public:
        humans(ros::NodeHandle nh, std::vector<float> cur_pose, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub);
        void predicted_motion(void);
        void show_predictions(std::vector<float> link_len,std::vector<float> link_r);
        float pub_model(int start_seq, int robot_step, float start_tm_in_robot_seq);
        float human_start_delay(int human) {
            if ((human<0)||(human>=data.size())) return 0.0;
            return data[human].get_start_delay();
        }
        int human_prior_robot_task(int human) {
            if ((human<0)||(human>=data.size())) return 0.0;
            return data[human].get_prior_robot_task();
        }
        int get_num_steps(void) {return data.size();}
    private:
        ros::Publisher human_model_pub;
        int num_steps = 0;
        std::vector<human> data;
        std::vector<float> start_pose;
        std::shared_ptr<ros::ServiceClient> predictor;
        ros::NodeHandle nh;
};
class robot_segment {
    public:
        robot_segment(ros::NodeHandle nh, int idx);
        double planned_time = 0.0;
        int get_goal_id(void){return goal_id;}
        moveit::planning_interface::MoveGroupInterface::Plan plan;
    private:
        int id = 0;
        std::string description = "";
        int type = 0;
        int prior_human_task = -1;
        double start_delay = 0.0;
        int goal_id;
};
class robot_sequence {
    public:
        robot_sequence(ros::NodeHandle nh, robot_state::RobotStatePtr state, robot_model::RobotModelPtr model, std::string plan_group);
        double plan_robot_segment(int seg_num, std::vector<double>& start_joint);
        int num_segments(void) {
            return data.size();
        }
    private:
        robot_state::RobotStatePtr state;
        robot_model::RobotModelPtr model;
        moveit::planning_interface::MoveGroupInterface move_group;
        int num_steps = 0;
        std::vector<robot_segment> data;
        std::vector<std::vector<double>> goals_angles;
        ros::NodeHandle nh;
        ros::Subscriber planner_sub;
        void perf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
        double plan_time = 0.0;
        ros::Time last_perf_received;
};
}