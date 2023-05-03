#include <ros/ros.h>
#include <string>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <stap_warp/human_prediction.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace stap_test {
class human {
    public:
        human(ros::NodeHandle nh, int idx, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub);
        void get_predicted_motion(std::vector<float> start_pose);
        void show_human(std::vector<float> link_len,std::vector<float> link_r);
        std::vector<float> get_last_pose(void) {
            return end_pose;
        }
    private:
        int id = 0;
        int prior_robot_task = -1;
        std::string description = "";
        double start_delay = 0.0;
        std::vector<float> reach_target;
        double end_delay = 0.0;
        bool arm_right = true;
        bool arm_left = false;
        std::vector<std::pair<float,std::vector<float>>> sequence;
        std::vector<float> end_pose;
        std::shared_ptr<ros::ServiceClient> predictor;
        std::shared_ptr<ros::Publisher> seq_pub;
        void forward_kinematics(std::vector<float> pose_elements, std::vector<Eigen::Vector3f> &link_centroids, std::vector<Eigen::Quaternionf> &link_quats);
        std::vector<float> link_lengths_;
        std::vector<float> radii;
};
class humans {
    public:
        humans(ros::NodeHandle nh, std::vector<float> cur_pose, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub);
        void predicted_motion(void);
        void show_predictions(std::vector<float> link_len,std::vector<float> link_r);
    private:
        int num_steps = 0;
        std::vector<human> data;
        std::vector<float> start_pose;
        std::shared_ptr<ros::ServiceClient> predictor;
    protected:
        ros::NodeHandle nh;
};
class robot_segment {
    public:
        robot_segment(ros::NodeHandle nh, int idx);
    private:
        int id = 0;
        std::string description = "";
        int type = 0;
        int prior_human_task = -1;
        double start_delay = 0.0;
        int goal_id;
        moveit::planning_interface::MoveGroupInterface::Plan plan;
};
class robot_sequence {
    public:
        robot_sequence(ros::NodeHandle nh);
    private:
        int num_steps = 0;
        std::vector<robot_segment> data;
        std::vector<std::vector<double>> goals_angles;
    protected:
        ros::NodeHandle nh;
};
}