#include <ros/ros.h>
#include <rosdyn_core/primitives.h>
#include <rosparam_utilities/rosparam_utilities.h>
// #include <human_probablistic_occupancy/human_probablistic_occupancy.h>
#include <velocity_scaling_iso15066/ssm15066.h>
#include <eigen3/Eigen/Core>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <avoidance_intervals/avoidance_model.h>
#include <std_msgs/Float64.h>

class stap_warper {
    public:
        stap_warper(ros::NodeHandle nh);
        void warp(moveit::planning_interface::MoveGroupInterface::Plan &plan, std::vector<std::pair<float,Eigen::MatrixXd>> &human_seq, double human_time_since_start, Eigen::VectorXd cur_pose);
    private:
        void scale_time_callback(const std_msgs::Float64::ConstPtr& msg);
        ssm15066::DeterministicSSMPtr ssm;
        rosdyn::ChainPtr chain_;
        std::vector<std::string> link_names;
        ros::NodeHandle nh;
        int max_iters = 1;
        double repulsion = 1.0;
        ros::Subscriber scale_time_sub;
        double path_time_pct = 0.0;

};