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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <mutex>

typedef Eigen::Array<bool,Eigen::Dynamic,1> ArrayXb;
namespace stap {
class stap_warper {
    public:
        stap_warper(ros::NodeHandle nh,robot_state::RobotStatePtr state, robot_model::RobotModelPtr model);
        void warp(std::vector<std::pair<float,Eigen::MatrixXd>> &human_seq, double human_time_since_start, Eigen::VectorXd cur_pose);
        void time_parameterize(trajectory_msgs::JointTrajectory &plan, std::vector<std::tuple<Eigen::ArrayXd,Eigen::ArrayXd,Eigen::ArrayXd,Eigen::ArrayXd>> &vel_profile);
    private:
        void scale_time_callback(const std_msgs::Float64::ConstPtr& msg);
        void act_traj_callback(const trajectory_msgs::JointTrajectory::ConstPtr& trj);
        ssm15066::DeterministicSSMPtr ssm;
        rosdyn::ChainPtr chain_;
        std::vector<std::string> link_names;
        ros::NodeHandle nh;
        int max_iters = 1;
        double repulsion = 0.0001;
        ros::Subscriber scale_time_sub;
        ros::Subscriber sub_act_trj;
        double path_time_pct = 0.0;
        ros::Publisher warp_pub;
        ros::Publisher blend_pub;
        int warp_iterations = 1;
        double attraction = 0.0001;
        Eigen::ArrayXd max_vels_ = Eigen::ArrayXd::Constant(6,1.5);
        Eigen::ArrayXd max_accels_ = Eigen::ArrayXd::Constant(6,1.5);
        robot_state::RobotStatePtr state;
        robot_model::RobotModelPtr model;
        moveit::planning_interface::MoveGroupInterface move_group;
        trajectory_msgs::JointTrajectory cur_traj;
        std::mutex trj_mtx;
        ros::Time last_warp_time = ros::Time::now();

};
}