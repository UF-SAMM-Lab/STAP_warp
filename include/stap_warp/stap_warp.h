#include <ros/ros.h>
#include <rosdyn_core/primitives.h>
#include <rosparam_utilities/rosparam_utilities.h>
// #include <human_probablistic_occupancy/human_probablistic_occupancy.h>
#include <velocity_scaling_iso15066/ssm15066.h>
#include <eigen3/Eigen/Core>

class stap_warper {
    public:
        stap_warper(ros::NodeHandle nh);
        void warp(moveit::planning_interface::MoveGroupInterface::Plan &plan, Eigen::MatrixXd co_pts);
    private:
        ssm15066::DeterministicSSMPtr ssm;

};