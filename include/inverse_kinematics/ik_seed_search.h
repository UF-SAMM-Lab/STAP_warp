//ik_seed_search.h
#include <ros/ros.h>
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace ik_search {
    bool float_vectors_equal(std::vector<double> v1, std::vector<double> v2);

    bool vector_contains_vector(std::vector<double> v1, std::vector<std::vector<double>> vecs);

    std::vector<std::vector<double>> ik_search(robot_state::RobotState robot_pose,std::string plan_group_name, const robot_state::JointModelGroup* joint_model_group_, geometry_msgs::Pose goal_pose);

    std::vector<double> ik_closest_sln(std::vector<double>current_angles, std::vector<std::vector<double>> all_ik_slns);
}