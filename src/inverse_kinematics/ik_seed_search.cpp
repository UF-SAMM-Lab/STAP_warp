#include <inverse_kinematics/ik_seed_search.h>

namespace ik_search {
    bool float_vectors_equal(std::vector<double> v1, std::vector<double> v2) {
        if (v1.size()!=v2.size()) {
            return false;
        } else {
            for (int i=0; i<int(v1.size());i++) {
                if (abs(v1[i]-v2[i])>0.01) {
                    return false;
                }
            }
            return true;
        }
    }

    bool vector_contains_vector(std::vector<double> v1, std::vector<std::vector<double>> vecs) {
        if (vecs.empty()|v1.empty()) {
            return false;
        } else {
            for (int i=0;i<int(vecs.size());i++) {
                if (float_vectors_equal(v1,vecs.at(i))) {
                    return true;
                }
            }
            return false;
        }
    }

    std::vector<std::vector<double>> ik_search(robot_state::RobotState robot_pose,std::string plan_group_name, const robot_state::JointModelGroup* joint_model_group_, geometry_msgs::Pose goal_pose) {
        std::vector<double> joint_angle_seed;
        std::vector<std::vector<double>> solutions;
        std::vector<double> sln_angles;
        for (int j1=0;j1<5;j1++) {
            for (int j2=0;j2<5;j2++) {
                for (int j3=0;j3<5;j3++) {
                    // std::cout<<j1<<","<<j2<<","<<j3<<std::endl;
                    //joint_angle_seed = {j1*6.28-3.14,j2*2.09-1.045,j3*3.49-1.745,0,0,0};
                    joint_angle_seed = {j1*0.2*6.28-3.14,j2*0.2*2.09-1.045,j3*0.2*3.49-1.745,0,0,0};
                    // joint_angle_seed = {0,0,0,0,0,0};
                    robot_pose.setJointGroupPositions(plan_group_name,joint_angle_seed);
                    // std::cout<<goal_pose<<std::endl;
                    if (robot_pose.setFromIK(joint_model_group_,goal_pose)) {
                        robot_pose.copyJointGroupPositions(plan_group_name,sln_angles);
                        if (solutions.empty()|(!vector_contains_vector(sln_angles,solutions))) {
                            solutions.push_back(sln_angles);
                        }
                    } else if ((j1==0) & (j2==0) & (j3==0)) {
                        return solutions;
                    }
                }
            }
        }
        return solutions;
    }
    std::vector<double> ik_closest_sln(std::vector<double>current_angles, std::vector<std::vector<double>> all_ik_slns) {
        double min_diff = 100;
        double sum_diff;
        std::vector<double> solution;
        for (int i=0;i<int(all_ik_slns.size());i++) {
            sum_diff = 0;
            for (int j=0;j<int(all_ik_slns.at(i).size());j++) {
                sum_diff = sum_diff+abs(all_ik_slns.at(i).at(j)-current_angles[j]);
            }
            if (sum_diff<min_diff) {
                min_diff = sum_diff;
                solution = all_ik_slns.at(i);
            }
        }
        return solution;
    }
}