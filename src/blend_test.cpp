#include <stap_warp/utilities.h>
#include <stap_warp/stap_warp.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib_msgs/GoalID.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <iostream>
#include <fstream>
#include <string>



int main(int argc, char** argv) {
    ros::init(argc,argv,"test_blend");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    trajectory_msgs::JointTrajectory plan;
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions.resize(6);
    pt.velocities.resize(6);
    pt.positions = {0.0,-1.5,0.0,0.0,0.0,0.0};
    pt.velocities = {0.0,0.0,0.0,0.0,0.0,0.0};
    plan.points.push_back(pt);
    pt.positions = {0.6,-1.3,0.6,0.5,0.5,0.1};
    pt.velocities = {0.0,0.0,0.0,0.0,0.0,0.0};
    plan.points.push_back(pt);
    pt.positions = {1.0,-1.0,0.6,0.5,0.5,0.1};
    pt.velocities = {0.0,0.0,0.0,0.0,0.0,0.0};
    plan.points.push_back(pt);
    pt.positions = {1.3,-0.8,0.6,0.5,0.5,0.1};
    pt.velocities = {0.0,0.0,0.0,0.0,0.0,0.0};
    plan.points.push_back(pt);
    pt.positions = {1.5,0.5,0.6,0.5,0.5,0.1};
    pt.velocities = {0.0,0.0,0.0,0.0,0.0,0.0};
    plan.points.push_back(pt);
    pt.positions = {-2.0,0.1,-0.6,-0.5,-1.0,0.1};
    pt.velocities = {-0.5,-0.2,0,0,0,0};
    plan.points.push_back(pt);
    pt.positions = {0.0,0.0,0.0,0.0,0.0,0.0};
    pt.velocities = {0.0,0.0,0.0,0.0,0.0,0.0};
    plan.points.push_back(pt);

    // state = move_group.getCurrentState();
    // moveit::core::robotStateToRobotStateMsg(*state,last_exec_plan.start_state_);
    // trj.setRobotTrajectoryMsg(*state,last_exec_plan.trajectory_);
    // trj.insertWayPoint(0,state,0);
    // iptp.computeTimeStamps(trj);
    // trj.getRobotTrajectoryMsg(last_exec_plan.trajectory_);

    moveit::planning_interface::MoveGroupInterface move_group("edo");
    std::string PLANNING_GROUP="edo";
    const robot_state::JointModelGroup* joint_model_group_ = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    robot_model_loader::RobotModelLoaderPtr robot_model_loader = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
    const robot_model::RobotModelPtr& model = robot_model_loader->getModel();

    planning_scene_monitor::PlanningSceneMonitorPtr monitor;
    monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    planning_scene::PlanningScenePtr ps_ptr = monitor->getPlanningScene();

    stap::stap_warper stap_warp(nh,move_group.getCurrentState(),model,ps_ptr,"edo");
    std::vector<std::tuple<Eigen::ArrayXd,Eigen::ArrayXd,Eigen::ArrayXd,Eigen::ArrayXd>> vel_profile;
    stap_warp.time_parameterize(plan,vel_profile);
    std::cout<<plan<<std::endl;
    double dt = 0.001;
    double t = 0;
    int k = 0;
    std::ofstream stream_file;
    stream_file.open("out_stream.csv");
    Eigen::ArrayXd p = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd v = Eigen::ArrayXd::Zero(6);
    std::cout<<std::get<0>(vel_profile[0])<<std::endl;
    std::cout<<std::get<1>(vel_profile[0])<<std::endl;
    std::cout<<std::get<2>(vel_profile[0])<<std::endl;
    std::cout<<std::get<3>(vel_profile[0])<<std::endl;
    Eigen::ArrayXd accs = std::get<0>(vel_profile[0]);
    Eigen::ArrayXd decs = std::get<1>(vel_profile[0]);
    Eigen::ArrayXd acc_times = std::get<2>(vel_profile[0]);
    Eigen::ArrayXd dec_times = std::get<3>(vel_profile[0]);
    while (t<plan.points.back().time_from_start.toSec()) {
      if ((t>plan.points[k+1].time_from_start.toSec()) && (k<vel_profile.size()-1)){
        k++;
        std::cout<<k<<std::endl;
        accs = std::get<0>(vel_profile[k]);
        decs = std::get<1>(vel_profile[k]);
        acc_times = std::get<2>(vel_profile[k]);
        dec_times = std::get<3>(vel_profile[k]);
        std::cout<<accs<<std::endl;
        std::cout<<decs<<std::endl;
        std::cout<<acc_times<<std::endl;
        std::cout<<dec_times<<std::endl;
      }
      for (int q=0;q<6;q++) {
        if (t<acc_times[q]) {
          v[q] += dt*accs[q];
          p[q] += 0.5*dt*dt*accs[q]+ dt*v[q];
        } else if ((t<plan.points[k+1].time_from_start.toSec()) && (t>=dec_times[q])) {
          v[q] += dt*decs[q];
          p[q] += 0.5*dt*dt*decs[q] + dt*v[q];
        } else {
          p[q] += v[q]*dt;
        }
      }
      // std::cout<<"p"<<p<<std::endl;
      stream_file<<t<<",";
      for (int j=0;j<6;j++) stream_file << p[j]<<",";
      for (int j=0;j<6;j++) stream_file << v[j]<<",";
      stream_file<<std::endl;
      t+=dt;
      // std::cout<<"v"<<v<<std::endl;
    }
    stream_file.close();
}