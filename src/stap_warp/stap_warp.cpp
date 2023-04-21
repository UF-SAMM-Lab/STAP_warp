#include <stap_warp/stap_warp.h>

stap_warper::stap_warper(ros::NodeHandle nh):nh(nh) {
    urdf::Model robo_model;
    robo_model.initParam("robot_description");
    std::string base_frame_ = "world";
    std::string tool_frame = "tip";
    if (!nh.getParam("base_frame", base_frame_))
    {
      ROS_ERROR("%s/base_frame not defined", nh.getNamespace().c_str());
      throw std::invalid_argument("base_frame is not defined");
    }
    if (!nh.getParam("tool_frame", tool_frame))
    {
      ROS_ERROR("%s/tool_frame not defined", nh.getNamespace().c_str());
      throw std::invalid_argument("base_frame is not defined");
    }
    Eigen::Vector3d grav;
    grav << 0, 0, -9.806;
    chain_ = rosdyn::createChain(robo_model, base_frame_, tool_frame, grav);
    ssm=std::make_shared<ssm15066::DeterministicSSM>(chain_,nh); 
    link_names = chain_->getLinksName();
}

void stap_warper::warp(moveit::planning_interface::MoveGroupInterface::Plan &plan, std::vector<std::pair<float,Eigen::MatrixXd>> human_seq, double time_since_start) {
    int start_p = 1;
    std::vector<Eigen::VectorXd> poses;
    std::vector<double> wp_times;
    poses.reserve(plan.trajectory_.joint_trajectory.points.size());
    wp_times.reserve(plan.trajectory_.joint_trajectory.points.size());
    for (int p=1;p<plan.trajectory_.joint_trajectory.points.size();p++) {
        if (plan.trajectory_.joint_trajectory.points[p].time_from_start.toSec()>time_since_start) {
          std::vector<double> pos = plan.trajectory_.joint_trajectory.points[p-1].positions;
          Eigen::Map<Eigen::VectorXd> tmp_vec(&pos[0], pos.size());
          poses.push_back(tmp_vec);
          wp_times.push_back(plan.trajectory_.joint_trajectory.points[p-1].time_from_start.toSec());
        }
    }
    std::vector<double> pos = plan.trajectory_.joint_trajectory.points.back().positions;
    Eigen::Map<Eigen::VectorXd> tmp_vec(&pos[0], pos.size());
    poses.push_back(tmp_vec);
    wp_times.push_back(plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec());

    for (int iter=0;iter<max_iters;iter++) {
        for (int p=1;p<poses.size();p++) {
            Eigen::VectorXd diff = poses[p]-poses[p-1];
            int num_steps = std::max(std::ceil(diff.norm()/0.1),1.0);
            double nominal_time = wp_times[p]-wp_times[p-1];
            Eigen::VectorXd dq = diff/nominal_time;
            for (int s=0;s<num_steps;s++) {
                double h_time = wp_times[p] + double(s/num_steps)*nominal_time-time_since_start;
                Eigen::VectorXd cur_pose = poses[p-1] + double(s/num_steps)*diff;
                // Eigen::Matrix6Xd jacobian = chain_->getJacobian(cur_pose);
                ssm->setPointCloud(human_seq[std::min((int)round(h_time*10),int(human_seq.size())-1)].second);
                std::vector<std::pair<double,Eigen::VectorXd>> scale_vects = ssm->computeScalesVectors(cur_pose,dq);
                std::cout<<"scale_vects:"<<scale_vects.size()<<std::endl;
                Eigen::VectorXd tau(diff.size());
                for (int j=0;j<dq.size();j++) {
                  Eigen::Matrix6Xd jacobian = chain_->getJacobianLink(cur_pose,link_names[j]);
                  tau += repulsion*std::max(1-scale_vects[j].first,0.0)*jacobian.block(0,0,3,jacobian.cols()).transpose()*scale_vects[j].second;
                }
            }
        }
    }
}
