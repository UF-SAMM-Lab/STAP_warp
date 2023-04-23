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
    scale_time_sub = nh.subscribe<std_msgs::Float64>("/execution_ratio",1,&stap_warper::scale_time_callback,this);
}

void stap_warper::scale_time_callback(const std_msgs::Float64::ConstPtr& msg) {
  path_time_pct = msg->data;
}

void stap_warper::warp(moveit::planning_interface::MoveGroupInterface::Plan &plan, std::vector<std::pair<float,Eigen::MatrixXd>> &human_seq, double human_time_since_start, Eigen::VectorXd cur_pose) {
    double path_time = path_time_pct*plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
    std::cout<<"start stap warp"<<path_time_pct<<","<<path_time<<std::endl;
    int start_p = 1;
    std::vector<Eigen::VectorXd> poses;
    std::vector<double> wp_times;
    poses.reserve(plan.trajectory_.joint_trajectory.points.size());
    wp_times.reserve(plan.trajectory_.joint_trajectory.points.size());
    bool in_plan = false;
    std::vector<double> pos = plan.trajectory_.joint_trajectory.points[0].positions;
    Eigen::VectorXd prev_pose = Eigen::VectorXd::Zero(pos.size());
    for (int i=0;i<pos.size();i++) prev_pose[i] = pos[i];
    double start_time;
    for (int p=1;p<plan.trajectory_.joint_trajectory.points.size();p++) {
      pos = std::vector<double>(plan.trajectory_.joint_trajectory.points[p].positions);
      Eigen::VectorXd tmp_vec = Eigen::VectorXd::Zero(pos.size());
      for (int i=0;i<pos.size();i++) tmp_vec[i] = pos[i];
      if (!in_plan) {
        // std::cout<<plan.trajectory_.joint_trajectory.points[p-1].time_from_start.toSec()<<","<<plan.trajectory_.joint_trajectory.points[p].time_from_start.toSec()<<","<<path_time<<std::endl;
        if ((path_time>=plan.trajectory_.joint_trajectory.points[p-1].time_from_start.toSec())&&(path_time<plan.trajectory_.joint_trajectory.points[p].time_from_start.toSec())) {
          start_time = path_time;
          poses.push_back(cur_pose);
          wp_times.push_back(0.0);
          in_plan = true;
        }
        // std::cout<<(prev_pose).transpose()<<"->"<<(tmp_vec).transpose()<<std::endl;
        // std::cout<<"cur:"<<cur_pose.transpose()<<std::endl;
        // Eigen::VectorXd diff1 = 1000*(cur_pose-tmp_vec);
        // Eigen::VectorXd diff2 = 1000*(cur_pose-prev_pose);
        // Eigen::ArrayXi sgn1 = diff1.array().round().sign().matrix().cast<int>().array();
        // Eigen::ArrayXi sgn2 = diff2.array().round().sign().matrix().cast<int>().array();
        // int sgn1_non_zero = sgn1.abs().sum();
        // int sgn2_non_zero = sgn2.abs().sum();
        // // Eigen::ArrayXi sgn3 = sgn1 * sgn2;

        // std::cout<<sgn1<<",\n"<<sgn2<<",\n"<<(sgn1*sgn2)<<std::endl;
        // std::cout<<"sgn1 non zero"<<sgn1_non_zero<<", sgn2:"<<sgn2_non_zero<<std::endl;
        // std::cout<<(sgn1*sgn2).sum()<<std::endl;
        // if (sgn1.cwiseProduct(sgn2).sum()==-1*std::min(sgn1_non_zero,sgn2_non_zero)) {
        //   poses.push_back(cur_pose);
        //   double pct = (cur_pose-prev_pose).norm()/(tmp_vec-prev_pose).norm();
        //   std::cout<<"p:"<<p<<" pct:"<<pct<<std::endl;
        //   start_time = pct*(plan.trajectory_.joint_trajectory.points[p].time_from_start-plan.trajectory_.joint_trajectory.points[p-1].time_from_start).toSec()+plan.trajectory_.joint_trajectory.points[p-1].time_from_start.toSec();
        //   wp_times.push_back(0.0);
        //   in_plan = true;
        // } else if ((cur_pose-prev_pose).norm()==0) {
        //   poses.push_back(cur_pose);
        //   start_time = plan.trajectory_.joint_trajectory.points[p-1].time_from_start.toSec();
        //   wp_times.push_back(0.0);
        //   in_plan = true;
        // }


      }
      if (in_plan) {
        poses.push_back(tmp_vec);
        wp_times.push_back(plan.trajectory_.joint_trajectory.points[p].time_from_start.toSec()-start_time);
      }
      prev_pose = tmp_vec;
    }
    // pos = std::vector<double>(plan.trajectory_.joint_trajectory.points.back().positions);
    // Eigen::Map<Eigen::VectorXd> tmp_vec(&pos[0], pos.size());
    // poses.push_back(tmp_vec);
    // wp_times.push_back(plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec());
    std::cout<<"poses:"<<poses.size()<<std::endl;
    std::cout<<"start stap warp loop"<<std::endl;
    for (int iter=0;iter<max_iters;iter++) {
        for (int p=1;p<poses.size();p++) {
            Eigen::VectorXd diff = poses[p]-poses[p-1];
            int num_steps = std::max(std::ceil(diff.norm()/0.1),1.0);
            double nominal_time = wp_times[p]-wp_times[p-1];
            std::cout<<"nominal time:"<<nominal_time<<std::endl;
            Eigen::VectorXd dq = diff/nominal_time;
            for (int s=0;s<num_steps;s++) {
                double h_time = wp_times[p-1] + double(s/num_steps)*nominal_time-human_time_since_start;
                std::cout<<"htime:"<<h_time<<std::endl;
                Eigen::VectorXd cur_pose = poses[p-1] + double(s/num_steps)*diff;
                // Eigen::Matrix6Xd jacobian = chain_->getJacobian(cur_pose);
                ssm->setPointCloud(human_seq[std::max(std::min(int(round(h_time*10)),int(human_seq.size())-1),0)].second);
                std::vector<std::pair<double,Eigen::Vector3d>> scale_vects = ssm->computeScalesVectors(cur_pose,dq);
                std::cout<<"scale_vects:"<<scale_vects.size()<<std::endl;
                Eigen::VectorXd tau(diff.size());
                for (int j=0;j<dq.size();j++) {
                  Eigen::Matrix6Xd jacobian = chain_->getJacobianLink(cur_pose,link_names[j]);
                  tau += repulsion*std::max(1-scale_vects[j].first,0.0)*jacobian.block(0,0,3,jacobian.cols()).transpose()*scale_vects[j].second;
                }
                std::cout<<"tau:"<<tau.transpose()<<std::endl;
            }
        }
    }
}
