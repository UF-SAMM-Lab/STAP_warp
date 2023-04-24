#include <stap_warp/stap_warp.h>

stap_warper::stap_warper(ros::NodeHandle nh):nh(nh) {
    urdf::Model robo_model;
    robo_model.initParam("robot_description");
    std::string base_frame_ = "world";
    std::string tool_frame = "tip";
    if (!nh.getParam("/stap_warp/base_frame", base_frame_))
    {
      ROS_ERROR("%sstap_warp/base_frame not defined", nh.getNamespace().c_str());
      throw std::invalid_argument("base_frame is not defined");
    }
    if (!nh.getParam("/stap_warp/tool_frame", tool_frame))
    {
      ROS_ERROR("%sstap_warp/tool_frame not defined", nh.getNamespace().c_str());
      throw std::invalid_argument("base_frame is not defined");
    }
    ROS_INFO_STREAM("/stap_warp/base frame"<<base_frame_);
    ROS_INFO_STREAM("/stap_warp/tool_frame"<<tool_frame);
    Eigen::Vector3d grav;
    grav << 0, 0, -9.806;
    chain_ = rosdyn::createChain(robo_model, base_frame_, tool_frame, grav);
    Eigen::VectorXd q(6);
    q.setZero();
    q[1] = 0.785;
    std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> Tbl_=chain_->getTransformations(q);
    for (int i=0;i<Tbl_.size();i++) {
      std::cout<<i<<","<<Tbl_.at(i).translation().transpose()<<std::endl;
    }
    ssm=std::make_shared<ssm15066::DeterministicSSM>(chain_,nh); 
    link_names = chain_->getLinksName();
    for (int i=0;i<link_names.size();i++) std::cout<<link_names[i]<<std::endl;
    scale_time_sub = nh.subscribe<std_msgs::Float64>("/execution_ratio",1,&stap_warper::scale_time_callback,this);
    warp_pub = nh.advertise<visualization_msgs::Marker>("/warp_points",1);
    if (!nh.getParam("/warp_repulsion",repulsion))
    {
      ROS_WARN("repulsion is not defined");
    } else {
      ROS_INFO_STREAM("repulsion:"<<repulsion);
    }
    if (!nh.getParam("/warp_attraction",attraction))
    {
      ROS_WARN("warp_attraction is not defined");
    }
    ROS_INFO_STREAM("attraction:"<<attraction);
    
    if (!nh.getParam("/warp_iterations",warp_iterations))
    {
      ROS_WARN("warp_iterations is not defined");
    } else {
      ROS_INFO_STREAM("warp_iterations:"<<warp_iterations);
    }
}

void stap_warper::scale_time_callback(const std_msgs::Float64::ConstPtr& msg) {
  path_time_pct = msg->data;
}

void stap_warper::warp(moveit::planning_interface::MoveGroupInterface::Plan &plan, std::vector<std::pair<float,Eigen::MatrixXd>> &human_seq, double human_time_since_start, Eigen::VectorXd cur_pose) {
    if (path_time_pct>=1.0) return;
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

      }
      if (in_plan) {
        poses.push_back(tmp_vec);
        wp_times.push_back(plan.trajectory_.joint_trajectory.points[p].time_from_start.toSec()-start_time);
      }
      prev_pose = tmp_vec;
    }

    std::cout<<"start stap warp loop"<<std::endl;
    std::cout<<"old poses:\n";
    for (int i=0;i<poses.size();i++) std::cout<<wp_times[i]<<":"<<poses[i].transpose()<<std::endl;
    bool first_pass = false;
    for (int iter=0;iter<warp_iterations;iter++) {
        std::vector<Eigen::VectorXd> new_poses;
        std::vector<double> new_wpt_times;

        new_poses.push_back(poses.front());
        new_wpt_times.push_back(wp_times.front());
        for (int p=1;p<poses.size();p++) {
            Eigen::VectorXd diff = poses[p]-poses[p-1];
            int num_steps = 1;
            if (!first_pass) num_steps = std::max(std::ceil(diff.norm()/0.1),1.0);
            first_pass = true;
            // std::cout<<"num steps:"<<num_steps<<std::endl;
            double nominal_time = wp_times[p]-wp_times[p-1];
            // std::cout<<"nominal time:"<<nominal_time<<std::endl;
            Eigen::VectorXd dq = diff/nominal_time;
            for (int s=1;s<=num_steps;s++) {
                if ((p==poses.size()-1)&&(s==num_steps)) continue;
                if ((p==1)&&(s==0)) continue;
                double pct = (double)s/(double)num_steps;
                double nom_time = wp_times[p-1] + pct*nominal_time;
                double h_time = nom_time+human_time_since_start;
                // std::cout<<"nom time:"<<nom_time<<", human_time_since_start:"<<human_time_since_start<<",htime:"<<h_time<<std::endl;
                Eigen::VectorXd cur_pose = poses[p-1] + pct*diff;
                Eigen::VectorXd nxt_pose = cur_pose;
                if (s<num_steps) {
                  nxt_pose = poses[p-1] + (double)(s+1)/(double)num_steps*diff;
                } else {
                  nxt_pose = poses[p];
                }
                // Eigen::Matrix6Xd jacobian = chain_->getJacobian(cur_pose);
                ssm->setPointCloud(human_seq[std::max(std::min(int(round(h_time*10)),int(human_seq.size())-1),0)].second);
                std::vector<std::pair<double,Eigen::Vector3d>> scale_vects = ssm->computeScalesVectors(cur_pose,dq);
                // std::cout<<"scale_vects:"<<scale_vects.size()<<std::endl;
                Eigen::VectorXd tau(diff.size());
                tau.setZero();
                for (int j=2;j<scale_vects.size();j++) {
                  Eigen::Matrix6Xd jacobian = chain_->getJacobianLink(cur_pose,link_names[j]);
                  Eigen::VectorXd tmp_tau =1.0*repulsion*(1.0/std::max(scale_vects[j].first,0.01)-1)*jacobian.block(0,0,3,jacobian.cols()).transpose()*scale_vects[j].second;//-attraction*((new_poses.back()-cur_pose)+(nxt_pose-cur_pose));
                  // std::cout<<"tau:"<<tmp_tau.transpose()<<std::endl;
                  tau += tmp_tau;
                }
                // std::cout<<"tau sum:"<<tau.transpose()<<std::endl;
                new_poses.push_back(cur_pose-1.0*tau);
                new_wpt_times.push_back(nom_time);
            }
        }
        new_poses.push_back(poses.back());
        new_wpt_times.push_back(wp_times.back());
        poses = new_poses;
        wp_times = new_wpt_times;
        for (int i=1;i<poses.size()-1;i++) {
          poses[i] = 0.333*(poses[i-1]+poses[i]+poses[i+1]);
        }
    }

    std::cout<<"new poses:\n";
    for (int i=0;i<poses.size();i++) std::cout<<wp_times[i]<<":"<<poses[i].transpose()<<std::endl;
    visualization_msgs::Marker mkr;
    mkr.header.frame_id = "world";
    mkr.id = 10000;
    mkr.type = 4;
    mkr.pose.position.x = 0;
    mkr.pose.position.y = 0;
    mkr.pose.position.z = 0;
    mkr.pose.orientation.w = 1;
    mkr.pose.orientation.x = 0;
    mkr.pose.orientation.y = 0;
    mkr.pose.orientation.z = 0;
    mkr.color.a = 1.0;
    mkr.color.b = 1.0;
    mkr.scale.x = 0.01;
    mkr.lifetime = ros::Duration();

    warp_pub.publish(mkr);
    for (int i=0;i<poses.size();i++) {
      Eigen::Affine3d T_base_tool = chain_->getTransformation(poses[i]);
      Eigen::Vector3d ee = T_base_tool.translation();
      geometry_msgs::Point pt;
      pt.x = ee[0];
      pt.y = ee[1];
      pt.z = ee[2];
      mkr.points.push_back(pt);
    }
    warp_pub.publish(mkr);
}
