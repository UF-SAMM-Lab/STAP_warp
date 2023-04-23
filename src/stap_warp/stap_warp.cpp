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
    warp_pub = nh.advertise<visualization_msgs::Marker>("/warp_points",1);
    if (!nh.getParam("/warp_repulsion",repulsion))
    {
      ROS_WARN("repulsion is not defined");
    }
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

      }
      if (in_plan) {
        poses.push_back(tmp_vec);
        wp_times.push_back(plan.trajectory_.joint_trajectory.points[p].time_from_start.toSec()-start_time);
      }
      prev_pose = tmp_vec;
    }

    std::cout<<"start stap warp loop"<<std::endl;
    for (int iter=0;iter<max_iters;iter++) {
        std::vector<Eigen::VectorXd> new_poses;
        std::vector<double> new_wpt_times;
        std::cout<<"old poses:\n";
        for (int i=0;i<poses.size();i++) std::cout<<wp_times[i]<<":"<<poses[i].transpose()<<std::endl;
        for (int p=1;p<poses.size();p++) {
            Eigen::VectorXd diff = poses[p]-poses[p-1];
            int num_steps = std::max(std::ceil(diff.norm()/0.1),1.0);
            double nominal_time = wp_times[p]-wp_times[p-1];
            // std::cout<<"nominal time:"<<nominal_time<<std::endl;
            Eigen::VectorXd dq = diff/nominal_time;
            for (int s=0;s<num_steps;s++) {
                double pct = (double)s/(double)num_steps;
                double nom_time = wp_times[p-1] + pct*nominal_time;
                double h_time = nom_time-human_time_since_start;
                // std::cout<<"htime:"<<h_time<<std::endl;
                Eigen::VectorXd cur_pose = poses[p-1] + pct*diff;
                // Eigen::Matrix6Xd jacobian = chain_->getJacobian(cur_pose);
                ssm->setPointCloud(human_seq[std::max(std::min(int(round(h_time*10)),int(human_seq.size())-1),0)].second);
                std::vector<std::pair<double,Eigen::Vector3d>> scale_vects = ssm->computeScalesVectors(cur_pose,dq);
                // std::cout<<"scale_vects:"<<scale_vects.size()<<std::endl;
                Eigen::VectorXd tau(diff.size());
                for (int j=0;j<dq.size();j++) {
                  Eigen::Matrix6Xd jacobian = chain_->getJacobianLink(cur_pose,link_names[j]);
                  tau += 1.0*repulsion*std::max(1-scale_vects[j].first,0.0)*jacobian.block(0,0,3,jacobian.cols()).transpose()*scale_vects[j].second;
                }
                std::cout<<tau<<std::endl;
                new_poses.push_back(cur_pose-1.0*tau);
                new_wpt_times.push_back(nom_time);
            }
        }
        poses = new_poses;
        wp_times = new_wpt_times;
        std::cout<<"new poses:\n";
        for (int i=0;i<poses.size();i++) std::cout<<wp_times[i]<<":"<<poses[i].transpose()<<std::endl;
    }
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
