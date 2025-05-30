#include <stap_warp/stap_warp.h>
namespace stap {
stap_warper::stap_warper(ros::NodeHandle nh, robot_state::RobotStatePtr state, robot_model::RobotModelPtr model,const planning_scene::PlanningScenePtr &planning_scene_, std::string plan_group):nh(nh),state(state),model(model),plan_group(plan_group),move_group(plan_group) {
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
    q_max = chain_->getQMax();
    q_min = chain_->getQMin();
    std::cout<<"q_max:"<<q_max.transpose()<<",q_min:"<<q_min.transpose()<<std::endl;
    Eigen::VectorXd q(6);
    q.setZero();
    q[1] = 0.785;
    std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> Tbl_=chain_->getTransformations(q);
    for (int i=0;i<Tbl_.size();i++) {
      std::cout<<i<<","<<Tbl_.at(i).translation().transpose()<<std::endl;
    }
    ros::NodeHandle ssm_nh("stap_warp");
    double min_dist = 0.2;
    nh.getParam("/warp/minimum_distance", min_dist);
    double react_time = 0.15;
    nh.getParam("/warp/reaction_time", react_time);
    ssm_nh.setParam("minimum_distance",min_dist);
    ssm_nh.setParam("reaction_time",react_time);
    ssm=std::make_shared<ssm15066::DeterministicSSM>(chain_,ssm_nh); 
    link_names = chain_->getLinksName();
    for (int i=0;i<link_names.size();i++) std::cout<<link_names[i]<<std::endl;
    scale_time_sub = nh.subscribe<std_msgs::Float64>("/execution_ratio",1,&stap_warper::scale_time_callback,this);
    warp_pub = nh.advertise<visualization_msgs::Marker>("/warp_points",1);
    warp_pub2 = nh.advertise<visualization_msgs::Marker>("/warp_points2",1);
    warp_pub3 = nh.advertise<visualization_msgs::Marker>("/warp_points3",1);
    warp_pub4 = nh.advertise<visualization_msgs::Marker>("/warp_points4",1);
    blend_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/planner_hw/microinterpolator/blend_trajectory",1);
    sub_act_trj = nh.subscribe<trajectory_msgs::JointTrajectory>("/planner_hw/microinterpolator/act_trajectory",1,&stap_warper::act_traj_callback,this);
    if (!nh.getParam("/warp/repulsion",repulsion))
    {
      ROS_WARN("repulsion is not defined");
    } else {
      ROS_INFO_STREAM("repulsion:"<<repulsion);
    }
    if (!nh.getParam("/warp/table_repulsion",table_repulsion))
    {
      ROS_WARN("/warp/table_repulsion is not defined");
    } else {
      ROS_INFO_STREAM("table_repulsion:"<<table_repulsion);
    }
    if (!nh.getParam("/warp/attraction",attraction))
    {
      ROS_WARN("warp/attraction is not defined");
    }
    ROS_INFO_STREAM("attraction:"<<attraction);
    
    if (!nh.getParam("/warp/iterations",warp_iterations))
    {
      ROS_WARN("warp/iterations is not defined");
    } else {
      ROS_INFO_STREAM("warp/iterations:"<<warp_iterations);
    }    
    if (!nh.getParam("/warp/smooth_steps",smooth_steps))
    {
      ROS_WARN("warp/smooth_steps is not defined");
    } else {
      ROS_INFO_STREAM("smooth_steps:"<<smooth_steps);
    }
    if (!nh.getParam("/warp/table_tolerance",table_tolerance))
    {
      ROS_WARN("warp/table_tolerance is not defined");
    } else {
      ROS_INFO_STREAM("table_tolerance:"<<table_tolerance);
    }
    if (!nh.getParam("/warp/connection_tol",connection_tol))
    {
      ROS_WARN("warp/connection_tol is not defined");
    } else {
      ROS_INFO_STREAM("connection_tol:"<<connection_tol);
    }
    if (!nh.getParam("/warp/cycle_time",cycle_time))
    {
      ROS_WARN("warp/cycle_time is not defined");
    } else {
      ROS_INFO_STREAM("cycle_time:"<<cycle_time);
    }
    if (!nh.getParam("/warp/direct_path_attraction",direct_path_attraction))
    {
      ROS_WARN("warp/direct_path_attraction is not defined");
    } else {
      ROS_INFO_STREAM("direct_path_attraction:"<<direct_path_attraction);
    }
    if (!nh.getParam("/warp/connection_min_dist",connection_min_dist))
    {
      ROS_WARN("warp/connection_min_dist is not defined");
    } else {
      ROS_INFO_STREAM("connection_min_dist:"<<connection_min_dist);
    }
    if (!nh.getParam("/warp/scale_jnt_ids",scale_vect_ids))
    {
      ROS_WARN("warp/scale_jnt_ids is not defined");
    } else {
      std::cout<<"scale_jnt_ids:";
      for (int i=0;i<scale_vect_ids.size();i++) std::cout<<scale_vect_ids[i]<<", ";
      std::cout<<std::endl;
    }
    if (!nh.getParam("/warp/goal_stop_tolerance",goal_stop_tolerance))
    {
      ROS_WARN("warp/goal_stop_tolerance is not defined");
    } else {
      ROS_INFO_STREAM("goal_stop_tolerance:"<<goal_stop_tolerance);
    }
    // scale_vect_ids = {3,4,5,8};
    planning_scene = planning_scene::PlanningScene::clone(planning_scene_);
    collision_detection::AllowedCollisionMatrix acm = planning_scene->getAllowedCollisionMatrix();
    std::vector<std::string> co_ids = {"human_torso","human_neck","human_l_upper", "human_l_fore","human_r_upper","human_r_fore"};
    // std::vector<std::string> robot_ids = {"edo_base_link","edo_link_1","edo_link_2","edo_link_3","edo_link_4","edo_link_5","edo_link_6","edo_link_ee","edo_link_ee_tip"};
    // for (int r=0;r<robot_ids.size();i++) {
      for (int i=0;i<co_ids.size();i++) {
        acm.setEntry(co_ids[i],true);
      }
    // }
    joint_names = state->getVariableNames();

    sub_ovr = nh.subscribe<std_msgs::Int64>("/speed_ovr",1,&stap_warper::speed_ovr_callback,this);
}

void stap_warper::act_traj_callback(const trajectory_msgs::JointTrajectory::ConstPtr& trj) {
  std::lock_guard<std::mutex> lock(trj_mtx);
  // std::cout<<"receiving actual trajectory\n";
  cur_traj = *trj;
}

void stap_warper::scale_time_callback(const std_msgs::Float64::ConstPtr& msg) {
  path_time_pct = msg->data;
}

void stap_warper::warp(std::vector<std::pair<float,Eigen::MatrixXd>> &human_seq, double human_time_since_start, Eigen::VectorXd cur_pose, sensor_msgs::JointState cur_js) {
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    if ((ros::Time::now()-last_warp_time).toSec()<cycle_time) return;
    if (cur_traj.points.size()<2) return;
    trj_mtx.lock();
    trajectory_msgs::JointTrajectory trj = cur_traj;
    trj_mtx.unlock();
    // std::cout<<"cur pose:"<<cur_pose.transpose()<<std::endl;
    if (path_time_pct>=1.0) return;
    Eigen::VectorXd goal_pose(cur_pose.size());
    for (int i=0;i<cur_pose.size();i++) goal_pose[i] = trj.points.back().positions[i];
    Eigen::VectorXd direct_path_diff = goal_pose-cur_pose;
    double direct_path_len = (direct_path_diff).norm();
    if (direct_path_len<goal_stop_tolerance) return;
    int num_steps = std::max(std::ceil(direct_path_len/0.1),1.0);
    Eigen::MatrixXd direct_path_pts;
    direct_path_pts.resize(cur_pose.size(),num_steps+1);
    for (int s=0;s<=num_steps;s++) {
      double pct = (double)s/(double)num_steps;
      direct_path_pts.col(s) = cur_pose+pct*direct_path_diff;
    }
    double path_time = path_time_pct*trj.points.back().time_from_start.toSec();
    // std::cout<<"start stap warp"<<path_time_pct<<","<<path_time<<std::endl;
    int start_p = 1;
    std::vector<Eigen::VectorXd> poses;
    std::vector<double> wp_times;
    poses.reserve(trj.points.size());
    wp_times.reserve(trj.points.size());
    bool in_plan = false;
    std::vector<double> pos = trj.points[0].positions;
    Eigen::VectorXd prev_pose = Eigen::VectorXd::Zero(pos.size());
    for (int i=0;i<pos.size();i++) prev_pose[i] = pos[i];
    double start_time;
    for (int p=1;p<trj.points.size();p++) {
      Eigen::VectorXd tmp_vec(trj.points[p].positions.size());
      for (int i=0;i<trj.points[p].positions.size();i++) tmp_vec[i] = trj.points[p].positions[i];
      if (!in_plan) {
        // std::cout<<trj.points[p-1].time_from_start.toSec()<<","<<trj.points[p].time_from_start.toSec()<<","<<path_time<<std::endl;
        if ((path_time>=trj.points[p-1].time_from_start.toSec())&&(path_time<trj.points[p].time_from_start.toSec())) {
          start_time = path_time;
          poses.push_back(cur_pose);
          wp_times.push_back(0.0);
          in_plan = true;
        }

      }
      if (in_plan) {
        poses.push_back(tmp_vec);
        // std::cout<<"poses:"<<tmp_vec.transpose()<<std::endl;
        wp_times.push_back(trj.points[p].time_from_start.toSec()-start_time);
      }
      prev_pose = tmp_vec;
    }
    // std::cout<<"goal pose:"<<poses.back().transpose()<<std::endl;
    double first_pt_time = wp_times[0];
    // std::cout<<"start stap warp loop"<<std::endl;
    // std::cout<<"old poses:\n";
    // for (int i=0;i<poses.size();i++) std::cout<<wp_times[i]<<":"<<poses[i].transpose()<<std::endl;
    bool first_pass = false;
    Eigen::Vector3d goal_position;
    state->setJointGroupPositions(plan_group,trj.points.back().positions);
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(state->getGlobalLinkTransform(link_names.back()),pose);
    goal_position[0] = pose.position.x;
    goal_position[1] = pose.position.y;
    goal_position[2] = pose.position.z;
    std::vector<int> ctrl_pts_to_avoid_obstacle;
    for (int iter=0;iter<warp_iterations;iter++) {
        std::vector<Eigen::VectorXd> new_poses;
        std::vector<double> new_wpt_times;

        // new_poses.push_back(poses.front());
        // new_wpt_times.push_back(wp_times.front());
        double last_wpt_time = 0.0;
        visualization_msgs::Marker mkr4;
        mkr4.header.frame_id = "world";
        mkr4.id = 10001;
        mkr4.type = 5;
        mkr4.pose.position.x = 0;
        mkr4.pose.position.y = 0;
        mkr4.pose.position.z = 0;
        mkr4.pose.orientation.w = 1;
        mkr4.pose.orientation.x = 0;
        mkr4.pose.orientation.y = 0;
        mkr4.pose.orientation.z = 0;
        mkr4.color.a = 1.0;
        mkr4.color.b = 0.0;
        mkr4.color.r = 1.0;
        mkr4.scale.x = 0.01;
        // mkr4.scale.y = 0.02;
        // mkr4.scale.z = 0.02;
        mkr4.lifetime = ros::Duration();
        int prev_attr_steps = 0;
        int prev_repl_steps = 0;
        Eigen::VectorXd last_repulsion_tau = Eigen::VectorXd::Zero(poses[0].size());
        int last_direct_step = 0;
        for (int p=1;p<poses.size();p++) {
            Eigen::VectorXd diff = poses[p]-poses[p-1];
            int num_steps = 1;
            if (!first_pass) num_steps = std::max(std::ceil(diff.norm()/0.1),1.0);
            // std::cout<<"num steps:"<<num_steps<<std::endl;
            double nominal_time = 0.0;
            for (int q=0;q<diff.size();q++){
              double jnt_time = abs(diff[q]/max_vels_[q]);
              nominal_time = std::max(nominal_time,jnt_time);
            }
            // std::cout<<"nominal time:"<<nominal_time<<std::endl;
            Eigen::VectorXd dq = diff/nominal_time;
            double diff_pct = 1.0/(double)num_steps;
            for (int s=0;s<num_steps;s++) {
                // if ((p==1)&&(s==0)) continue;
                double pct = (double)s/(double)num_steps;
                // double nom_time = wp_times[p-1] + pct*nominal_time;
                // std::cout<<"nom time:"<<nom_time<<", human_time_since_start:"<<human_time_since_start<<",htime:"<<h_time<<std::endl;
                Eigen::VectorXd cur_q = poses[p-1] + pct*diff;
                if (cur_q.sum()==0) std::cout<<"cur q is zero: "<<poses[p-1].transpose()<<std::endl;
                // double min_time_diff = diff
                double nom_time = last_wpt_time + diff_pct*nominal_time;
                double h_time = nom_time+human_time_since_start;
                // std::cout<<"nom time:"<<nom_time<<", human_time_since_start:"<<human_time_since_start<<",htime:"<<h_time<<std::endl;
                // Eigen::VectorXd nxt_pose = cur_q;
                // if (s<num_steps) {
                // nxt_pose = poses[p-1] + (double)(s+1)/(double)num_steps*diff;
                // } else {
                //   nxt_pose = poses[p];
                // }
                // Eigen::Matrix6Xd jacobian = chain_->getJacobian(cur_pose);
                if (human_seq.empty()) {
                  ssm->setPointCloud(Eigen::MatrixXd(3,0));
                } else {
                  ssm->setPointCloud(human_seq[std::max(std::min(int(round(h_time*10)),int(human_seq.size())-1),0)].second);
                }
                // std::cout<<"q:"<<cur_q.transpose()<<std::endl;
                // std::cout<<"dq:"<<dq.transpose()<<std::endl;
                // std::vector<std::pair<double,Eigen::Vector3d>> scale_vects = ssm->computeScalesVectors(cur_q,dq);
                std::vector<std::vector<std::pair<double,Eigen::Vector3d>>> scale_vects = ssm->computeMoreScalesVectors(cur_q,dq);
                Eigen::VectorXd tau(diff.size());
                std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> T_base_all_links = chain_->getTransformations(cur_q);
                tau.setZero();
                bool need_attr = false;
                double min_scale = 1.0;
                for (int jj=0;jj<scale_vect_ids.size();jj++) {
                  int j = scale_vect_ids[jj];
                  Eigen::Matrix6Xd jacobian = chain_->getJacobianLink(cur_q,link_names[j]);
                  // if (scale_vects[j].first<1.0) {
                  //   need_attr = true;
                  //   min_scale = std::min(min_scale,scale_vects[j].first);
                  //   // Eigen::Vector3d force_vec = scale_vects[j].second.cross(Eigen::Vector3d::UnitZ()).normalized();
                  //   // force_vec = (0.5*(force_vec+scale_vects[j].second)).normalized();
                  //   Eigen::Vector3d force_vec = scale_vects[j].second;
                  //   Eigen::VectorXd tmp_tau = repulsion*(1.0-std::max(scale_vects[j].first,0.0))*jacobian.block(0,0,3,jacobian.cols()).transpose()*scale_vects[j].second;//-attraction*((new_poses.back()-cur_pose)+(nxt_pose-cur_pose));
                  //   // Eigen::VectorXd tmp_tau =1.0*repulsion*(1.0/std::max(scale_vects[j].first,0.1)-1)*jacobian.block(0,0,3,jacobian.cols()).transpose()*scale_vects[j].second;//-attraction*((new_poses.back()-cur_pose)+(nxt_pose-cur_pose));
                  //   tau += tmp_tau;
                  //   geometry_msgs::Point pt1;
                  //   pt1.x = T_base_all_links[j](0,3);
                  //   pt1.y = T_base_all_links[j](1,3);
                  //   pt1.z = T_base_all_links[j](2,3);
                  //   geometry_msgs::Point pt2;
                  //   pt2.x = T_base_all_links[j](0,3) + 0.1*force_vec[0];
                  //   pt2.y = T_base_all_links[j](1,3) + 0.1*force_vec[1];
                  //   pt2.z = T_base_all_links[j](2,3) + 0.1*force_vec[2];
                  //   mkr4.points.push_back(pt1);
                  //   mkr4.points.push_back(pt2);
                  //   // std::cout<<"j:"<<j<<",vec:"<<scale_vects[j].second.transpose()<<",f vec:"<<force_vec.transpose()<<", scale:"<<scale_vects[j].first<<", tau:"<<tmp_tau.transpose()<<std::endl;
                  // }
                  for (int k=0;k<scale_vects[j].size();k++) {
                    if (scale_vects[j][k].first<1.0) {
                      need_attr = true;
                      min_scale = std::min(min_scale,scale_vects[j][k].first);
                      Eigen::Vector3d force_vec = scale_vects[j][k].second;
                      Eigen::VectorXd tmp_tau = repulsion*(1.0-std::max(scale_vects[j][k].first,0.0))*jacobian.block(0,0,3,jacobian.cols()).transpose()*scale_vects[j][k].second;//-attraction*((new_poses.back()-cur_pose)+(nxt_pose-cur_pose));
                      // Eigen::VectorXd tmp_tau =1.0*repulsion*(1.0/std::max(scale_vects[j].first,0.1)-1)*jacobian.block(0,0,3,jacobian.cols()).transpose()*scale_vects[j].second;//-attraction*((new_poses.back()-cur_pose)+(nxt_pose-cur_pose));
                      tau += tmp_tau;
                      geometry_msgs::Point pt1;
                      pt1.x = T_base_all_links[j](0,3);
                      pt1.y = T_base_all_links[j](1,3);
                      pt1.z = T_base_all_links[j](2,3);
                      geometry_msgs::Point pt2;
                      pt2.x = T_base_all_links[j](0,3) + 0.1*force_vec[0];
                      pt2.y = T_base_all_links[j](1,3) + 0.1*force_vec[1];
                      pt2.z = T_base_all_links[j](2,3) + 0.1*force_vec[2];
                      mkr4.points.push_back(pt1);
                      mkr4.points.push_back(pt2);
                      // std::cout<<"j:"<<j<<",vec:"<<scale_vects[j].second.transpose()<<",f vec:"<<force_vec.transpose()<<", scale:"<<scale_vects[j].first<<", tau:"<<tmp_tau.transpose()<<std::endl;
                    }
                  }
                }
                nom_time = last_wpt_time + (1.0/std::max(min_scale*global_override,0.01))*diff_pct*nominal_time;
                if (prev_repl_steps>0) {
                  tau+=((double)prev_repl_steps/((double)smooth_steps+1.0))*last_repulsion_tau;
                  prev_repl_steps--;
                }
                if (need_attr) {
                  last_repulsion_tau = tau;
                  prev_repl_steps = smooth_steps;
                  Eigen::Matrix6Xd jacobian = chain_->getJacobian(cur_q);
                  Eigen::VectorXd t_attr = attraction*jacobian.block(0,0,3,jacobian.cols()).transpose()*(goal_position-T_base_all_links.back().translation()).normalized();
                  // std::cout<<"attr:"<<t_attr.transpose()<<std::endl;
                  tau-=t_attr;
                  prev_attr_steps = smooth_steps;
                } else if (prev_attr_steps>0) {
                  Eigen::Matrix6Xd jacobian = chain_->getJacobian(cur_q);
                  Eigen::VectorXd t_attr = ((double)prev_attr_steps/(double)smooth_steps)*attraction*jacobian.block(0,0,3,jacobian.cols()).transpose()*(goal_position-T_base_all_links.back().translation()).normalized();
                  // std::cout<<"attr2:"<<prev_attr_steps<<":"<<t_attr.transpose()<<std::endl;
                  tau-=t_attr;
                  prev_attr_steps -= 1;
                }
                if (table_tolerance>0) {
                  for (int j=2;j<T_base_all_links.size();j++) {
                    if (T_base_all_links[j](2,3)<table_tolerance) {
                      Eigen::Matrix6Xd jacobian = chain_->getJacobianLink(cur_q,link_names[j]);
                      Eigen::VectorXd tmp_tau = table_repulsion*(table_tolerance-T_base_all_links[j](2,3))*jacobian.block(0,0,3,jacobian.cols()).transpose()*Eigen::Vector3d::UnitZ();//-attraction*((new_poses.back()-cur_pose)+(nxt_pose-cur_pose));
                      // std::cout<<"too low! table tau:"<<tmp_tau.transpose()<<std::endl;
                      tau -= tmp_tau;
                    }
                  }
                }

                //pull path toward direct path
                double min_dist=std::numeric_limits<double>::infinity();
                for (int j=last_direct_step;j<direct_path_pts.cols();j++) {
                  double dist = (direct_path_pts.col(j)-cur_q).norm();
                  if (dist<=min_dist) {
                    last_direct_step = j;
                    min_dist = dist;
                  } else {
                    break;
                  }
                }
                // std::cout<<"last_direct_step:"<<last_direct_step<<","<<direct_path_pts.cols()<<std::endl;
                Eigen::VectorXd direct_path_tau = -direct_path_attraction*(direct_path_pts.col(last_direct_step)-cur_q);
                // std::cout<<direct_path_pts.col(last_direct_step).transpose()<<","<<cur_q.transpose()<<","<<direct_path_tau.transpose()<<std::endl;
                tau += direct_path_tau;
                // std::cout<<"tau sum:"<<tau.transpose()<<std::endl;
                Eigen::VectorXd new_q = cur_q-1.0*tau;                
                
                state->setJointGroupPositions(plan_group, new_q);
                state->update();
                if (!state->satisfiesBounds())
                {
                  ROS_ERROR_STREAM("state is not valid:"<<new_q.transpose()<<"--"<<cur_q.transpose());
                  new_poses.push_back(cur_q);
                  new_wpt_times.push_back(nom_time);
                  continue;
                }

                if (!planning_scene->isStateValid(*state,plan_group))
                {
                  // ROS_ERROR_STREAM("new q is in collision:"<<new_q.transpose());
                  state->setJointGroupPositions(plan_group, cur_q);
                  state->update();
                  if (!planning_scene->isStateValid(*state,plan_group)) {
                    ROS_ERROR_STREAM("new and cur q are in collision:"<<new_q.transpose()<<"--"<<cur_q.transpose());
                  }
                  new_poses.push_back(cur_q);
                  ctrl_pts_to_avoid_obstacle.push_back(int(new_poses.size())-1);
                  new_wpt_times.push_back(nom_time);
                  continue;
                }

                new_q= new_q.cwiseMin(q_max).cwiseMax(q_min);
                if (!new_poses.empty()) {
                  Eigen::VectorXd new_diff_time = ((new_q-new_poses.back()).array().abs()/max_vels_).matrix();
                  nom_time = last_wpt_time + (1.0/std::max(min_scale,0.01))*new_diff_time.maxCoeff();
                }
                new_poses.push_back(new_q);
                new_wpt_times.push_back(nom_time);
                last_wpt_time = nom_time;
            }
        }

        warp_pub4.publish(mkr4);
        first_pass = true;
        new_poses.push_back(poses.back());
        new_wpt_times.push_back(wp_times.back());
        poses = new_poses;    
        visualization_msgs::Marker mkr2;
        mkr2.header.frame_id = "world";
        mkr2.id = 10001;
        mkr2.type = 7;
        mkr2.pose.position.x = 0;
        mkr2.pose.position.y = 0;
        mkr2.pose.position.z = 0;
        mkr2.pose.orientation.w = 1;
        mkr2.pose.orientation.x = 0;
        mkr2.pose.orientation.y = 0;
        mkr2.pose.orientation.z = 0;
        mkr2.color.a = 1.0;
        mkr2.color.b = 1.0;
        mkr2.color.r = 1.0;
        mkr2.scale.x = 0.02;
        mkr2.scale.y = 0.02;
        mkr2.scale.z = 0.02;
        mkr2.lifetime = ros::Duration();
        // std::cout<<"new poses:\n";
        for (int i=0;i<poses.size();i++) {
          // std::cout<<wp_times[i]<<":"<<poses[i].transpose()<<std::endl;
          state->setJointGroupPositions(plan_group,poses[i]);
          tf::poseEigenToMsg(state->getGlobalLinkTransform(link_names.back()),pose);
          mkr2.points.push_back(pose.position);
          
        }
        warp_pub2.publish(mkr2);
        // std::cin.ignore();
        wp_times = new_wpt_times;
        // for (int i=1;i<poses.size()-1;i++) {
        //   poses[i] = 0.5*(poses[i-1]+poses[i]);//+poses[i+1]);
        // }
    }


    visualization_msgs::Marker mkr2;
    mkr2.header.frame_id = "world";
    mkr2.id = 10001;
    mkr2.type = 7;
    mkr2.pose.position.x = 0;
    mkr2.pose.position.y = 0;
    mkr2.pose.position.z = 0;
    mkr2.pose.orientation.w = 1;
    mkr2.pose.orientation.x = 0;
    mkr2.pose.orientation.y = 0;
    mkr2.pose.orientation.z = 0;
    mkr2.color.a = 1.0;
    mkr2.color.b = 1.0;
    mkr2.color.g = 1.0;
    mkr2.scale.x = 0.04;
    mkr2.scale.y = 0.04;
    mkr2.scale.z = 0.04;
    mkr2.lifetime = ros::Duration();

    trajectory_msgs::JointTrajectory new_plan;
    new_plan.joint_names = joint_names;//state->getJointStateGroup(plan_group)->getJointNames();
    // new_plan.joint_names = {"edo_joint_1","edo_joint_2","edo_joint_3","edo_joint_4","edo_joint_5","edo_joint_6"};
    Eigen::VectorXd diff = Eigen::VectorXd::Zero(6);
    trajectory_msgs::JointTrajectoryPoint pt;
    // for (int j=0;j<6;j++) pt.positions.push_back(cur_pose[j]);
    // for (int j=0;j<6;j++) pt.velocities.push_back(0.0);
    // std::cout<<cur_js<<std::endl;
    for (int j=0;j<6;j++) pt.accelerations.push_back(0.0);
    for (int j=0;j<6;j++) pt.effort.push_back(0.0);
    for (int j=0;j<6;j++) pt.positions.push_back(cur_js.position[j]);
    for (int j=0;j<6;j++) pt.velocities.push_back(cur_js.velocity[j]);
    // pt.accelerations = cur_js.acceleration;
    new_plan.points.push_back(pt);
    state->setJointGroupPositions(plan_group,pt.positions);
    tf::poseEigenToMsg(state->getGlobalLinkTransform(link_names.back()),pose);
    mkr2.points.push_back(pose.position);
    diff = (poses[1]-poses[0]).normalized();
    prev_pose = cur_pose;
    for (int i=1;i<poses.size();i++) {
      Eigen::VectorXd new_diff = (poses[i]-poses[i-1]).normalized();
      if ((((abs(new_diff.dot(diff))<connection_tol)&&((poses[i-1]-cur_pose).norm()>0.8))&&((poses[i-1]-prev_pose).norm()>connection_min_dist))){//||(std::find(ctrl_pts_to_avoid_obstacle.begin(),ctrl_pts_to_avoid_obstacle.end(),i)!=ctrl_pts_to_avoid_obstacle.end())) {
        for (int j=0;j<6;j++) pt.positions[j] = poses[i-1][j];
        for (int j=0;j<6;j++) pt.velocities[j] = 0.0;
        for (int j=0;j<6;j++) pt.accelerations[j] = 0.0;
        for (int j=0;j<6;j++) pt.effort[j] = 0.0;
        new_plan.points.push_back(pt);
        state->setJointGroupPositions(plan_group,pt.positions);
        tf::poseEigenToMsg(state->getGlobalLinkTransform(link_names.back()),pose);
        mkr2.points.push_back(pose.position);
        diff = new_diff;
        prev_pose = poses[i-1];
      }
    }
    for (int j=0;j<6;j++) pt.positions[j] = poses.back()[j];
    for (int j=0;j<6;j++) pt.velocities[j] = 0.0;
    for (int j=0;j<6;j++) pt.accelerations[j] = 0.0;
    for (int j=0;j<6;j++) pt.effort[j] = 0.0;
    new_plan.points.push_back(pt);

    warp_pub3.publish(mkr2);
    if (new_plan.points.size()==2) return;
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    robot_state::RobotStatePtr state = move_group.getCurrentState();
    // moveit::core::robotStateToRobotStateMsg(*state,new_plan.start_state_);
    robot_trajectory::RobotTrajectory tp_trj(model,plan_group);
    tp_trj.setRobotTrajectoryMsg(*state,new_plan);
    iptp.computeTimeStamps(tp_trj);
    moveit_msgs::RobotTrajectory trj_msg;
    tp_trj.getRobotTrajectoryMsg(trj_msg);
    new_plan = trj_msg.joint_trajectory;
    // trj = new_plan;
    // std::cout<<new_plan.points.back().time_from_start.toSec();
    new_plan.points.erase(new_plan.points.begin());
    // new_plan.points.erase(new_plan.points.begin());

    // std::cout<<"new plan:\n";
    // std::cout<<new_plan<<std::endl;
    // for (int i=0;i<new_plan.points.size();i++) {
    //   for (int j=0;j<6;j++) std::cout<<new_plan.points[i].positions[j]<<",";
    //   std::cout<<std::endl;
    // }

    new_plan.joint_names = joint_names;
    // new_plan.joint_names = {"edo_joint_1","edo_joint_2","edo_joint_3","edo_joint_4","edo_joint_5","edo_joint_6"};
    blend_pub.publish(new_plan);

    last_warp_time = ros::Time::now();

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
    Eigen::VectorXd q1(6);
    Eigen::VectorXd q2(6);
    for (int i=1;i<new_plan.points.size();i++) {
      for (int q=0;q<6;q++) q1[q] = new_plan.points[i-1].positions[q];
      for (int q=0;q<6;q++) q2[q] = new_plan.points[i].positions[q];
      Eigen::VectorXd diff = q2-q1;
      int n = std::ceil(diff.norm()/0.1);
      for (int j=0;j<n;j++) {
        double pct = (double)j/(double)n;
        Eigen::VectorXd q3 = q1+pct*diff;
        state->setJointGroupPositions(plan_group,q3);
        tf::poseEigenToMsg(state->getGlobalLinkTransform(link_names.back()),pose);
        mkr.points.push_back(pose.position);
      }
    }
    warp_pub.publish(mkr);

    std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    // ROS_INFO_STREAM("warp took " << time_span.count() << " seconds");
}

void stap_warper::time_parameterize(trajectory_msgs::JointTrajectory &plan, std::vector<std::tuple<Eigen::ArrayXd,Eigen::ArrayXd,Eigen::ArrayXd,Eigen::ArrayXd>> &vel_profile) {
 
  vel_profile.clear();
  double last_end_time = 0.0;
  for (int i=1;i<plan.points.size();i++) {
    std::cout<<"starting: "<<i<<std::endl;
    Eigen::ArrayXd q_start(plan.points[i-1].positions.size());
    for (int q=0;q<q_start.size();q++) q_start[q] = plan.points[i-1].positions[q];
    Eigen::ArrayXd q_end(plan.points[i].positions.size());
    for (int q=0;q<q_end.size();q++) q_end[q] = plan.points[i].positions[q];
    Eigen::ArrayXd q_dot_start(plan.points[i-1].positions.size());
    for (int q=0;q<q_dot_start.size();q++) q_dot_start[q] = plan.points[i-1].velocities[q];
    Eigen::ArrayXd q_dot_end(plan.points[i].positions.size());
    for (int q=0;q<q_dot_end.size();q++) q_dot_end[q] = plan.points[i].velocities[q];
    Eigen::ArrayXd diff = q_end-q_start;
    Eigen::ArrayXd abs_diff = diff.abs();
    Eigen::ArrayXd sgn1 = Eigen::ArrayXd::Zero(abs_diff.size());
    for (int q=0;q<sgn1.size();q++) sgn1[q] = 1.0*(diff[q]>0.0)-1.0*(diff[q]<0.0);
    std::cout<<"sgn1"<<sgn1<<std::endl;
    Eigen::ArrayXd times_acc_to_full_speed = ((sgn1*max_vels_-q_dot_start)/max_accels_).abs();
    double time_acc_to_full_speed = times_acc_to_full_speed.maxCoeff();
    Eigen::ArrayXd times_to_decc_from_full_speed = ((sgn1*max_vels_-q_dot_end)/max_accels_).abs();
    double time_to_decc_from_full_speed = times_to_decc_from_full_speed.matrix().maxCoeff();
    Eigen::ArrayXd dq_to_full_speed = 0.5*sgn1*max_accels_*times_acc_to_full_speed*times_acc_to_full_speed+q_dot_start*times_acc_to_full_speed;
    Eigen::ArrayXd dq_full_to_next_speed = -0.5*sgn1*max_accels_*times_to_decc_from_full_speed*times_to_decc_from_full_speed+sgn1*max_vels_*times_to_decc_from_full_speed;
    Eigen::ArrayXd dq_tot = dq_to_full_speed+dq_full_to_next_speed;
    Eigen::ArrayXd full_spd_q = diff.abs()-dq_tot.abs();
    Eigen::ArrayXd times_at_full_spd = full_spd_q/max_vels_;
    Eigen::ArrayXd full_spd_tot_times = times_at_full_spd+times_acc_to_full_speed+times_to_decc_from_full_speed;
    double full_spd_tot_time = 0.0;
    int full_spd_jnt = 0;
    for (int q=0;q<times_acc_to_full_speed.size();q++) {
      if (full_spd_tot_times[q]>full_spd_tot_time) {
        full_spd_tot_time = full_spd_tot_times[q];
        full_spd_jnt = q;
      }
    }
    ArrayXb may_go_too_fast = (full_spd_tot_time*q_dot_start).array().abs()>abs_diff.array();
    // for (int q=0;q<may_go_too_fast.size();q++) {
    //   if (may_go_too_fast[q]) sgn1[q]=-sgn1[q];
    // }
    Eigen::ArrayXd accel_stop_time = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd deccel_start_time = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd accelerations = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd deccelerations = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd t1_ = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd t2_ = Eigen::ArrayXd::Zero(6);
    Eigen::ArrayXd alt_a = Eigen::ArrayXd::Zero(6);

    ArrayXb spd_limit_jnts = ArrayXb::Constant(6,true);
    if (times_at_full_spd[full_spd_jnt]>0) {
      spd_limit_jnts[full_spd_jnt] = false;
      accelerations[full_spd_jnt] = max_accels_[full_spd_jnt];
      deccelerations[full_spd_jnt] = max_accels_[full_spd_jnt];
      accel_stop_time[full_spd_jnt] = times_acc_to_full_speed[full_spd_jnt];
      deccel_start_time[full_spd_jnt] = times_acc_to_full_speed[full_spd_jnt]+times_at_full_spd[full_spd_jnt];
      double mid_t = times_acc_to_full_speed[full_spd_jnt]+0.5*times_at_full_spd[full_spd_jnt];
      Eigen::ArrayXd c = (sgn1*max_vels_-q_dot_start)/(sgn1*max_vels_-q_dot_end);
      t2_ = (diff-full_spd_tot_time*sgn1*max_vels_)/(0.5*(c*c-1)*(q_dot_end-q_dot_start)/(c-1)+c*(q_dot_start-sgn1*max_vels_));
      Eigen::ArrayXd alt_t2 = (diff-full_spd_tot_time*sgn1*max_vels_)/(q_dot_start-sgn1*max_vels_);
      for (int q=0;q<t2_.size();q++) {
        if (c[q]==1.0) {
          t2_[q] = alt_t2[q];
        }
      }
      t1_ = c*t2_;
      alt_a = ((q_dot_end-q_dot_start)/(c-1)/t2_).abs();
      Eigen::ArrayXd alt_alt_a = ((sgn1*max_vels_-q_dot_start)/c/t2_).abs();
      for (int q=0;q<t2_.size();q++) {
        if (c[q]==1.0) {
          alt_a[q] = alt_alt_a[q];
        }
      }
    }
    std::cout<<"full spd time:"<<full_spd_tot_time<<std::endl;
    Eigen::ArrayXd qm_dot = max_vels_;
    Eigen::ArrayXd tm;
    Eigen::ArrayXd t_fm;
    Eigen::ArrayXd sgn = sgn1;
    Eigen::ArrayXd avg_spd_times = abs_diff/(0.5*(q_dot_end+q_dot_start).abs());
    while ((qm_dot>0).all()) {
      sgn = sgn1;
      tm = (sgn*qm_dot-q_dot_start).abs()/max_accels_;
      t_fm = (sgn*qm_dot-q_dot_end).abs()/max_accels_;
      for (int q=0;q<tm.size();q++) {
        if (spd_limit_jnts[q]) {
          if ((tm[q]+t_fm[q])>avg_spd_times[q]) sgn[q] = -sgn1[q];
        }
      }
      tm = (sgn*qm_dot-q_dot_start).abs()/max_accels_;
      t_fm = (sgn*qm_dot-q_dot_end).abs()/max_accels_;
      Eigen::ArrayXd diff2 = 0.5*max_accels_*(tm*tm-t_fm*t_fm)+qm_dot*t_fm+q_dot_start*tm;
      ArrayXb diff_bool = ((abs_diff-diff2.abs())>=0);
      if (diff_bool.all()) break;
      for (int q=0;q<qm_dot.size();q++) {
        if (!diff_bool[q]) qm_dot[q]-=0.01;
      }
    }
    Eigen::ArrayXd tot_time = tm + t_fm;
    double max_tot_time = 0.0;
    int limit_joint = 0;
    for (int q=0;q<tot_time.size();q++) {
      if (tot_time[q]>max_tot_time) {
        max_tot_time = tot_time[q];
        limit_joint = q;
      }
    }
    double mid_time = tm[limit_joint];
    double end_time = tot_time[limit_joint];
    if (times_at_full_spd[full_spd_jnt]>0) {
      end_time = full_spd_tot_time;
      mid_time = 0.5*end_time;
    }
    Eigen::MatrixXd knowns = Eigen::MatrixXd::Zero(2,diff.size());
    knowns.row(0) = (diff-end_time*q_dot_start).matrix();
    knowns.row(1) = (q_dot_end-q_dot_start).matrix();
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2,2);
    A(0,0) = 0.5*mid_time*mid_time+mid_time*(end_time-mid_time);
    A(0,1) = -0.5*(end_time-mid_time)*(end_time-mid_time);
    A(1,0) = mid_time;
    A(1,1) = -(end_time-mid_time);
    Eigen::MatrixXd a_d = A.inverse()*knowns;
    std::cout<<"knowns"<<knowns<<std::endl;
    std::cout<<"A"<<A<<std::endl;
    std::cout<<"a_d"<<a_d<<std::endl;
    for (int q=0;q<spd_limit_jnts.size();q++) {
      if (spd_limit_jnts[q]) {
        accelerations[q] = abs(a_d(0,q));
        deccelerations[q] = abs(a_d(1,q));
        accel_stop_time[q] = mid_time;
        deccel_start_time[q] = mid_time;
      }
    }
    Eigen::ArrayXd mid_spds = q_dot_start+mid_time*a_d.row(0).transpose().array();
    ArrayXb tmp_comparitor = (mid_spds.abs()>max_vels_);
    for (int q=0;q<6;q++) {
      if (spd_limit_jnts[q] && tmp_comparitor[q]) {
          accel_stop_time[q] = t1_[q];
          deccel_start_time[q] = full_spd_tot_time-t2_[q];
          accelerations[q] = alt_a[q];
          deccelerations[q] = alt_a[q];
          std::cout<<q<<",a:"<<accelerations[q]<<std::endl;
          std::cout<<q<<",d:"<<deccelerations[q]<<std::endl;
      }
    }
    if (times_at_full_spd[full_spd_jnt]>0) {
      mid_spds[full_spd_jnt] = sgn[full_spd_jnt]*max_vels_[full_spd_jnt];
    }
    std::cout<<"end time:"<<end_time<<std::endl;
    std::cout<<sgn<<std::endl;
    std::cout<<accelerations<<std::endl;
    std::cout<<accel_stop_time<<std::endl;
    std::cout<<deccelerations<<std::endl;
    std::cout<<deccel_start_time<<std::endl;
    plan.points[i].time_from_start = ros::Duration(end_time + last_end_time);
    vel_profile.emplace_back(sgn*accelerations,-1.0*sgn*deccelerations,accel_stop_time+last_end_time,deccel_start_time+last_end_time);
    last_end_time += end_time;
  }
}


void stap_warper::speed_ovr_callback(const std_msgs::Int64::ConstPtr& msg) {
    global_override = (double)msg->data/100.0;
}

}