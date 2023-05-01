#include <stap_warp/stap_warp.h>
namespace stap {
stap_warper::stap_warper(ros::NodeHandle nh, robot_state::RobotStatePtr state, robot_model::RobotModelPtr model):nh(nh),state(state),model(model),move_group("edo") {
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
    blend_pub = nh.advertise<trajectory_msgs::JointTrajectory>("/planner_hw/microinterpolator/blend_trajectory",1);
    sub_act_trj = nh.subscribe<trajectory_msgs::JointTrajectory>("/planner_hw/microinterpolator/act_trajectory",1,&stap_warper::act_traj_callback,this);
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

void stap_warper::act_traj_callback(const trajectory_msgs::JointTrajectory::ConstPtr& trj) {
  std::lock_guard<std::mutex> lock(trj_mtx);
  std::cout<<"receiving actual trajectory\n";
  cur_traj = *trj;
}

void stap_warper::scale_time_callback(const std_msgs::Float64::ConstPtr& msg) {
  path_time_pct = msg->data;
}

void stap_warper::warp(std::vector<std::pair<float,Eigen::MatrixXd>> &human_seq, double human_time_since_start, Eigen::VectorXd cur_pose) {
    if ((ros::Time::now()-last_warp_time).toSec()<0.1) return;
    if (cur_traj.points.size()<2) return;
    trj_mtx.lock();
    trajectory_msgs::JointTrajectory trj = cur_traj;
    trj_mtx.unlock();
    std::cout<<"cur pose:"<<cur_pose.transpose()<<std::endl;
    if (path_time_pct>=1.0) return;
    double path_time = path_time_pct*trj.points.back().time_from_start.toSec();
    std::cout<<"start stap warp"<<path_time_pct<<","<<path_time<<std::endl;
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
        std::cout<<"poses:"<<tmp_vec.transpose()<<std::endl;
        wp_times.push_back(trj.points[p].time_from_start.toSec()-start_time);
      }
      prev_pose = tmp_vec;
    }
    double first_pt_time = wp_times[0];
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
            // std::cout<<"num steps:"<<num_steps<<std::endl;
            double nominal_time = 0.0;
            for (int q=0;q<diff.size();q++){
              double jnt_time = abs(diff[q]/max_vels_[q]);
              nominal_time = std::max(nominal_time,jnt_time);
            }
            // std::cout<<"nominal time:"<<nominal_time<<std::endl;
            Eigen::VectorXd dq = diff/nominal_time;
            double diff_pct = 1.0/(double)num_steps;
            for (int s=1;s<=num_steps;s++) {
                if ((p==poses.size()-1)&&(s==num_steps)) continue;
                if ((p==1)&&(s==0)) continue;
                double pct = (double)s/(double)num_steps;
                // double nom_time = wp_times[p-1] + pct*nominal_time;
                // std::cout<<"nom time:"<<nom_time<<", human_time_since_start:"<<human_time_since_start<<",htime:"<<h_time<<std::endl;
                Eigen::VectorXd cur_pose = poses[p-1] + pct*diff;
                // double min_time_diff = diff
                double nom_time = new_wpt_times.back() + diff_pct*nominal_time;
                double h_time = nom_time+human_time_since_start;
                Eigen::VectorXd nxt_pose = cur_pose;
                if (s<num_steps) {
                  nxt_pose = poses[p-1] + (double)(s+1)/(double)num_steps*diff;
                } else {
                  nxt_pose = poses[p];
                }
                // Eigen::Matrix6Xd jacobian = chain_->getJacobian(cur_pose);
                ssm->setPointCloud(human_seq[std::max(std::min(int(round(h_time*10)),int(human_seq.size())-1),0)].second);
                std::vector<std::pair<double,Eigen::Vector3d>> scale_vects = ssm->computeScalesVectors(cur_pose,dq);
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
        first_pass = true;
        new_poses.push_back(poses.back());
        new_wpt_times.push_back(wp_times.back());
        poses = new_poses;
        wp_times = new_wpt_times;
        // for (int i=1;i<poses.size()-1;i++) {
        //   poses[i] = 0.5*(poses[i-1]+poses[i]);//+poses[i+1]);
        // }
    }

    std::cout<<"new poses:\n";
    for (int i=0;i<poses.size();i++) std::cout<<wp_times[i]<<":"<<poses[i].transpose()<<std::endl;
    
    trajectory_msgs::JointTrajectory new_plan;
    new_plan.joint_names = {"edo_joint_1","edo_joint_2","edo_joint_3","edo_joint_4","edo_joint_5","edo_joint_6"};
    Eigen::VectorXd diff = Eigen::VectorXd::Zero(6);
    trajectory_msgs::JointTrajectoryPoint pt;
    for (int j=0;j<6;j++) pt.positions.push_back(cur_pose[j]);
    for (int j=0;j<6;j++) pt.velocities.push_back(0.0);
    for (int j=0;j<6;j++) pt.accelerations.push_back(0.0);
    for (int j=0;j<6;j++) pt.effort.push_back(0.0);
    new_plan.points.push_back(pt);
    for (int i=1;i<poses.size();i++) {
      Eigen::VectorXd new_diff = (poses[i]-poses[i-1]).normalized();
      if (((abs(new_diff.dot(diff))<0.9)&&((poses[i]-cur_pose).norm()>0.5))||(i==poses.size()-1)) {
        for (int j=0;j<6;j++) pt.positions[j] = poses[i][j];
        for (int j=0;j<6;j++) pt.velocities[j] = 0.0;
        for (int j=0;j<6;j++) pt.accelerations[j] = 0.0;
        for (int j=0;j<6;j++) pt.effort[j] = 0.0;
        new_plan.points.push_back(pt);
        diff = new_diff;
      }
    }
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    robot_state::RobotStatePtr state = move_group.getCurrentState();
    // moveit::core::robotStateToRobotStateMsg(*state,new_plan.start_state_);
    robot_trajectory::RobotTrajectory tp_trj(model,"edo");
    tp_trj.setRobotTrajectoryMsg(*state,new_plan);
    iptp.computeTimeStamps(tp_trj);
    moveit_msgs::RobotTrajectory trj_msg;
    tp_trj.getRobotTrajectoryMsg(trj_msg);
    new_plan = trj_msg.joint_trajectory;
    // trj = new_plan;
    std::cout<<new_plan.points.back().time_from_start.toSec();
    new_plan.points.erase(new_plan.points.begin());
    // new_plan.points.erase(new_plan.points.begin());

    std::cout<<"new plan:\n";
    for (int i=0;i<new_plan.points.size();i++) {
      for (int j=0;j<6;j++) std::cout<<new_plan.points[i].positions[j]<<",";
      std::cout<<std::endl;
    }


    new_plan.joint_names = {"edo_joint_1","edo_joint_2","edo_joint_3","edo_joint_4","edo_joint_5","edo_joint_6"};
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
    for (int i=0;i<new_plan.points.size();i++) {
      state->setJointGroupPositions("edo",new_plan.points[i].positions);
      geometry_msgs::Pose pose;
      tf::poseEigenToMsg(state->getGlobalLinkTransform("edo_link_6"),pose);
      // Eigen::Affine3d T_base_tool = chain_->getTransformation(poses[i]);
      // Eigen::Vector3d ee = T_base_tool.translation();
      // geometry_msgs::Point pt;
      // pt.x = ee[0];
      // pt.y = ee[1];
      // pt.z = ee[2];
      mkr.points.push_back(pose.position);
    }
    warp_pub.publish(mkr);
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

}