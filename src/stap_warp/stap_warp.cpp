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
    rosdyn::ChainPtr chain_ = rosdyn::createChain(robo_model, base_frame_, tool_frame, grav);
    ssm=std::make_shared<ssm15066::DeterministicSSM>(chain_,nh); 
}

void stap_warper::warp(moveit::planning_interface::MoveGroupInterface::Plan &plan, std::vector<std::pair<double,Eigen::MatrixXd>> human_seq, double time_since_start) {
    int start_p = 1;
    for (int p=1;p<plan.trajectory_.joint_trajectory.points.size();p++) {
        if (plan.trajectory_.joint_trajectory.points[start_p].time_from_start.toSec()>time_since_start) start_p++;
    }
    for (int iter=0;iter<max_iters;i++) {
        for (int p=start_p;p<plan.trajectory_.joint_trajectory.points.size();p++) {
            std::vector<double> pos1 = plan.trajectory_.joint_trajectory.points[p-1].positions
            std::vector<double> pos2 = plan.trajectory_.joint_trajectory.points[p].positions
            Eigen::VectorXd pose1(pos.data());
            Eigen::VectorXd pose2(pos.data());
            Eigen::VectorXd diff = pose2-pose1;
            int num_steps = std::max(std::ceil(diff.norm()/0.1),1);
            double nominal_time = (plan.trajectory_.joint_trajectory.points[p].time_from_start-plan.trajectory_.joint_trajectory.points[p-1].time_from_start).toSec();
            Eigen::VectorXd dq = diff/nominal_time;
            for (int s=0;s<num_steps;s++) {
                double h_time = plan.trajectory_.joint_trajectory.points[p].time_from_start.toSec()-time_from_start;
                Eigen::VectorXd cur_pose = pose1+ double(s/num_steps)*diff;
                Eigen::Matrix6Xd jacobian = chain_->getJacobian(cur_pose);
                ssm->setPointCloud(human_seq[t_step].second)
                double scaling = ssm->computeScaling(cur_pose,dq);
                Eigen::Vector3d F_rep = repulsion*std::max(1-scaling,0);
            }
        }
    }
}

Eigen::VectorXd stap_warper::apf_pose(const Eigen::VectorXd current_pose, const Eigen::Vector3d& ee_tgt) {
  return apf_pose(current_pose,ee_tgt,0);
}

Eigen::VectorXd stap_warper::apf_pose(const Eigen::VectorXd current_pose, const Eigen::Vector3d& ee_tgt, int iteration) {
  
  Eigen::Vector3d current_ee_pos = chain_->getTransformation(current_pose).translation();
  Eigen::Vector3d F_att = -apf_zeta*(ee_tgt-current_ee_pos);
  // std::cout<<"F_att:"<<F_att<<std::endl;
  Eigen::Matrix6Xd jacobian = chain_->getJacobian(current_pose);
  // std::cout<<"jacobian:"<<jacobian.block(0,0,3,jacobian.cols())<<std::endl;
  Eigen::VectorXd tau = jacobian.block(0,0,3,jacobian.cols()).transpose()*F_att;
  Eigen::VectorXd end_pose = current_pose-apf_alpha*tau;
  // std::cout<<"ee_tgt:"<<ee_tgt.transpose()<<",iter:"<<iteration<<", diff:"<<(end_pose-current_pose).norm()<<std::endl;
  // std::cout<<"end_pose:"<<end_pose.transpose()<<std::endl;
  if ((iteration<200)&&((end_pose-current_pose).norm()>0.01)) {
    end_pose = apf_pose(end_pose,ee_tgt,iteration+1);
  }
  return end_pose;
}