#include <stap_warp/sequence.h>

namespace stap_test {

humans::humans(ros::NodeHandle nh, std::vector<float> cur_pose, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub):nh(nh),predictor(predictor) {
    nh.getParam("/test_sequence/human_sequence/start_pose", start_pose);
    if (start_pose.size()==0) {
        start_pose = cur_pose;
    }
    if (!nh.getParam("/test_sequence/human_sequence/length", num_steps))
    {
      ROS_ERROR("/test_sequence/human_sequence/length is not defined in sequence.yaml");
    }
    for (int i=0; i<num_steps;i++) {
        data.emplace_back(nh,i,predictor,seq_pub);
    }
}

void humans::predicted_motion(void) {
    std::vector<float> pose = start_pose;
    for (int i=0;i<data.size();i++) {
        data[i].get_predicted_motion(pose);
        pose = data[i].get_last_pose();
    }
}

void humans::show_predictions(std::vector<float> link_len,std::vector<float> link_r) {
    for (int i=0;i<data.size();i++) {
        data[i].show_human(link_len,link_r);
    }
}

human::human(ros::NodeHandle nh, int idx, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub):predictor(predictor),seq_pub(seq_pub) {
    id = idx;
    std::cout<<"human "<<idx<<":";
    nh.getParam("/test_sequence/human_sequence/"+std::to_string(idx)+"/description", description);
    std::cout<<description<<",";
    nh.getParam("/test_sequence/human_sequence/"+std::to_string(idx)+"/after_robot_task", prior_robot_task);
    std::cout<<" prior robot task:"<<prior_robot_task<<",";
    nh.getParam("/test_sequence/human_sequence/"+std::to_string(idx)+"/start_delay", start_delay);
    std::cout<<" start dly:"<<start_delay<<",";
    nh.getParam("/test_sequence/human_sequence/"+std::to_string(idx)+"/reach_target", reach_target);
    std::cout<<" reach tgt:";
    for (int q=0;q<3;q++) std::cout<<reach_target[q]<<",";
    std::string arm_string;
    nh.getParam("/test_sequence/human_sequence/"+std::to_string(idx)+"/arm", arm_string);
    std::cout<<" arm:"<<arm_string<<std::endl;
    if (arm_string=="left") arm_left = true;
    arm_right = !arm_left;
    nh.getParam("/human_link_radii", radii);
}

void human::get_predicted_motion(std::vector<float> start_pose) {
    stap_warp::human_prediction srv;
    srv.request.start_pose = start_pose;
    srv.request.reach_target = reach_target;
    srv.request.active_hand = !arm_right;
    if (predictor->call(srv)) {
        int n_cols = srv.response.pose_sequence.layout.dim[0].size;
        int i = 0;
        while (true) {
            std::pair<float,std::vector<float>> seq_step;
            seq_step.first = srv.response.pose_sequence.data[i];
            for (int j=1;j<n_cols;j++) {
                seq_step.second.push_back(srv.response.pose_sequence.data[i+j]);
            }
            sequence.push_back(seq_step);
            i+=n_cols;
            if (i>srv.response.pose_sequence.data.size()-n_cols) break;
            ROS_INFO_STREAM("human "<<id<<" received sequence with "<<sequence.size()<<" steps");
        }
        end_pose = sequence.back().second;
    }
}
    
void human::forward_kinematics(std::vector<float> pose_elements, std::vector<Eigen::Vector3f> &link_centroids, std::vector<Eigen::Quaternionf> &link_quats) {

    Eigen::Vector3f pelvis_loc = {pose_elements[0],pose_elements[1],pose_elements[2]};
    // pelvis_loc = transform_to_world*pelvis_loc;
    // Eigen::Quaternionf quat_to_world(transform_to_world.rotation());

    Eigen::Quaternionf z_axis_quat(0,0,0,1);
    Eigen::Quaternionf x_axis_quat(0.707,0,-0.707,0);
    std::vector<Eigen::Quaternionf> quats;
    Eigen::Quaternionf q;
    for (int i=0;i<7;i++){
        q = Eigen::Quaternionf(pose_elements[i*4+3],pose_elements[i*4+4],pose_elements[i*4+5],pose_elements[i*4+6]);
        quats.push_back(q);
        // ROS_INFO_STREAM("quat "<<q.w()<<" "<<q.vec().transpose());
    }
    link_centroids.clear();
    link_quats.clear();
    std::vector<Eigen::Vector3f> human_points;
    Eigen::Quaternionf z_spine = quats[0]*z_axis_quat*quats[0].inverse();
    Eigen::Quaternionf x_spine = quats[0]*x_axis_quat*quats[0].inverse();
    // link_quats.push_back(z_spine);
    Eigen::Vector3f spine_top = pelvis_loc+link_lengths_[0]*z_spine.vec();
    human_points.push_back(pelvis_loc);
    human_points.push_back(spine_top);
    link_centroids.push_back(pelvis_loc+0.5*link_lengths_[0]*z_spine.vec());
    // ROS_INFO_STREAM("spine top "<<spine_top.transpose());
    Eigen::Quaternionf z_neck = quats[1]*z_axis_quat*quats[1].inverse();
    Eigen::Quaternionf x_neck = quats[1]*x_axis_quat*quats[1].inverse();
    // link_quats.push_back(z_neck);
    Eigen::Vector3f head = spine_top+link_lengths_[1]*z_neck.vec();
    human_points.push_back(head);
    link_centroids.push_back(spine_top+0.5*link_lengths_[1]*z_neck.vec());
    // ROS_INFO_STREAM("head top "<<head.transpose());
    Eigen::Quaternionf z_shoulders = quats[2]*z_axis_quat*quats[2].inverse();
    Eigen::Vector3f l_shoulder = spine_top-0.5*link_lengths_[2]*z_shoulders.vec();
    human_points.push_back(l_shoulder);
    // ROS_INFO_STREAM("l_shoulder "<<l_shoulder.transpose());
    Eigen::Quaternionf z_e1 = quats[3]*z_axis_quat*quats[3].inverse();
    Eigen::Quaternionf x_e1 = quats[3]*x_axis_quat*quats[3].inverse();
    // link_quats.push_back(x_e1);
    Eigen::Vector3f e1 = l_shoulder+link_lengths_[3]*z_e1.vec();
    human_points.push_back(e1);
    link_centroids.push_back(l_shoulder+0.5*link_lengths_[3]*z_e1.vec());
    Eigen::Quaternionf z_w1 = quats[4]*z_axis_quat*quats[4].inverse();
    Eigen::Quaternionf x_w1 = quats[4]*x_axis_quat*quats[4].inverse();
    // link_quats.push_back(x_w1);
    Eigen::Vector3f w1 = e1+(link_lengths_[4]+0.1)*z_w1.vec();
    human_points.push_back(w1);
    link_centroids.push_back(e1+0.5*(link_lengths_[4])*z_w1.vec());
    Eigen::Vector3f r_shoulder = spine_top+0.5*link_lengths_[2]*z_shoulders.vec();
    human_points.push_back(r_shoulder);
    // ROS_INFO_STREAM("r_shoulder "<<r_shoulder.transpose());
    Eigen::Quaternionf z_e2 = quats[5]*z_axis_quat*quats[5].inverse();
    Eigen::Quaternionf x_e2 = quats[5]*x_axis_quat*quats[5].inverse();
    // link_quats.push_back(x_e2);
    Eigen::Vector3f e2 = r_shoulder+link_lengths_[5]*z_e2.vec();
    human_points.push_back(e2);
    link_centroids.push_back(r_shoulder+0.5*link_lengths_[5]*z_e2.vec());
    Eigen::Quaternionf z_w2 = quats[6]*z_axis_quat*quats[6].inverse();
    Eigen::Quaternionf x_w2 = quats[6]*x_axis_quat*quats[6].inverse();
    // link_quats.push_back(x_w2);
    Eigen::Vector3f w2 = e2+(link_lengths_[6]+0.1)*z_w2.vec();
    human_points.push_back(w2);
    link_centroids.push_back(e2+0.5*(link_lengths_[6])*z_w2.vec());

    link_quats.push_back(quats[0]);
    link_quats.push_back(quats[1]);
    link_quats.push_back(quats[3]);
    link_quats.push_back(quats[4]);
    link_quats.push_back(quats[5]);
    link_quats.push_back(quats[6]);

    
}

void human::show_human(std::vector<float> link_len,std::vector<float> link_r) {
    link_lengths_ = link_len;
    radii = link_r;
    Eigen::Vector3f start_color;
    start_color[0] = 0;
    start_color[1] = 1;
    start_color[2] = 0;
    Eigen::Vector3f color_diff(1,0,0);
    color_diff-=start_color;
    ros::Rate r(10);
    for (int i=0;i<sequence.size();i++) {
        std::vector<Eigen::Vector3f> link_centroids;
        std::vector<Eigen::Quaternionf> link_quats;
        forward_kinematics(sequence[i].second,link_centroids,link_quats);
        std::vector<int> ids = {0,1,3,4,5,6};
        visualization_msgs::MarkerArray mkr_arr;
        for (int b=0;b<6;b++) {
            int a = ids[b];
            visualization_msgs::Marker mkr;
            mkr.id=mkr_arr.markers.size()+1;
            mkr.lifetime = ros::Duration(5);
            mkr.type=mkr.CYLINDER;
            Eigen::Vector3f c = color_diff*((double)i/(double)sequence.size())+start_color;
            mkr.color.r=c[0];
            mkr.color.b=c[1];
            mkr.color.g=c[2];
            mkr.color.a=0.2;
            mkr.pose.position.x = link_centroids[b][0];
            mkr.pose.position.y = link_centroids[b][1];
            mkr.pose.position.z = link_centroids[b][2];
            mkr.pose.orientation.x = link_quats[b].x();
            mkr.pose.orientation.y = link_quats[b].y();
            mkr.pose.orientation.z = link_quats[b].z();
            mkr.pose.orientation.w = link_quats[b].w();
            mkr.scale.x = 2*radii[a];
            mkr.scale.y = 2*radii[a];
            mkr.scale.z = link_lengths_[a];
            mkr.header.frame_id = "world";
            mkr_arr.markers.push_back(mkr);
        }
        seq_pub->publish(mkr_arr);
        r.sleep();
    }
}

robot_sequence::robot_sequence(ros::NodeHandle nh):nh(nh) {
    if (!nh.getParam("/test_sequence/robot_sequence/length", num_steps))
    {
      ROS_ERROR("/test_sequence/robot_sequence/length is not defined in sequence.yaml");
    }
    for (int i=0; i<num_steps;i++) {
        data.emplace_back(nh,i);
    }    
    int num_robot_poses = 0;
    if (!nh.getParam("/test_sequence/robot_poses/length", num_robot_poses))
    {
      ROS_ERROR("/test_sequence/robot_poses/length is not defined in sequence.yaml");
    }
    std::vector<double> pose(6);
    for (int i=0; i<num_robot_poses;i++) {
        nh.getParam("/test_sequence/robot_poses/"+std::to_string(i), pose);
        goals_angles.push_back(pose);
        std::cout<<"goal "<<i<<":";
        for (int q=0;q<6;q++) std::cout<<goals_angles.back()[q]<<",";
        std::cout<<std::endl;
    }

}
robot_segment::robot_segment(ros::NodeHandle nh, int idx) {
    id = idx;
    std::cout<<"robot seg "<<idx<<":";
    nh.getParam("/test_sequence/robot_sequence/"+std::to_string(idx)+"/description", description);
    std::cout<<description<<",";
    nh.getParam("/test_sequence/robot_sequence/"+std::to_string(idx)+"/goal", goal_id);
    std::cout<<" goal id:"<<goal_id<<",";
    nh.getParam("/test_sequence/robot_sequence/"+std::to_string(idx)+"/type", type);
    std::cout<<" type:"<<type<<",";
    nh.getParam("/test_sequence/robot_sequence/"+std::to_string(idx)+"/after_human_task", prior_human_task);
    std::cout<<" prior_human_task:"<<prior_human_task<<std::endl;
}
}