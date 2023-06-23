#include <stap_warp/sequence.h>

namespace stap_test {

humans::humans(ros::NodeHandle nh, std::vector<float> cur_pose, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub,std::shared_ptr<avoidance_intervals::skeleton> skel, std::shared_ptr<ros::Publisher> pub_txt, int test_num, Eigen::Isometry3f transform_to_world):nh(nh),predictor(predictor),skel(skel),seq_pub(seq_pub),pub_txt(pub_txt), transform_to_world(transform_to_world) {
    nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/start_pose", start_pose);
    if (start_pose.size()==0) {
        start_pose = cur_pose;
    }
    if (!nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/length", num_steps))
    {
      ROS_ERROR_STREAM("/test_sequence/"<<test_num<<"/human_sequence/length is not defined in sequence.yaml");
    }
    data.clear();
    data.reserve(num_steps);
    for (int i=0; i<num_steps;i++) {
        data.emplace_back(nh,i,predictor,seq_pub,prediction_dt,test_num,transform_to_world);
    }
    human_model_pub = nh.advertise<stap_warp::joint_seq>("human_model_seq", 0,false);
    human_done_srv = nh.serviceClient<stap_warp::human_motion_done>("human_motion_done");
    human_reset_srv = nh.serviceClient<stap_warp::human_motion_reset>("human_human_motion_resetmotion_done");
    wrist_trace_pub = nh.advertise<visualization_msgs::Marker>("prediction_wrist_traces", 0,false);
    reach_tgt_pub = nh.advertise<visualization_msgs::Marker>("reach_tgt", 0,false);
    nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/simulated_switch_times", sim_switch_times);
    std::cout<<"sim switch times:";
    for (int i =0;i<sim_switch_times.size();i++) std::cout<<sim_switch_times[i]<<",";
    std::cout<<std::endl;
}

float humans::pub_model(int start_seq, int robot_step, float start_tm_in_robot_seq) {
    ROS_INFO_STREAM("pub human start:"<<start_seq<<", steps:"<<data.size()<<", robot step:"<<robot_step<<", time into robot seq:"<<start_tm_in_robot_seq);
    if ((start_seq<0)||(start_seq>=data.size())) return 0.0;
    generate_full_sequence(start_seq,robot_step,start_tm_in_robot_seq);
    stap_warp::joint_seq seq_msg;
    float elapsed_tm = std::round(10.0*start_tm_in_robot_seq)*0.1;
    int s = 0;
    float dt = prediction_dt;
    bool some_points = false;
    for (s=start_seq;s<data.size();s++) {
        // ROS_INFO_STREAM("prior:"<<data[s].get_prior_robot_task()<<",robot_step"<<robot_step);
        if (data[s].get_prior_robot_task()>=robot_step) break;
        if ((data[s].get_start_delay()>0.0)&&(data[s].get_seq_size()>0))  {
            float start_tm = elapsed_tm;
            while (elapsed_tm - start_tm<data[s].get_start_delay()) {
                if (elapsed_tm>0) {
                    stap_warp::joint_seq_elem seq_elem;
                    seq_elem.time = elapsed_tm;
                    // ROS_INFO_STREAM("seq time1:"<<seq_elem.time);
                    seq_elem.joint_pos = std::get<1>(data[s].get_seq(0));
                    seq_elem.quats = std::get<2>(data[s].get_seq(0));
                    seq_msg.sequence.push_back(seq_elem);
                    some_points = true;
                }
                elapsed_tm += dt;
            }
        }
        // ROS_INFO_STREAM("adding prediction for human step "<<s);
        for (int i=0;i<data[s].get_seq_size();i++) {
            float seq_time = std::get<0>(data[s].get_seq(i))+elapsed_tm;
            // ROS_INFO_STREAM("seq time2:"<<seq_time<<", elpased tm:"<<elapsed_tm<<" from human "<<s<<", step "<<i<<", tm "<<std::get<0>(data[s].get_seq(i)));
            if (seq_time>=0.0) {
                stap_warp::joint_seq_elem seq_elem;
                seq_elem.time = seq_time;
                seq_elem.joint_pos = std::get<1>(data[s].get_seq(i));
                seq_elem.quats = std::get<2>(data[s].get_seq(i));
                seq_msg.sequence.push_back(seq_elem);
                some_points = true;
            }
        }
        elapsed_tm += std::get<0>(data[s].get_seq(data[s].get_seq_size()-1))+dt;
    }
    if ((!some_points) && (s>0)) {
        if ((data[s-1].get_seq_size()>0)) {
            stap_warp::joint_seq_elem seq_elem;
            seq_elem.time = 0.0;
            // ROS_INFO_STREAM("seq time3:"<<seq_elem.time<<","<<s-1);
            seq_elem.joint_pos = std::get<1>(data[s-1].get_seq(data[s-1].get_seq_size()-1));
            seq_elem.quats = std::get<2>(data[s-1].get_seq(data[s-1].get_seq_size()-1));
            seq_msg.sequence.push_back(seq_elem);
        }
    }
    human_model_pub.publish(seq_msg);
    std::lock_guard<std::mutex> l(joint_seq_mtx);
    skel->joint_seq = full_joint_seq_f;
    skel->quat_seq = full_quat_seq;
    return elapsed_tm-start_tm_in_robot_seq;
}

void humans::generate_full_sequence(int start_seq, int robot_step, float start_tm_in_robot_seq,bool skip_first_delay) {
    if ((start_seq<0)||(start_seq>=data.size())) return;
    float elapsed_tm = 0.0;
    if (!skip_first_delay) {
        elapsed_tm = std::round(10.0*start_tm_in_robot_seq)*0.1;
    } else {
        elapsed_tm = 0.0;
    }
    int s = 0;
    float dt = 0.1;
    std::lock_guard<std::mutex> l(joint_seq_mtx);
    full_joint_seq.clear();
    full_joint_seq_f.clear();
    full_quat_seq.clear();
    bool some_points = false;
    visualization_msgs::Marker mkr;
    mkr.id=106;
    mkr.lifetime = ros::Duration(0.0);
    mkr.type=mkr.LINE_STRIP;
    mkr.color.r=1.0;
    mkr.color.b=0.0;
    mkr.color.g=0.0;
    mkr.color.a=1.0;
    mkr.pose.position.x = 0;
    mkr.pose.position.y = 0;
    mkr.pose.position.z = 0;
    mkr.pose.orientation.x = 0;
    mkr.pose.orientation.y = 0;
    mkr.pose.orientation.z = 0;
    mkr.pose.orientation.w = 1;
    mkr.scale.x = 0.01;
    mkr.scale.y = 0.01;
    mkr.scale.z = 0.01;
    mkr.header.frame_id = "world";
    for (s=start_seq;s<data.size();s++) {
        if (data[s].get_prior_robot_task()>=robot_step) break;
        if ((data[s].get_start_delay()>0.0))  {
            float start_tm = elapsed_tm;
            while (elapsed_tm - start_tm<data[s].get_start_delay()) {
                if ((elapsed_tm>0) && (data[s].get_seq_size()>0)) {
                    full_joint_seq.emplace_back(elapsed_tm,std::get<3>(data[s].get_seq(0)));
                    Eigen::MatrixXd tmp_jnts_mat = std::get<3>(data[s].get_seq(0));
                    std::vector<Eigen::Vector3f> tmp_jnts;
                    for (int c=0;c<tmp_jnts_mat.cols();c++) tmp_jnts.emplace_back(tmp_jnts_mat.col(c).cast<float>());
                    full_joint_seq_f.emplace_back(elapsed_tm,tmp_jnts);
                    std::vector<Eigen::Quaternionf> tmp_quats;
                    std::vector<float> tmp_quat_vec = std::get<2>(data[s].get_seq(0));
                    for (int q=0;q<7;q++) tmp_quats.emplace_back(tmp_quat_vec[q*4+3],tmp_quat_vec[q*4+4],tmp_quat_vec[q*4+5],tmp_quat_vec[q*4+6]);
                    full_quat_seq.emplace_back(elapsed_tm,tmp_quats);
                    some_points = true;
                }
                elapsed_tm += dt;
            }
        }
        for (int i=0;i<data[s].get_seq_size();i++) {
            float seq_time = std::get<0>(data[s].get_seq(i))+elapsed_tm;
            if (seq_time>=0.0) {
                full_joint_seq.emplace_back(seq_time,std::get<3>(data[s].get_seq(i)));
                Eigen::MatrixXd tmp_jnts_mat = std::get<3>(data[s].get_seq(i));
                // std::cout<<"test:"<<tmp_jnts_mat.col(5).transpose()<<":"<<tmp_jnts_mat.col(8).transpose()<<std::endl;
                geometry_msgs::Point pt;
                pt.x = tmp_jnts_mat(0,5);
                pt.y = tmp_jnts_mat(1,5);
                pt.z = tmp_jnts_mat(2,5);
                // // std::cout<<pt<<std::endl;
                // mkr.points.push_back(pt);
                // pt.x = tmp_jnts_mat(0,8);
                // pt.y = tmp_jnts_mat(1,8);
                // pt.z = tmp_jnts_mat(2,8);
                mkr.points.push_back(pt);
                // std::cout<<pt<<std::endl;
                std::vector<Eigen::Vector3f> tmp_jnts;
                for (int c=0;c<tmp_jnts_mat.cols();c++) tmp_jnts.emplace_back(tmp_jnts_mat.col(c).cast<float>());
                full_joint_seq_f.emplace_back(seq_time,tmp_jnts);
                std::vector<Eigen::Quaternionf> tmp_quats;
                std::vector<float> tmp_quat_vec = std::get<2>(data[s].get_seq(i));
                for (int q=0;q<7;q++) tmp_quats.emplace_back(tmp_quat_vec[q*4+3],tmp_quat_vec[q*4+4],tmp_quat_vec[q*4+5],tmp_quat_vec[q*4+6]);
                full_quat_seq.emplace_back(seq_time,tmp_quats);
                some_points = true;
            }
        }
        elapsed_tm += std::get<0>(data[s].get_seq(data[s].get_seq_size()-1))+dt;
        skip_first_delay=false;
        // std::cout<<"s:"<<s<<"-"<<full_joint_seq.size()<<std::endl;
    }
    if ((!some_points) && (s>0)) {
        if (data[s-1].get_seq_size()>0) {
            full_joint_seq.emplace_back(0.0,std::get<3>(data[s-1].get_seq(data[s-1].get_seq_size()-1)));
            Eigen::MatrixXd tmp_jnts_mat = std::get<3>(data[s-1].get_seq(data[s-1].get_seq_size()-1));
            // std::cout<<"test2:"<<tmp_jnts_mat.col(5).transpose()<<":"<<tmp_jnts_mat.col(8).transpose()<<std::endl;
            geometry_msgs::Point pt;
            pt.x = tmp_jnts_mat(0,5);
            pt.y = tmp_jnts_mat(1,5);
            pt.z = tmp_jnts_mat(2,5);
            // mkr.points.push_back(pt);
            // pt.x = tmp_jnts_mat(0,8);
            // pt.y = tmp_jnts_mat(1,8);
            // pt.z = tmp_jnts_mat(2,8);
            mkr.points.push_back(pt);
            std::vector<Eigen::Vector3f> tmp_jnts;
            for (int c=0;c<tmp_jnts_mat.cols();c++) tmp_jnts.emplace_back(tmp_jnts_mat.col(c).cast<float>());
            full_joint_seq_f.emplace_back(0.0,tmp_jnts);
            std::vector<Eigen::Quaternionf> tmp_quats;
            std::vector<float> tmp_quat_vec = std::get<2>(data[s-1].get_seq(data[s-1].get_seq_size()-1));
            for (int q=0;q<7;q++) tmp_quats.emplace_back(tmp_quat_vec[q*4+3],tmp_quat_vec[q*4+4],tmp_quat_vec[q*4+5],tmp_quat_vec[q*4+6]);
            full_quat_seq.emplace_back(0.0,tmp_quats);
            // ROS_INFO_STREAM("no points:"<<s-1);
        }
    }
    wrist_trace_pub.publish(mkr);
}

void humans::show_reach_tgt(int step_num) {
    if ((step_num<0)||(step_num>data.size()-1)) return;
    visualization_msgs::Marker mkr2;
    mkr2.id=107;
    mkr2.lifetime = ros::Duration(0.0);
    mkr2.type=mkr2.SPHERE_LIST;
    mkr2.color.r=0.0;
    mkr2.color.b=1.0;
    mkr2.color.g=0.0;
    mkr2.color.a=1.0;
    mkr2.pose.position.x = 0;
    mkr2.pose.position.y = 0;
    mkr2.pose.position.z = 0;
    mkr2.pose.orientation.x = 0;
    mkr2.pose.orientation.y = 0;
    mkr2.pose.orientation.z = 0;
    mkr2.pose.orientation.w = 1;
    mkr2.scale.x = 0.04;
    mkr2.scale.y = 0.04;
    mkr2.scale.z = 0.04;
    mkr2.header.frame_id = "world";
    geometry_msgs::Point pt;
    if (!data[step_num].both_arms) {
        pt.x = data[step_num].reach_target[0];
        pt.y = data[step_num].reach_target[1];
        pt.z = data[step_num].reach_target[2];
        mkr2.points.push_back(pt);
    } else {
        pt.x = data[step_num].reach_target_left[0];
        pt.y = data[step_num].reach_target_left[1];
        pt.z = data[step_num].reach_target_left[2];
        mkr2.points.push_back(pt);
        pt.x = data[step_num].reach_target_right[0];
        pt.y = data[step_num].reach_target_right[1];
        pt.z = data[step_num].reach_target_right[2];
        mkr2.points.push_back(pt);
    }
    wrist_trace_pub.publish(mkr2);
}

void humans::save_full_seq(std::string file_name) {
    std::string log_file= ros::package::getPath("stap_warp")+"/data/"+file_name;
    std::ofstream outFile(log_file,std::ofstream::trunc);

    for (int i=0;i<full_quat_seq.size();i++) {
        outFile<<full_quat_seq[i].first<<","<<full_joint_seq_f[i].second[0][0]<<","<<full_joint_seq_f[i].second[0][1]<<","<<full_joint_seq_f[i].second[0][2]<<",";
        for (int j=0;j<full_quat_seq[i].second.size();j++) {
            outFile<<full_quat_seq[i].second[j].w()<<","<<full_quat_seq[i].second[j].x()<<","<<full_quat_seq[i].second[j].y()<<","<<full_quat_seq[i].second[j].z()<<",";
        }
        outFile<<"\n";
        // std::cout<<"wrote a line\n";
    }
    outFile.close();
}

void humans::predicted_motion(void) {
    std::vector<float> pose = start_pose;
    for (int i=0;i<data.size();i++) {
        data[i].get_predicted_motion(pose);
        data[i].set_nominal_seq();
        data[i].get_last_pose(pose);
    }
}

void humans::show_predictions(std::vector<float> link_len,std::vector<float> link_r) {
    for (int i=0;i<data.size();i++) {
        visualization_msgs::Marker mkr;
        mkr.id=105;
        mkr.lifetime = ros::Duration(5.0);
        mkr.type=mkr.TEXT_VIEW_FACING;
        mkr.color.r=1.0;
        mkr.color.b=1.0;
        mkr.color.g=1.0;
        mkr.color.a=1.0;
        mkr.pose.position.x = -1;
        mkr.pose.position.y = 0;
        mkr.pose.position.z = 1;
        mkr.pose.orientation.x = 0;
        mkr.pose.orientation.y = 0;
        mkr.pose.orientation.z = 0;
        mkr.pose.orientation.w = 1;
        mkr.scale.z = 0.3;
        mkr.header.frame_id = "world";
        mkr.text = data[i].description;
        pub_txt->publish(gen_overlay_text(data[i].description));
        data[i].show_human(link_len,link_r);
    }
    
}

bool humans::simulate_step(int step_num, double elapsed_time, std::vector<float>& current_pose) {
    if ((step_num<0)||(step_num>data.size()-1)) return true;
    std::vector<std::tuple<float,std::vector<float>,std::vector<float>,Eigen::MatrixXd>> step_seq = data[step_num].get_nominal_seq();
    int i =0;
    for (i=0;i<step_seq.size();i++) {
        if (elapsed_time<std::get<0>(step_seq[i])) break;
    }
    if (i>step_seq.size()-1) i = step_seq.size()-1;
    current_pose = std::get<2>(step_seq[i]);
    // std::cout<<i<<","<<step_seq.size()<<std::endl;
    data[step_num].show_step(i);
    data[step_num].done = elapsed_time>=data[step_num].get_step_end_time();
    return data[step_num].done;
}

bool humans::is_step_done(int step) {
    if ((step<0)||(step>data.size()-1)) return false;
    if (data[step].done) return true;
    if (!data[step].check_pos) return true;
    stap_warp::human_motion_done srv;
    if (data[step].both_arms) {
        bool left_arm_done = false;
        srv.request.reach_target = data[step].reach_target_left;
        srv.request.active_hand = true;
        if (human_done_srv.call(srv)) {
            left_arm_done=srv.response.done;
        }
        bool right_arm_done = false;
        srv.request.reach_target = data[step].reach_target_right;
        srv.request.active_hand = false;
        if (human_done_srv.call(srv)) {
            right_arm_done=srv.response.done;
        }
        data[step].done = left_arm_done && right_arm_done;
        return left_arm_done && right_arm_done;
    } else {
        srv.request.reach_target = data[step].get_tgt();
        srv.request.active_hand = !data[step].arm_right;
        if (human_done_srv.call(srv)) {
            data[step].done = srv.response.done;
            return srv.response.done;
        }
    }
    ROS_ERROR("human done server call failed");
    return false;
}

void humans::reset_motion_done(void) {
    stap_warp::human_motion_reset srv;
    srv.request.reset = true;
    human_reset_srv.call(srv);
}

void humans::update_predictions(int cur_step, std::vector<float> cur_pose, int robot_step, double current_robot_time) {
    std::vector<float> pose = cur_pose;
    for (int i=cur_step;i<data.size();i++) {
        if (data[i].get_prior_robot_task()>=robot_step) break;
        data[i].get_predicted_motion(pose);
        data[i].get_last_pose(pose);
    }
    generate_full_sequence(cur_step, robot_step, current_robot_time,true);
}

void humans::pub_descrition(int step_num) {
    if ((step_num<0)||(step_num>data.size()-1)) return;
    pub_txt->publish(gen_overlay_text(data[step_num].description));
}

void humans::show_sequence(void) {
    Eigen::Vector3f start_color;
    start_color[0] = 0;
    start_color[1] = 1;
    start_color[2] = 0;
    Eigen::Vector3f color_diff(1,0,0);
    color_diff-=start_color;
    for (int i=0;i<full_quat_seq.size();i++) {
        // std::vector<Eigen::Vector3f> link_centroids;
        // std::vector<Eigen::Quaternionf> link_quats;
        // Eigen::Matrix3Xd joint_vec = Eigen::MatrixXd::Zero(3,15);
        // data[0].forward_kinematics(full_quat_seq[i].second,link_centroids,link_quats,joint_vec);
        std::vector<int> ids = {0,1,3,4,5,6};
        std::vector<int> ids2 = {0,1,3,4,6,7};
        std::vector<int> ids3 = {1,2,4,5,7,8};
        visualization_msgs::MarkerArray mkr_arr;
        for (int b=0;b<6;b++) {
            int a = ids[b];
            visualization_msgs::Marker mkr;
            mkr.id=30000+7*i+b;
            mkr.lifetime = ros::Duration(5.0);
            mkr.type=mkr.CYLINDER;
            Eigen::Vector3f c = color_diff*((double)i/(double)full_quat_seq.size())+start_color;
            mkr.color.r=c[0];
            mkr.color.b=c[1];
            mkr.color.g=c[2];
            mkr.color.a=1.0;
            int j1 = ids2[b];
            int j2 = ids3[b];
            Eigen::Vector3f tmp_vec = 0.5*(full_joint_seq_f[i].second[j1]+full_joint_seq_f[i].second[j2]);
            mkr.pose.position.x = tmp_vec[0];
            mkr.pose.position.y = tmp_vec[1];
            mkr.pose.position.z = tmp_vec[2];
            mkr.pose.orientation.x = full_quat_seq[i].second[a].x();
            mkr.pose.orientation.y = full_quat_seq[i].second[a].y();
            mkr.pose.orientation.z = full_quat_seq[i].second[a].z();
            mkr.pose.orientation.w = full_quat_seq[i].second[a].w();
            mkr.scale.x = 2*data[0].radii[a];
            mkr.scale.y = 2*data[0].radii[a];
            mkr.scale.z = data[0].link_lengths_[a];
            mkr.header.frame_id = "world";
            mkr_arr.markers.push_back(mkr);
        }
        seq_pub->publish(mkr_arr);
    }
}

human::human(ros::NodeHandle nh, int idx, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub, float prediction_dt, int test_num, Eigen::Isometry3f transform_to_world):predictor(predictor),seq_pub(seq_pub),prediction_dt(prediction_dt), transform_to_world(transform_to_world) {
    id = idx;
    std::cout<<"test_num:"<<test_num<<", human "<<idx<<":";
    nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/show_human_rate",show_human_rate);
    nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/"+std::to_string(idx)+"/description", description);
    std::cout<<description<<",";
    nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/"+std::to_string(idx)+"/after_robot_task", prior_robot_task);
    std::cout<<" prior robot task:"<<prior_robot_task<<",";
    nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/"+std::to_string(idx)+"/start_delay", start_delay);
    std::cout<<" start dly:"<<start_delay<<",";
    std::string arm_string;
    nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/"+std::to_string(idx)+"/arm", arm_string);
    if (arm_string=="both") {
        std::cout<<"what now?"<<std::endl;
        nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/"+std::to_string(idx)+"/reach_target_left", reach_target_left);
        nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/"+std::to_string(idx)+"/reach_target_right", reach_target_right);
        std::cout<<"left reach tgt:";
        for (int q=0;q<3;q++) std::cout<<reach_target_left[q]<<",";
        std::cout<<"right reach tgt:";
        for (int q=0;q<3;q++) std::cout<<reach_target_right[q]<<",";
    }
    else {
        nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/"+std::to_string(idx)+"/reach_target", reach_target);
        std::cout<<" reach tgt:";
        for (int q=0;q<3;q++) std::cout<<reach_target[q]<<",";
    }
    std::cout<<" arm:"<<arm_string<<std::endl;
    if (arm_string=="both") {
        both_arms = true;
        arm_left = false;
        arm_right = false;
    }
    else {
        both_arms = false;
        if (arm_string=="left") arm_left = true;
        arm_right = !arm_left;
    }
    nh.getParam("/human_link_radii", radii);
    nh.getParam("/test_sequence/"+std::to_string(test_num)+"/human_sequence/"+std::to_string(idx)+"/check_pos", check_pos);
    transform_to_world_inv = transform_to_world.inverse();
}

void human::show_step(int step_num) {
    if ((step_num<0)||(step_num>nom_sequence.size())) return;
    std::vector<Eigen::Vector3f> link_centroids;
    std::vector<Eigen::Quaternionf> link_quats;
    Eigen::Matrix3Xd joint_vec = Eigen::MatrixXd::Zero(3,15);
    forward_kinematics(std::get<2>(nom_sequence[step_num]),link_centroids,link_quats,joint_vec);
    std::vector<int> ids = {0,1,3,4,5,6};
    visualization_msgs::MarkerArray mkr_arr;
    for (int b=0;b<6;b++) {
        int a = ids[b];
        visualization_msgs::Marker mkr;
        mkr.id=mkr_arr.markers.size()+1;
        mkr.lifetime = ros::Duration(5);
        mkr.type=mkr.CYLINDER;
        mkr.color.r=1.0;//c[0];
        mkr.color.b=0.0;//c[1];
        mkr.color.g=0.0;//c[2];
        mkr.color.a=1.0;
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
}

std::vector<float> human::transform_pose_to_UF(std::vector<float> input_pose) {
    std::vector<float> output_pose;
    Eigen::Vector3f pelvis_loc = transform_to_world_inv*Eigen::Vector3f(input_pose[0],input_pose[1],input_pose[2]);
    for (int i=0;i<3;i++) output_pose.push_back(pelvis_loc[i]);
    Eigen::Quaternionf q;
    Eigen::Quaternionf quat_transform(transform_to_world_inv.rotation());
    for (int i=0;i<7;i++){
        q = quat_transform*Eigen::Quaternionf(input_pose[i*4+3],input_pose[i*4+4],input_pose[i*4+5],input_pose[i*4+6]);
        output_pose.push_back(q.w());
        output_pose.push_back(q.x());
        output_pose.push_back(q.y());
        output_pose.push_back(q.z());
    }
    return output_pose;
}

std::vector<float> human::transform_point_to_UF(std::vector<float> p) {
    std::vector<float>out_p;
    Eigen::Vector3f v = transform_to_world_inv*Eigen::Vector3f(p[0],p[1],p[2]);
    for (int i=0;i<3;i++) out_p.push_back(v[i]);
    return out_p;
}

std::vector<float> human::transform_point_to_SW(std::vector<float> p) {
    std::vector<float>out_p;
    Eigen::Vector3f v = transform_to_world*Eigen::Vector3f(p[0],p[1],p[2]);
    for (int i=0;i<3;i++) out_p.push_back(v[i]);
    return out_p;
}

std::vector<float> human::transform_pose_to_SW(std::vector<float> input_pose) {
    std::vector<float> output_pose;
    Eigen::Vector3f pelvis_loc = transform_to_world*Eigen::Vector3f(input_pose[0],input_pose[1],input_pose[2]);
    for (int i=0;i<3;i++) output_pose.push_back(pelvis_loc[i]);
    Eigen::Quaternionf q;
    Eigen::Quaternionf quat_transform(transform_to_world.rotation());
    for (int i=0;i<7;i++){
        q = quat_transform*Eigen::Quaternionf(input_pose[i*4+3],input_pose[i*4+4],input_pose[i*4+5],input_pose[i*4+6]);
        output_pose.push_back(q.w());
        output_pose.push_back(q.x());
        output_pose.push_back(q.y());
        output_pose.push_back(q.z());
    }
    return output_pose;
}

void human::get_predicted_motion(std::vector<float> start_pose) {
    sequence.clear();
    if (done) return;
    if (!both_arms) {
        stap_warp::human_prediction srv;
        //transform start_pose into UF world frame
        srv.request.start_pose = transform_pose_to_UF(start_pose);
        srv.request.reach_target = transform_point_to_UF(reach_target);
        srv.request.active_hand = !arm_right;
        srv.request.dt = prediction_dt;
        if (predictor->call(srv)) {
            int n_cols = srv.response.pose_sequence.layout.dim[0].size;
            // ROS_INFO_STREAM(description<<" ncols:"<<n_cols);
            // std::cout<<srv.response.pose_sequence<<std::endl;
            int i = 0;
            while (true) {
                std::tuple<float,std::vector<float>,std::vector<float>> seq_step;
                std::vector<float> quat_vec;
                std::vector<float> joint_vec;
                for (int j=1;j<n_cols;j++) {
                    quat_vec.push_back(srv.response.pose_sequence.data[i+j]);
                }
                quat_vec = transform_pose_to_SW(quat_vec);
                Eigen::Matrix3Xd joint_mat = Eigen::MatrixXd::Zero(3,15);
                std::vector<Eigen::Vector3f> link_centroids;
                std::vector<Eigen::Quaternionf> link_quats;
                forward_kinematics(quat_vec,link_centroids,link_quats,joint_mat);
                // std::cout<<joint_mat<<std::endl;
                for (int j=0;j<joint_mat.cols();j++) {
                    for (int k=0;k<3;k++) joint_vec.push_back(joint_mat.col(j)[k]);
                }
                // dt, joint positions vector, pose+quat elements vector, and joint positions 3x15 matrix
                sequence.emplace_back(srv.response.pose_sequence.data[i],joint_vec,quat_vec,joint_mat);
                i+=n_cols;
                if (i>srv.response.pose_sequence.data.size()-n_cols) break;
            }
            // ROS_INFO_STREAM("human "<<id<<" received sequence with "<<sequence.size()<<" steps");
            end_pose = std::get<2>(sequence.back());
        }
    } else {
        stap_warp::human_prediction srv;
        srv.request.start_pose = transform_pose_to_UF(start_pose);
        srv.request.reach_target = transform_point_to_UF(reach_target_left);
        srv.request.active_hand = true;
        srv.request.dt = prediction_dt;
        std::vector<std::pair<float,std::vector<float>>> left_quats;
        if (predictor->call(srv)) {
            int n_cols = srv.response.pose_sequence.layout.dim[0].size;
            // ROS_INFO_STREAM(description<<" ncols:"<<n_cols);
            // std::cout<<srv.response.pose_sequence<<std::endl;
            int i = 0;
            while (true) {
                std::vector<float> quat_vec;
                for (int j=1;j<n_cols;j++) {
                    quat_vec.push_back(srv.response.pose_sequence.data[i+j]);
                }
                quat_vec = transform_pose_to_SW(quat_vec);
                left_quats.emplace_back(srv.response.pose_sequence.data[i],quat_vec);
                i+=n_cols;
                if (i>srv.response.pose_sequence.data.size()-n_cols) break;
            }
        }
        srv.request.reach_target = transform_point_to_UF(reach_target_right);
        srv.request.active_hand = false;
        std::vector<std::pair<float,std::vector<float>>> right_quats;
        if (predictor->call(srv)) {
            int n_cols = srv.response.pose_sequence.layout.dim[0].size;
            // ROS_INFO_STREAM(description<<" ncols:"<<n_cols);
            // std::cout<<srv.response.pose_sequence<<std::endl;
            int i = 0;
            while (true) {
                std::vector<float> quat_vec;
                for (int j=1;j<n_cols;j++) {
                    quat_vec.push_back(srv.response.pose_sequence.data[i+j]);
                }
                quat_vec = transform_pose_to_SW(quat_vec);
                right_quats.emplace_back(srv.response.pose_sequence.data[i],quat_vec);
                i+=n_cols;
                if (i>srv.response.pose_sequence.data.size()-n_cols) break;
            }
        }
        sequence.clear();
        std::vector<float> left_arm_quats;
        std::vector<float> right_arm_quats;
        for (int i=0;i<std::max(left_quats.size(),right_quats.size());i++) {
            // std::cout<<i<<","<<std::max(left_quats.size(),right_quats.size())<<std::endl;
            bool use_left = (i<left_quats.size());
            bool use_right = (i<right_quats.size());
            bool use_both = use_left & use_right;
            Eigen::Vector3f pelvis;
            float time = 0.0;
            std::vector<float> quat;
            if (use_both) {
                pelvis = 0.5*(Eigen::Vector3f(left_quats[i].second[0],left_quats[i].second[1],left_quats[i].second[2])+Eigen::Vector3f(right_quats[i].second[0],right_quats[i].second[1],right_quats[i].second[2]));
                time = left_quats[i].first;
            } else if (use_left) {
                pelvis =Eigen::Vector3f(left_quats[i].second[0],left_quats[i].second[1],left_quats[i].second[2]);
                time = left_quats[i].first;
            } else if (use_right) {
                pelvis = Eigen::Vector3f(right_quats[i].second[0],right_quats[i].second[1],right_quats[i].second[2]);
                time = right_quats[i].first;
            }
            for (int j=0;j<3;j++) quat.push_back(pelvis[j]);
            for (int q=0;q<3;q++) {
                Eigen::Vector4f tmp_quat;
                if (use_both) {
                    tmp_quat = 0.5*(Eigen::Vector4f(left_quats[i].second[q*4+3],left_quats[i].second[q*4+4],left_quats[i].second[q*4+5],left_quats[i].second[q*4+6])+Eigen::Vector4f(right_quats[i].second[q*4+3],right_quats[i].second[q*4+4],right_quats[i].second[q*4+5],right_quats[i].second[q*4+6]));
                } else if (use_left) {
                    tmp_quat = Eigen::Vector4f(left_quats[i].second[q*4+3],left_quats[i].second[q*4+4],left_quats[i].second[q*4+5],left_quats[i].second[q*4+6]);
                } else if (use_right) {
                    tmp_quat = Eigen::Vector4f(right_quats[i].second[q*4+3],right_quats[i].second[q*4+4],right_quats[i].second[q*4+5],right_quats[i].second[q*4+6]);
                }
                for (int j=0;j<4;j++) quat.push_back(tmp_quat[j]);
            }
            if (use_left) {
                left_arm_quats = std::vector<float>(left_quats[i].second.begin()+23,left_quats[i].second.end());
            }
            if (use_right) {
                right_arm_quats = std::vector<float>(right_quats[i].second.begin()+15,right_quats[i].second.begin()+23);
            }
                
            for (int j=0;j<8;j++) quat.push_back(right_arm_quats[j]);
            for (int j=0;j<8;j++) quat.push_back(left_arm_quats[j]);
            std::vector<float> joint_vec;
            Eigen::Matrix3Xd joint_mat = Eigen::MatrixXd::Zero(3,15);
            std::vector<Eigen::Vector3f> link_centroids;
            std::vector<Eigen::Quaternionf> link_quats;
            forward_kinematics(quat,link_centroids,link_quats,joint_mat);
            // std::cout<<joint_mat<<std::endl;
            for (int j=0;j<joint_mat.cols();j++) {
                for (int k=0;k<3;k++) joint_vec.push_back(joint_mat.col(j)[k]);
            }
            // for (int j=0;j<quat.size();j++) std::cout<<quat[j]<<",";
            // std::cout<<std::endl;
            sequence.emplace_back(time,joint_vec,quat,joint_mat);
        }
        end_pose = std::get<2>(sequence.back());
    }
                
}
    
void human::forward_kinematics(std::vector<float> pose_elements, std::vector<Eigen::Vector3f>& link_centroids, std::vector<Eigen::Quaternionf>& link_quats, Eigen::Matrix3Xd& human_points) {
    // std::cout<<pose_elements.size()<<std::endl;
    Eigen::Vector3f pelvis_loc = Eigen::Vector3f(pose_elements[0],pose_elements[1],pose_elements[2]);
    // std::cout<<pelvis_loc.transpose()<<std::endl;
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
    link_centroids.reserve(6);
    link_quats.clear();
    link_quats.reserve(6);
    // std::cout<<"here:\n";
    Eigen::Quaternionf z_spine = quats[0]*z_axis_quat*quats[0].inverse();
    Eigen::Quaternionf x_spine = quats[0]*x_axis_quat*quats[0].inverse();
    // link_quats.push_back(z_spine);
    // std::cout<<"here:\n";
    // std::cout<<z_spine.vec()<<std::endl;
    // std::cout<<pelvis_loc.transpose()<<std::endl;
    // std::cout<<link_lengths_[0]<<std::endl;
    Eigen::Vector3f spine_top = pelvis_loc+link_lengths_[0]*z_spine.vec();
    // std::cout<<"pelvis:\n";
    // std::cout<<pelvis_loc.cast<double>()<<std::endl;
    human_points.col(0) = pelvis_loc.cast<double>();
    // std::cout<<human_points<<std::endl;
    human_points.col(1) = spine_top.cast<double>();
    link_centroids.push_back(pelvis_loc+0.5*link_lengths_[0]*z_spine.vec());
    // ROS_INFO_STREAM("spine top "<<spine_top.transpose());
    Eigen::Quaternionf z_neck = quats[1]*z_axis_quat*quats[1].inverse();
    Eigen::Quaternionf x_neck = quats[1]*x_axis_quat*quats[1].inverse();
    // link_quats.push_back(z_neck);
    Eigen::Vector3f head = spine_top+link_lengths_[1]*z_neck.vec();
    human_points.col(2) = head.cast<double>();
    link_centroids.push_back(spine_top+0.5*link_lengths_[1]*z_neck.vec());
    // ROS_INFO_STREAM("head top "<<head.transpose());
    Eigen::Quaternionf z_shoulders = quats[2]*z_axis_quat*quats[2].inverse();
    Eigen::Vector3f l_shoulder = spine_top-0.5*link_lengths_[2]*z_shoulders.vec();
    human_points.col(3) = l_shoulder.cast<double>();
    // ROS_INFO_STREAM("l_shoulder "<<l_shoulder.transpose());
    Eigen::Quaternionf z_e1 = quats[3]*z_axis_quat*quats[3].inverse();
    Eigen::Quaternionf x_e1 = quats[3]*x_axis_quat*quats[3].inverse();
    // link_quats.push_back(x_e1);
    Eigen::Vector3f e1 = l_shoulder+link_lengths_[3]*z_e1.vec();
    human_points.col(4) = e1.cast<double>();
    link_centroids.push_back(l_shoulder+0.5*link_lengths_[3]*z_e1.vec());
    Eigen::Quaternionf z_w1 = quats[4]*z_axis_quat*quats[4].inverse();
    Eigen::Quaternionf x_w1 = quats[4]*x_axis_quat*quats[4].inverse();
    // link_quats.push_back(x_w1);
    Eigen::Vector3f w1 = e1+(link_lengths_[4]+0.1)*z_w1.vec();
    human_points.col(5) = w1.cast<double>();
    link_centroids.push_back(e1+0.5*(link_lengths_[4])*z_w1.vec());
    Eigen::Vector3f r_shoulder = spine_top+0.5*link_lengths_[2]*z_shoulders.vec();
    human_points.col(6) = r_shoulder.cast<double>();
    // ROS_INFO_STREAM("r_shoulder "<<r_shoulder.transpose());
    Eigen::Quaternionf z_e2 = quats[5]*z_axis_quat*quats[5].inverse();
    Eigen::Quaternionf x_e2 = quats[5]*x_axis_quat*quats[5].inverse();
    // link_quats.push_back(x_e2);
    Eigen::Vector3f e2 = r_shoulder+link_lengths_[5]*z_e2.vec();
    human_points.col(7) = e2.cast<double>();
    link_centroids.push_back(r_shoulder+0.5*link_lengths_[5]*z_e2.vec());
    Eigen::Quaternionf z_w2 = quats[6]*z_axis_quat*quats[6].inverse();
    Eigen::Quaternionf x_w2 = quats[6]*x_axis_quat*quats[6].inverse();
    // link_quats.push_back(x_w2);
    Eigen::Vector3f w2 = e2+(link_lengths_[6]+0.1)*z_w2.vec();
    human_points.col(8) = w2.cast<double>();
    link_centroids.push_back(e2+0.5*(link_lengths_[6])*z_w2.vec());

    link_quats.push_back(quats[0]);
    link_quats.push_back(quats[1]);
    link_quats.push_back(quats[3]);
    link_quats.push_back(quats[4]);
    link_quats.push_back(quats[5]);
    link_quats.push_back(quats[6]);
    Eigen::Vector3f spine_mid = 0.5*(spine_top+pelvis_loc);
    human_points.col(9) = spine_mid.cast<double>();
    human_points.col(10) = 0.5*(spine_top+head).cast<double>();
    human_points.col(11) = 0.5*(l_shoulder+e1).cast<double>();
    human_points.col(12) = 0.5*(w1+e1).cast<double>();
    human_points.col(13) = 0.5*(r_shoulder+e2).cast<double>();
    human_points.col(14) = 0.5*(w2+e2).cast<double>();
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
    ros::Rate r(show_human_rate);
    float tm = 0.0;
    while(tm<start_delay) {
        tm+=prediction_dt;
        r.sleep();
    }
    for (int i=0;i<sequence.size();i++) {
        std::vector<Eigen::Vector3f> link_centroids;
        std::vector<Eigen::Quaternionf> link_quats;
        Eigen::Matrix3Xd joint_vec = Eigen::MatrixXd::Zero(3,15);
        forward_kinematics(std::get<2>(sequence[i]),link_centroids,link_quats,joint_vec);
        std::vector<int> ids = {0,1,3,4,5,6};
        visualization_msgs::MarkerArray mkr_arr;
        for (int b=0;b<6;b++) {
            int a = ids[b];
            visualization_msgs::Marker mkr;
            mkr.id=mkr_arr.markers.size()+1;
            mkr.lifetime = ros::Duration(5);
            mkr.type=mkr.CYLINDER;
            Eigen::Vector3f c = color_diff*((double)i/(double)sequence.size())+start_color;
            mkr.color.r=1.0;//c[0];
            mkr.color.b=0.0;//c[1];
            mkr.color.g=0.0;//c[2];
            mkr.color.a=1.0;
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

std::tuple<float,std::vector<float>,std::vector<float>,Eigen::MatrixXd> human::get_seq(int i) {
    if (sequence.empty()) return std::make_tuple(0.0,std::vector<float>(),std::vector<float>(),Eigen::MatrixXd());
    return sequence[i];
}
int human::get_seq_size(void) {
    return sequence.size();
}

robot_sequence::robot_sequence(ros::NodeHandle nh, robot_state::RobotStatePtr state, robot_model::RobotModelPtr model, std::string plan_group, std::shared_ptr<stap_test::humans> human_data,std::shared_ptr<data_recorder> rec,const planning_scene::PlanningScenePtr &planning_scene_,std::shared_ptr<ros::Publisher> pub_txt):nh(nh),state(state),model(model),plan_group(plan_group),move_group(plan_group),human_data(human_data),rec(rec),stap_warper(nh,move_group.getCurrentState(),model,planning_scene_,plan_group),pub_txt(pub_txt) {
     
    if (!nh.getParam("/sequence_test/test_num",test_num))
    {
      ROS_WARN("/sequence_test/test_num is not set");
    }  

    if (!nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_sequence/length", num_steps))
    {
      ROS_ERROR_STREAM("/test_sequence/"<<test_num<<"/robot_sequence/length is not defined in sequence.yaml");
    }
    for (int i=0; i<num_steps;i++) {
        data.emplace_back(nh,i,test_num);
    }    
    int num_robot_poses = 0;
    if (!nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_poses/length", num_robot_poses))
    {
      ROS_ERROR_STREAM("/test_sequence/"<<test_num<<"/robot_poses/length is not defined in sequence.yaml");
    }

    if (!nh.getParam("/sequence_test/output_positions",output_positions))
    {
      ROS_WARN("/sequence_test/output_positions is not set");
    }
    ROS_INFO_STREAM(output_positions ? "outputting positions":"not outputting positions");

    if (!nh.getParam("/sequence_test/tip_link",tip_link))
    {
      ROS_WARN("/sequence_test/tip_link is not set");
    }
    ROS_INFO_STREAM("tip_link:"<<tip_link);

    if (!nh.getParam("/sequence_test/tip_offset",tip_offset))
    {
      ROS_WARN("/sequence_test/tip_offset is not set");
    }
    ROS_INFO_STREAM("tip_offset:"<<tip_offset);

    std::vector<double> pose(6);
    for (int i=0; i<num_robot_poses;i++) {
        nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_poses/"+std::to_string(i), pose);
        goals_angles.push_back(pose);
        std::cout<<"goal "<<i<<":";
        for (int q=0;q<6;q++) std::cout<<goals_angles.back()[q]<<",";
        std::cout<<std::endl;
    }
    move_group.setPlanningPipelineId("irrt_avoid");
    move_group.setPlannerId("irrta");
    move_group.setMaxVelocityScalingFactor(1.0);            //time parametrization
    move_group.setMaxAccelerationScalingFactor(1.0);    //time parametrization
    double planning_time = 5.0;
    nh.getParam("/test_sequence/" + std::to_string(test_num) + "/planning_time", planning_time);
    move_group.setPlanningTime(planning_time);
    planner_sub = nh.subscribe<std_msgs::Float64MultiArray>("/solver_performance",1,&robot_sequence::perf_callback,this);
    last_perf_received = ros::Time::now();
    nom_plan_pub = nh.advertise<visualization_msgs::Marker>("/nominal_plan",1);
    grip_pos_pub = nh.advertise<std_msgs::Float32MultiArray>("/set_gripper_open_close", 1);
    grip_pub = nh.advertise<std_msgs::Bool>("/open_gripper", 1);        
    std_msgs::Float32MultiArray gripper_pos;
    gripper_pos.data.push_back(30.0);
    gripper_pos.data.push_back(0);
    grip_pos_pub.publish(gripper_pos);
    ros::Duration(0.1).sleep();
    std_msgs::Bool gripper_msg;
    gripper_msg.data = true;
    grip_pub.publish(gripper_msg);    
    std::string ctrl_ns = "manipulator";
    if (!nh.getParam("/ctrl_ns",ctrl_ns))
    {
      ROS_WARN("ctrl_ns is not defined");
    }
    sub_goal = nh.subscribe<control_msgs::FollowJointTrajectoryActionGoal>(ctrl_ns+"/follow_joint_trajectory/goal",1,&robot_sequence::goal_callback,this);
    pub_cancel_traj = nh.advertise<actionlib_msgs::GoalID>( ctrl_ns+"/follow_joint_trajectory/cancel", 0,false);
    centroids_pub = nh.advertise<geometry_msgs::PoseArray>( "/centroids", 0,false);
    pc_pub = nh.advertise<sensor_msgs::PointCloud>( "/occupancy", 0,false);
    Eigen::Vector3d workspace_lb={-1,-1,0.5};
    Eigen::Vector3d workspace_ub={1,1,2.5};
    
    std::vector<double> ws_lb_param;

    if (nh.getParam("workspace_lower_bounds_xyz",ws_lb_param))
    {
        for (int i=0;i<3;i++) workspace_lb[i] = ws_lb_param[i];
    } else {
        ROS_DEBUG("workspace_lower_bounds_xyz is not set, default={-1,-1,0.5}");
    }  

    std::vector<double> ws_ub_param;

    if (nh.getParam("workspace_upper_bounds_xyz",ws_ub_param))
    {
        for (int i=0;i<3;i++) workspace_ub[i] = ws_ub_param[i];
    } else {
        ROS_DEBUG("workspace_lower_bounds_xyz is not set, default={1,1,2.5}");
    }
    unsigned int npnt=50;
    grid_ = std::make_shared<human_occupancy::OccupancyGrid>(workspace_lb,workspace_ub,npnt);
    link_names = move_group.getLinkNames();
}

void robot_sequence::perf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.empty()) return;
    plan_time = std::min(msg->data.back(),20.0);
    ROS_INFO_STREAM("perf plan time:"<<plan_time);
    last_perf_received = ros::Time::now();
}

double robot_sequence::plan_robot_segment(int seg_num, std::vector<double>& start_joint) {
    if ((seg_num<0)||(seg_num>data.size())) return 0.0;
    visualization_msgs::Marker mkr;
    mkr.id=105;
    mkr.lifetime = ros::Duration(5.0);
    mkr.type=mkr.TEXT_VIEW_FACING;
    mkr.color.r=1.0;
    mkr.color.b=1.0;
    mkr.color.g=1.0;
    mkr.color.a=1.0;
    mkr.pose.position.x = -1;
    mkr.pose.position.y = 0;
    mkr.pose.position.z = 1;
    mkr.pose.orientation.x = 0;
    mkr.pose.orientation.y = 0;
    mkr.pose.orientation.z = 0;
    mkr.pose.orientation.w = 1;
    mkr.scale.z = 0.3;
    mkr.header.frame_id = "world";
    mkr.text = "robot plan:" + std::to_string(seg_num) + ":\n" + data[seg_num].description;
    pub_txt->publish(gen_overlay_text("robot plan:" + std::to_string(seg_num) + "-" + data[seg_num].description));
    ROS_INFO_STREAM("planning for segment:"<<seg_num<<" with goal id:"<<data[seg_num].get_goal_id());

    if (data[seg_num].get_type()==0) {
        if(fs::create_directory(ros::package::getPath("stap_warp")+"/plans/" + plan_group))
            std::cout << "Created a directory\n";
        if(fs::create_directory(ros::package::getPath("stap_warp")+"/plans/" + plan_group + "/test_" + std::to_string(test_num)))
            std::cout << "Created a directory\n";
        if(fs::create_directory(ros::package::getPath("stap_warp")+"/plans/" + plan_group + "/test_" + std::to_string(test_num)+"/"+data[seg_num].pipeline + "_" + data[seg_num].planner))
            std::cout << "Created a directory\n";
        std::string bag_file= ros::package::getPath("stap_warp")+"/plans/" + plan_group + "/test_" + std::to_string(test_num) + "/"+data[seg_num].pipeline + "_" + data[seg_num].planner + "/sequence_" + std::to_string(seg_num) + "_plan.bag";
        if (access( bag_file.c_str(), F_OK ) != -1) {
            rosbag::Bag bag; 
            bag.open(bag_file, rosbag::bagmode::Read);
            std::vector<std::string> topics;
            topics.push_back("sequence_" + std::to_string(seg_num) + "_planning_time");
            topics.push_back("sequence_" + std::to_string(seg_num) + "_start_state");
            topics.push_back("sequence_" + std::to_string(seg_num) + "_trajectorie");
            std::vector<double> planning_times;
            std::vector<moveit_msgs::RobotState> start_states;
            std::vector<moveit_msgs::RobotTrajectory> trajectories;    
            for(rosbag::MessageInstance const m: rosbag::View(bag)) {
                std_msgs::Float64::ConstPtr plan_time_msg = m.instantiate<std_msgs::Float64>();
                if (plan_time_msg!=nullptr) planning_times.push_back((double)plan_time_msg->data);
                moveit_msgs::RobotState::ConstPtr start_state_msg = m.instantiate<moveit_msgs::RobotState>();
                if (start_state_msg!=nullptr) start_states.push_back(*start_state_msg);
                moveit_msgs::RobotTrajectory::ConstPtr trajectory_msg = m.instantiate<moveit_msgs::RobotTrajectory>();
                if (trajectory_msg!=nullptr) trajectories.push_back(*trajectory_msg);
            }
            data[seg_num].plan.start_state_ = start_states[0];
            data[seg_num].plan.trajectory_ = trajectories[0];
            data[seg_num].plan.planning_time_ = planning_times[0];
            bag.close();
            if (output_positions) {
                state->setVariablePositions(data[seg_num].plan.trajectory_.joint_trajectory.points.back().positions);
                const Eigen::Affine3d& tip_tf = state->getGlobalLinkTransform(tip_link);
                Eigen::Vector3d tip_pos = tip_tf.translation()+tip_tf.rotation()*Eigen::Vector3d(0,0,tip_offset);
                Eigen::Quaterniond tip_q(tip_tf.rotation());
                ROS_INFO_STREAM("tip_pos:"<<tip_pos[0]<<","<<tip_pos[1]<<","<<tip_pos[2]<<","<<tip_q.w()<<","<<tip_q.x()<<","<<tip_q.y()<<","<<tip_q.z());
            }
            return planning_times[0];
        } else {
            move_group.setPlanningPipelineId(data[seg_num].pipeline);
            move_group.setPlannerId(data[seg_num].planner);
            for (int i = 0;i<6;i++) std::cout<<start_joint[i]<<",";
            std::cout<<"->";
            for (int i = 0;i<6;i++) std::cout<<goals_angles[data[seg_num].get_goal_id()][i]<<",";
            std::cout<<std::endl;
            state = move_group.getCurrentState();
            state->setVariablePositions(start_joint);
            move_group.setStartState(*state);
            move_group.setJointValueTarget(goals_angles[data[seg_num].get_goal_id()]);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            ros::Time plan_req_time = ros::Time::now();
            bool plan_success = (move_group.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
            std::cout<<(last_perf_received-plan_req_time).toSec()<<std::endl;
            if (data[seg_num].pipeline=="irrt_avoid") {
                while ((last_perf_received-plan_req_time).toSec()<0.0) {
                    ros::Duration(0.1).sleep();
                    ROS_INFO("waiting for planner time estimate");
                }
            } else {
                plan_time = plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
            }
            if (plan_success) {
                data[seg_num].plan = plan;
                data[seg_num].planned_time = plan_time;
                std_msgs::Float64 plan_time_msg;
                plan_time_msg.data = plan_time;
                rosbag::Bag bag; 
                bag.open(bag_file, rosbag::bagmode::Write);
                bag.write("sequence_" + std::to_string(seg_num) + "_planning_time",ros::Time::now(),plan_time_msg);
                bag.write("sequence_" + std::to_string(seg_num) + "_start_state",ros::Time::now(),plan.start_state_);
                bag.write("sequence_" + std::to_string(seg_num) + "_trajectorie",ros::Time::now(),plan.trajectory_);
                bag.close();
                start_joint = plan.trajectory_.joint_trajectory.points.back().positions;
                if (output_positions) {
                    state->setVariablePositions(data[seg_num].plan.trajectory_.joint_trajectory.points.back().positions);
                    const Eigen::Affine3d& tip_tf = state->getGlobalLinkTransform(tip_link);
                    Eigen::Vector3d tip_pos = tip_tf.translation()+tip_tf.rotation()*Eigen::Vector3d(0,0,tip_offset);
                    Eigen::Quaterniond tip_q(tip_tf.rotation());
                    ROS_INFO_STREAM("tip_pos:"<<tip_pos[0]<<","<<tip_pos[1]<<","<<tip_pos[2]<<","<<tip_q.w()<<","<<tip_q.x()<<","<<tip_q.y()<<","<<tip_q.z());
                }
                return plan_time;
            } else {
                ROS_ERROR("planning failed");
                return 0.0;
            }
        }
    } else {
        data[seg_num].planned_time =1.0;
    }
    return data[seg_num].planned_time;
}

void robot_sequence::goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg) {
  std::lock_guard<std::mutex> lck(goal_mtx);
  goal_id = msg->goal_id.id;
//   ROS_INFO_STREAM("read"<<goal_id);
}

void robot_sequence::segment_thread_fn(int seg_num) {
    segment_active = true;
    rec->plan_time = data[seg_num].planned_time;
    if ((seg_num<0)||(seg_num>data.size())) return;
    if (data[seg_num].get_type()==0) {
        if (data[seg_num].pipeline=="irrt_avoid") {
            move_group.asyncExecute(data[seg_num].plan);
            Eigen::VectorXd goal_vec(6);
            for (int i=0;i<6;i++) goal_vec[i] = data[seg_num].plan.trajectory_.joint_trajectory.points.back().positions[i];
            ros::Duration(0.1).sleep();
            ros::Time p_start = ros::Time::now();
            while ((rec->joint_pos_vec-goal_vec).norm()>0.001) {
                human_data->joint_seq_mtx.lock();
                std::vector<std::pair<float,Eigen::MatrixXd>> human_seq = human_data->full_joint_seq;
                human_data->joint_seq_mtx.unlock();
                stap_warper.warp(human_seq,std::max((ros::Time::now()-p_start).toSec(),0.0),rec->joint_pos_vec,rec->get_current_joint_state());
                ros::Duration(0.01).sleep();
            }
        } else if (data[seg_num].pipeline=="dirrt") {
            move_group.setPlanningPipelineId(data[seg_num].pipeline);
            move_group.setPlannerId(data[seg_num].planner);
            move_group.setPlanningTime(1.0);
            move_group.setStartStateToCurrentState();
            move_group.setJointValueTarget(goals_angles[data[seg_num].get_goal_id()]);

            centroids_pub.publish(rec->pose_msg);
            std::vector<Eigen::Vector3f> pts = rec->live_human_points;
            Eigen::Vector3f p;
            if (pts.size()>1) {
                p = 0.5*(pts[0]+pts[1]);
                pts.push_back(p);
                p = 0.5*(pts[2]+pts[1]);
                pts.push_back(p);
                p = 0.5*(pts[3]+pts[4]);
                pts.push_back(p);
                p = 0.5*(pts[4]+pts[5]);
                pts.push_back(p);
                p = 0.5*(pts[6]+pts[7]);
                pts.push_back(p);
                p = 0.5*(pts[7]+pts[8]);
                pts.push_back(p);
            }
            set_occupancy(pts);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool plan_success = false;
            while (!plan_success) {
                plan_success = (move_group.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                if (!plan_success) ros::Duration(0.1).sleep();
            }
            data[seg_num].plan = plan;
            Eigen::VectorXd goal_vec(6);
            for (int i=0;i<6;i++) goal_vec[i] = data[seg_num].plan.trajectory_.joint_trajectory.points.back().positions[i];
            move_group.asyncExecute(data[seg_num].plan);

            bool dist_ready = false;
            int prev_spd_scale = 0.0;
            ros::Time prev_plan_time=ros::Time::now()-ros::Duration(1.0);
            bool replan_needed = false;
            ros::Time last_motion_time = ros::Time::now();
            
            double prev_min_dist = rec->get_min_dist();

            while ((rec->joint_pos_vec-goal_vec).norm()>0.001) {
                if (rec->joint_vel_vec.cwiseAbs().sum()>0.01) last_motion_time = ros::Time::now();
                replan_needed = ((ros::Time::now()-last_motion_time).toSec()>1.0);
                //if stopped replan
            
                if (((((ros::Time::now()-prev_plan_time).toSec()>1.0) && (rec->spd_scale<50)) && (rec->spd_scale-prev_spd_scale<=0))||replan_needed) {
                    move_group.stop();
                    int t=0;
                    while (rec->joint_vel_vec.cwiseAbs().sum()>0.001) {
                        goal_id_msg.stamp = ros::Time::now();
                        goal_mtx.lock();
                        goal_id_msg.id = goal_id;
                        ROS_INFO_STREAM("send"<<goal_id);
                        goal_mtx.unlock();
                        pub_cancel_traj.publish(goal_id_msg);
                        t++;
                    }
                    ROS_INFO_STREAM("stopped");
                    centroids_pub.publish(rec->pose_msg);
                    // show_human.stop_show_human();
                    ros::Duration(0.1).sleep();
                    // std::cin.ignore();

                    plan_success = false;
                    t = 0;
                    while ((!plan_success) && (t<20)) {
                        // for (int j=0; j<rec.joint_positions.size();j++) std::cout<<rec.joint_positions[j]<<",";
                        std::cout<<std::endl;
                        move_group.setPlanningTime(0.5);
                        state->setVariablePositions(rec->joint_positions);
                        move_group.setStartState(*state);
                        move_group.setJointValueTarget(plan.trajectory_.joint_trajectory.points.back().positions);
                        plan_success = (move_group.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                        t++;
                    }
                    // co_human.resume_obs();
                    // show_human.resume_show_human();
                    ros::Duration(0.1).sleep();
                    bool move_success = (move_group.asyncExecute(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                    ros::Duration(0.1).sleep();
                    prev_plan_time=ros::Time::now();
                    last_motion_time = ros::Time::now();
                }
                prev_spd_scale = rec->spd_scale;
                ros::Duration(0.01).sleep();
            }
        } else {
            move_group.asyncExecute(data[seg_num].plan);
            Eigen::VectorXd goal_vec(6);
            for (int i=0;i<6;i++) goal_vec[i] = data[seg_num].plan.trajectory_.joint_trajectory.points.back().positions[i];
            ros::Duration(0.1).sleep();
            while ((rec->joint_pos_vec-goal_vec).norm()>0.001) {
                ros::Duration(0.01).sleep();
            }
        }
    }
    else if (data[seg_num].get_type()==1) {
        //close gripper
        std_msgs::Bool gripper_msg;
        gripper_msg.data = false;
        grip_pub.publish(gripper_msg);
        ros::Duration(0.5).sleep();
    }
    else if (data[seg_num].get_type()==2) {
        //open gripper  
        std_msgs::Float32MultiArray gripper_pos;
        gripper_pos.data.push_back(data[seg_num].get_gripper_pct());
        gripper_pos.data.push_back(0);
        grip_pos_pub.publish(gripper_pos);
        ros::Duration(0.1).sleep();
        std_msgs::Bool gripper_msg;
        gripper_msg.data = true;
        grip_pub.publish(gripper_msg);
        ros::Duration(0.5).sleep();
    }
    segment_active = false;
}

bool robot_sequence::do_segment(int seg_num) {
    if (segment_active) return false;
    if (segment_thread.joinable()) segment_thread.join();
    visualization_msgs::Marker mkr;
    mkr.id=105;
    mkr.lifetime = ros::Duration(5.0);
    mkr.type=mkr.TEXT_VIEW_FACING;
    mkr.color.r=1.0;
    mkr.color.b=1.0;
    mkr.color.g=1.0;
    mkr.color.a=1.0;
    mkr.pose.position.x = -1;
    mkr.pose.position.y = 0;
    mkr.pose.position.z = 1;
    mkr.pose.orientation.x = 0;
    mkr.pose.orientation.y = 0;
    mkr.pose.orientation.z = 0;
    mkr.pose.orientation.w = 1;
    mkr.scale.z = 0.2;
    mkr.header.frame_id = "world";
    mkr.text = "robot exec:"+std::to_string(seg_num)+"-"+ data[seg_num].description;
    pub_txt->publish(gen_overlay_text(mkr.text));
    pub_plan(nom_plan_pub,data[seg_num].plan,state,plan_group,link_names.back());
    segment_thread = std::thread(&robot_sequence::segment_thread_fn,this,seg_num);
    return segment_active;
}

int robot_sequence::get_prior_human_step(int seg_num) {
    if ((seg_num<0)||(seg_num>data.size())) return 1000;
    return data[seg_num].get_prior_human_task();
}

void robot_sequence::set_gripper(bool open) {
    std_msgs::Bool gripper_msg;
    gripper_msg.data = open;
    grip_pub.publish(gripper_msg);
}

robot_segment::robot_segment(ros::NodeHandle nh, int idx, int test_num) {
    id = idx;
    std::cout<<"robot seg "<<idx<<":";
    nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_sequence/"+std::to_string(idx)+"/description", description);
    std::cout<<description<<",";
    nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_sequence/"+std::to_string(idx)+"/goal", goal_id);
    std::cout<<" goal id:"<<goal_id<<",";
    nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_sequence/"+std::to_string(idx)+"/type", type);
    std::cout<<" type:"<<type<<",";
    if (type>0) {
        nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_sequence/"+std::to_string(idx)+"/pct", gripper_pct);
    }
    nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_sequence/"+std::to_string(idx)+"/after_human_task", prior_human_task);
    std::cout<<" prior_human_task:"<<prior_human_task<<std::endl;
    bool use_nonrestrictive = false;
    bool tmp = false;
    tmp = nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_sequence/"+std::to_string(idx)+"/use_nonrestrictive_planner",use_nonrestrictive);
    tmp = tmp && use_nonrestrictive;
    if (tmp) {
        nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_sequence/nonrestrictive_pipeline", pipeline);
        nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_sequence/nonrestrictive_planner", planner);
    } else {
        nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_sequence/"+std::to_string(idx)+"/pipeline", pipeline);
        nh.getParam("/test_sequence/" + std::to_string(test_num) + "/robot_sequence/"+std::to_string(idx)+"/planner", planner);
    }
}

jsk_rviz_plugins::OverlayText gen_overlay_text(std::string txt) {
    jsk_rviz_plugins::OverlayText msg;
    msg.action = msg.ADD;
    msg.width = 800;
    msg.height = 100;
    msg.left = 10;
    msg.top = 800;
    msg.text_size = 24;
    msg.line_width = 6;
    msg.font = "DejaVu Sans Mono";
    msg.text = txt;
    msg.fg_color.r = 25.0/255.0;
    msg.fg_color.g = 1.0;
    msg.fg_color.b = 240.0/255.0;
    msg.fg_color.a = 1.0;
    msg.bg_color.r = 0.0;
    msg.bg_color.g = 0.0;
    msg.bg_color.b = 0.0;
    msg.bg_color.a = 0.2;
    return msg;
}
void robot_sequence::set_occupancy(std::vector<Eigen::Vector3f> avoid_pts) {
    visualization_msgs::Marker obs_pts;
    geometry_msgs::PoseArray pc_pose_array;
    pc_pose_array.header.frame_id="world";
    avoidance_intervals::display_markers(avoid_pts,obs_pts);
    geometry_msgs::Pose pose;
    for (geometry_msgs::Point p:obs_pts.points) {
        pose.position = p;
        pc_pose_array.poses.push_back(pose);
    }
    grid_->update(pc_pose_array);
    sensor_msgs::PointCloud pc;
    pc = grid_->toPointCloud();
    pc_pub.publish(pc);

    // pub_timer = nh_.createTimer(ros::Duration(1.0/50.0), &human_occupancy_helper::set_occupancy_timer, this);
}


std::vector<float> transform_point_to_UF(std::vector<float> p, Eigen::Isometry3f transform_to_world_inv) {
    std::vector<float>out_p;
    Eigen::Vector3f v = transform_to_world_inv*Eigen::Vector3f(p[0],p[1],p[2]);
    for (int i=0;i<3;i++) out_p.push_back(v[i]);
    return out_p;
}

std::vector<float> transform_point_to_SW(std::vector<float> p, Eigen::Isometry3f transform_to_world) {
    std::vector<float>out_p;
    Eigen::Vector3f v = transform_to_world*Eigen::Vector3f(p[0],p[1],p[2]);
    for (int i=0;i<3;i++) out_p.push_back(v[i]);
    return out_p;
}

std::vector<float> transform_pose_to_SW(std::vector<float> input_pose, Eigen::Isometry3f transform_to_world) {
    std::vector<float> output_pose;
    Eigen::Vector3f pelvis_loc = transform_to_world*Eigen::Vector3f(input_pose[0],input_pose[1],input_pose[2]);
    for (int i=0;i<3;i++) output_pose.push_back(pelvis_loc[i]);
    Eigen::Quaternionf q;
    Eigen::Quaternionf quat_transform(transform_to_world.rotation());
    for (int i=0;i<7;i++){
        q = quat_transform*Eigen::Quaternionf(input_pose[i*4+3],input_pose[i*4+4],input_pose[i*4+5],input_pose[i*4+6]);
        output_pose.push_back(q.w());
        output_pose.push_back(q.x());
        output_pose.push_back(q.y());
        output_pose.push_back(q.z());
    }
    return output_pose;
}

std::vector<float> transform_pose_to_UF(std::vector<float> input_pose, Eigen::Isometry3f transform_to_world_inv) {
    std::vector<float> output_pose;
    Eigen::Vector3f pelvis_loc = transform_to_world_inv*Eigen::Vector3f(input_pose[0],input_pose[1],input_pose[2]);
    for (int i=0;i<3;i++) output_pose.push_back(pelvis_loc[i]);
    Eigen::Quaternionf q;
    Eigen::Quaternionf quat_transform(transform_to_world_inv.rotation());
    for (int i=0;i<7;i++){
        q = quat_transform*Eigen::Quaternionf(input_pose[i*4+3],input_pose[i*4+4],input_pose[i*4+5],input_pose[i*4+6]);
        output_pose.push_back(q.w());
        output_pose.push_back(q.x());
        output_pose.push_back(q.y());
        output_pose.push_back(q.z());
    }
    return output_pose;
}
}