#include <stap_warp/sequence.h>

namespace stap_test {

humans::humans(ros::NodeHandle nh, std::vector<float> cur_pose, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub,std::shared_ptr<avoidance_intervals::skeleton> skel):nh(nh),predictor(predictor),skel(skel),seq_pub(seq_pub) {
    nh.getParam("/test_sequence/human_sequence/start_pose", start_pose);
    if (start_pose.size()==0) {
        start_pose = cur_pose;
    }
    if (!nh.getParam("/test_sequence/human_sequence/length", num_steps))
    {
      ROS_ERROR("/test_sequence/human_sequence/length is not defined in sequence.yaml");
    }
    data.clear();
    data.reserve(num_steps);
    for (int i=0; i<num_steps;i++) {
        data.emplace_back(nh,i,predictor,seq_pub,prediction_dt);
    }
    human_model_pub = nh.advertise<stap_warp::joint_seq>("human_model_seq", 0,false);
    human_done_srv = nh.serviceClient<stap_warp::human_motion_done>("human_motion_done");
    human_reset_srv = nh.serviceClient<stap_warp::human_motion_reset>("human_human_motion_resetmotion_done");
}

float humans::pub_model(int start_seq, int robot_step, float start_tm_in_robot_seq) {
    ROS_INFO_STREAM("pub mode start:"<<start_seq<<","<<data.size());
    if ((start_seq<0)||(start_seq>=data.size())) return 0.0;
    generate_full_sequence(start_seq,robot_step,start_tm_in_robot_seq);
    stap_warp::joint_seq seq_msg;
    float elapsed_tm = std::round(10.0*start_tm_in_robot_seq)*0.1;
    int s = 0;
    float dt = prediction_dt;
    for (s=start_seq;s<data.size();s++) {
        ROS_INFO_STREAM("prior:"<<data[s].get_prior_robot_task()<<",robot_step"<<robot_step);
        if (data[s].get_prior_robot_task()>=robot_step) break;
        if (data[s].get_start_delay()>0.0)  {
            float start_tm = elapsed_tm;
            while (elapsed_tm - start_tm<data[s].get_start_delay()) {
                if (elapsed_tm>0) {
                    stap_warp::joint_seq_elem seq_elem;
                    seq_elem.time = elapsed_tm;
                    ROS_INFO_STREAM("seq time1:"<<seq_elem.time);
                    seq_elem.joint_pos = std::get<1>(data[s].get_seq(0));
                    seq_elem.quats = std::get<2>(data[s].get_seq(0));
                    seq_msg.sequence.push_back(seq_elem);
                }
                elapsed_tm += dt;
            }
        }
        ROS_INFO_STREAM("adding prediction for human step "<<s);
        for (int i=0;i<data[s].get_seq_size();i++) {
            float seq_time = std::get<0>(data[s].get_seq(i))+elapsed_tm;
            if (seq_time>=0.0) {
                stap_warp::joint_seq_elem seq_elem;
                seq_elem.time = seq_time;
                ROS_INFO_STREAM("seq time2:"<<seq_elem.time);
                seq_elem.joint_pos = std::get<1>(data[s].get_seq(i));
                seq_elem.quats = std::get<2>(data[s].get_seq(i));
                seq_msg.sequence.push_back(seq_elem);
            }
        }
        elapsed_tm += std::get<0>(data[s].get_seq(data[s].get_seq_size()-1))+dt;
    }
    human_model_pub.publish(seq_msg);
    std::lock_guard<std::mutex> l(joint_seq_mtx);
    skel->joint_seq = full_joint_seq_f;
    skel->quat_seq = full_quat_seq;
    return elapsed_tm-start_tm_in_robot_seq;
}

void humans::generate_full_sequence(int start_seq, int robot_step, float start_tm_in_robot_seq) {
    if ((start_seq<0)||(start_seq>=data.size())) return;
    float elapsed_tm = std::round(10.0*start_tm_in_robot_seq)*0.1;
    int s = 0;
    float dt = 0.1;
    std::lock_guard<std::mutex> l(joint_seq_mtx);
    full_joint_seq.clear();
    full_joint_seq_f.clear();
    full_quat_seq.clear();
    for (s=start_seq;s<data.size();s++) {
        if (data[s].get_prior_robot_task()>=robot_step) break;
        if (data[s].get_start_delay()>0.0)  {
            float start_tm = elapsed_tm;
            while (elapsed_tm - start_tm<data[s].get_start_delay()) {
                if (elapsed_tm>0) {
                    full_joint_seq.emplace_back(elapsed_tm,std::get<3>(data[s].get_seq(0)));
                    Eigen::MatrixXd tmp_jnts_mat = std::get<3>(data[s].get_seq(0));
                    std::vector<Eigen::Vector3f> tmp_jnts;
                    for (int c=0;c<tmp_jnts_mat.cols();c++) tmp_jnts.emplace_back(tmp_jnts_mat.col(c).cast<float>());
                    full_joint_seq_f.emplace_back(elapsed_tm,tmp_jnts);
                    std::vector<Eigen::Quaternionf> tmp_quats;
                    std::vector<float> tmp_quat_vec = std::get<2>(data[s].get_seq(0));
                    for (int q=0;q<7;q++) tmp_quats.emplace_back(tmp_quat_vec[q*4+3],tmp_quat_vec[q*4+4],tmp_quat_vec[q*4+5],tmp_quat_vec[q*4+6]);
                    full_quat_seq.emplace_back(elapsed_tm,tmp_quats);
                }
                elapsed_tm += dt;
            }
        }
        for (int i=0;i<data[s].get_seq_size();i++) {
            float seq_time = std::get<0>(data[s].get_seq(i))+elapsed_tm;
            if (seq_time>=0.0) {
                full_joint_seq.emplace_back(elapsed_tm,std::get<3>(data[s].get_seq(i)));
                Eigen::MatrixXd tmp_jnts_mat = std::get<3>(data[s].get_seq(i));
                std::vector<Eigen::Vector3f> tmp_jnts;
                for (int c=0;c<tmp_jnts_mat.cols();c++) tmp_jnts.emplace_back(tmp_jnts_mat.col(c).cast<float>());
                full_joint_seq_f.emplace_back(elapsed_tm,tmp_jnts);
                std::vector<Eigen::Quaternionf> tmp_quats;
                std::vector<float> tmp_quat_vec = std::get<2>(data[s].get_seq(i));
                for (int q=0;q<7;q++) tmp_quats.emplace_back(tmp_quat_vec[q*4+3],tmp_quat_vec[q*4+4],tmp_quat_vec[q*4+5],tmp_quat_vec[q*4+6]);
                full_quat_seq.emplace_back(elapsed_tm,tmp_quats);
            }
        }
        elapsed_tm += std::get<0>(data[s].get_seq(data[s].get_seq_size()))+dt;
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

bool humans::is_step_done(int step) {
    if ((step<0)||(step>data.size())) return false;
    stap_warp::human_motion_done srv;
    srv.request.reach_target = data[step].get_tgt();
    srv.request.active_hand = !data[step].arm_right;
    if (human_done_srv.call(srv)) {
        return srv.response.done;
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
        data[i].get_predicted_motion(pose);
        pose = data[i].get_last_pose();
    }
    generate_full_sequence(cur_step, robot_step, current_robot_time);
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

human::human(ros::NodeHandle nh, int idx, std::shared_ptr<ros::ServiceClient> predictor,std::shared_ptr<ros::Publisher> seq_pub, float prediction_dt):predictor(predictor),seq_pub(seq_pub),prediction_dt(prediction_dt) {
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
    srv.request.dt = prediction_dt;
    sequence.clear();
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
            Eigen::Matrix3Xd joint_mat = Eigen::MatrixXd::Zero(3,15);
            std::vector<Eigen::Vector3f> link_centroids;
            std::vector<Eigen::Quaternionf> link_quats;
            forward_kinematics(quat_vec,link_centroids,link_quats,joint_mat);
            std::cout<<joint_mat<<std::endl;
            for (int j=0;j<joint_mat.cols();j++) {
                for (int k=0;k<3;k++) joint_vec.push_back(joint_mat.col(j)[k]);
            }
            sequence.emplace_back(srv.response.pose_sequence.data[i],joint_vec,quat_vec,joint_mat);
            i+=n_cols;
            if (i>srv.response.pose_sequence.data.size()-n_cols) break;
        }
        ROS_INFO_STREAM("human "<<id<<" received sequence with "<<sequence.size()<<" steps");
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
    ros::Rate r(10);
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
            mkr.color.r=c[0];
            mkr.color.b=c[1];
            mkr.color.g=c[2];
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
    return sequence[i];
}
int human::get_seq_size(void) {
    return sequence.size();
}

robot_sequence::robot_sequence(ros::NodeHandle nh, robot_state::RobotStatePtr state, robot_model::RobotModelPtr model, std::string plan_group, std::shared_ptr<stap_test::humans> human_data,std::shared_ptr<data_recorder> rec,const planning_scene::PlanningScenePtr &planning_scene_):nh(nh),state(state),model(model),move_group(plan_group),human_data(human_data),rec(rec),stap_warper(nh,move_group.getCurrentState(),model,planning_scene_) {
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
    move_group.setPlanningPipelineId("irrt_avoid");
    move_group.setPlannerId("irrta");
    double planning_time = 5.0;
    nh.getParam("/sequence_test/planning_time", planning_time);
    move_group.setPlanningTime(planning_time);
    planner_sub = nh.subscribe<std_msgs::Float64MultiArray>("/solver_performance",1,&robot_sequence::perf_callback,this);
    last_perf_received = ros::Time::now();
    nom_plan_pub = nh.advertise<visualization_msgs::Marker>("/marker_visualization_topic",1);
    grip_pos_pub = nh.advertise<std_msgs::Float32MultiArray>("/set_gripper_open_close", 1);
    grip_pub = nh.advertise<std_msgs::Bool>("/open_gripper", 1);        
    std_msgs::Float32MultiArray gripper_pos;
    gripper_pos.data.push_back(30.0);
    gripper_pos.data.push_back(0);
    grip_pos_pub.publish(gripper_pos);
}

void robot_sequence::perf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.empty()) return;
    plan_time = msg->data.back();
    ROS_INFO_STREAM("perf plan time:"<<plan_time);
    last_perf_received = ros::Time::now();
}

double robot_sequence::plan_robot_segment(int seg_num, std::vector<double>& start_joint) {
    if ((seg_num<0)||(seg_num>data.size())) return 0.0;
    ROS_INFO_STREAM("planning for segment:"<<seg_num);
    if (data[seg_num].get_type()==0) {
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
        while ((last_perf_received-plan_req_time).toSec()<0.0) {
            ros::Duration(0.1).sleep();
            ROS_INFO("waiting for planner time estimate");
        }
        if (plan_success) {
            data[seg_num].plan = plan;
            data[seg_num].planned_time = plan_time;
            start_joint = plan.trajectory_.joint_trajectory.points.back().positions;
            return plan_time;
        } else {
            ROS_ERROR("planning failed");
            return 0.0;
        }
    } else {
        data[seg_num].planned_time =1.0;
    }
    return data[seg_num].planned_time;
}

void robot_sequence::segment_thread_fn(int seg_num) {
    segment_active = true;
    if ((seg_num<0)||(seg_num>data.size())) return;
    if (data[seg_num].get_type()==0) {
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
            ros::Duration(0.1).sleep();
        }
    }
    else if (data[seg_num].get_type()==1) {
        std_msgs::Bool gripper_msg;
        gripper_msg.data = true;
        grip_pub.publish(gripper_msg);
        ros::Duration(0.5).sleep();
    }
    else if (data[seg_num].get_type()==2) {
        std_msgs::Bool gripper_msg;
        gripper_msg.data = false;
        grip_pub.publish(gripper_msg);
        ros::Duration(0.5).sleep();
    }
    segment_active = false;
}

bool robot_sequence::do_segment(int seg_num) {
    if (segment_active) return false;
    if (segment_thread.joinable()) segment_thread.join();
    pub_plan(nom_plan_pub,data[seg_num].plan,state);
    segment_thread = std::thread(&robot_sequence::segment_thread_fn,this,seg_num);
    return segment_active;
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