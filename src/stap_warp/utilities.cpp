#include <stap_warp/utilities.h>

void clearObstacles(void) {
    moveit::planning_interface::PlanningSceneInterface current_scene;
    std::vector<std::string> obstacles;
    obstacles = current_scene.getKnownObjectNames();
    // std::cout<<"num obstacles = "<<obstacles.size()<<std::endl;
    for (int i=0;i<obstacles.size();i++) {
        if (obstacles[i]=="table") {
            obstacles.erase(obstacles.begin()+i);
        }
    }
    while (obstacles.size()>0) {
        current_scene.removeCollisionObjects(obstacles);
        obstacles = current_scene.getKnownObjectNames();
        for (int i=0;i<obstacles.size();i++) {
            // std::cout<<obstacles[i]<<std::endl;
            if (obstacles[i]=="table") {
                obstacles.erase(obstacles.begin()+i);
            }
        }
        // std::cout<<"num obstacles = "<<obstacles.size()<<std::endl;
    }
}
human_publisher::human_publisher(ros::NodeHandle nh, bool sim):nh_(nh) {
    show_human_timer = nh.createTimer(ros::Duration(0.033), &human_publisher::show_human_thread, this);
    show_human_timer.stop();
    if (sim) {
        pub_poses_timer = nh.createTimer(ros::Duration(0.033), &human_publisher::pub_poses_thread, this);
        pub_poses_timer.stop();
    }
}

void human_publisher::start_show_human(double start_time2) {
  start_time = ros::Time::now();
  show_human_elapsed_time = start_time2;
  show_human_timer.start();
}
void human_publisher::start_poses(double start_time2) {
  start_time = ros::Time::now();
  pub_poses_elapsed_time = start_time2;
  pub_poses_timer.start();
}
void human_publisher::stop_show_human(void) {
  show_human_timer.stop();
}
void human_publisher::stop_poses(void) {
  pub_poses_timer.stop();
}
void human_publisher::resume_show_human(void) {
  start_time = ros::Time::now();
  show_human_timer.start();
}
void human_publisher::resume_poses(void) {
  pose_start_time = ros::Time::now();
  pub_poses_timer.start();
}

void human_publisher::show_human_thread(const ros::TimerEvent& event) {
  std_msgs::Float32 time_disp;
  show_human_elapsed_time += (event.current_real-start_time).toSec();
  time_disp.data = show_human_elapsed_time;
  model_pub.publish(time_disp);
  start_time = ros::Time::now();
}
void human_publisher::pub_poses_thread(const ros::TimerEvent& event) {
  std::cout<<"pub_poses_elapsed_time:"<<pub_poses_elapsed_time<<std::endl;
  pub_poses_elapsed_time += (event.current_real-pose_start_time).toSec();
  geometry_msgs::PoseArray poses = skel->get_pose_at_time(std::max(pub_poses_elapsed_time,0.0));
  geometry_msgs::Pose p;
  if (poses.poses.size()>1) {
    p.position.x = 0.5*(poses.poses[0].position.x+poses.poses[1].position.x);
    p.position.y = 0.5*(poses.poses[0].position.y+poses.poses[1].position.y);
    p.position.z = 0.5*(poses.poses[0].position.z+poses.poses[1].position.z);
    poses.poses.push_back(p);
    p.position.x = 0.5*(poses.poses[1].position.x+poses.poses[2].position.x);
    p.position.y = 0.5*(poses.poses[1].position.y+poses.poses[2].position.y);
    p.position.z = 0.5*(poses.poses[1].position.z+poses.poses[2].position.z);
    poses.poses.push_back(p);
    p.position.x = 0.5*(poses.poses[3].position.x+poses.poses[4].position.x);
    p.position.y = 0.5*(poses.poses[3].position.y+poses.poses[4].position.y);
    p.position.z = 0.5*(poses.poses[3].position.z+poses.poses[4].position.z);
    poses.poses.push_back(p);
    p.position.x = 0.5*(poses.poses[5].position.x+poses.poses[4].position.x);
    p.position.y = 0.5*(poses.poses[5].position.y+poses.poses[4].position.y);
    p.position.z = 0.5*(poses.poses[5].position.z+poses.poses[4].position.z);
    poses.poses.push_back(p);
    p.position.x = 0.5*(poses.poses[6].position.x+poses.poses[7].position.x);
    p.position.y = 0.5*(poses.poses[6].position.y+poses.poses[7].position.y);
    p.position.z = 0.5*(poses.poses[6].position.z+poses.poses[7].position.z);
    poses.poses.push_back(p);
    p.position.x = 0.5*(poses.poses[8].position.x+poses.poses[7].position.x);
    p.position.y = 0.5*(poses.poses[8].position.y+poses.poses[7].position.y);
    p.position.z = 0.5*(poses.poses[8].position.z+poses.poses[7].position.z);
    poses.poses.push_back(p);
  }
  poses.header.frame_id="world";
  poses.header.stamp=ros::Time::now();
  poses_pub.publish(poses);
  std_msgs::Float32MultiArray skel_pts_msg;
    for (int i=0;i<poses.poses.size();i++) {
        skel_pts_msg.data.push_back(poses.poses[i].position.x);
        skel_pts_msg.data.push_back(poses.poses[i].position.y);
        skel_pts_msg.data.push_back(poses.poses[i].position.z);
    }
    skel_pub.publish(skel_pts_msg);

    std::vector<Eigen::Quaternionf> human_quats = skel->get_quats_at_time(std::max(pub_poses_elapsed_time,0.0));
  std_msgs::Float32MultiArray skel_quats_msg;
    for (int i=0;i<4;i++) skel_quats_msg.data.push_back(0.0);
    for (int i=0;i<human_quats.size();i++) {
        skel_quats_msg.data.push_back(human_quats[i].w());
        skel_quats_msg.data.push_back(human_quats[i].x());
        skel_quats_msg.data.push_back(human_quats[i].y());
        skel_quats_msg.data.push_back(human_quats[i].z());
    }
    skel_quat_pub.publish(skel_quats_msg);
  pose_start_time = ros::Time::now();
}
void human_publisher::empty_poses(void) {
  geometry_msgs::PoseArray poses;
  poses.header.frame_id="world";
  poses.header.stamp=ros::Time::now();
  poses_pub.publish(poses);
}

data_recorder::data_recorder(ros::NodeHandle nh,std::string log_file_full_path, const planning_scene::PlanningScenePtr &planning_scene_ptr, avoidance_intervals::skeleton *skel_ptr, moveit::core::RobotModelConstPtr robot_model, robot_state::RobotStatePtr state, std::vector<double> human_link_len, std::vector<double> human_link_radii, rosdyn::ChainPtr chain,std::string solvperflog_file_full_path):nh_(nh),logFile(log_file_full_path,std::ofstream::app),ps_ptr(planning_scene_ptr),human_link_len_(human_link_len),human_link_radii_(human_link_radii), state_(state), chain_(chain),solvperflogFile(solvperflog_file_full_path,std::ofstream::app),jsLogFile("joint_state_record.csv",std::ofstream::trunc) {
  
    ready_ = false;
  spd_sub = nh_.subscribe<std_msgs::Int64>("/safe_ovr_1",1,&data_recorder::spd_callback,this);
  joint_states_sub = nh_.subscribe<sensor_msgs::JointState>("/joint_states",1,&data_recorder::jnt_state_callback,this);
//   joint_states_sub = nh_.subscribe<sensor_msgs::JointState>("/joint_states",1,&data_recorder::spd_callback,this);
//   joint_states_sub = nh_.subscribe<sensor_msgs::JointState>("/joint_states",1,&data_recorder::spd_callback,this);
  pose_sub = nh_.subscribe<geometry_msgs::PoseArray>("/poses",1,&data_recorder::pose_callback,this);

  skeleton_sub = nh_.subscribe<std_msgs::Float32MultiArray>("/skeleton",1,&data_recorder::skeleton_callback,this);
  camera_keypoints = std::vector<float>(15*3,0.0);
    sub_live_quats = nh.subscribe<std_msgs::Float32MultiArray>("/skeleton_quats",1,&data_recorder::skel_quats_cb,this);
    sub_dist = nh.subscribe<std_msgs::Float32>("/min_robot_human_dist",1,&data_recorder::dist_callback,this);
    time_pub = nh.advertise<std_msgs::Float32>("/cycle_time",1);
    solver_perf_sub=nh.subscribe<std_msgs::Float64MultiArray>("/solver_performance",1,&data_recorder::perf_callback,this);

  record_timer = nh_.createTimer(ros::Duration(1.0/60.0), &data_recorder::record_thread, this);
  record_timer.stop();

  human_pose_pts = std::vector<double>(29*2*3,0.0);

  skel = skel_ptr;
  min_distance = std::numeric_limits<double>::max();
  log_start_time = ros::Time::now();

}

void data_recorder::dist_callback(const std_msgs::Float32::ConstPtr& msg) {
  
    std::lock_guard<std::mutex> lck(mtx);
    min_distance = msg->data;
}

void data_recorder::spd_callback(const std_msgs::Int64::ConstPtr& msg) {
  spd_scale = msg->data;
}
void data_recorder::perf_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.empty()) return;
  avoid_plan_time = msg->data.back();
  ROS_INFO_STREAM("plan time:"<< plan_time);
  int stride = 3;
  for (int j=0;j<stride;j++) solvperflogFile<<msg->layout.dim[j].label<<",";
    solvperflogFile<<" \n";
  for (int i=0;i<msg->data.size();i+=stride) {
    for (int j=0;j<stride;j++) solvperflogFile<<msg->data[i+j]<<",";
    solvperflogFile<<" \n";
  }
}

void data_recorder::skeleton_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lck(mtx3);
    camera_keypoints = msg->data;
    std::lock_guard<std::mutex> lck2(mtx4);
    live_human_points.clear();
    for (int i=0;i<int(double(camera_keypoints.size())/3.0);i++) {
        live_human_points.push_back(Eigen::Vector3f(camera_keypoints[i*3],camera_keypoints[i*3+1],camera_keypoints[i*3+2]));
    }
}

void data_recorder::skel_quats_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lck(mtx5);
    std::vector<float> pose_elements = msg->data;
    live_human_quats.clear();
    for (int i=0;i<7;i++){
        live_human_quats.push_back(Eigen::Quaternionf(pose_elements[i*4+4],pose_elements[i*4+5],pose_elements[i*4+6],pose_elements[i*4+7]));
    }
}

void data_recorder::jnt_state_callback(const sensor_msgs::JointState::ConstPtr& msg) {
    double dt = 1.0;
    if (joint_state_ready) {
        dt = (msg->header.stamp-last_joint_state_time).toSec();
    }
    if (joint_names.empty()) {
        for (int i=0;i<msg->name.size();i++) joint_names.push_back(msg->name[i]);
        n_dof_ = int(joint_names.size());
        std::cout<<n_dof_<<std::endl;
        joint_positions = std::vector<double>(n_dof_,0.0);
        joint_velocities = std::vector<double>(n_dof_,0.0);
        joint_accelerations = std::vector<double>(n_dof_,0.0);
        joint_effort = std::vector<double>(n_dof_,0.0);
    }
    if (n_dof_==0) return;
    joint_pos_vec.resize(n_dof_);
    joint_pos_vec.setZero();
    joint_vel_vec.resize(n_dof_);
    joint_vel_vec.setZero();
    for (int i=0;i<n_dof_;i++) {
        if (&msg->velocity!=0) {
            if (int(msg->velocity.size())==n_dof_) {
                if (joint_state_ready) joint_accelerations[i] = (msg->velocity[i]-joint_velocities[i])/dt;
                // joint_velocities[i] = msg->velocity[i];
                if (joint_state_ready) joint_velocities[i] = (msg->position[i]-joint_positions[i])/dt;
            }
        // } catch(...) {
        //     ROS_WARN("no joint velocities!");
        }
        joint_positions[i] = msg->position[i];
        joint_pos_vec[i] = joint_positions[i];
    
        if (&msg->velocity!=0) {
            if (int(msg->velocity.size())==n_dof_) {
                joint_vel_vec[i] = joint_velocities[i];
                joint_effort[i] = msg->effort[i];
            }
        // } catch(...) {
        //     ROS_WARN("no joint velocities!");
        }
    }
    std::vector< Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d> > ee_twist=chain_->getTwist(joint_pos_vec,joint_vel_vec);
    double tmp_tangential_speed = 0.0;
    for (size_t il=0;il<ee_twist.size();il++) tmp_tangential_speed = std::max(tmp_tangential_speed,ee_twist.at(il).block(0,0,3,1).norm());
    last_joint_state_time = msg->header.stamp;
    joint_state_ready = true;
    std::lock_guard<std::mutex> lck(mtx2);
    tangential_speed = tmp_tangential_speed;
    jsLogFile<<(ros::Time::now()-log_start_time).toSec()<<",";
    for (int i=0;i<n_dof_;i++) jsLogFile << joint_positions[i]<<",";
    for (int i=0;i<n_dof_;i++) jsLogFile << joint_velocities[i]<<",";
    for (int i=0;i<n_dof_;i++) jsLogFile << joint_accelerations[i]<<",";
    jsLogFile <<"\n";
    std::lock_guard<std::mutex> lock(js_mtx);
    cur_js = *msg;
}
void data_recorder::pose_callback(const geometry_msgs::PoseArray::ConstPtr& msg) {
    ready_ = true;
    for (int i=0;i<msg->poses.size();i++) {
        human_pose_pts[i*3] = msg->poses[i].position.x;
        human_pose_pts[i*3+1] = msg->poses[i].position.y;
        human_pose_pts[i*3+2] = msg->poses[i].position.z;
    }
    pose_msg = *msg;
   
}

void data_recorder::record_thread(const ros::TimerEvent& event) {
    // // state_->setVariablePositions(joint_positions);

    // std::vector<std::string> obstacles;
    // obstacles = psi.getKnownObjectNames();
    // std::cout<<"num obstacles = "<<obstacles.size()<<std::endl;
    // for (int i=0;i<obstacles.size();i++) {
    //     std::cout<<obstacles[i]<<",";
    // }
    // std::cout<<std::endl;
    // collision_detection::AllowedCollisionMatrix acm = ps_ptr->getAllowedCollisionMatrix();
    // std::vector<std::string> acm_names;
    // acm.getAllEntryNames(acm_names);
    // std::vector<std::string> human_links = {"l_fore","l_upper","neck","r_fore","r_upper","torso"};
    // std::vector<std::string> robot_links = {"shoulder_link","upper_arm_link","forearm_link","wrist_1_link","wrist_2_link","wrist_3_link","robotiq_simplified"};
    // for (int i=0;i<acm_names.size()-1;i++){
    //     for (int j=i+1;j<acm_names.size();j++) {
    //         acm.setEntry(acm_names[i],acm_names[j],true);
    //     }
    //     for (int j=0;j<human_links.size();j++) {
    //         if (std::find(robot_links.begin(), robot_links.end(), acm_names[i]) != robot_links.end()) 
    //             acm.setEntry(acm_names[i],human_links[j],false);
    //     }
    // }
    // moveit_msgs::AllowedCollisionMatrix acm_msg;
    // acm.getMessage(acm_msg);
    // ROS_INFO_STREAM(acm_msg);
    // // acm.setEntry("forearm_link", "lower_beam", true);
    // // acm.setEntry("lower_beam", "forearm_link", true);
    
    // collision_detection::CollisionRequest req;
    // req.distance = true;
    // req.contacts = true;
    // req.max_contacts = 1000;
    // collision_detection::CollisionResult res;
    // ps_ptr->checkCollision(req,res,ps_ptr->getCurrentState(),acm);
    // ROS_INFO_STREAM("min_dist:"<<res.distance);
    // collision_detection::CollisionResult::ContactMap::const_iterator it;
    // for(it = res.contacts.begin();
    //     it != res.contacts.end();
    //     ++it)
    // {
    // ROS_INFO("Contact between: %s and %s",
    //         it->first.first.c_str(),
    //         it->first.second.c_str());
    // }
    if (log_lines==0) {
        logFile<<"t,spd scale,";
        for (int i=0;i<n_dof_;i++) {
            logFile<<joint_names[i]<<" pos,";
        }
        for (int i=0;i<n_dof_;i++) {
            logFile<<joint_names[i]<<" vel,";
        }
        for (int i=0;i<n_dof_;i++) {
            logFile<<joint_names[i]<<" acc,";
        }
        for (int i=0;i<n_dof_;i++) {
            logFile<<joint_names[i]<<" effort,";
        }
        for (int i=0;i<int(int(human_pose_pts.size())/3);i++) {
            logFile<<"act human j"<<i<<" x, y, z,";
        }
        for (int j=0;j<15;j++) {
            logFile<<"skel pt "<<j<<" x, y, z,";
        }
        logFile<<"pred pelvis x, y, z,";
        logFile<<"pred spine top x, y, z,";
        logFile<<"pred head top x, y, z,";
        logFile<<"pred l shoulder x, y, z,";
        logFile<<"pred l elbow x, y, z,";
        logFile<<"pred l wrist x, y, z,";
        logFile<<"pred r shoulder x, y, z,";
        logFile<<"pred r elbow x, y, z,";
        logFile<<"pred r wrist x, y, z,";
        logFile<<"plan time,min sep dist, tip speed\n";  
    }
    double t = (event.current_real-start_time).toSec();
    human_quats = skel->get_quats_at_time(t);
    std_msgs::Float32 time_msg;
    time_msg.data = t;
    time_pub.publish(time_msg);
    std::cout<<t<<", dist:"<<min_distance<<std::endl;
    logFile<<t<<","<<spd_scale<<",";
    for (int i=0;i<n_dof_;i++) {
        logFile<<joint_positions[i]<<",";
    }
    for (int i=0;i<n_dof_;i++) {
        logFile<<joint_velocities[i]<<",";
    }
    for (int i=0;i<n_dof_;i++) {
        logFile<<joint_accelerations[i]<<",";
    }
    for (int i=0;i<n_dof_;i++) {
        logFile<<joint_effort[i]<<",";
    }
    for (int i=0;i<human_pose_pts.size();i++) {
        logFile<<human_pose_pts[i]<<",";
    }
    mtx3.lock();
    for (int j=0;j<45;j++) {
        if (j<int(camera_keypoints.size())) {
            logFile<<camera_keypoints[j]<<",";
        } else {
            logFile<<" ,";
        }
    }
    mtx3.unlock();
    pred_human_pose = skel->get_pose_at_time(t);
    for (int i=0;i<9;i++) {
        if (i < pred_human_pose.poses.size()) {
            logFile<<pred_human_pose.poses[i].position.x<<",";
            logFile<<pred_human_pose.poses[i].position.y<<",";
            logFile<<pred_human_pose.poses[i].position.z<<",";
        } else {
            logFile<<" , , ,";
        }
    }
    std::lock_guard<std::mutex> lck(mtx);
    std::lock_guard<std::mutex> lck2(mtx2);
    logFile<<plan_time<<","<<min_distance<<","<<tangential_speed<<"\n";   
    log_lines++; 
}

void data_recorder::start(void) {
    log_lines = 0;
    start_time = ros::Time::now();
    record_timer.start();
}

void data_recorder::stop(void) {
    record_timer.stop();
    ready_ = false;
}

double data_recorder::get_min_dist(void) {
    std::lock_guard<std::mutex> lck(mtx);
    return min_distance;
}

data_recorder::~data_recorder() {
    logFile.close();
    solvperflogFile.close();
    jsLogFile.close();
}

human_occupancy_helper::human_occupancy_helper(ros::NodeHandle nh):nh_(nh) {
    pc_pub=nh_.advertise<sensor_msgs::PointCloud>("/occupancy",1);
    centroids_pub = nh_.advertise<geometry_msgs::PoseArray>("/centroids",1);

    Eigen::Vector3d workspace_lb={-1,-1,0.5};
    Eigen::Vector3d workspace_ub={1,1,2.5};
    
    std::vector<double> ws_lb_param;

    if (nh_.getParam("workspace_lower_bounds_xyz",ws_lb_param))
    {
        for (int i=0;i<3;i++) workspace_lb[i] = ws_lb_param[i];
    } else {
        ROS_DEBUG("workspace_lower_bounds_xyz is not set, default={-1,-1,0.5}");
    }  

    std::vector<double> ws_ub_param;

    if (nh_.getParam("workspace_upper_bounds_xyz",ws_ub_param))
    {
        for (int i=0;i<3;i++) workspace_ub[i] = ws_ub_param[i];
    } else {
        ROS_DEBUG("workspace_lower_bounds_xyz is not set, default={1,1,2.5}");
    }
    unsigned int npnt=50;
    grid_ = std::make_shared<human_occupancy::OccupancyGrid>(workspace_lb,workspace_ub,npnt);
}
void human_occupancy_helper::set_occupancy(std::vector<Eigen::VectorXf> avoid_pts) {
    visualization_msgs::Marker obs_pts;
    pc_pose_array.poses.clear();
    pc_pose_array.header.frame_id="world";
    avoidance_intervals::display_markers(avoid_pts,obs_pts);
    geometry_msgs::Pose pose;
    for (geometry_msgs::Point p:obs_pts.points) {
        pose.position = p;
        pc_pose_array.poses.push_back(pose);
    }
    grid_->update(pc_pose_array);
    pc = grid_->toPointCloud();
    pc_pub.publish(pc);

    centroids_pub.publish(pc_pose_array);
    // pub_timer = nh_.createTimer(ros::Duration(1.0/50.0), &human_occupancy_helper::set_occupancy_timer, this);
}

void human_occupancy_helper::set_occupancy_timer(const ros::TimerEvent& event) {
    // grid_->update(pc_pose_array);
    // pc = grid_->toPointCloud();
    centroids_pub.publish(pc_pose_array);
    pc_pub.publish(pc);
}

void disp_sub_callback(const visualization_msgs::Marker::ConstPtr& msg) {
    
}
humanCollisionObjects::humanCollisionObjects(ros::NodeHandle node_handle, const planning_scene::PlanningScenePtr &planning_scene_ptr, std::vector<double> lengths, std::vector<double> radii, double min_dist):planning_scene_(planning_scene_ptr),nh(node_handle),link_lengths_(lengths),link_radii_(radii), min_dist_(min_dist)  {
  
  planning_scene.is_diff = true;
  planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  ros::ServiceClient planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();


  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
        
  ros::WallDuration(1.0).sleep();
  timer_start = ros::Time::now();
  act_lengths = link_lengths_;
  act_radii = link_radii_;
}

void humanCollisionObjects::update_timer(const ros::TimerEvent& event) {
  elapsed_time += (event.current_real-timer_start).toSec();
  updateCollisionObjects(std::max(elapsed_time,0.0));
  timer_start = ros::Time::now();
}

void humanCollisionObjects::start_live_obs(void) {
  sub_live_quats = nh.subscribe<std_msgs::Float32MultiArray>("/skeleton_quats",1,&humanCollisionObjects::liveCollisionObjectsCallback,this);
}

void humanCollisionObjects::stop_live_obs(void) {
  sub_live_quats.shutdown();
}

void humanCollisionObjects::liveCollisionObjectsCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  // if (pause_live) return;
  forward_kinematics(msg->data);
  // ROS_INFO_STREAM("link_quats"<<link_quats.size());
  if (planning_scene.world.collision_objects.empty()) {
      collisionObjects.clear();
      for (int i=0;i<link_quats.size();i++) {
        collisionObjects.push_back(createCollisionObject(link_centroids[i],link_quats[i],act_lengths[i],act_radii[i],co_ids[i]));
      }
      // psi.addCollisionObjects(collisionObjects);
    } else {
      for (int i=0;i<link_quats.size();i++) {
          moveCollisionObject(collisionObjects[i],link_centroids[i],link_quats[i]);
      }
      // psi.applyCollisionObjects(collisionObjects);

    }
    planning_scene.world.collision_objects = collisionObjects;

    planning_scene_diff_publisher.publish(planning_scene);
}
void humanCollisionObjects::resume_live_obs(void) {
  inflate = false;
  removeHumans();
  for (int i=0;i<link_radii_.size();i++) act_radii[i] = link_radii_[i];
  // pause_live = false;
}

void humanCollisionObjects::inflate_live_obs(void) {
  // pause_live = true;
  removeHumans();
  for (int i=0;i<link_radii_.size();i++) act_radii[i] = link_radii_[i]+min_dist_;
}

void humanCollisionObjects::start_obs(double start_time) {
  elapsed_time = start_time;
  resume_obs();
}
void humanCollisionObjects::resume_obs(void) {
  inflate = false;
  removeHumans();
  for (int i=0;i<link_radii_.size();i++) act_radii[i] = link_radii_[i];
  updateCollisionObjects(std::max(elapsed_time,0.0));
  timer_start = ros::Time::now();
  udpate_timer = nh.createTimer(ros::Duration(1.0/10.0), &humanCollisionObjects::update_timer, this);
}

void humanCollisionObjects::inflate_obs(void) {
  udpate_timer.stop();
  removeHumans();
  for (int i=0;i<link_radii_.size();i++) act_radii[i] = link_radii_[i]+min_dist_;
  updateCollisionObjects(std::max(elapsed_time,0.0));
}

void humanCollisionObjects::pause_obs(void) {
  udpate_timer.stop();
}

void humanCollisionObjects::updateCollisionObjects(double t) {
  // std::cout<<"t:"<<t<<","<<dt<<","<<int(t/dt)<<std::endl;
  forward_kinematics(pose_sequence[std::min(int(t/dt),int(pose_sequence.size())-1)]);
  // ROS_INFO_STREAM("link_quats"<<link_quats.size());
  if (planning_scene.world.collision_objects.empty()) {
      collisionObjects.clear();
      for (int i=0;i<link_quats.size();i++) {
        collisionObjects.push_back(createCollisionObject(link_centroids[i],link_quats[i],act_lengths[i],act_radii[i],co_ids[i]));
      }
      // psi.addCollisionObjects(collisionObjects);
    } else {
      for (int i=0;i<link_quats.size();i++) {
          moveCollisionObject(collisionObjects[i],link_centroids[i],link_quats[i]);
      }
      // psi.applyCollisionObjects(collisionObjects);

    }
    planning_scene.world.collision_objects = collisionObjects;

    planning_scene_diff_publisher.publish(planning_scene);
}

std::vector<Eigen::Vector3f> humanCollisionObjects::getLinkData(double t) {
  // std::cout<<"t:"<<t<<","<<dt<<","<<int(t/dt)<<std::endl;
  forward_kinematics(pose_sequence[std::min(int(t/dt),int(pose_sequence.size())-1)]);
  return human_points;
}

void humanCollisionObjects::removeHumans(void) {
  std::cout<<"removing human"<<std::endl;
  for (int i=0;i<collisionObjects.size();i++) {
      collisionObjects[i].operation = collisionObjects[i].REMOVE;
      planning_scene.world.collision_objects[i] = collisionObjects[i];
  }
      // psi.applyCollisionObjects(collisionObjects);
  //publish the updated planning scene
  planning_scene_diff_publisher.publish(planning_scene);
  while (!planning_scene.world.collision_objects.empty()) {
      collisionObjects.pop_back();
      planning_scene.world.collision_objects.pop_back();
  }
}

moveit_msgs::CollisionObject humanCollisionObjects::createCollisionObject(Eigen::Vector3f pos, Eigen::Quaternionf quat, double length, double radius, std::string id)
{
    moveit_msgs::CollisionObject collisionObj;
    collisionObj.header.frame_id = "world";
    collisionObj.id = id;
    shape_msgs::SolidPrimitive co_shape;
    co_shape.type = co_shape.CYLINDER;
    co_shape.dimensions.resize(2);
    co_shape.dimensions[0] = length;
    co_shape.dimensions[1] = radius;

    geometry_msgs::Pose co_pose;
    co_pose.position.x = pos[0];
    co_pose.position.y = pos[1];
    co_pose.position.z = pos[2];
    co_pose.orientation.x = quat.x();
    co_pose.orientation.y = quat.y();
    co_pose.orientation.z = quat.z();
    co_pose.orientation.w = quat.w();  
    collisionObj.pose=co_pose;
    collisionObj.primitives.push_back(co_shape);
    co_pose.position.x = 0;
    co_pose.position.y = 0;
    co_pose.position.z = 0;
    co_pose.orientation.x = 0;
    co_pose.orientation.y = 0;
    co_pose.orientation.z = 0;
    co_pose.orientation.w = 1;
    collisionObj.primitive_poses.push_back(co_pose);
    collisionObj.operation = collisionObj.ADD;
    // collisionObj.pose.orientation.w = 1.0;

    return collisionObj;
}


void humanCollisionObjects::moveCollisionObject(moveit_msgs::CollisionObject &msg, Eigen::Vector3f pos, Eigen::Quaternionf quat)
{
    geometry_msgs::Pose co_pose;
    msg.pose.position.x = pos[0];
    msg.pose.position.y = pos[1];
    msg.pose.position.z = pos[2];

    // std::cout<<quat.w()<<","<<quat.x()<<","<<quat.y()<<","<<quat.z()<<std::endl;
    msg.pose.orientation.x = quat.x();
    msg.pose.orientation.y = quat.y();
    msg.pose.orientation.z = quat.z();
    msg.pose.orientation.w = quat.w();
    msg.primitives.clear();
    msg.operation = msg.MOVE;
}

void humanCollisionObjects::forward_kinematics(std::vector<float> pose_elements) {
    // ROS_INFO_STREAM("forward kinematics "<<pose_elements.size());
    // if (prediction.data[31]>0) {
    //     ROS_INFO_STREAM("error");
    //     ROS_INFO_STREAM(prediction);
    // }
    Eigen::Vector3f pelvis_loc = {pose_elements[1],pose_elements[2],pose_elements[3]};
    pelvis_loc = transform_to_world*pelvis_loc;
    Eigen::Quaternionf quat_to_world(transform_to_world.rotation());

    Eigen::Quaternionf z_axis_quat(0,0,0,1);
    Eigen::Quaternionf x_axis_quat(0.707,0,-0.707,0);
    std::vector<Eigen::Quaternionf> quats;
    Eigen::Quaternionf q;
    for (int i=0;i<7;i++){
        q = quat_to_world*Eigen::Quaternionf(pose_elements[i*4+4],pose_elements[i*4+5],pose_elements[i*4+6],pose_elements[i*4+7]);
        quats.push_back(q);
        // ROS_INFO_STREAM("quat "<<q.w()<<" "<<q.vec().transpose());
    }
    link_centroids.clear();
    link_quats.clear();
    human_points.clear();
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
    Eigen::Vector3f l_shoulder = spine_top-0.5*shoulder_len*z_shoulders.vec();
    human_points.push_back(l_shoulder);
    // ROS_INFO_STREAM("l_shoulder "<<l_shoulder.transpose());
    Eigen::Quaternionf z_e1 = quats[3]*z_axis_quat*quats[3].inverse();
    Eigen::Quaternionf x_e1 = quats[3]*x_axis_quat*quats[3].inverse();
    // link_quats.push_back(x_e1);
    Eigen::Vector3f e1 = l_shoulder+link_lengths_[2]*z_e1.vec();
    human_points.push_back(e1);
    link_centroids.push_back(l_shoulder+0.5*link_lengths_[2]*z_e1.vec());
    Eigen::Quaternionf z_w1 = quats[4]*z_axis_quat*quats[4].inverse();
    Eigen::Quaternionf x_w1 = quats[4]*x_axis_quat*quats[4].inverse();
    // link_quats.push_back(x_w1);
    Eigen::Vector3f w1 = e1+(link_lengths_[3]+0.1)*z_w1.vec();
    human_points.push_back(w1);
    link_centroids.push_back(e1+0.5*(link_lengths_[3])*z_w1.vec());
    Eigen::Vector3f r_shoulder = spine_top+0.5*shoulder_len*z_shoulders.vec();
    human_points.push_back(r_shoulder);
    // ROS_INFO_STREAM("r_shoulder "<<r_shoulder.transpose());
    Eigen::Quaternionf z_e2 = quats[5]*z_axis_quat*quats[5].inverse();
    Eigen::Quaternionf x_e2 = quats[5]*x_axis_quat*quats[5].inverse();
    // link_quats.push_back(x_e2);
    Eigen::Vector3f e2 = r_shoulder+link_lengths_[4]*z_e2.vec();
    human_points.push_back(e2);
    link_centroids.push_back(r_shoulder+0.5*link_lengths_[4]*z_e2.vec());
    Eigen::Quaternionf z_w2 = quats[6]*z_axis_quat*quats[6].inverse();
    Eigen::Quaternionf x_w2 = quats[6]*x_axis_quat*quats[6].inverse();
    // link_quats.push_back(x_w2);
    Eigen::Vector3f w2 = e2+(link_lengths_[5]+0.1)*z_w2.vec();
    human_points.push_back(w2);
    link_centroids.push_back(e2+0.5*(link_lengths_[5])*z_w2.vec());

    link_quats.push_back(quats[0]);
    link_quats.push_back(quats[1]);
    link_quats.push_back(quats[3]);
    link_quats.push_back(quats[4]);
    link_quats.push_back(quats[5]);
    link_quats.push_back(quats[6]);

    
}

void humanCollisionObjects::read_human_task(int task_num, Eigen::Isometry3f transform) {
    transform_to_world = transform;
    std::string file_name = ros::package::getPath("human_motion_prediction")+"/task_"+std::to_string(task_num) + "_t_means.csv";
    std::ifstream myfile (file_name); 
    std::string line;
    std::string prev_line;
    std::vector<std::string> all_lines;
    double prev_time = 0;
    std::stringstream ss;
    dt = 0;

    if (myfile.is_open()) { 
        // ROS_INFO_STREAM("reading file ");
        while (std::getline(myfile,line)) {
            all_lines.push_back(line);
        }

        ROS_INFO_STREAM("all lines "<<all_lines.size());
        myfile.close();
        pose_sequence.clear();
        for (int i=0;i<all_lines.size();i++) {
          std::vector<float> pose_elements;
          ss=std::stringstream(all_lines[i]);
          std::string substr;
          while (std::getline(ss,substr,',')) {
              pose_elements.push_back(std::stof(substr.c_str()));
          }   

          if ((dt==0)&&(pose_elements[0]>prev_time)) dt = pose_elements[0]-prev_time;

          pose_sequence.push_back(pose_elements);

        }

    }

}
