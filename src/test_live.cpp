#include <stap_warp/utilities.h>
#include <stap_warp/stap_warp.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib_msgs/GoalID.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <stap_warp/sequence.h>

std::vector<Eigen::Vector3f> pt_cloud;
std::mutex mtx;
std::mutex goal_mtx;
std::string goal_id;
ros::Time start_tm;
double start_duration;
std::mutex skel_mtx;
std::mutex stat_mtx;                //x,  y ,  z  ,  w,  x,  y,  z,  w,  x,  y,  z,    w,  x,    y,  z,w,x,   y,  z,  w,  x,  y,  z,  w,  x,  y,  z,  w,  x,   y,  z,  
std::vector<float> human_quat_pose = {0,0.75,-0.03,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.707,0.0,0.707,0.0,0,0,-1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,-1.0,0.0};
std_msgs::Float32MultiArray human_stat;

void disp_sub_callback(const visualization_msgs::Marker::ConstPtr& msg) {
    std::lock_guard<std::mutex> lck(mtx);
    pt_cloud.clear();
    for (int i=0;i<msg->points.size();i++) {
        pt_cloud.emplace_back(msg->points[i].x,msg->points[i].y,msg->points[i].z);
    }
}

void goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg) {
  std::lock_guard<std::mutex> lck(goal_mtx);
  goal_id = msg->goal_id.id;
  ROS_INFO_STREAM("read"<<goal_id);
}

void start_timer(const ros::TimerEvent& event) {
  std::stringstream stream;
  start_duration = 3.0-(event.current_real - start_tm).toSec();
  stream << std::fixed<< std::setprecision(1) << start_duration;
  std::cout<<std::endl;
  std::cout<<"***********\n";
  std::cout<<"*** "<<stream.str()<<" ***"<<std::endl;
  std::cout<<"***********\n";
  std::cout<<std::endl;
}

void skel_quats_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size()<2) return;
    std::lock_guard<std::mutex> lck(skel_mtx);
    human_quat_pose = std::vector<float>(msg->data.begin()+1,msg->data.end());
    // live_human_quats.clear();
    // for (int i=0;i<7;i++){
    //     live_human_quats.push_back(Eigen::Quaternionf(pose_elements[i*4+4],pose_elements[i*4+5],pose_elements[i*4+6],pose_elements[i*4+7]));
    // }
}

void human_done_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
  std::lock_guard<std::mutex> l(stat_mtx);
  human_stat = *msg;
}

int main(int argc, char** argv) {
    ros::init(argc,argv,"test_plan");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    std::shared_ptr<avoidance_intervals::skeleton> test_skeleton = std::make_shared<avoidance_intervals::skeleton>(nh,"/predicted_skeleton",0.05,0.1);

    Eigen::Vector3f workspace_lb={-1,-1,0.5};
    Eigen::Vector3f workspace_ub={1,1,2.5};
    
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
    double grid_spacing=0.05;
    if (!nh.getParam("/grid_spacing",grid_spacing))
    {
      ROS_WARN("grid_spacing is not defined");
    }
    double stap_offset=0.00;
    if (!nh.getParam("/stap_offset",stap_offset))
    {
      ROS_WARN("stap_offset is not defined");
    }

    avoidance_intervals::modelPtr model_ = std::make_shared<avoidance_intervals::model>(workspace_lb,workspace_ub,grid_spacing,6,nh);


    std::vector<Eigen::VectorXf> avoid_pts;
    test_skeleton->publish_pts(avoid_pts);

    // ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "human_markers", 0 );

    ros::Publisher model_pub = nh.advertise<std_msgs::Float32>( "/model_disp_req", 0,false);
    ros::Publisher model_all_pub = nh.advertise<std_msgs::Bool>( "/model_disp_all_req", 0,false);
    ros::Publisher centroids_pub = nh.advertise<geometry_msgs::PoseArray>( "/centroids", 0,false);
    ros::Publisher nom_plan_pub = nh.advertise<visualization_msgs::Marker>("/nominal_plan",1);
    ros::Publisher pub_human_status = nh.advertise<visualization_msgs::Marker>("/human_status",1);
    ros::Publisher pub_pause_tracking = nh.advertise<std_msgs::Bool>("/pause_tracking",1);
    ros::Subscriber sub_quats = nh.subscribe<std_msgs::Float32MultiArray>("/skeleton_quats",1,skel_quats_cb);
    ros::Subscriber sub_human_done = nh.subscribe<std_msgs::Float32MultiArray>("/human_task_status",1,human_done_cb);
    std::shared_ptr<ros::Publisher> pub_txt = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::Marker>("stap_description", 0,false));
    std::string plan_group = "manipulator";
    if (!nh.getParam("/plan_group",plan_group))
    {
      ROS_WARN("plan_group is not defined");
    }
    std::string ctrl_ns = "manipulator";
    if (!nh.getParam("/ctrl_ns",ctrl_ns))
    {
      ROS_WARN("ctrl_ns is not defined");
    }
    bool simulate_human = false;
    if (!nh.getParam("/test_live/simulate_human",simulate_human))
    {
      ROS_WARN("simulate_human is not defined");
    }
    ros::Publisher pub_cancel_traj = nh.advertise<actionlib_msgs::GoalID>( ctrl_ns+"/follow_joint_trajectory/cancel", 0,false);
    actionlib_msgs::GoalID goal_id_msg;
    ros::Subscriber sub_goal = nh.subscribe<control_msgs::FollowJointTrajectoryActionGoal>(ctrl_ns+"/follow_joint_trajectory/goal",1,goal_callback);
    //do a test with no obsctacles:
    std::vector<double> start_joint = {55,-69,87,-108,90,35}; 
    for (int i = 0;i<6;i++) {
      start_joint[i] *= 3.14/180;
    }
    moveit::planning_interface::MoveGroupInterface move_group(plan_group);
    // actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> mg_action_client = move_group.getMoveGroupClient();
    const robot_state::JointModelGroup* joint_model_group_ = move_group.getCurrentState()->getJointModelGroup(plan_group);
    robot_model_loader::RobotModelLoaderPtr robot_model_loader = robot_model_loader::RobotModelLoaderPtr(new robot_model_loader::RobotModelLoader("robot_description"));
    const robot_model::RobotModelPtr& model = robot_model_loader->getModel();
    std::shared_ptr<planning_scene::PlanningScene> scene(new planning_scene::PlanningScene(model));
    planning_scene_monitor::PlanningSceneMonitorPtr monitor;
    monitor.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    planning_scene::PlanningScenePtr ps_ptr = monitor->getPlanningScene();
    if(monitor->getPlanningScene())
    {
      monitor->startSceneMonitor("/move_group/monitored_planning_scene");
      monitor->startWorldGeometryMonitor();
      monitor->startStateMonitor();
    }
    else
    {
      exit(EXIT_FAILURE);
    }

    std::string resp = "y";
    clearObstacles();


    start_joint = {55,-69,87,-108,90,35}; 
    int test_num=1;
    if (!nh.getParam("/test_num",test_num))
    {
      ROS_ERROR("test_num is not defined");
    }
    std::string planning_pipeline;
    if (!nh.getParam("/tests/"+std::to_string(test_num)+"/planning_pipeline",planning_pipeline))
    {
      ROS_ERROR_STREAM("planning_pipeline is not defined for test "<<test_num);
      return 0;
    }
    std::string planner_id;
    if (!nh.getParam("/tests/"+std::to_string(test_num)+"/planner_id",planner_id))
    {
      ROS_ERROR_STREAM("planner_id is not defined for test "<<test_num);
      return 0;
    }

    std::string log_file_name;
    if (!nh.getParam("/tests/"+std::to_string(test_num)+"/log_file_name",log_file_name))
    {
      ROS_ERROR_STREAM("log_file_name is not defined for test "<<test_num);
      return 0;
    }

    move_group.setPlanningPipelineId(planning_pipeline);
    move_group.setPlannerId(planner_id);

    nh.getParam("/tests/"+std::to_string(test_num)+"/robot_start", start_joint);
    for (int i = 0;i<6;i++) {
      start_joint[i] *= 3.14/180;
    }
    bool use_warp = true;
    nh.getParam("/use_warp",use_warp);
    nh.getParam("/tests/"+std::to_string(test_num)+"/robot_start", start_joint);
    for (int i = 0;i<6;i++) {
      start_joint[i] *= 3.14/180;
    }
    robot_state::RobotStatePtr state;
    state = move_group.getCurrentState();
    state->setVariablePositions(start_joint);
    move_group.setStartState(*state);
    int human_task_num;
    bool use_collision_objects_initially;
    bool use_collision_objects_continuously;
    std::vector<double> goal_joint = {55,-69,87,-108,90,35}; 
    nh.getParam("/tests/"+std::to_string(test_num)+"/robot_goal", goal_joint);
    nh.getParam("/tests/"+std::to_string(test_num)+"/human_task_num", human_task_num);
    nh.getParam("/tests/"+std::to_string(test_num)+"/use_collision_objects_initially", use_collision_objects_initially);
    nh.getParam("/tests/"+std::to_string(test_num)+"/use_collision_objects_continuously", use_collision_objects_continuously);
    double plan_time = 60;
    nh.getParam("/planning_time", plan_time);
    nh.getParam("/tests/"+std::to_string(test_num)+"/planning_time", plan_time);
    Eigen::VectorXd goal_vec(goal_joint.size());
    for (int i = 0;i<6;i++) {
      goal_joint[i] *= 3.14/180;
      goal_vec[i] = goal_joint[i];
    }
    std::cout<<"human_task_num:"<<human_task_num<<std::endl;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success;
    bool move_success;
    std::vector<double> workcell_transform(7,0.0);
    workcell_transform[6] = 1.0;
    Eigen::Isometry3f transform_to_world = Eigen::Isometry3f::Identity();
    if (nh.getParam("/tests/"+std::to_string(test_num)+"/workcell_transform", workcell_transform)) {
      transform_to_world.linear() = Eigen::Matrix3f(Eigen::Quaternionf(workcell_transform[3],workcell_transform[4],workcell_transform[5],workcell_transform[6]));
      transform_to_world.translation() = Eigen::Vector3f(workcell_transform[0],workcell_transform[1],workcell_transform[2]);
    }
    ROS_INFO_STREAM(transform_to_world.matrix());

    std::vector<float> human_link_lengths;// = {0.569,0.194,0.328,0.285,0.357,0.285,0.45}; 
    std::vector<float> human_link_radii;// = {0.12,0.05,0.1,0.04,0.03,0.04,0.03}; 
    double min_dist = 0.0;
    if (!nh.getParam("/human_link_lengths", human_link_lengths)) ROS_ERROR("couldn't read human dimensions");
    if (!nh.getParam("/human_link_radii", human_link_radii)) ROS_ERROR("couldn't read human radii");
    if (!nh.getParam("/minimum_distance", min_dist)) ROS_ERROR("couldn't read minimum_distance");
    std::vector<double> human_link_lengths2; 
    std::vector<double> human_link_radii2; 
    test_skeleton->link_lengths_.resize(human_link_lengths.size());
    test_skeleton->link_radii_.resize(human_link_lengths.size());
    for (int i=0;i<human_link_radii.size();i++) {
      std::cout<<human_link_lengths[i]<<",";
      test_skeleton->link_lengths_[i] = (float)human_link_lengths[i];
      test_skeleton->link_radii_[i] = (float)human_link_radii[i];
      if (i==2) continue;
      human_link_lengths2.push_back(human_link_lengths[i]);
      human_link_radii2.push_back(human_link_radii[i]);
    }
    std::cout<<std::endl;

    // test_skeleton->link_lengths_ = {0.5,0.2,0.35,0.35,0.35,0.35,0.35};
    // test_skeleton->link_radii_ = {0.12,0.05,0.1,0.04,0.03,0.04,0.03};
    ROS_INFO_STREAM("reading file ");
    avoid_pts = test_skeleton->read_human_task(human_task_num,transform_to_world);
    ROS_INFO_STREAM("avoid pts size "<<avoid_pts.size());
    double avoid_offset = 0.0;
    nh.getParam("/test_sharework_cell/avoid_time_offset",avoid_offset);
    for (int i=0;i<avoid_pts.size();i++) {
      avoid_pts[i][3] += avoid_offset;
      avoid_pts[i][4] += avoid_offset;
    }  
    std_msgs::Bool show_all;
    show_all.data = true;
    double est_plan_time;

    humanCollisionObjects co_human(nh,scene,human_link_lengths2,human_link_radii2, 0.05);
    co_human.read_human_task(human_task_num,transform_to_world);


    human_publisher show_human(nh);
    show_human.skel = test_skeleton;
    show_human.model_pub = model_pub;
    show_human.stop_show_human();

    human_occupancy_helper human_occupany(nh);
    ros::Subscriber disp_sub = nh.subscribe<visualization_msgs::Marker>("/human_markers",1,disp_sub_callback);
  
    if (planner_id=="multigoal_ssm_test") {
      test_skeleton->publish_pts(avoid_pts);
      ros::Duration(2.0).sleep();
      if (1) {
        for (int i =0; i < int(test_skeleton->end_time_*10);i++) {
          // std_msgs::Float32 time_disp;
          // time_disp.data = float(i/10);
          // model_pub.publish(time_disp);
          // ros::Duration(0.05).sleep();
          // std::cout<<"pts in cloud:"<<pt_cloud.size()<<std::endl;
          // mtx.lock();
          // human_occupany.set_occupancy(pt_cloud);
          // mtx.unlock();
          geometry_msgs::PoseArray poses = test_skeleton->get_pose_at_time(std::max(double(i/10.0),0.0));
          std::vector<Eigen::Vector3f> pts;
          for (int i=0;i<poses.poses.size();i++) pts.emplace_back(poses.poses[i].position.x,poses.poses[i].position.y,poses.poses[i].position.z);
          geometry_msgs::Pose p;
          if (pts.size()>1) {
            p.position.x = 0.5*(poses.poses[0].position.x+poses.poses[1].position.x);
            p.position.y = 0.5*(poses.poses[0].position.y+poses.poses[1].position.y);
            p.position.z = 0.5*(poses.poses[0].position.z+poses.poses[1].position.z);
            pts.emplace_back(p.position.x,p.position.y,p.position.z);
            p.position.x = 0.5*(poses.poses[1].position.x+poses.poses[2].position.x);
            p.position.y = 0.5*(poses.poses[1].position.y+poses.poses[2].position.y);
            p.position.z = 0.5*(poses.poses[1].position.z+poses.poses[2].position.z);
            pts.emplace_back(p.position.x,p.position.y,p.position.z);
            p.position.x = 0.5*(poses.poses[3].position.x+poses.poses[4].position.x);
            p.position.y = 0.5*(poses.poses[3].position.y+poses.poses[4].position.y);
            p.position.z = 0.5*(poses.poses[3].position.z+poses.poses[4].position.z);
            pts.emplace_back(p.position.x,p.position.y,p.position.z);
            p.position.x = 0.5*(poses.poses[5].position.x+poses.poses[4].position.x);
            p.position.y = 0.5*(poses.poses[5].position.y+poses.poses[4].position.y);
            p.position.z = 0.5*(poses.poses[5].position.z+poses.poses[4].position.z);
            pts.emplace_back(p.position.x,p.position.y,p.position.z);
            p.position.x = 0.5*(poses.poses[6].position.x+poses.poses[7].position.x);
            p.position.y = 0.5*(poses.poses[6].position.y+poses.poses[7].position.y);
            p.position.z = 0.5*(poses.poses[6].position.z+poses.poses[7].position.z);
            pts.emplace_back(p.position.x,p.position.y,p.position.z);
            p.position.x = 0.5*(poses.poses[8].position.x+poses.poses[7].position.x);
            p.position.y = 0.5*(poses.poses[8].position.y+poses.poses[7].position.y);
            p.position.z = 0.5*(poses.poses[8].position.z+poses.poses[7].position.z);
            pts.emplace_back(p.position.x,p.position.y,p.position.z);
          }
          human_occupany.set_occupancy(pts);
        }
      } else {
        model_all_pub.publish(show_all);
        ros::Duration(0.01).sleep();
          std::cout<<"pts in cloud:"<<pt_cloud.size()<<std::endl;
        mtx.lock();
        human_occupany.set_occupancy(pt_cloud);
        mtx.unlock();
      }
    }

    urdf::Model urdf_model;
    urdf_model.initParam("robot_description");
    std::string base_frame_ = "world";
    std::string tool_frame = "open_tip";
    // double step_ = 0.1;
    if (!nh.getParam("base_frame", base_frame_))
    {
      ROS_ERROR("%s/base_frame not defined", nh.getNamespace().c_str());
      throw std::invalid_argument("base_frame is not defined");
    }
    if (!nh.getParam("tool_frame", tool_frame))
    {
      ROS_ERROR("%s/tool_frame not defined", nh.getNamespace().c_str());
      throw std::invalid_argument("tool_frame is not defined");
    }
    // if (!nh.getParam("computation_step", step_))
    // {
    //   ROS_ERROR("%s/computation_step not defined, use 0.1", nh_.getNamespace().c_str());
    //   step_=0.1;
    // }

    Eigen::Vector3d grav;
    grav << 0, 0, -9.806;

    rosdyn::ChainPtr chain = rosdyn::createChain(urdf_model, base_frame_, tool_frame, grav);
    std::string log_file= ros::package::getPath("stap_warp")+"/data/"+ log_file_name +"_live.csv";
    std::string log_file2= ros::package::getPath("stap_warp")+"/data/"+ log_file_name +"_solver_perf_live.csv";
    data_recorder rec(nh,log_file, scene,test_skeleton,model,state,human_link_lengths2,human_link_radii2,chain,log_file2);
    ros::Timer start_human = nh.createTimer(ros::Duration(0.1),start_timer);
    start_human.stop();

    int num_tests=1;
    nh.getParam("/num_tests", num_tests);
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    stap::stap_warper stap_warp(nh,move_group.getCurrentState(),model,ps_ptr);

    std::shared_ptr<ros::ServiceClient> predictor = std::make_shared<ros::ServiceClient>(nh.serviceClient<stap_warp::human_prediction>("predict_human"));
    while (!predictor->exists()) {
      ROS_INFO("waiting for predict_human service");
      ros::Duration(0.1).sleep();
    }
    //setup the human predictors
    std::shared_ptr<ros::Publisher> human_pub = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("predicted_human", 0,false));
    // std::shared_ptr<ros::Publisher> human_model_pub = std::make_shared<ros::Publisher>(nh.advertise<stap_warp::joint_seq>("human_model_seq", 0,false));
    skel_mtx.lock();
    std::cout<<human_quat_pose.size()<<std::endl;
    int human_seq = 0;
    nh.getParam("/tests/"+std::to_string(test_num) + "/human_sequence_num", human_seq);
    ROS_INFO_STREAM("loading human sequence:"<<human_seq);
    std::shared_ptr<stap_test::humans> human_data = std::make_shared<stap_test::humans>(nh,human_quat_pose,predictor,human_pub,test_skeleton,pub_txt,human_seq);
    skel_mtx.unlock();
    human_data->set_dimensions(human_link_lengths,human_link_radii);
    human_data->predicted_motion();
    human_data->generate_full_sequence(0,0,0.0);
    ROS_INFO_STREAM("human sequence:"<<human_data->full_joint_seq.size()<<" steps");
    human_data->show_predictions(human_link_lengths,human_link_radii);

    for (int test_num=0;test_num<num_tests;test_num++) {
      std_msgs::Bool pause_track_msg;
      if (planning_pipeline=="irrt_avoid") {
        pause_track_msg.data = true;
        pub_pause_tracking.publish(pause_track_msg);
      }
      std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
      test_skeleton->publish_pts(std::vector<Eigen::VectorXf>());
      // human_occupany.set_occupancy(std::vector<Eigen::VectorXf>());
      ros::Duration(2.0).sleep();
      model_all_pub.publish(show_all);

      move_group.setPlanningPipelineId("ompl");
      move_group.setPlannerId("BiTRRT");
      move_group.setStartStateToCurrentState();
      move_group.setJointValueTarget(start_joint);
      move_group.setPlanningTime(2.0);
      plan_success = (move_group.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (plan_success) {
        plans.push_back(plan);
      } else {
        ROS_ERROR("planning failed");
        num_tests++;
        continue;
      }

      test_skeleton->publish_pts(avoid_pts);
      test_skeleton->publish_sequence(0.0);
      if (use_collision_objects_initially) co_human.updateCollisionObjects(0.0);

      geometry_msgs::PoseArray poses = test_skeleton->get_pose_at_time(0.0);
      geometry_msgs::Pose p;
      if (poses.poses.size()>1) {
        p.position.x = 0.5*(poses.poses[0].position.x+poses.poses[1].position.x);
        p.position.y = 0.5*(poses.poses[0].position.y+poses.poses[1].position.y);
        p.position.z = 0.5*(poses.poses[0].position.z+poses.poses[1].position.z);
        poses.poses.push_back(p);
      }
      poses.header.frame_id="world";
      poses.header.stamp=ros::Time::now();

      centroids_pub.publish(poses);
      ros::Duration(5.0).sleep();
      model_all_pub.publish(show_all);

      move_group.setPlanningPipelineId(planning_pipeline);
      move_group.setPlannerId(planner_id);
      state->setVariablePositions(start_joint);
      move_group.setStartState(*state);
      move_group.setJointValueTarget(goal_joint);
      move_group.setPlanningTime(plan_time);
      plan_success = (move_group.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (plan_success) {
        plans.push_back(plan);
      } else {
        ROS_ERROR("planning failed");
        num_tests++;
        continue;
      }    

      pause_track_msg.data = false;
      pub_pause_tracking.publish(pause_track_msg);
      
      est_plan_time = plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
      // avoid_offset = plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
      // for (int i=0;i<avoid_pts.size();i++) {
      //   avoid_pts[i][3] -= avoid_offset;
      //   avoid_pts[i][4] -= avoid_offset;
      // }
      // test_skeleton->publish_pts(avoid_pts);
      // test_skeleton->publish_sequence(avoid_offset);

      test_skeleton->publish_pts(std::vector<Eigen::VectorXf>());
      // human_occupany.set_occupancy(std::vector<Eigen::VectorXf>());

      ros::Duration(2.0).sleep();
      model_all_pub.publish(show_all);

      move_group.setPlanningPipelineId("ompl");
      move_group.setPlannerId("BiTRRT");
      state->setVariablePositions(goal_joint);
      move_group.setStartState(*state);
      // move_group.setStartStateToCurrentState();
      std::vector<double> home_joint = {0.0,0.0,0.0,0.0,0.0,0.0};
      move_group.setJointValueTarget(home_joint);
      move_group.setPlanningTime(2.0);
      plan_success = (move_group.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if (plan_success) {
        plans.push_back(plan);
      } else {
        ROS_ERROR("planning failed");
        num_tests++;
        continue;
      }
      rec.plan_time = est_plan_time;
      if (((test_num-1)%5==0)||(test_num%5==0)||(test_num>15)) rec.plan_time = rec.avoid_plan_time;
      test_skeleton->publish_pts(avoid_pts);
      ros::Duration(1.0).sleep();
      bool showing_human = false;
      rec.stop();

      while (!rec.ready()) {
        ROS_INFO_STREAM("/poses not publishing yet");
        ros::Duration(1.0).sleep();
      }
      co_human.removeHumans();
      // if ((planner_id!="dirrt_paper")&&(planner_id!="multigoal_ssm_test")) co_human.start_live_obs();
      co_human.start_live_obs();
      ros::Time p_start = ros::Time::now();
      for (int i=0;i<plans.size();i++) {
        if (i==1) {
          ROS_WARN("Press enter when the human is in place.  When you press enter, the human should repeat the recorded motion.");

          start_tm = ros::Time::now();
          std::cout<<"Starting in:\n";
          start_human.start();
          ros::Duration(3.0).sleep();
          std::cout<<"Go!\n";
          start_human.stop();
          // std::cin.ignore();
        
          show_human.start_time = ros::Time::now();
          show_human.start_show_human();
          p_start = ros::Time::now();
          showing_human = true;

          rec.start();
        // } else if (i==2) {
        //   show_human.pose_start_time = ros::Time::now();
        //   pub_poses_timer.start();
        //   showing_human = true;
        } else {
          if (showing_human) {
            rec.stop();
            ROS_INFO_STREAM("finished with time:"<<rec.t<<" seconds.");
            showing_human = false;
            test_skeleton->publish_pts(std::vector<Eigen::VectorXf>());
            ros::Duration(0.1).sleep();
            std_msgs::Float32 t_msg;
            t_msg.data = 0.0;
            model_pub.publish(t_msg);
            show_human.stop_show_human();
          }
        }
        if (i==2) {
          ROS_WARN("press enter when the human backs away from the robot");
          // std::cin.ignore();
        }
        if ((i==1) && use_collision_objects_continuously){

                    // co_human.start_obs();
          move_group.asyncExecute(plans[i]);
          bool dist_ready = false;
          int prev_spd_scale = 0.0;
          ros::Time prev_plan_time=ros::Time::now()-ros::Duration(1.0);
          bool replan_needed = false;
          ros::Time last_motion_time = ros::Time::now();
          
          double prev_min_dist = rec.get_min_dist();
          while ((rec.joint_pos_vec-goal_vec).norm()>0.001) {
            if (rec.joint_vel_vec.cwiseAbs().sum()>0.01) last_motion_time = ros::Time::now();
            replan_needed = ((ros::Time::now()-last_motion_time).toSec()>1.0);
            //if stopped replan
            if (planner_id=="multigoal_ssm_test") {
              std::vector<Eigen::Vector3f> pts = rec.live_human_points;
              Eigen::Vector3f p;
              if (poses.poses.size()>1) {
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
              human_occupany.set_occupancy(pts);
            }
            if ((planner_id=="dirrt_paper")||(planner_id=="multigoal_ssm_test")) {
              if (((((ros::Time::now()-prev_plan_time).toSec()>1.0) && (rec.spd_scale<50)) && (rec.spd_scale-prev_spd_scale<=0))||replan_needed) {
                move_group.stop();
                int t=0;
                while (rec.joint_vel_vec.cwiseAbs().sum()>0.001) {
                  goal_id_msg.stamp = ros::Time::now();
                  goal_mtx.lock();
                  goal_id_msg.id = goal_id;
                  ROS_INFO_STREAM("send"<<goal_id);
                  goal_mtx.unlock();
                  pub_cancel_traj.publish(goal_id_msg);
                  t++;
                }
                ROS_INFO_STREAM("stopped");
                centroids_pub.publish(rec.pose_msg);
                show_human.stop_show_human();
                ros::Duration(0.1).sleep();
                // std::cin.ignore();

                plan_success = false;
                t = 0;
                while ((!plan_success) && (t<20)) {
                  for (int j=0; j<rec.joint_positions.size();j++) std::cout<<rec.joint_positions[j]<<",";
                  std::cout<<std::endl;
                  move_group.setPlanningPipelineId(planning_pipeline);
                  move_group.setPlannerId(planner_id);
                  move_group.setPlanningTime(0.2);
                  state->setVariablePositions(rec.joint_positions);
                  move_group.setStartState(*state);
                  move_group.setJointValueTarget(goal_joint);
                  plan_success = (move_group.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                  t++;
                }
                // co_human.resume_obs();
                show_human.resume_show_human();
                ros::Duration(0.1).sleep();
                move_success = (move_group.asyncExecute(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ros::Duration(0.1).sleep();
                prev_plan_time=ros::Time::now();
                last_motion_time = ros::Time::now();
              }
              prev_spd_scale = rec.spd_scale;
            } else {
              if (rec.get_min_dist()>min_dist+0.05) dist_ready = true;
              if (((rec.get_min_dist()<0.4)&&((ros::Time::now()-prev_plan_time).toSec()>1.0) && (rec.get_min_dist()-prev_min_dist<0) && (dist_ready)) || replan_needed) {
                move_group.stop();
                int t=0;
                while (rec.joint_vel_vec.cwiseAbs().sum()>0.001) {
                  goal_id_msg.stamp = ros::Time::now();
                  goal_mtx.lock();
                  goal_id_msg.id = goal_id;
                  ROS_INFO_STREAM("send"<<goal_id);
                  goal_mtx.unlock();
                  pub_cancel_traj.publish(goal_id_msg);
                  t++;
                }
                ROS_INFO_STREAM("stopped");
                // mg_action_client.cancelAllGoals();
                show_human.stop_show_human();
                co_human.inflate_live_obs();
                ros::Duration(0.1).sleep();
                // std::cin.ignore();

                plan_success = false;
                t=0;
                while ((!plan_success) && (t<20)) {
                  for (int j=0; j<rec.joint_positions.size();j++) std::cout<<rec.joint_positions[j]<<",";
                  std::cout<<std::endl;
                  move_group.setPlanningPipelineId(planning_pipeline);
                  move_group.setPlannerId(planner_id);
                  move_group.setPlanningTime(0.2);
                  state->setVariablePositions(rec.joint_positions);
                  move_group.setStartState(*state);
                  move_group.setJointValueTarget(goal_joint);
                  plan_success = (move_group.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                  t++;
                }
                co_human.resume_live_obs();
                show_human.resume_show_human();
                ros::Duration(0.1).sleep();
                move_success = (move_group.asyncExecute(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
                ros::Duration(0.1).sleep();
                prev_plan_time=ros::Time::now();
                last_motion_time = ros::Time::now();
                prev_min_dist = rec.get_min_dist();
              }
            }
            ros::Duration(0.03).sleep();
          }  
        } else {

        if (i==1) {
            ROS_INFO_STREAM("plan size:"<<plans[i].trajectory_.joint_trajectory.points.size());

            pub_plan(nom_plan_pub,plans[i],state);
            move_group.asyncExecute(plans[i]);
            int human_step=0;
            ros::Duration(0.1).sleep();
            visualization_msgs::Marker mkr;
            mkr.id=105;
            mkr.lifetime = ros::Duration(0.0);
            mkr.type=mkr.TEXT_VIEW_FACING;
            mkr.color.r=1.0;
            mkr.color.b=1.0;
            mkr.color.g=1.0;
            mkr.color.a=1.0;
            mkr.pose.position.x = -0.8;
            mkr.pose.position.y = 0;
            mkr.pose.position.z = 1;
            mkr.pose.orientation.x = 1;
            mkr.pose.orientation.y = 0;
            mkr.pose.orientation.z = 0;
            mkr.pose.orientation.w = 0;
            mkr.scale.z = 0.2;
            mkr.header.frame_id = "world";
            ros::Time human_start_time = ros::Time::now();
            ros::Time h_switch_tm = ros::Time::now();
            ros::Rate r1(10);
            while (((rec.joint_pos_vec-goal_vec).norm()>0.001)&&(ros::ok())) {
              std::stringstream str;
              if (human_step<human_data->get_num_steps()) {
                if (simulate_human) {
                  ROS_INFO_STREAM("time:"<<(ros::Time::now()-human_start_time).toSec()<<", sim step:"<<human_data->sim_switch_times[human_step]);
                  if ((ros::Time::now()-human_start_time).toSec()>human_data->sim_switch_times[human_step]) {
                    ROS_INFO_STREAM("human step "<<human_step<<" is done");
                    human_step++;
                    h_switch_tm = ros::Time::now();
                  }
                }
                else if (human_data->is_step_done(human_step)) {
                  ROS_INFO_STREAM("human step "<<human_step<<" is done");
                  human_step++;
                  h_switch_tm = ros::Time::now();
                  human_data->reset_motion_done();
                }
                str<<"step:"<<human_step;
                stat_mtx.lock();
                if (human_stat.data.size()>6) {
                  str<<", e:";
                  str<<round(human_stat.data[6]*100)*0.01;
                  str<<", spd:";
                  str<<round(human_stat.data[4]*100)*0.01;
                }
                stat_mtx.unlock();
                human_data->update_predictions(human_step,human_quat_pose,0,std::max(0.0,std::min((h_switch_tm-ros::Time::now()).toSec(),(double)human_data->human_start_delay(human_step))));
              } else {
                str<<"Human Task Done!";
                human_data->full_joint_seq.clear();
              }
              mkr.text = str.str();
              pub_human_status.publish(mkr);
              human_data->show_reach_tgt(human_step);
              //std::max((ros::Time::now()-p_start).toSec(),0.0);
              if (use_warp) {
                stap_warp.warp(human_data->full_joint_seq,0.0,rec.joint_pos_vec,rec.get_current_joint_state());
              }
              // r1.sleep();
            }
            mkr.text = "Robot HRI segment is done!";
            pub_human_status.publish(mkr);
          } else {
            move_group.execute(plans[i]);
          }

          move_group.execute(plans[i]);
          ros::Duration(0.1).sleep();
        }
      }
      co_human.stop_live_obs();
      co_human.removeHumans();

      ros::Duration(2.0).sleep();
      if (test_num<num_tests-1) ROS_INFO("Repeating the test");

    }

    ROS_INFO_STREAM("made it!");
    return 0;
}
