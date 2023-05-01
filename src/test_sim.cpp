#include <stap_warp/utilities.h>
#include <stap_warp/stap_warp.h>

#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib_msgs/GoalID.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>

std::vector<Eigen::VectorXf> pt_cloud;
std::mutex mtx;
std::mutex goal_mtx;
std::string goal_id;

void disp_sub_callback(const visualization_msgs::Marker::ConstPtr& msg) {
    std::lock_guard<std::mutex> lck(mtx);
    pt_cloud.clear();
    for (int i=0;i<msg->points.size();i++) {
        pt_cloud.push_back(Eigen::Vector3f(msg->points[i].x,msg->points[i].y,msg->points[i].z));
    }
}

void goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg) {
  std::lock_guard<std::mutex> lck(goal_mtx);
  goal_id = msg->goal_id.id;
  ROS_INFO_STREAM("read"<<goal_id);
}

int main(int argc, char** argv) {
    ros::init(argc,argv,"test_plan");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    avoidance_intervals::skeleton test_skeleton(nh,"/predicted_skeleton",0.05,0.1);

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

    avoidance_intervals::modelPtr model_ = std::make_shared<avoidance_intervals::model>(workspace_lb,workspace_ub,grid_spacing,6,nh);

    std::vector<Eigen::VectorXf> avoid_pts;
    test_skeleton.publish_pts(avoid_pts);

    // ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "human_markers", 0 );

    ros::Publisher model_pub = nh.advertise<std_msgs::Float32>( "/model_disp_req", 0,false);
    ros::Publisher model_all_pub = nh.advertise<std_msgs::Bool>( "/model_disp_all_req", 0,false);
    ros::Publisher centroids_pub = nh.advertise<geometry_msgs::PoseArray>( "/centroids", 0,false);
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
    ros::Publisher pub_cancel_traj = nh.advertise<actionlib_msgs::GoalID>( ctrl_ns+"/follow_joint_trajectory/cancel", 0,false);
    actionlib_msgs::GoalID goal_id_msg;
    ros::Subscriber sub_goal = nh.subscribe<control_msgs::FollowJointTrajectoryActionGoal>(ctrl_ns+"/follow_joint_trajectory/goal",1,goal_callback);
    //do a test with no obsctacles:
    static const std::string PLANNING_GROUP = plan_group;
    std::vector<double> start_joint = {55,-69,87,-108,90,35}; 
    for (int i = 0;i<6;i++) {
      start_joint[i] *= 3.14/180;
    }
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    // actionlib::SimpleActionClient<moveit_msgs::MoveGroupAction> mg_action_client = move_group.getMoveGroupClient();
    const robot_state::JointModelGroup* joint_model_group_ = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
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
    std::vector<int> test_nums;

    std::string test_num_string;
    int test_num_int;
    if (!nh.getParam("/test_num",test_num_string))
    {
      if (!nh.getParam("/test_num",test_num_int))
      {
        ROS_ERROR("test_num is not defined");
        return 0;
      } else {
        test_nums.push_back(test_num_int);
      }
    } else {
      std::stringstream ss(test_num_string);
      std::string s;
      int i=0;
      while (std::getline(ss, s, ',')) {
          test_nums.push_back(std::stof(s));
          i++;
      }
    }
    std::cout<<"tests:";
    for (int i = 0;i<test_nums.size();i++) std::cout<<test_nums[i]<<",";
    std::cout<<std::endl;
    for (int test_num:test_nums) {

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
      bool pub_occ_centroids = false;
      if (!nh.getParam("/tests/"+std::to_string(test_num)+"/publish_occupancy_centoroids",pub_occ_centroids))
      {
        ROS_WARN_STREAM("pub_occ_centroids is not defined for test "<<test_num<<",assuming false");
      }
      
      bool get_plan_time_from_perf = false;
      if (!nh.getParam("/tests/"+std::to_string(test_num)+"/get_plan_time_from_perf",get_plan_time_from_perf))
      {
        ROS_WARN_STREAM("get_plan_time_from_perf is not defined for test "<<test_num<<",assuming false");
      }

      move_group.setPlanningPipelineId(planning_pipeline);
      move_group.setPlannerId(planner_id);

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
      bool move_success = false;
      std::vector<double> workcell_transform(7,0.0);
      
      Eigen::Isometry3f transform_to_world = Eigen::Isometry3f::Identity();
      if (nh.getParam("/tests/"+std::to_string(test_num)+"/workcell_transform", workcell_transform)) {
        transform_to_world.linear() = Eigen::Matrix3f(Eigen::Quaternionf(workcell_transform[3],workcell_transform[4],workcell_transform[5],workcell_transform[6]));
        transform_to_world.translation() = Eigen::Vector3f(workcell_transform[0],workcell_transform[1],workcell_transform[2]);
      }
      ROS_INFO_STREAM(transform_to_world.matrix());

      std::vector<double> human_link_lengths;// = {0.569,0.194,0.328,0.285,0.357,0.285,0.45}; 
      std::vector<double> human_link_radii;// = {0.12,0.05,0.1,0.04,0.03,0.04,0.03}; 
      double min_dist = 0.0;
      if (!nh.getParam("/human_link_lengths", human_link_lengths)) ROS_ERROR("couldn't read human dimensions");
      if (!nh.getParam("/human_link_radii", human_link_radii)) ROS_ERROR("couldn't read human radii");
      if (!nh.getParam("/minimum_distance", min_dist)) ROS_ERROR("couldn't read minimum_distance");
      std::vector<double> human_link_lengths2; 
      std::vector<double> human_link_radii2; 
      test_skeleton.link_lengths_.resize(human_link_lengths.size());
      test_skeleton.link_radii_.resize(human_link_lengths.size());
      for (int i=0;i<human_link_radii.size();i++) {
        std::cout<<human_link_lengths[i]<<",";
        test_skeleton.link_lengths_[i] = (float)human_link_lengths[i];
        test_skeleton.link_radii_[i] = (float)human_link_radii[i];
        if (i==2) continue;
        human_link_lengths2.push_back(human_link_lengths[i]);
        human_link_radii2.push_back(human_link_radii[i]);
      }
      std::cout<<std::endl;

      // test_skeleton.link_lengths_ = {0.5,0.2,0.35,0.35,0.35,0.35,0.35};
      // test_skeleton.link_radii_ = {0.12,0.05,0.1,0.04,0.03,0.04,0.03};
      ROS_INFO_STREAM("reading file ");
      avoid_pts = test_skeleton.read_human_task(human_task_num,transform_to_world);
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


      human_publisher show_human(nh,true);
      show_human.skel = & test_skeleton;
      show_human.model_pub = model_pub;
      show_human.skel_pub = nh.advertise<std_msgs::Float32MultiArray> ("/skeleton_points",1);
      show_human.skel_quat_pub = nh.advertise<std_msgs::Float32MultiArray> ("/skeleton_quats",1);
      show_human.poses_pub = nh.advertise<geometry_msgs::PoseArray> ("/poses",1);
      show_human.stop_show_human();
      show_human.stop_poses();

      human_occupancy_helper human_occupany(nh);
      ros::Subscriber disp_sub = nh.subscribe<visualization_msgs::Marker>("/human_markers",1,disp_sub_callback);
    
      if (pub_occ_centroids) {
        test_skeleton.publish_pts(avoid_pts);
        ros::Duration(2.0).sleep();
        if (0) {
          for (int i =0; i < int(test_skeleton.end_time_*10);i++) {
            std_msgs::Float32 time_disp;
            time_disp.data = float(i/10);
            model_pub.publish(time_disp);
            ros::Duration(0.01).sleep();
            std::cout<<"pts in cloud:"<<pt_cloud.size()<<std::endl;
            mtx.lock();
            human_occupany.set_occupancy(pt_cloud);
            mtx.unlock();
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
      std::string log_file= ros::package::getPath("stap_warp")+"/data/"+ log_file_name +"_sim.csv";
      std::string log_file2= ros::package::getPath("stap_warp")+"/data/"+ log_file_name +"_solver_perf_sim.csv";
      data_recorder rec(nh,log_file, scene,&test_skeleton,model,state,human_link_lengths2,human_link_radii2,chain,log_file2);
      int num_tests=1;
      nh.getParam("/num_tests", num_tests);
      move_group.setMaxVelocityScalingFactor(1.0);
      move_group.setMaxAccelerationScalingFactor(1.0);

      stap::stap_warper stap_warp(nh,move_group.getCurrentState(),model);

      for (int iter=0;iter<num_tests;iter++) {
        std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans;
        test_skeleton.publish_pts(std::vector<Eigen::VectorXf>());
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

          ROS_WARN_STREAM("path1 time:"<<plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec());
        } else {
          ROS_ERROR("planning failed");
          num_tests++;
          continue;
        }

        // test_skeleton.publish_pts(avoid_pts);
        // test_skeleton.publish_sequence(0.0);


        // ros::Duration(5.0).sleep();
        // test_skeleton.publish_pts(std::vector<Eigen::VectorXf>());

        // ros::Duration(5.0).sleep();
        test_skeleton.publish_pts(avoid_pts);
        // test_skeleton.publish_sequence(0.0);
        if (use_collision_objects_initially) co_human.updateCollisionObjects(0.0);

        geometry_msgs::PoseArray poses = test_skeleton.get_pose_at_time(0.0);
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

        if (pub_occ_centroids) centroids_pub.publish(poses);

        show_human.pose_start_time = ros::Time::now();
        show_human.start_poses();
        ros::Duration(0.1).sleep();
        show_human.stop_poses();
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
        

        est_plan_time = plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec();
        ROS_WARN_STREAM("path p="<<plans.size()-1<<" time:"<<est_plan_time<<", plan size"<<plans[1].trajectory_.joint_trajectory.points.size());
        ros::Duration(0.5).sleep();
        rec.plan_time = est_plan_time;
        if (get_plan_time_from_perf) rec.plan_time = rec.avoid_plan_time;

        test_skeleton.publish_pts(std::vector<Eigen::VectorXf>());
        // human_occupany.set_occupancy(std::vector<Eigen::VectorXf>());

        ros::Duration(2.0).sleep();
        model_all_pub.publish(show_all);

        move_group.setPlanningPipelineId("ompl");
        move_group.setPlannerId("BiTRRT");
        state->setVariablePositions(goal_joint);
        move_group.setStartState(*state);
        // move_group.setStartStateToCurrentState();
        move_group.setJointValueTarget(start_joint);
        move_group.setPlanningTime(2.0);
        plan_success = (move_group.plan(plan)==moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (plan_success) {
          plans.push_back(plan);
          ROS_WARN_STREAM("path1 time:"<<plan.trajectory_.joint_trajectory.points.back().time_from_start.toSec());

        } else {
          ROS_ERROR("planning failed");
          num_tests++;
          continue;
        }
        poses.poses.clear();
        if (pub_occ_centroids) centroids_pub.publish(poses);

        // rec.plan_time = est_plan_time;
        test_skeleton.publish_pts(avoid_pts);
        ros::Duration(1.0).sleep();
        bool showing_human = false;
        for (int i=0;i<plans.size();i++) {
          if (i==1) {
            // ROS_WARN("Press enter when the human is in place.  When you press enter, the human should repeat the recorded motion.");
            // std::cin.ignore();
          
            show_human.start_time = ros::Time::now();
            show_human.pose_start_time = ros::Time::now();
            show_human.start_show_human();
            show_human.start_poses();
            showing_human = true;

            rec.start();
          // } else if (i==2) {
          //   show_human.pose_start_time = ros::Time::now();
          //   pub_poses_timer.start();
          //   showing_human = true;
          } else {
            if (showing_human) {
              rec.stop();
              showing_human = false;
              test_skeleton.publish_pts(std::vector<Eigen::VectorXf>());
              ros::Duration(0.1).sleep();
              std_msgs::Float32 t_msg;
              t_msg.data = 0.0;
              model_pub.publish(t_msg);
              show_human.empty_poses();
              show_human.stop_show_human();
              show_human.stop_poses();
              poses.poses.clear();
              if ((test_num-3)%5==0) centroids_pub.publish(poses);
            }
          }
          // if (i==2) {
            // ROS_WARN("press enter when the human backs away from the robot");
            // std::cin.ignore();
          // }
          if ((i==1) && use_collision_objects_continuously){
            // co_human.start_obs();
            move_group.asyncExecute(plans[i]);
            bool dist_ready = false;
            int prev_spd_scale = 0.0;
            ros::Time prev_plan_time=ros::Time::now()-ros::Duration(1.0);
            bool replan_needed = false;
            ros::Time last_motion_time = ros::Time::now();
            double prev_min_dist = rec.get_min_dist();
            co_human.removeHumans();
            if ((test_num-3)%5!=0)  co_human.start_obs();
            while ((rec.joint_pos_vec-goal_vec).norm()>0.001) {
              if (rec.joint_vel_vec.cwiseAbs().sum()>0.01) last_motion_time = ros::Time::now();
              replan_needed = ((ros::Time::now()-last_motion_time).toSec()>1.0);
              //if stopped replan
            
              if (pub_occ_centroids)  {
                ROS_INFO_STREAM("replan dirrt");
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
                  // mg_action_client.cancelAllGoals();
                  show_human.stop_show_human();
                  show_human.stop_poses();
                  // co_human.inflate_obs();
                  // co_human.pause_obs();
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
                  show_human.resume_poses();
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
                  show_human.stop_poses();
                  co_human.inflate_obs();
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
                  co_human.resume_obs();
                  show_human.resume_show_human();
                  show_human.resume_poses();
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
            if ((i==1) && ((test_num-3)%5!=0)) co_human.start_obs();
            ros::Time p_start = ros::Time::now();
            if (i==1) {
              ROS_INFO_STREAM("plan size:"<<plans[i].trajectory_.joint_trajectory.points.size());
              move_group.asyncExecute(plans[i]);
              while ((rec.joint_pos_vec-goal_vec).norm()>0.001) {
                stap_warp.warp(plans[i],model_->joint_seq,std::max((ros::Time::now()-p_start).toSec()+0.2,0.0),rec.joint_pos_vec);
                ros::Duration(0.1).sleep();
              }
            } else {
              move_group.execute(plans[i]);
            }
            ROS_INFO_STREAM("exec took:"<<(ros::Time::now()-p_start).toSec());
            if (i==1) co_human.removeHumans();
          }
        }

        ros::Duration(2.0).sleep();
        if (iter<num_tests-1) ROS_INFO("Repeating the test");

      }
    }

    ROS_INFO_STREAM("made it!");
    return 0;
}
