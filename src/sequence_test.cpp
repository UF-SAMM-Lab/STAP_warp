#include <stap_warp/utilities.h>
#include <stap_warp/stap_warp.h>
#include <stap_warp/sequence.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <actionlib_msgs/GoalID.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <stap_warp/human_prediction.h>
#include <memory>
#include <mutex>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>


std::mutex mtx;
std::mutex goal_mtx;
std::string goal_id;
std::mutex skel_mtx;
ros::Time start_tm;
double start_duration;
std::vector<float> human_quat_pose = {-1.737439036369323730e-01,9.085529446601867676e-01,-3.478607162833213806e-02,9.981507693313929064e-01,-5.998364698210649493e-02,9.849049592449109214e-03,0.000000000000000000e+00,9.580149214898834309e-01,1.740551826395725366e-01,-2.278424973507739981e-01,0.000000000000000000e+00,7.123834513833746662e-01,1.841394957996581871e-01,6.772019375945099728e-01,-0.000000000000000000e+00,2.844575261446070646e-01,1.394740396576789054e-01,-9.484887495807520219e-01,0.000000000000000000e+00,7.097945123379674204e-01,4.777340680356021441e-01,5.176503747637605235e-01,-0.000000000000000000e+00,2.843504163844787769e-01,-3.630602545215000920e-01,8.873173571438557339e-01,0.000000000000000000e+00,7.148535069955277432e-01,2.736450296645902003e-01,-6.435082449169243768e-01,0.000000000000000000e+00};


void goal_callback(const control_msgs::FollowJointTrajectoryActionGoal::ConstPtr& msg) {
  std::lock_guard<std::mutex> lck(goal_mtx);
  goal_id = msg->goal_id.id;
  ROS_INFO_STREAM("read"<<goal_id);
}

// void segment_thread(moveit::planning_interface::MoveGroupInterface::Plan plan) {
//     ROS_INFO_STREAM("plan size:"<<plans.trajectory_.joint_trajectory.points.size());
//     move_group.asyncExecute(plans);
//     ros::Time p_start = ros::Time::now();
//     ros::Duration(0.1).sleep();
//     while ((rec.joint_pos_vec-goal_vec).norm()>0.001) {
//         stap_warp.warp(model_->joint_seq,std::max((ros::Time::now()-p_start).toSec(),0.0),rec.joint_pos_vec,rec.get_current_joint_state());
//         ros::Duration(0.1).sleep();
//     }
// }

void skel_quats_cb(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size()<2) return;
    std::lock_guard<std::mutex> lck(skel_mtx);
    human_quat_pose = std::vector<float>(msg->data.begin()+1,msg->data.end());
    // live_human_quats.clear();
    // for (int i=0;i<7;i++){
    //     live_human_quats.push_back(Eigen::Quaternionf(pose_elements[i*4+4],pose_elements[i*4+5],pose_elements[i*4+6],pose_elements[i*4+7]));
    // }
}

int main(int argc, char** argv) {
    ros::init(argc,argv,"test_plan");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

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
    bool simulated = true;
    if (!nh.getParam("/sequence_test/simulated",simulated))
    {
      ROS_WARN("/sequence_test/simulated is not set");
    }
    
    ros::Publisher pub_cancel_traj = nh.advertise<actionlib_msgs::GoalID>( ctrl_ns+"/follow_joint_trajectory/cancel", 0,false);
    actionlib_msgs::GoalID goal_id_msg;
    ros::Subscriber sub_goal = nh.subscribe<control_msgs::FollowJointTrajectoryActionGoal>(ctrl_ns+"/follow_joint_trajectory/goal",1,goal_callback);
    ros::Subscriber sub_quats = nh.subscribe<std_msgs::Float32MultiArray>("/skeleton_quats",1,skel_quats_cb);
    ros::Publisher pub_pause_tracking = nh.advertise<std_msgs::Bool>("/pause_tracking",1);
    std::shared_ptr<ros::Publisher> pub_txt = std::make_shared<ros::Publisher>(nh.advertise<jsk_rviz_plugins::OverlayText>("stap_description", 0,false));
    pub_txt->publish(stap_test::gen_overlay_text("this is some text"));
    
    //do a test with no obsctacles:
    std::vector<double> start_joint;
    std::vector<double> goal_joint;

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
    std::vector<float> human_link_lengths;// = {0.569,0.194,0.328,0.285,0.357,0.285,0.45}; 
    std::vector<float> human_link_radii;// = {0.12,0.05,0.1,0.04,0.03,0.04,0.03}; 
    double min_dist = 0.0;
    if (!nh.getParam("/human_link_lengths", human_link_lengths)) ROS_ERROR("couldn't read human dimensions");
    if (!nh.getParam("/human_link_radii", human_link_radii)) ROS_ERROR("couldn't read human radii");
    if (!nh.getParam("/minimum_distance", min_dist)) ROS_ERROR("couldn't read minimum_distance");
    if (!nh.getParam("/minimum_distance", min_dist)) ROS_ERROR("couldn't read minimum_distance");
    std::vector<double> human_link_lengths2; 
    std::vector<double> human_link_radii2; 
    for (int i=0;i<human_link_radii.size();i++) {
      std::cout<<human_link_lengths[i]<<",";
      // test_skeleton.link_lengths_[i] = (float)human_link_lengths[i];
      // test_skeleton.link_radii_[i] = (float)human_link_radii[i];
      if (i==2) continue;
      human_link_lengths2.push_back(human_link_lengths[i]);
      human_link_radii2.push_back(human_link_radii[i]);
    }
    std::cout<<std::endl;

    std::string resp = "y";
    clearObstacles();

    moveit::planning_interface::PlanningSceneInterface current_scene;
    // add collision box object to avoid hitting fixtures
    ros::ServiceClient planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();
    moveit_msgs::ApplyPlanningScene srv;
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;
    planning_scene_msg.world.collision_objects.push_back(createCollisionBox(Eigen::Vector3f(0.4,0.3,0.15),Eigen::Vector3f(-0.4,0.4,0.075),Eigen::Quaternionf(0,1,0,0),"fixtures"));
    srv.request.scene = planning_scene_msg;
    // srv.request. = ps_ptr->getAllowedCollisionMatrix();
    planning_scene_diff_client.call(srv);

    std::shared_ptr<ros::ServiceClient> predictor = std::make_shared<ros::ServiceClient>(nh.serviceClient<stap_warp::human_prediction>("predict_human"));
    while (!predictor->exists()) {
      ROS_INFO("waiting for predict_human service");
      ros::Duration(0.1).sleep();
    }
    std::shared_ptr<avoidance_intervals::skeleton> test_skeleton = std::make_shared<avoidance_intervals::skeleton>(nh,"/predicted_skeleton",0.05,0.1);
    std::shared_ptr<ros::Publisher> human_pub = std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("predicted_human", 0,false));
    // std::shared_ptr<ros::Publisher> human_model_pub = std::make_shared<ros::Publisher>(nh.advertise<stap_warp::joint_seq>("human_model_seq", 0,false));
    skel_mtx.lock();
    std::cout<<human_quat_pose.size()<<std::endl;
    std::shared_ptr<stap_test::humans> human_data = std::make_shared<stap_test::humans>(nh,human_quat_pose,predictor,human_pub,test_skeleton,pub_txt,1);
    skel_mtx.unlock();
    human_data->set_dimensions(human_link_lengths,human_link_radii);

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
    std::string log_file= ros::package::getPath("stap_warp")+"/data/sequence_sim.csv";
    std::string log_file2= ros::package::getPath("stap_warp")+"/data/sequence_solver_perf_sim.csv";
    std::shared_ptr<data_recorder> rec = std::make_shared<data_recorder>(nh,log_file, scene,test_skeleton,model,move_group.getCurrentState(),human_link_lengths2,human_link_radii2,chain,log_file2);
    stap_test::robot_sequence robot_data(nh,move_group.getCurrentState(),model,plan_group,human_data,rec,ps_ptr,pub_txt);
    human_data->predicted_motion();
    human_data->generate_full_sequence(0,0,0.0);
    human_data->save_full_seq("sequence_human.csv");
    human_data->generate_full_sequence(5,10,0.0);
    human_data->save_full_seq("sequence_human2.csv");
    human_data->generate_full_sequence(14,18,0.0);
    human_data->save_full_seq("sequence_human3.csv");
    human_data->show_predictions(human_link_lengths,human_link_radii);
    int seg_num=0;
    //pre-plan all robot motions
    int h = 0;
    int next_h = 0;
    double last_human_time = 0.0;
    double segment_time = 0.0;
    double human_done_time = 0.0;

    std_msgs::Bool pause_track_msg;
    pause_track_msg.data = true;
    pub_pause_tracking.publish(pause_track_msg);
    std::vector<double> last_waypoint = move_group.getCurrentJointValues();
    //plan robot segment nominal paths based on nominal predictions
    for (int i=0;i<robot_data.num_segments();i++) {
      ROS_INFO_STREAM("next h:"<<next_h);
      if (next_h<human_data->get_num_steps()) {
        if (i>human_data->human_prior_robot_task(next_h)) {
          ROS_INFO_STREAM("setting h to "<<next_h<<","<<i<<","<<human_data->human_prior_robot_task(next_h));
          h=next_h;
          segment_time = 0.0;
        }
      }      
      while ((human_data->human_prior_robot_task(next_h)<i)&&(next_h<human_data->get_num_steps())) {
        next_h++;
      }
      last_human_time = human_data->pub_model(h,i,-segment_time);
      // human_data->show_sequence();
      // std::cout<<"see the motion"<<std::endl;
      // std::cin.ignore();
      ros::Duration(0.5).sleep();
      double planned_time = robot_data.plan_robot_segment(i,last_waypoint);
      segment_time += planned_time;
      ROS_INFO_STREAM("preplanned robot segment "<<i<<" with est. time of "<<planned_time);
    }
    pause_track_msg.data = false;
    pub_pause_tracking.publish(pause_track_msg);
    ROS_INFO_STREAM("ready to start when a person is in the cell");
    if (!simulated) {
      while (!rec->ready()) {
        ROS_INFO_STREAM_ONCE("/poses not publishing yet");
        ros::Duration(1.0).sleep();
      }
    }

    robot_data.set_gripper(true);


    ros::Rate r(100);
    //now attempt execution
    int robot_step = 0;
    int human_step = 0;
    human_data->reset_motion_done();
    rec->stop();

    ros::Time human_start_time = ros::Time::now();
    rec->start();
    while (ros::ok()) {
      if (robot_step>=robot_data.num_segments()) break;
      if (!robot_data.is_segment_active()) {
        if (robot_data.get_prior_human_step(robot_step)<human_step) {
          robot_data.do_segment(robot_step);
          robot_step++;
        } else {
          ROS_INFO_STREAM_THROTTLE(10,"waiting to start robot segment "<<robot_step<<" until human step "<<robot_data.get_prior_human_step(robot_step)<<" is done");
        }
      }
      std::cout<<"robot step:"<<robot_step<<", human step:"<<human_step<<", seq:"<<human_data->full_joint_seq.size()<<std::endl;

      human_data->pub_descrition(human_step);
      human_data->update_predictions(human_step,human_quat_pose,robot_step,std::max(0.0,std::min((human_start_time-ros::Time::now()).toSec(),(double)human_data->human_start_delay(human_step))));
      if (simulated) {
        double elapsed_tm = (ros::Time::now()-human_start_time).toSec();
        if ((human_data->simulate_step(human_step,elapsed_tm,human_quat_pose)) && (robot_step>human_data->human_prior_robot_task(human_step+1))) {
          human_step++;
          human_start_time = ros::Time::now();
        }

      } else if ((human_data->is_step_done(human_step)&&((ros::Time::now()-human_start_time).toSec()>0.8*human_data->get_step_end_time(human_step)))) {
        human_step++;
        human_start_time = ros::Time::now();
        human_data->reset_motion_done();
      }
      // robot_data.update_prediction();
      r.sleep();
    } 
    while (!robot_data.is_segment_active()) ros::Duration(0.1).sleep();
    rec->stop();
    while (human_step<human_data->get_num_steps()) {
      ROS_INFO_STREAM_THROTTLE(5,"waiting for the human to finish tasks:"<<human_step);
      if (simulated) {
        double elapsed_tm = (ros::Time::now()-human_start_time).toSec();
        if ((human_data->simulate_step(human_step,elapsed_tm,human_quat_pose)) && (robot_step>human_data->human_prior_robot_task(human_step+1))) {
          human_step++;
          human_start_time = ros::Time::now();
        }

      } else if (human_data->is_step_done(human_step)) {
        human_step++;
        human_data->reset_motion_done();
      }
      if (human_step>=human_data->get_num_steps()) break;
      ros::Duration(0.01).sleep();
    }
    ROS_INFO("done!");



}