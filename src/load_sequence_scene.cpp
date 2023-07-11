#include <stap_warp/utilities.h>
#include <stap_warp/stap_warp.h>
#include <stap_warp/sequence.h>

std::mutex mtx;
std::mutex goal_mtx;
std::string goal_id;
std::mutex skel_mtx;
ros::Time start_tm;
double start_duration;

int main(int argc, char** argv) {
    ros::init(argc,argv,"load_sequence_scene");
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
    int test_num = 1; 
    if (!nh.getParam("/load_scene/test_num",test_num))
    {
      ROS_WARN("/load_scene/test_num is not set");
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

    clearObstacles();

    moveit::planning_interface::PlanningSceneInterface current_scene;
    // add collision box object to avoid hitting fixtures
    ros::ServiceClient planning_scene_diff_client = nh.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    planning_scene_diff_client.waitForExistence();
    moveit_msgs::ApplyPlanningScene srv;
    moveit_msgs::PlanningScene planning_scene_msg;
    planning_scene_msg.is_diff = true;

    int num_collision_boxes = 0;
    if (!nh.getParam("/test_sequence/" + std::to_string(test_num) + "/collision_boxes/length",num_collision_boxes))
    {
      ROS_WARN_STREAM("/test_sequence/"<<std::to_string(test_num)<<"/collision_boxes/length is not set");
    } 

    for (int i=0;i<num_collision_boxes;i++) {
      std::vector<double> box_params; 
      if (!nh.getParam("/test_sequence/" + std::to_string(test_num) + "/collision_boxes/" + std::to_string(i) + "/origin_xyz_wxyz_dims",box_params))
      {
        ROS_WARN_STREAM("/test_sequence/"<<std::to_string(test_num)<<"/collision_boxes/" << std::to_string(i)<<"/origin_xyz_wxyz_dims is not set");
      } 
      std::string box_name = "";
      if (!nh.getParam("/test_sequence/" + std::to_string(test_num) + "/collision_boxes/" + std::to_string(i) + "/name",box_name))
      {
        ROS_WARN_STREAM("/test_sequence/"<<std::to_string(test_num)<<"/collision_boxes/" << std::to_string(i)<<"/name is not set");
      } 
      Eigen::Vector3f box_origin(box_params[0],box_params[1],box_params[2]);
      // box_origin = transform_to_world*box_origin;
      Eigen::Quaternionf box_quat(box_params[3],box_params[4],box_params[5],box_params[6]);
      // box_quat = Eigen::Quaternionf(transform_to_world.rotation())*box_quat;
      std::cout<<"box:"<<box_origin[0]<<","<<box_origin[1]<<","<<box_origin[2]<<","<<box_quat.w()<<","<<box_quat.x()<<","<<box_quat.y()<<","<<box_quat.z()<<std::endl;
      planning_scene_msg.world.collision_objects.push_back(createCollisionBox(Eigen::Vector3f(box_params[7],box_params[8],box_params[9]),box_origin,box_quat,box_name));
    }

    srv.request.scene = planning_scene_msg;
    // srv.request. = ps_ptr->getAllowedCollisionMatrix();
    planning_scene_diff_client.call(srv);
    
    ROS_INFO("done!");

}