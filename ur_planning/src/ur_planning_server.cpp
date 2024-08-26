#include <rclcpp/service.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <thread>

#include "ur_custom_msgs/srv/move_to.hpp"
#include "ur_custom_msgs/srv/get_pose.hpp"

// All source files that use ROS logging should define a file-specific
// static const rclcpp::Logger named LOGGER, located at the top of the file
// and inside the namespace with the narrowest scope (if there is one)
static const rclcpp::Logger LOGGER = rclcpp::get_logger("ur_planning_server");

class URPlanningServer : public rclcpp::Node
{
  public:
    URPlanningServer(const rclcpp::Node::SharedPtr move_group_node)
    : Node("ur_planning_server"), 
      move_group_node(move_group_node),
      move_group(move_group_node, "arm")
    {
      ////////////////////////// MoveIt Setup //////////////////////////////////////////////////////////
      
      // We spin up a SingleThreadedExecutor for the current state monitor to get information
      // about the robot's state.
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(move_group_node);
      std::thread([&executor]() { executor.spin(); }).detach();

      static const std::string PLANNING_GROUP = "arm";

      // The move group interface
      move_group = moveit::planning_interface::MoveGroupInterface(move_group_node, PLANNING_GROUP);

      // Raw pointers are frequently used to refer to the planning group for improved performance.
      joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


      ////////////////////////// Service Setup //////////////////////////////////////////////////////////
      auto my_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

      // rclcpp::ServiceOptions options;
      // options.callback_group = my_callback_group;

      moveto_service = this->create_service<ur_custom_msgs::srv::MoveTo>(
        "ur_planning/move_to",
        std::bind(&URPlanningServer::moveto_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        my_callback_group
      );

      // getpose_service = this->create_service<ur_custom_msgs::srv::GetPose>(
      //   "ur_planning/get_pose",
      //   std::bind(&URPlanningServer::getpose_callback, this, std::placeholders::_1, std::placeholders::_2)
      // );
      

      // We can print the name of the reference frame for this robot.
      RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());

      // We can also print the name of the end-effector link for this group.
      RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
    }
    
  private:
    rclcpp::Node::SharedPtr move_group_node;
    const moveit::core::JointModelGroup* joint_model_group;
    geometry_msgs::msg::Pose target_pose;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    rclcpp::Service<ur_custom_msgs::srv::MoveTo>::SharedPtr moveto_service;
    // rclcpp::Service<ur_custom_msgs::srv::GetPose>::SharedPtr getpose_service;
    moveit::planning_interface::MoveGroupInterface move_group;
    // Planning scene will be used for collision setup
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    void moveto_callback(const std::shared_ptr<ur_custom_msgs::srv::MoveTo::Request> request,
                          std::shared_ptr<ur_custom_msgs::srv::MoveTo::Response> response)
    {
      RCLCPP_INFO(LOGGER, "Received request to move to pose");
      target_pose = request->target_pose;
      move_group.setPoseTarget(target_pose);

      bool success = (this->move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (!success)
      {
        response->success = false;
        return;
      }
      
      this->move_group.move(); 
      response->success = true;
    }

    // void getpose_callback(const std::shared_ptr<ur_custom_msgs::srv::GetPose::Request> request,
    //                       std::shared_ptr<ur_custom_msgs::srv::GetPose::Response> response)
    // {
    //   RCLCPP_INFO(LOGGER, "Received request to get pose");
    //   response->current_pose = move_group.getCurrentPose().pose;
    // }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node = rclcpp::Node::make_shared("ur_planning_move_group", node_options);

  auto ur_planning_server = std::make_shared<URPlanningServer>(move_group_node);

  // rclcpp::executors::MultiThreadedExecutor main_executor;
  // main_executor.add_node(ur_planning_server);
  // std::thread([&executor]() { main_executor.spin(); }).detach();
  
  while (rclcpp::ok())
  {
    rclcpp::spin_some(ur_planning_server);
  }

  return 0;
}