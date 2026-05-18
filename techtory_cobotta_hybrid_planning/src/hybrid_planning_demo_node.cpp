#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/hybrid_planner.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hybrid_planning_demo_node");

  RCLCPP_INFO(node->get_logger(), "Starting Hybrid Planning Demo Node");

  auto action_client = rclcpp_action::create_client<moveit_msgs::action::HybridPlanner>(
      node, "run_hybrid_planning");

  if (!action_client->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node->get_logger(), "Hybrid Planner action server not available after waiting");
    return 1;
  }

  auto goal_msg = moveit_msgs::action::HybridPlanner::Goal();
  
  // Set up the motion plan request
  goal_msg.motion_sequence.items.resize(1);
  goal_msg.motion_sequence.items[0].req.group_name = "arm";
  goal_msg.motion_sequence.items[0].req.max_velocity_scaling_factor = 0.1;
  goal_msg.motion_sequence.items[0].req.max_acceleration_scaling_factor = 0.1;
  goal_msg.motion_sequence.items[0].req.allowed_planning_time = 5.0;

  // We want to move joint 1 a little bit
  robot_model_loader::RobotModelLoader robot_model_loader(node, "robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  moveit::core::RobotState goal_state(kinematic_model);
  goal_state.setToDefaultValues();

  // Try to set joint values
  std::vector<double> joint_values;
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("arm");
  goal_state.copyJointGroupPositions(joint_model_group, joint_values);
  
  // modify first joint (adjust safely for cobotta)
  if (!joint_values.empty()) {
      joint_values[0] += 0.2; 
  }
  goal_state.setJointGroupPositions(joint_model_group, joint_values);

  moveit_msgs::msg::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  goal_msg.motion_sequence.items[0].req.goal_constraints.push_back(joint_goal);

  RCLCPP_INFO(node->get_logger(), "Sending Hybrid Planning Goal");

  auto send_goal_options = rclcpp_action::Client<moveit_msgs::action::HybridPlanner>::SendGoalOptions();
  send_goal_options.result_callback =
      [node](const rclcpp_action::ClientGoalHandle<moveit_msgs::action::HybridPlanner>::WrappedResult& result) {
        switch (result.code)
        {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node->get_logger(), "Hybrid Planning goal succeeded!");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node->get_logger(), "Hybrid Planning goal was aborted: %s", result.result->error_message.c_str());
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node->get_logger(), "Hybrid Planning goal was canceled");
            break;
          default:
            RCLCPP_ERROR(node->get_logger(), "Unknown result code");
            break;
        }
        rclcpp::shutdown();
      };

  action_client->async_send_goal(goal_msg, send_goal_options);

  rclcpp::spin(node);

  return 0;
}
