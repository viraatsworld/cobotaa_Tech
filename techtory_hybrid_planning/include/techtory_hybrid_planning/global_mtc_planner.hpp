// Global Planner plugin (demo3) for the Techtory Cobotta cell.
//
// Implements moveit::hybrid_planning::GlobalPlannerInterface using MoveIt Task
// Constructor (MTC). The incoming MotionPlanRequest is interpreted in the
// "welding" convention used by the upstream hybrid_planning_demo example:
//
//   goal_constraints[0].position_constraints[0].target_point_offset  -> start xyz
//   goal_constraints[0].orientation_constraints[0].orientation       -> start quat
//   goal_constraints[0].position_constraints[1].target_point_offset  -> goal  xyz
//   goal_constraints[0].orientation_constraints[1].orientation       -> goal  quat
//
// From those two poses the plugin builds an Approach -> Weld -> Retreat MTC
// task. Pair it with hybrid_planning_demo3_node, which encodes the request in
// exactly this layout.
//
// Ported from UR10e_welding_demo/hybrid_planning_demo (Author: Henning Kayser),
// adapted to the Cobotta "arm" group and the cobotta_pro_tool0 TCP. The
// UR-specific processit_tasks dependency has been removed.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/global_planner/global_planner_interface.hpp>
#include <moveit/task_constructor/task.h>

namespace techtory_hybrid_planning
{
// Component loaded by the Global Planner node; computes a full trajectory with
// MoveIt Task Constructor.
class GlobalMTCPlannerComponent : public moveit::hybrid_planning::GlobalPlannerInterface
{
public:
  bool initialize(const rclcpp::Node::SharedPtr& node) override;
  bool reset() noexcept override;
  moveit_msgs::msg::MotionPlanResponse
  plan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::GlobalPlanner>> global_goal_handle)
      override;

private:
  moveit::task_constructor::TaskPtr task_;
  rclcpp::Node::SharedPtr node_ptr_;
};
}  // namespace techtory_hybrid_planning
