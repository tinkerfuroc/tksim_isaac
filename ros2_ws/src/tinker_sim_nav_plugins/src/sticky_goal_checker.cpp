#include "tinker_sim_nav_plugins/sticky_goal_checker.hpp"

#include <cmath>

#include "angles/angles.h"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"

namespace tinker_sim_nav_plugins
{

void StickyGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  SimpleGoalChecker::initialize(parent, plugin_name, costmap_ros);
  auto node = parent.lock();
  logger_ = node->get_logger();
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".release_margin", rclcpp::ParameterValue(release_margin_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".goal_match_tolerance", rclcpp::ParameterValue(goal_match_tolerance_));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".goal_match_yaw_tolerance",
    rclcpp::ParameterValue(goal_match_yaw_tolerance_));
  node->get_parameter(plugin_name + ".release_margin", release_margin_);
  node->get_parameter(plugin_name + ".goal_match_tolerance", goal_match_tolerance_);
  node->get_parameter(plugin_name + ".goal_match_yaw_tolerance", goal_match_yaw_tolerance_);
}

void StickyGoalChecker::reset()
{
  // Called on every new path, including replans of the same goal; the latch
  // is cleared by isGoalReached when the goal itself changes.
  SimpleGoalChecker::reset();
}

void StickyGoalChecker::unlatch(const char * reason, double value)
{
  latched_ = false;
  RCLCPP_INFO(logger_, "%s: xy latch cleared (%s %.3f)", plugin_name_.c_str(), reason, value);
}

bool StickyGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  if (!stateful_) {
    latched_ = false;
    return SimpleGoalChecker::isGoalReached(query_pose, goal_pose, velocity);
  }
  const double dist = std::hypot(
    query_pose.position.x - goal_pose.position.x, query_pose.position.y - goal_pose.position.y);
  if (latched_) {
    const double moved = std::hypot(
      goal_pose.position.x - latched_goal_.position.x,
      goal_pose.position.y - latched_goal_.position.y);
    const double turned = std::fabs(angles::shortest_angular_distance(
        tf2::getYaw(goal_pose.orientation), tf2::getYaw(latched_goal_.orientation)));
    if (moved > goal_match_tolerance_) {
      unlatch("goal moved", moved);
    } else if (turned > goal_match_yaw_tolerance_) {
      unlatch("goal turned", turned);
    } else if (dist > xy_goal_tolerance_ + release_margin_) {
      unlatch("released at", dist);
    }
  }
  if (!latched_) {
    if (dist > xy_goal_tolerance_) {
      return false;
    }
    latched_ = true;
    latched_goal_ = goal_pose;
    RCLCPP_INFO(
      logger_, "%s: xy latch set at %.3f (goal %.2f, %.2f)", plugin_name_.c_str(), dist,
      goal_pose.position.x, goal_pose.position.y);
  }
  const double dyaw = angles::shortest_angular_distance(
    tf2::getYaw(query_pose.orientation), tf2::getYaw(goal_pose.orientation));
  return std::fabs(dyaw) < yaw_goal_tolerance_;
}

bool StickyGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance)
{
  SimpleGoalChecker::getTolerances(pose_tolerance, vel_tolerance);
  if (latched_) {
    pose_tolerance.position.x = xy_goal_tolerance_ + release_margin_;
    pose_tolerance.position.y = xy_goal_tolerance_ + release_margin_;
  }
  return true;
}

}  // namespace tinker_sim_nav_plugins

PLUGINLIB_EXPORT_CLASS(tinker_sim_nav_plugins::StickyGoalChecker, nav2_core::GoalChecker)
