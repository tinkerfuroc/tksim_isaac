#ifndef TINKER_SIM_NAV_PLUGINS__STICKY_GOAL_CHECKER_HPP_
#define TINKER_SIM_NAV_PLUGINS__STICKY_GOAL_CHECKER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "nav2_controller/plugins/simple_goal_checker.hpp"
#include "rclcpp/rclcpp.hpp"

namespace tinker_sim_nav_plugins
{

/**
 * SimpleGoalChecker with an xy latch that holds for the goal, not the path.
 *
 * Humble's controller_server calls reset() on every new path (the BT replans
 * about once a second), which clears SimpleGoalChecker's stateful latch, and
 * RPP takes its rotate-to-goal-heading threshold from getTolerances() with no
 * hysteresis. Parked at the xy tolerance, 1-3 cm of AMCL jitter during the
 * goal-yaw pivot flips RPP between the goal-yaw and path-heading pivots
 * (Bug A, side_table_02).
 *
 * Here the latch is set once the pose is inside xy_goal_tolerance and kept
 * across reset() while the goal stays within goal_match_tolerance of the
 * latched one. While latched, isGoalReached checks yaw only, and
 * getTolerances reports xy_goal_tolerance + release_margin, so RPP keeps
 * pivoting to the goal yaw. Beyond that release radius the latch drops and
 * xy is required again. controller_server hands the goal over in the local
 * costmap frame (odom), so every AMCL correction moves the goal: during a
 * 120 deg pivot at update_min_a 0.1 they add up to ~5 cm (live, 2026-10-12).
 * goal_match_tolerance (0.15) only has to tell a new goal from the same one;
 * a real correction is caught by the release radius, since it moves the goal
 * relative to the robot. RPP compares the CARROT distance, which equals the goal
 * distance only below min_lookahead_dist: keep xy + release_margin under it.
 */
class StickyGoalChecker : public nav2_controller::SimpleGoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void reset() override;
  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist & velocity) override;
  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override;

  bool latched() const {return latched_;}

protected:
  void unlatch(const char * reason, double value);

  double release_margin_{0.04};
  double goal_match_tolerance_{0.15};
  double goal_match_yaw_tolerance_{0.1};
  bool latched_{false};
  geometry_msgs::msg::Pose latched_goal_;
  rclcpp::Logger logger_{rclcpp::get_logger("StickyGoalChecker")};
};

}  // namespace tinker_sim_nav_plugins

#endif  // TINKER_SIM_NAV_PLUGINS__STICKY_GOAL_CHECKER_HPP_
