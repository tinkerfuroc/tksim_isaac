#include <cmath>
#include <memory>

#include "gtest/gtest.h"
#include "nav2_core/goal_checker.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tinker_sim_nav_plugins/sticky_goal_checker.hpp"

using geometry_msgs::msg::Pose;
using geometry_msgs::msg::Twist;
using tinker_sim_nav_plugins::StickyGoalChecker;

namespace
{

Pose pose(double x, double y, double yaw)
{
  Pose p;
  p.position.x = x;
  p.position.y = y;
  p.orientation = nav2_util::geometry_utils::orientationAroundZAxis(yaw);
  return p;
}

// xy 0.2, yaw 0.25, release margin 0.04: the sim overlay's values.
class StickyGoalCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("sticky_goal_checker_test");
    node_->declare_parameter("gc.xy_goal_tolerance", 0.2);
    node_->declare_parameter("gc.yaw_goal_tolerance", 0.25);
    node_->declare_parameter("gc.release_margin", 0.04);
    checker_.initialize(node_, "gc", nullptr);
  }

  double reported_xy()
  {
    Pose tol;
    Twist vel;
    EXPECT_TRUE(checker_.getTolerances(tol, vel));
    return tol.position.x;
  }

  bool reached(const Pose & robot) {return checker_.isGoalReached(robot, goal_, Twist());}

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  StickyGoalChecker checker_;
  Pose goal_ = pose(1.17, 1.75, M_PI);
};

}  // namespace

TEST_F(StickyGoalCheckerTest, OutsideTheToleranceIsNotReachedAndNotLatched)
{
  EXPECT_FALSE(reached(pose(1.17, 1.50, M_PI)));  // 0.25 m
  EXPECT_FALSE(checker_.latched());
  EXPECT_DOUBLE_EQ(reported_xy(), 0.2);
}

TEST_F(StickyGoalCheckerTest, InsideLatchesAndWidensTheReportedTolerance)
{
  EXPECT_FALSE(reached(pose(1.17, 1.56, 0.0)));  // 0.19 m, yaw off
  EXPECT_TRUE(checker_.latched());
  EXPECT_DOUBLE_EQ(reported_xy(), 0.24);
}

TEST_F(StickyGoalCheckerTest, LatchSurvivesAReplanOfTheSameGoal)
{
  // Bug A: AMCL jitter during the goal-yaw pivot puts the estimate 1-2 cm
  // outside the tolerance right after a replan reset the stock latch.
  reached(pose(1.17, 1.56, 0.0));
  checker_.reset();
  EXPECT_TRUE(reached(pose(1.17, 1.53, M_PI - 0.1)));  // 0.22 m, yaw within
  EXPECT_TRUE(checker_.latched());
}

TEST_F(StickyGoalCheckerTest, ReplanEndingAFewCentimetresAwayKeepsTheLatch)
{
  reached(pose(1.17, 1.56, 0.0));
  checker_.reset();
  goal_ = pose(1.20, 1.75, M_PI);  // planner ended one cell over
  EXPECT_TRUE(reached(pose(1.17, 1.56, M_PI)));
  EXPECT_TRUE(checker_.latched());
}

TEST_F(StickyGoalCheckerTest, AmclCorrectionsAddingUpDuringThePivotKeepTheLatch)
{
  // The goal arrives in odom, so AMCL corrections during a pivot move it:
  // ~5 cm over a 120 deg pivot at update_min_a 0.1 (live). Still the same goal.
  reached(pose(1.17, 1.56, 0.0));
  goal_ = pose(1.17, 1.67, M_PI);  // moved 8 cm; robot 0.11 m from it
  EXPECT_TRUE(reached(pose(1.17, 1.56, M_PI)));
  EXPECT_TRUE(checker_.latched());
}

TEST_F(StickyGoalCheckerTest, ARealCorrectionBeyondTheReleaseRadiusRequiresXyAgain)
{
  // A >0.24 m localization correction while latched: not reached, latch
  // dropped, and RPP is back on the 0.2 m tolerance, so it drives back
  // instead of pivoting 0.3 m off.
  reached(pose(1.17, 1.56, 0.0));
  EXPECT_FALSE(reached(pose(1.17, 1.45, M_PI)));  // 0.30 m, yaw within
  EXPECT_FALSE(checker_.latched());
  EXPECT_DOUBLE_EQ(reported_xy(), 0.2);
  EXPECT_TRUE(reached(pose(1.17, 1.60, M_PI)));  // back inside
}

TEST_F(StickyGoalCheckerTest, ANewGoalClearsTheLatch)
{
  reached(pose(1.17, 1.56, 0.0));
  checker_.reset();
  goal_ = pose(1.17, 1.40, M_PI);  // 0.35 m away from the latched goal
  EXPECT_TRUE(reached(pose(1.17, 1.56, M_PI)));  // 0.16 m: latched afresh on the new goal
  EXPECT_TRUE(checker_.latched());
  goal_ = pose(3.0, 3.0, M_PI);
  EXPECT_FALSE(reached(pose(1.17, 1.56, M_PI)));
  EXPECT_FALSE(checker_.latched());
  EXPECT_DOUBLE_EQ(reported_xy(), 0.2);
}

TEST_F(StickyGoalCheckerTest, ANewGoalHeadingClearsTheLatch)
{
  reached(pose(1.17, 1.56, 0.0));
  goal_ = pose(1.17, 1.75, M_PI / 2);
  reached(pose(1.17, 1.50, M_PI / 2));  // 0.25 m: outside xy for the new goal
  EXPECT_FALSE(checker_.latched());
}

TEST_F(StickyGoalCheckerTest, NotStatefulMeansNoLatch)
{
  node_->set_parameter(rclcpp::Parameter("gc.stateful", false));
  EXPECT_FALSE(reached(pose(1.17, 1.56, 0.0)));
  EXPECT_FALSE(checker_.latched());
  EXPECT_DOUBLE_EQ(reported_xy(), 0.2);
}

TEST(StickyGoalCheckerPlugin, LoadsThroughPluginlib)
{
  pluginlib::ClassLoader<nav2_core::GoalChecker> loader("nav2_core", "nav2_core::GoalChecker");
  auto checker = loader.createSharedInstance("tinker_sim_nav_plugins::StickyGoalChecker");
  EXPECT_NE(checker, nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
