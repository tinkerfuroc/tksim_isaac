#include <gtest/gtest.h>
#include "tinker_sim_control/clock_schedule.hpp"
using tinker_sim_control::ClockSchedule;
TEST(ClockSchedule, WaitsForClockAndDoesNotAdvancePausedControl) {
  ClockSchedule schedule(10);
  EXPECT_FALSE(schedule.poll(0));
  ASSERT_EQ(schedule.poll(100), 10);
  EXPECT_FALSE(schedule.poll(100));
  EXPECT_FALSE(schedule.poll(109));
  ASSERT_EQ(schedule.poll(110), 10);
}
TEST(ClockSchedule, RewindRebasesImmediatelyWithoutNegativePeriod) {
  ClockSchedule schedule(10);
  ASSERT_EQ(schedule.poll(10000), 10);
  ASSERT_EQ(schedule.poll(10030), 30);
  ASSERT_EQ(schedule.poll(100), 10);
  EXPECT_FALSE(schedule.poll(100));
  ASSERT_EQ(schedule.poll(110), 10);
}
TEST(ClockSchedule, RepeatedResetToZeroKeepsControlPaused) {
  ClockSchedule schedule(10);
  ASSERT_EQ(schedule.poll(1000), 10);
  ASSERT_EQ(schedule.poll(0), 10);
  EXPECT_FALSE(schedule.poll(0));
  ASSERT_EQ(schedule.poll(10), 10);
}
