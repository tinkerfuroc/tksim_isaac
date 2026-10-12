// Standard ROS controller manager with native-owned clock scheduling.
#include <chrono>
#include <memory>
#include <thread>
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_helpers.hpp"
#include "tinker_sim_control/clock_schedule.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto manager = std::make_shared<controller_manager::ControllerManager>(
      executor, "controller_manager");
  const auto rate = manager->get_update_rate();
  if (rate == 0) {
    RCLCPP_ERROR(manager->get_logger(), "Controller update rate must be positive");
    rclcpp::shutdown();
    return 1;
  }
  const auto affinity = manager->get_parameter_or<int>("cpu_affinity", -1);
  if (affinity >= 0) {
    const auto result = realtime_tools::set_current_thread_affinity(affinity);
    if (!result.first) RCLCPP_WARN(manager->get_logger(), "%s", result.second.c_str());
  }
  if (manager->get_parameter_or<bool>("lock_memory", realtime_tools::has_realtime_kernel())) {
    const auto result = realtime_tools::lock_memory();
    if (!result.first) RCLCPP_WARN(manager->get_logger(), "%s", result.second.c_str());
  }
  const auto priority = manager->get_parameter_or<int>("thread_priority", 50);
  std::thread control([manager, rate, priority] {
    if (realtime_tools::has_realtime_kernel() &&
        !realtime_tools::configure_sched_fifo(priority)) {
      RCLCPP_WARN(manager->get_logger(), "Could not enable FIFO controller scheduling");
    }
    const auto period = std::chrono::nanoseconds(1000000000LL / rate);
    tinker_sim_control::ClockSchedule schedule(period.count());
    auto next = std::chrono::steady_clock::now();
    while (rclcpp::ok()) {
      const auto now = manager->now();
      const auto elapsed = schedule.poll(now.nanoseconds());
      if (elapsed) {
        const auto dt = rclcpp::Duration::from_nanoseconds(*elapsed);
        manager->read(now, dt);
        manager->update(now, dt);
        manager->write(now, dt);
      }
      // Detect rewind within one wall update interval; paused time does not
      // advance the controller. Preserve positive elapsed simulation periods.
      next += period;
      const auto wall = std::chrono::steady_clock::now();
      if (next < wall) next = wall + period;
      std::this_thread::sleep_until(next);
    }
  });
  executor->add_node(manager);
  executor->spin();
  rclcpp::shutdown();
  control.join();
  return 0;
}
