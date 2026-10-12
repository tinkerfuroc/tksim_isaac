#pragma once
#include <cstdint>
#include <optional>
namespace tinker_sim_control {
class ClockSchedule {
 public:
  explicit ClockSchedule(std::int64_t period) : period_(period) {}
  std::optional<std::int64_t> poll(std::int64_t now) {
    if (!initialized_) {
      if (now <= 0) return std::nullopt;
      initialized_ = true; previous_ = now;
      return period_;
    }
    if (now < previous_) {
      previous_ = now;
      return period_;
    }
    const auto elapsed = now - previous_;
    if (elapsed < period_) return std::nullopt;
    previous_ = now;
    return elapsed;
  }
 private:
  std::int64_t period_;
  std::int64_t previous_{0};
  bool initialized_{false};
};
}  // namespace tinker_sim_control
