cmake_minimum_required(VERSION 3.16)
project(tinker_sim_control)
find_package(ament_cmake REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)

add_executable(controller_manager_node src/controller_manager_node.cpp)
target_compile_features(controller_manager_node PRIVATE cxx_std_17)
target_include_directories(controller_manager_node PRIVATE include)
ament_target_dependencies(controller_manager_node controller_manager rclcpp realtime_tools)
install(TARGETS controller_manager_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_clock_schedule test/test_clock_schedule.cpp)
  target_compile_features(test_clock_schedule PRIVATE cxx_std_17)
  target_include_directories(test_clock_schedule PRIVATE include)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(clock_recovery test/test_clock_recovery.py TIMEOUT 90
    ENV CONTROLLER_MANAGER_EXECUTABLE=$<TARGET_FILE:controller_manager_node>
        ROS_DOMAIN_ID=173 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1)
endif()
ament_package()
