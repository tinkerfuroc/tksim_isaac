cmake_minimum_required(VERSION 3.16)
project(tinker_sim_nav_plugins)
find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)

set(deps angles geometry_msgs nav2_controller nav2_core nav2_util pluginlib rclcpp rclcpp_lifecycle tf2)

add_library(sticky_goal_checker SHARED src/sticky_goal_checker.cpp)
target_compile_features(sticky_goal_checker PUBLIC cxx_std_17)
target_include_directories(sticky_goal_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(sticky_goal_checker ${deps})
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS sticky_goal_checker
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sticky_goal_checker test/test_sticky_goal_checker.cpp)
  target_link_libraries(test_sticky_goal_checker sticky_goal_checker)
endif()
ament_package()
