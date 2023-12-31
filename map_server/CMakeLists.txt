cmake_minimum_required(VERSION 3.8)
project(map_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Add include directories
include_directories(include)

#add_executable(pre_approach_v2_node src/pre_approach_v2.cpp)
#ament_target_dependencies(pre_approach_v2_node rclcpp sensor_msgs geometry_msgs nav_msgs)
#rosidl_target_interfaces(pre_approach_v2_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install the executable
install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch params files
install(DIRECTORY
  launch
  config
  rviz
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
