cmake_minimum_required(VERSION 3.8)
project(nav2_apps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)


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


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
  DEPENDENCIES std_msgs  # Add it when you use other dependency in the file 
)

# Add include directories
include_directories(include)

add_executable(approach_service_server_node src/approach_service_server.cpp)
ament_target_dependencies(approach_service_server_node rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2_ros tf2)
rosidl_target_interfaces(approach_service_server_node ${PROJECT_NAME} "rosidl_typesupport_cpp")


# Install the executable
install(TARGETS
  approach_service_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/move_shelf_to_ship.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch params files
install(DIRECTORY
  launch
  src
  srv
  scripts
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
