cmake_minimum_required(VERSION 3.8)
project(robotik_pfadplannung)

# C++ Standard festlegen
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# LOADING DEPENDENCIES & PACKAGES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)


# GENERATING CUSTOM INTERFACES (Messages & Services)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GraphState.msg"
  "srv/SetDoorState.srv"
  "srv/GetPath.srv"
  DEPENDENCIES geometry_msgs
)

# Include folder for Header
include_directories(include)

# MAKE & BIND THE EXECUTABLES

# test_planner
add_executable(test_planner 
  src/test_planner.cpp
  src/a_star_solver.cpp
)
target_include_directories(test_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(test_planner
  rclcpp
  geometry_msgs
  visualization_msgs
)
rosidl_get_typesupport_target(test_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(test_planner "${test_target}")

install(TARGETS test_planner
  DESTINATION lib/${PROJECT_NAME}
)

# graph_provider (Braucht den Solver nicht, bleibt so!)
add_executable(graph_provider src/graph_provider.cpp)
ament_target_dependencies(graph_provider
  rclcpp
  geometry_msgs
  visualization_msgs
)
rosidl_get_typesupport_target(ch_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(graph_provider "${ch_target}")

install(TARGETS graph_provider
  DESTINATION lib/${PROJECT_NAME}
)

# path_planner
add_executable(path_planner 
  src/path_planner.cpp
  src/a_star_solver.cpp
)
ament_target_dependencies(path_planner
  rclcpp
  geometry_msgs
  visualization_msgs
  nav_msgs
)
rosidl_get_typesupport_target(cpp_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(path_planner "${cpp_target}")

install(TARGETS path_planner
  DESTINATION lib/${PROJECT_NAME}
)


# path_follower
add_executable(path_follower src/path_follower.cpp)
ament_target_dependencies(path_follower
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS path_follower
  DESTINATION lib/${PROJECT_NAME}
)

#find_start_pose
add_executable(find_start_pose src/find_start_pose.cpp)
ament_target_dependencies(find_start_pose
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav2_util
  sensor_msgs
  )
install(TARGETS find_start_pose
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
