cmake_minimum_required(VERSION 3.8)
project(online_line_finder)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(robotik_pfadplannung REQUIRED)


add_executable(online_line_finder src/online_line_finder.cpp)


ament_target_dependencies(online_line_finder
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  robotik_pfadplannung
)



install(TARGETS online_line_finder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
