cmake_minimum_required(VERSION 3.8)
project(voronoi_planner_v)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# 1. Alle benötigten Pakete finden
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 2. Das Executable erstellen
add_executable(voronoi_planner_node src/voronoi_planner.cpp)

# 3. Alle Abhängigkeiten (inklusive OpenCV) sauber verknüpfen
ament_target_dependencies(voronoi_planner_node 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  visualization_msgs
  OpenCV
)

# 4. Installation konfigurieren
install(TARGETS voronoi_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch maps
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
