cmake_minimum_required(VERSION 3.8)
project(osm_robot_navigation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/route_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME route_planner
)

install(PROGRAMS
  scripts/location_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME location_gui
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
