cmake_minimum_required(VERSION 3.8)
project(osm_robot_navigation)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  src/route_planner_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME route_planner
)

install(PROGRAMS
  src/location_gui.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME location_gui
)

install(PROGRAMS
  src/path_follower_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME path_follower
)

install(DIRECTORY launch rviz urdf audibot_description/meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
