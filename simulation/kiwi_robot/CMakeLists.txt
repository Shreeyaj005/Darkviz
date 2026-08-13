cmake_minimum_required(VERSION 3.8)
project(kiwi_robot)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  config
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/autonomous_controller.py
    scripts/navigation_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/omni_drive_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME omni_drive_node
)
ament_package()
