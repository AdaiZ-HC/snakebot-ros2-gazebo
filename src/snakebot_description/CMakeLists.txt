cmake_minimum_required(VERSION 3.20)
project(snakebot_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY urdf config launch worlds tools
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/snake_behavior.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/snake_behavior.py
  DESTINATION share/${PROJECT_NAME}/scripts)
install(FILES scripts/behavior_core.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/behavior_core.py DESTINATION share/${PROJECT_NAME}/scripts)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
