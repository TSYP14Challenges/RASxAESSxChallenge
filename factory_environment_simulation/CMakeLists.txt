cmake_minimum_required(VERSION 3.8)
project(factory_environment_simulation)

find_package(ament_cmake REQUIRED)

# Optional ROS 2 resource installation; the standalone runner needs no build.
install(DIRECTORY launch worlds models licenses scripts step1 step2 step3
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
  PATTERN "*validation.json" EXCLUDE
  PATTERN "offline_*.json" EXCLUDE
  PATTERN "route_preview.*" EXCLUDE)
install(FILES README.md NOTICE.txt DESTINATION share/${PROJECT_NAME})
install(PROGRAMS run.sh DESTINATION share/${PROJECT_NAME})

ament_package()
