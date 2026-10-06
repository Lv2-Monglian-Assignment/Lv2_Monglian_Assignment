cmake_minimum_required(VERSION 3.8)
project(tracker_bringup)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
# The module config is the single editable source; install it for ROS launch lookup.
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../../../config/hsv.yaml"
        DESTINATION share/${PROJECT_NAME}/config)
ament_package()
