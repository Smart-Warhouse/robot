cmake_minimum_required(VERSION 3.8)
project(forklift_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf config launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/fork_joint_state_publisher.py DESTINATION lib/${PROJECT_NAME})

ament_package()
