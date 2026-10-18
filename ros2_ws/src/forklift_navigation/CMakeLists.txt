cmake_minimum_required(VERSION 3.8)
project(forklift_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps behavior_trees DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/make_filter_masks.py DESTINATION lib/${PROJECT_NAME})

ament_package()
