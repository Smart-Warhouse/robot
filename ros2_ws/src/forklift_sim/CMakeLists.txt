cmake_minimum_required(VERSION 3.8)
project(forklift_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_world.py DESTINATION lib/${PROJECT_NAME})

ament_package()
