cmake_minimum_required(VERSION 3.8)
project(forklift_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds maps urdf config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/generate_world.py scripts/cmd_vel_relay.py scripts/wheel_odom_stub.py
        DESTINATION lib/${PROJECT_NAME})

# Headless Gazebo integration tests (need ros_gz + an X display / Xvfb): only .github/workflows/sim.yml enables them
# with -DFORKLIFT_SIM_TESTS=ON, so the plain `colcon test` of ci.yml stays free of a running simulator.
option(FORKLIFT_SIM_TESTS "run the Gazebo twin launch tests" OFF)
if(BUILD_TESTING AND FORKLIFT_SIM_TESTS)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/launch/sim_sensors_drive.launch.py TARGET sim_sensors_drive TIMEOUT 600)
  add_launch_test(test/launch/sim_nav.launch.py TARGET sim_nav TIMEOUT 900)
endif()

ament_package()
