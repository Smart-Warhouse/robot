cmake_minimum_required(VERSION 3.8)
project(forklift_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch systemd DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/forklift-start.sh DESTINATION lib/${PROJECT_NAME})

# Single source of truth for the lane graph: the fleet server's map (code/backend/app/warehouse_map.yaml) is
# installed as share/forklift_bringup/config/warehouse_map.yaml, so robot and server always plan on the same graph.
set(WAREHOUSE_MAP "${CMAKE_CURRENT_SOURCE_DIR}/../../../backend/app/warehouse_map.yaml")
if(EXISTS "${WAREHOUSE_MAP}")
  install(FILES "${WAREHOUSE_MAP}" DESTINATION share/${PROJECT_NAME}/config)
else()
  message(WARNING "warehouse_map.yaml not found at ${WAREHOUSE_MAP}; pass warehouse_map:=<file> at launch")
endif()

ament_package()
