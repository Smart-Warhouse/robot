cmake_minimum_required(VERSION 3.8)
project(forklift_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch systemd DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/forklift-start.sh DESTINATION lib/${PROJECT_NAME})

# Single source of truth for the lane graph: the fleet server's map (backend repo: app/warehouse_map.yaml) is
# installed as share/forklift_bringup/config/warehouse_map.yaml, so robot and server always plan on the same graph.
# Repos live side by side in one workspace folder (docs/ backend/ robot/ ai/ ...); override with SW_WORKSPACE.
if(DEFINED ENV{SW_WORKSPACE})
  set(SW_WORKSPACE "$ENV{SW_WORKSPACE}")
else()
  get_filename_component(SW_WORKSPACE "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
endif()
set(WAREHOUSE_MAP "${SW_WORKSPACE}/backend/app/warehouse_map.yaml")
if(EXISTS "${WAREHOUSE_MAP}")
  install(FILES "${WAREHOUSE_MAP}" DESTINATION share/${PROJECT_NAME}/config)
else()
  message(WARNING "warehouse_map.yaml not found at ${WAREHOUSE_MAP}; pass warehouse_map:=<file> at launch")
endif()

ament_package()
