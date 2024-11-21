cmake_minimum_required(VERSION 3.8)
project(beliv_vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tier4_api_msgs REQUIRED)
find_package(tier4_api_utils REQUIRED)
find_package(tier4_debug_msgs REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
# find_package(dbw_fca_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(dataspeed_ulc_msgs REQUIRED)
# find_package(dataspeed_dbw_common REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(autoware_vehicle_info_utils REQUIRED)
find_package(ds_dbw_msgs REQUIRED)
find_package(ds_dbw_can REQUIRED)
# find_package(ament_cmake REQUIRED)

# ament_auto_find_build_dependencies()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_executable(beliv_veh_interface
  src/beliv_veh_interface.cpp
  src/beliv_veh_interface_node.cpp
)

ament_target_dependencies(beliv_veh_interface
  "std_msgs"
  "geometry_msgs"
  "tier4_api_msgs"
  "tier4_api_utils"
  "tier4_debug_msgs"
  "tier4_external_api_msgs"
  "tier4_vehicle_msgs"
  # "dbw_fca_msgs"
  "sensor_msgs"
  # "dataspeed_ulc_msgs"
  # "dataspeed_dbw_common"
  "autoware_control_msgs"
  "autoware_vehicle_msgs"
  "can_msgs"
  "message_filters"
  "autoware_vehicle_info_utils"
  "ds_dbw_msgs"
  "ds_dbw_can"
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

install(
  TARGETS
    beliv_veh_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/sys_enable_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  src
  launch
)
