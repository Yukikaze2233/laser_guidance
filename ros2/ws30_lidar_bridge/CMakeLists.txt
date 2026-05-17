cmake_minimum_required(VERSION 3.16)
project(ws30_lidar_bridge)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Threads REQUIRED)

# ---- Locate the standalone WS30 core library -----------------------------------
set(CORE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../..")
set(CORE_INCLUDE_DIRS
  "${CORE_ROOT}/external/ws30_lidar_core/include")
find_library(WS30_LIDAR_CORE_LIB
  ws30_lidar_core
  PATHS
    "${CORE_ROOT}/external/ws30_lidar_core/build"
    "${CORE_ROOT}/build/external/ws30_lidar_core"
  NO_DEFAULT_PATH)
if(NOT WS30_LIDAR_CORE_LIB)
  find_library(WS30_LIDAR_CORE_LIB ws30_lidar_core)
endif()
if(NOT WS30_LIDAR_CORE_LIB)
  message(FATAL_ERROR "ws30_lidar_core library not found. Build it first: cmake -S external/ws30_lidar_core -B external/ws30_lidar_core/build -DCMAKE_BUILD_TYPE=Release && cmake --build external/ws30_lidar_core/build")
endif()

message(STATUS "WS30 lidar core library: ${WS30_LIDAR_CORE_LIB}")
message(STATUS "WS30 lidar core include: ${CORE_INCLUDE_DIRS}")

# ---- Bridge node ----------------------------------------------------------------
add_executable(ws30_lidar_node
  src/ws30_lidar_node.cpp)
target_include_directories(ws30_lidar_node PRIVATE ${CORE_INCLUDE_DIRS})
target_link_libraries(ws30_lidar_node
  ${WS30_LIDAR_CORE_LIB}
  Threads::Threads
  rt)
target_compile_definitions(ws30_lidar_node PRIVATE
  RMCS_LASER_GUIDANCE_DEFAULT_CONFIG_PATH="${CORE_ROOT}/config/default.yaml")
ament_target_dependencies(ws30_lidar_node
  rclcpp sensor_msgs diagnostic_msgs std_srvs)

# ---- Install --------------------------------------------------------------------
install(TARGETS ws30_lidar_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
