cmake_minimum_required(VERSION 3.10)
project(vehicle_core)

# ================================
#   C++ / compiler configuration
# ================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Conservative warnings (do NOT turn into errors to avoid breaking existing code)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# ================================
#   Shared headers (Message.h)
# ================================
set(SHARED_ROOT "" CACHE PATH "Path to Shared headers (contains Message.h)")
if(NOT SHARED_ROOT)
  set(_guess1 "${CMAKE_CURRENT_SOURCE_DIR}/../../../Shared")
  set(_guess2 "${CMAKE_CURRENT_SOURCE_DIR}/../../Shared")
  if(EXISTS "${_guess1}/Message.h")
    set(SHARED_ROOT "${_guess1}")
  elseif(EXISTS "${_guess2}/Message.h")
    set(SHARED_ROOT "${_guess2}")
  else()
    message(FATAL_ERROR "Could not find Message.h. Set -DSHARED_ROOT=/absolute/path/to/Shared")
  endif()
endif()

# ================================
#   Dependencies
# ================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(robot_localization REQUIRED)

# OpenCV is usually pulled by cv_bridge; make it optional to avoid breaking a working setup
find_package(OpenCV QUIET)

# GStreamer (appsink) via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gobject-2.0
  glib-2.0
)

# Make these includes visible to all targets that need them
include_directories(${GSTREAMER_INCLUDE_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS})

# ================================
#   Interfaces (msgs / srvs)
# ================================
# Keep your existing list exactly; DriveFeedback.msg included
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DriveTarget.msg"
  "msg/InternalDriveCommand.msg"
  "msg/ManualDriveCommand.msg"
  "msg/LightsCommand.msg"
  "msg/MaxSpeed.msg"
  "msg/AutopilotCommand.msg"
  "msg/LightsStatus.msg"
  "msg/GeneralStatus.msg"
  "msg/DriveStatus.msg"
  "msg/AutopilotStatus.msg"
  "msg/TelemetryData.msg"
  "msg/Error.msg"
  "msg/ServerInit.msg"
  "msg/AutopilotConfig.msg"
  "msg/PerceptionProfile.msg"
  "msg/StateMode.msg"
  "msg/StateEvent.msg"
  "msg/DriveFeedback.msg"
  "msg/IMUStatus.msg"
  "srv/RequestInternalControl.srv"
  "srv/ReleaseInternalControl.srv"
  "srv/SetAutopilotProfile.srv"
  DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# ================================
#   Install public headers
# ================================
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY data/ DESTINATION share/${PROJECT_NAME}/data)
ament_export_include_directories(include)

# Also export deps for downstream tools/containers
ament_export_dependencies(
  rclcpp rclcpp_components std_msgs std_srvs
  geometry_msgs nav_msgs sensor_msgs tf2
  tf2_ros
  cv_bridge
)
if(OpenCV_FOUND)
  ament_export_dependencies(OpenCV)
endif()

# ==========================================================
#                COMPONENT LIBRARIES (NODES)
# ==========================================================
# Helper macro to reduce repetition
function(register_component target_name cpp_file plugin_qualname)
  add_library(${target_name} SHARED ${cpp_file})
  target_include_directories(${target_name} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${SHARED_ROOT}
  )
  ament_target_dependencies(${target_name}
    rclcpp rclcpp_components std_msgs std_srvs
  )
  # Give access to generated message typesupport
  rosidl_target_interfaces(${target_name}
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
  rclcpp_components_register_nodes(${target_name} "${plugin_qualname}")
  install(TARGETS ${target_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endfunction()

# --- State Manager component ---
add_library(state_manager_node_component SHARED src/state_manager.cpp)
target_include_directories(state_manager_node_component PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SHARED_ROOT}
)
ament_target_dependencies(state_manager_node_component rclcpp rclcpp_components std_msgs std_srvs)
rosidl_target_interfaces(state_manager_node_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(state_manager_node_component "vehicle_core::StateManagerNode")
install(TARGETS state_manager_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- heading calibrator component ---
add_library(heading_calibrator_node_component SHARED src/heading_calibrator_node.cpp)
target_include_directories(heading_calibrator_node_component PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SHARED_ROOT}
)
ament_target_dependencies(heading_calibrator_node_component rclcpp rclcpp_components std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs robot_localization tf2)
rosidl_target_interfaces(heading_calibrator_node_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(heading_calibrator_node_component "vehicle_core::HeadingCalibratorNode")
install(TARGETS heading_calibrator_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- Drive Arbiter component ---
add_library(drive_arbiter_node_component SHARED src/drive_arbiter.cpp)
target_include_directories(drive_arbiter_node_component PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SHARED_ROOT}
)
ament_target_dependencies(drive_arbiter_node_component rclcpp rclcpp_components std_msgs std_srvs)
rosidl_target_interfaces(drive_arbiter_node_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(drive_arbiter_node_component "vehicle_core::DriveArbiterNode")
install(TARGETS drive_arbiter_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- GT-U7 GPS component ---
add_library(gt_u7_gps_node_component SHARED 
  src/gt_u7_gps_node.cpp
  src/Devices/GtU7Gps.cpp
)
target_include_directories(gt_u7_gps_node_component PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SHARED_ROOT}
)
ament_target_dependencies(gt_u7_gps_node_component
  rclcpp rclcpp_components sensor_msgs
)
# if your node uses your package's custom msgs, keep this line:
rosidl_target_interfaces(gt_u7_gps_node_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rclcpp_components_register_nodes(gt_u7_gps_node_component "vehicle_core::GtU7GpsNode")
install(TARGETS gt_u7_gps_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- TCP Server component ---
add_library(tcp_server_node_component SHARED src/tcp_server_node.cpp)
target_include_directories(tcp_server_node_component PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SHARED_ROOT}
)
ament_target_dependencies(tcp_server_node_component
  rclcpp rclcpp_components std_msgs std_srvs
)
rosidl_target_interfaces(tcp_server_node_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rclcpp_components_register_nodes(tcp_server_node_component "vehicle_core::TcpServerNode")
install(TARGETS tcp_server_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- I2C Bridge component ---
add_library(i2c_bridge_node SHARED 
  src/i2c_bridge_node.cpp
  src/Devices/I2C/I2CBus.cpp
  src/Devices/I2C/I2CBusRegistry.cpp
  src/Devices/I2C/I2CDevice.cpp
  src/Devices/BNO055.cpp
  src/Devices/DriveSubsys.cpp
  src/Devices/LightingSubsys.cpp
)
target_include_directories(i2c_bridge_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(i2c_bridge_node
  rclcpp rclcpp_components std_msgs sensor_msgs nav_msgs tf2
)
rosidl_target_interfaces(i2c_bridge_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rclcpp_components_register_nodes(i2c_bridge_node "vehicle_core::I2cBridgeNode")
install(TARGETS i2c_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- Autopilot component ---
add_library(autopilot_node_component SHARED 
  src/Autopilot/autopilot_node.cpp
  src/Autopilot/KmlPathLoader.cpp
  src/Autopilot/PurePursuit.cpp
)
target_include_directories(autopilot_node_component PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SHARED_ROOT}
)
ament_target_dependencies(autopilot_node_component rclcpp rclcpp_components std_msgs std_srvs geometry_msgs sensor_msgs nav_msgs robot_localization tf2)
rosidl_target_interfaces(autopilot_node_component ${PROJECT_NAME} "rosidl_typesupport_cpp")
rclcpp_components_register_nodes(autopilot_node_component "vehicle_core::AutopilotNode")
install(TARGETS autopilot_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
target_link_libraries(autopilot_node_component
  Geographic
)

# --- Ackermann wheel odometry component ---
add_library(ackermann_odometry_node_component SHARED
  src/ackermann_odometry_node.cpp
)

target_include_directories(ackermann_odometry_node_component PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${SHARED_ROOT}
)

ament_target_dependencies(ackermann_odometry_node_component
  rclcpp
  rclcpp_components
  geometry_msgs
  nav_msgs
  tf2_ros
)

rosidl_target_interfaces(
  ackermann_odometry_node_component
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

rclcpp_components_register_nodes(
  ackermann_odometry_node_component
  "vehicle_core::AckermannOdometryNode"
)

install(TARGETS ackermann_odometry_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- RTSP Streamer component ---
add_library(rtsp_streamer_node SHARED
  src/rtsp_streamer_node.cpp
)
target_include_directories(rtsp_streamer_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GSTREAMER_INCLUDE_DIRS}
  ${SHARED_ROOT}
)
target_link_libraries(rtsp_streamer_node
  ${GSTREAMER_LIBRARIES}
  gstrtspserver-1.0
)
ament_target_dependencies(rtsp_streamer_node
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_nodes(rtsp_streamer_node
  "vehicle_core::RtspStreamerNode"
)
install(TARGETS rtsp_streamer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# --- GST Camera component ---
add_library(gst_camera_component SHARED src/gst_camera_node.cpp)
target_include_directories(gst_camera_component PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GSTREAMER_INCLUDE_DIRS}
)
target_compile_options(gst_camera_component PRIVATE ${GSTREAMER_CFLAGS_OTHER})
ament_target_dependencies(gst_camera_component
  rclcpp rclcpp_components sensor_msgs cv_bridge
)
target_link_libraries(gst_camera_component ${GSTREAMER_LIBRARIES})
if(OpenCV_FOUND)
  target_link_libraries(gst_camera_component ${OpenCV_LIBS})
endif()
rosidl_target_interfaces(gst_camera_component
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rclcpp_components_register_nodes(gst_camera_component "vehicle_core::GstCameraNode")
install(TARGETS gst_camera_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ==========================================================
#           OPTIONAL: CUSTOM CONTAINER EXECUTABLE
# ==========================================================
option(BUILD_VEHICLE_CORE_CONTAINER "Build custom component container exe" ON)
if(BUILD_VEHICLE_CORE_CONTAINER)
  add_executable(vehicle_core_container src/vehicle_core_container.cpp)
  ament_target_dependencies(vehicle_core_container rclcpp rclcpp_components)
  install(TARGETS vehicle_core_container
    RUNTIME DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ================================
#   Install resources (cfg/launch)
# ================================
install(DIRECTORY cfg launch
  DESTINATION share/${PROJECT_NAME}
)

# ================================
#   ament package
# ================================
ament_package()
