cmake_minimum_required(VERSION 3.8)
project(cam_lidar_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-error")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(rclpy REQUIRED)
option(BUILD_RVIZ_PANEL "Build the optional RViz calibration panel" ON)
if(BUILD_RVIZ_PANEL)
  find_package(rviz_common REQUIRED)
  find_package(rviz_default_plugins REQUIRED)
  find_package(Qt5 REQUIRED Core Widgets)
endif()
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

## Setup include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(-DQT_NO_KEYWORDS)
set(CMAKE_AUTOMOC ${BUILD_RVIZ_PANEL})

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RunOptimise.action"
  "srv/Optimise.srv"
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

# Get typesupport target name for the generated interfaces
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  cv_bridge
  image_transport
  geometry_msgs
  pcl_conversions
  sensor_msgs
  std_msgs
  message_filters
  tf2
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  ament_index_cpp
  visualization_msgs
)

# Library
add_library(cam_lidar_calibration_core SHARED
  src/feature_extractor.cpp
  src/load_params.cpp
  src/optimiser.cpp
)
target_link_libraries(cam_lidar_calibration_core
  ${OpenCV_LIBS}
  ${Eigen3_LIBRARIES}
  ${PCL_LIBRARIES}
)
ament_target_dependencies(cam_lidar_calibration_core ${dependencies})
target_link_libraries(cam_lidar_calibration_core "${cpp_typesupport_target}")

# Feature extraction node
add_executable(feature_extraction_node src/feature_extraction_node.cpp)
target_link_libraries(feature_extraction_node
  cam_lidar_calibration_core
  ${OpenCV_LIBS}
  ${Eigen3_LIBRARIES}
)
ament_target_dependencies(feature_extraction_node ${dependencies})
target_link_libraries(feature_extraction_node "${cpp_typesupport_target}")

# Assessment node
add_executable(assess_node src/assess_calibration.cpp)
target_link_libraries(assess_node
  cam_lidar_calibration_core
  ${OpenCV_LIBS}
  ${Eigen3_LIBRARIES}
)
ament_target_dependencies(assess_node ${dependencies})
target_link_libraries(assess_node "${cpp_typesupport_target}")

if(BUILD_RVIZ_PANEL)
# RViz panel library (component)
add_library(cam_lidar_panel SHARED
  src/cam_lidar_panel.cpp
)
target_link_libraries(cam_lidar_panel
  cam_lidar_calibration_core
  Qt5::Widgets
)
ament_target_dependencies(cam_lidar_panel ${dependencies} rviz_common rviz_default_plugins)
target_link_libraries(cam_lidar_panel "${cpp_typesupport_target}")

  install(TARGETS cam_lidar_panel LIBRARY DESTINATION lib)
  pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
endif()

# Install
install(TARGETS
  feature_extraction_node
  assess_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  cam_lidar_calibration_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY data/
  DESTINATION share/${PROJECT_NAME}/data
)

install(PROGRAMS
  scripts/publish_camera_info.py
  scripts/assess_jazzy.py
  scripts/decompress_image.py
  scripts/calibrate_bag.py
  scripts/publish_calibration.py
  scripts/calibration_math.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  enable_testing()
  add_executable(normal_diversity_test tests/normal_diversity_test.cpp)
  target_link_libraries(normal_diversity_test ${OpenCV_LIBS})
  add_test(NAME normal_diversity COMMAND normal_diversity_test)
endif()

ament_export_include_directories(include)
ament_export_libraries(cam_lidar_calibration_core)
ament_export_dependencies(${dependencies})
ament_package()
