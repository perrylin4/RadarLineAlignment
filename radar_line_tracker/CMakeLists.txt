cmake_minimum_required(VERSION 3.8)
project(radar_line_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

set(MVS_ROOT /opt/MVS)
include_directories(${MVS_ROOT}/include)
link_directories(${MVS_ROOT}/lib/64)

add_executable(line_tracker_node src/line_tracker_node.cpp)

target_include_directories(line_tracker_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

set_target_properties(line_tracker_node PROPERTIES
  BUILD_RPATH "${MVS_ROOT}/lib/64"
  INSTALL_RPATH "${MVS_ROOT}/lib/64"
)

ament_target_dependencies(line_tracker_node
  rclcpp
  sensor_msgs
  geometry_msgs
)

target_link_libraries(line_tracker_node
  ${OpenCV_LIBS}
  MvCameraControl
)

install(TARGETS line_tracker_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
