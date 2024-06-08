cmake_minimum_required(VERSION 3.8)
project(charuco_calib_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(hand_eye_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(charuco_calib_server
  src/charuco-calib-server.cpp
  src/charuco-calibrator.cpp
  src/utils/utils.cpp 
)

#Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

ament_target_dependencies(charuco_calib_server
  rclcpp
  geometry_msgs
  sensor_msgs
  OpenCV
  cv_bridge
  tf2
  hand_eye_msgs
)

target_link_libraries(charuco_calib_server
  ${OpenCV_LIBRARIES}
)

install(TARGETS charuco_calib_server
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
