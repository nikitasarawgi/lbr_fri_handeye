cmake_minimum_required(VERSION 3.8)
project(auto-hand-eye-calib)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(DEFINED ENV{CONDA_PREFIX})
    message("CONDA_PREFIX is defined: $ENV{CONDA_PREFIX}")
else()
    message(FATAL_ERROR "CONDA_PREFIX is not defined. Please activate your conda environment.")
endif()

set(CONDA_ENV_LIB_PATH $ENV{CONDA_PREFIX}/lib)

# Set the PKG_CONFIG_PATH environment variable
set(ENV{PKG_CONFIG_PATH} "${CONDA_ENV_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

# set(CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
list(APPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})

set(MIN_EIGEN_VERSION 3.2.0)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(glog REQUIRED PATHS $ENV{CONDA_PREFIX})
find_package(Ceres REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(hand_eye_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)



# # Add external code
# add_subdirectory(solverAXXB)

# Include directories
include_directories(
  include
  ${CONDA_ENV_LIB_PATH}
  ${EIGEN_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add executables
add_executable(auto-hand-eye-calib 
  src/auto-hand-eye-calib-test.cpp
  src/auto-hand-eye-calib.cpp 
  src/utils/utils.cpp
  src/solverAXXB/solver_utils.cpp 
  src/solverAXXB/zhixy/conventionalaxxbsvdsolver.cc
  src/solverAXXB/zhixy/extendedaxxbelilambdasvdsolver.cc
  src/solverAXXB/zhixy/andreffextendedaxxbsolver.cc
  src/solverAXXB/tsaiAXXB/tsaiaxxb.cpp
  src/solverAXXB/camodocalDanii/camodocaldanii.cpp
  src/solverAXXB/camodocalDanii/camodocal/calib/HandEyeCalibration.cc
)

# Add link directories
link_directories(${CONDA_ENV_LIB_PATH})
# Set the LD_LIBRARY_PATH environment variable
set(ENV{LD_LIBRARY_PATH} ${CONDA_ENV_LIB_PATH}:$ENV{LD_LIBRARY_PATH})

# target_link_libraries(auto-hand-eye-calib ${CMAKE_INSTALL_PREFIX}/lib)
target_link_libraries(auto-hand-eye-calib
  rclcpp::rclcpp
  # solverAXXB
  ${EIGEN_LIBRARIES}
  ${OpenCV_LIBRARIES}
  # PRIVATE
  glog::glog
  ${CERES_LIBRARIES}
)

ament_target_dependencies(auto-hand-eye-calib 
  rclcpp
  glog
  Ceres
  moveit_msgs
  Eigen3
  geometry_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
  hand_eye_msgs
  OpenCV
  cv_bridge
)

install(TARGETS 
  auto-hand-eye-calib
  DESTINATION lib/${PROJECT_NAME}
)

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
