cmake_minimum_required (VERSION 2.4.6)

include ($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include ($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set (ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_init ()
add_definitions (-Wall -O3 -pipe -funroll-loops)
rosbuild_add_boost_directories ()
rosbuild_check_for_sse ()

## add eigen functionality
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
include_directories (${CMAKE_CURRENT_BINARY_DIR})

rosbuild_add_executable (nu_skeletonviewer src/nu_skeletonviewer.cpp)
target_link_libraries (pcl_visualization)

rosbuild_add_compile_flags (nu_skeletonviewer ${SSE_FLAGS})


