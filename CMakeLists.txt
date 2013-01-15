cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

add_definitions("-DNOT_BULLETSIM")
add_definitions("-DEIGEN_DEFAULT_TO_ROW_MAJOR")
add_definitions("-DHDFUTIL_USE_BOOST")

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/interactive_segmentation.cpp src/utils_pcl.cpp src/cloud_ops.cpp src/alignment.cpp src/table.cpp src/segmentation.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(test_seg src/test_interactive_segmentation.cpp)
target_link_libraries(test_seg ${PROJECT_NAME})

rosbuild_add_executable(interactive_segmentation_service src/interactive_segmentation_service.cpp)
target_link_libraries(interactive_segmentation_service ${PROJECT_NAME})

include_directories("/usr/include/vtk-5.6")

rosbuild_add_executable(test_int_seg src/test_interactive_segmentation.cpp)
target_link_libraries(test_int_seg ${PROJECT_NAME})


rosbuild_add_executable(test_segmentation src/test_segmentation.cpp)
target_link_libraries(test_segmentation ${PROJECT_NAME} vtkCommon vtkFiltering)

rosbuild_add_executable(test_align_mugs src/test_align_mugs.cpp)
target_link_libraries(test_align_mugs ${PROJECT_NAME} hdfutil ${PCL_LIBRARIES} vtkCommon vtkFiltering)
 
add_subdirectory(hdfutil)