cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(pcl_hello)
find_package(PCL 1.8 REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_to_pcl pcd_to_pcl.cpp)
add_executable(featureExtract featureExtract.cpp)

target_link_libraries(pcd_to_pcl ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(featureExtract ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})