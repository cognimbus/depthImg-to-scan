cmake_minimum_required(VERSION 2.8.3)
project(depth_to_scan)

add_compile_options(-std=c++14)

set(CMAKE_CXX_FLAGS "-ffunction-sections -fdata-sections")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "-Wl,--as-needed,--gc-sections")

find_package(OpenCV  REQUIRED )

find_package(catkin REQUIRED COMPONENTS
  
  roscpp  
  std_msgs
  sensor_msgs
  cv_bridge  
  image_transport
  tf
  tf2
  depth_image_proc
  image_geometry
  pcl_ros

)



catkin_package()


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${OpenCV_INCLUDE_DIRS}

)


# dpeth_to_grayscale

add_executable(depth_to_scan_node
  src/depth_to_scan/depth_to_scan_node.cpp

)

# add_dependencies(depth_to_grayscale_node
#  {${PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS}
# )

target_link_libraries(depth_to_scan_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
