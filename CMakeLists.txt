cmake_minimum_required(VERSION 2.8.3)
project(dyret_hardware)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -std=c++11 -march=native" )

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  dyret_common
  dyret_utils
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(dynamixel_server src/dynamixel_server.cpp)
target_link_libraries(dynamixel_server ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/lib/libdxl_x64_cpp.so)
add_dependencies(dynamixel_server ${catkin_EXPORTED_TARGETS})
