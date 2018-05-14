cmake_minimum_required(VERSION 2.8.3)
project(dyret_hardware)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -std=c++11 -march=native" )

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  std_msgs 
  dyret_common
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include/dynamixel_sdk
  ${catkin_INCLUDE_DIRS}
)


add_executable(dynamixel_server src/dynamixel_server.cpp
    src/dynamixel_sdk/packet_handler.cpp
    src/dynamixel_sdk/protocol1_packet_handler.cpp
    src/dynamixel_sdk/protocol2_packet_handler.cpp
    src/dynamixel_sdk/group_sync_read.cpp
    src/dynamixel_sdk/group_sync_write.cpp
    src/dynamixel_sdk/group_bulk_read.cpp
    src/dynamixel_sdk/group_bulk_write.cpp
    src/dynamixel_sdk/port_handler.cpp
    src/dynamixel_sdk/port_handler_linux.cpp    
)


add_dependencies(dynamixel_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_server ${catkin_LIBRARIES})

install(TARGETS dynamixel_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
