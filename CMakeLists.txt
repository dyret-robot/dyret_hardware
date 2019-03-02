cmake_minimum_required(VERSION 3.1)
project(dyret_hardware)

# This is allowed as of Melodic
set(CMAKE_CXX_STANDARD 14)
# Set warning flags to get prettiest and good C++
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror -Wno-unused -Wold-style-cast")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror -Wno-unused")

find_package(catkin REQUIRED COMPONENTS 
  message_generation
  roscpp 
  std_msgs 
  dyret_common
)

add_message_files(
  DIRECTORY msg
  FILES
  ActuatorBoardCommand.msg
  ActuatorBoardState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  LIBRARIES dynamixel_wrapper 
)

include_directories(
  SYSTEM
  include/dynamixel_sdk
  ${catkin_INCLUDE_DIRS}
)

include_directories(include)

add_executable(hardware_manager src/hardware_manager.cpp
    src/dynamixel_wrapper.cpp
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
add_dependencies(hardware_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(hardware_manager dynamixel_wrapper ${catkin_LIBRARIES})


add_library(dynamixel_wrapper src/dynamixel_wrapper.cpp) 
add_dependencies(dynamixel_wrapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(dynamixel_wrapper ${catkin_LIBRARIES})

add_executable(tmpMon src/tmpMon.cpp)
add_dependencies(tmpMon ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tmpMon ${catkin_LIBRARIES})

