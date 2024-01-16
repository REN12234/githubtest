cmake_minimum_required(VERSION 3.16.0)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ggdb") # ${CMAKE_CXX_FLAGS} global variable / -03 is to let the compiler optimize our code / ggdb generate debug information for gdb 
# add_compile_options(-std=c++11)
project(base_controller)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  hardware_interface
  message_generation
)
# add_message_files(
#   FILES
# )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
      roscpp 
      std_msgs 
      geometry_msgs
      hardware_interface
  DEPENDS Boost
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  Utils/common
  Utils/interface
  Utils/multiThread
  Utils/time
)

add_library(${PROJECT_NAME}
  src/hardware_interface.cpp
  Utils/interface/uart.cpp
  Utils/multiThread/Loop.cpp
  Utils/time/AbsoluteTimer.cpp
)

###########
## Build ##
###########

add_executable(base_controller_node src/base_controller.cpp)
target_link_libraries(base_controller_node
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
)
add_dependencies(base_controller_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
