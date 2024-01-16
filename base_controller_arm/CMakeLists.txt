cmake_minimum_required(VERSION 3.16.0)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ggdb") # ${CMAKE_CXX_FLAGS} global variable / -03 is to let the compiler optimize our code / ggdb generate debug information for gdb 
project(base_controller_arm)
add_compile_options(-std=c++11)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -ggdb") # ${CMAKE_CXX_FLAGS} global variable / -03 is to let the compiler optimize our code / ggdb generate debug information for gdb 
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  hardware_interface
  message_generation
  # controller_manager

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
  INCLUDE_DIRS 
  include
  LIBRARIES 
  ${PROJECT_NAME}
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  geometry_msgs
  hardware_interface
  # controller_manager
  DEPENDS Boost
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  thirdParty/common
  thirdParty/interface
  thirdParty/multiThread
  thirdParty/time
  thirdParty/ParamHandler
  /home/$ENV{USER}/mujoco-2.2.1/include
  thirdParty/kortex_api/include
  thirdParty/kortex_api/include/client
  thirdParty/kortex_api/include/client_stubs
  thirdParty/kortex_api/include/common
  thirdParty/kortex_api/include/google
  thirdParty/kortex_api/include/messages

)
link_directories(
  ${PROJECT_NAME}
  ${catkin_LIB_DIRS}
  # /home/$ENV{USER}/mujoco-2.2.1/lib
  thirdParty/kortex_api/lib/release
)
add_library(${PROJECT_NAME}
  src/hardware_interface_arm.cpp
  src/KortexRobot.cpp
  src/ArmModel.cpp
  thirdParty/interface/uart.cpp
  thirdParty/time/AbsoluteTimer.cpp
  thirdParty/multiThread/Loop.cpp
  thirdParty/kortex_api
  
)

list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")
find_package(PkgConfig)
pkg_search_module(PINO REQUIRED pinocchio)
if(PINO_FOUND)
  message("Pinocchio found")
  message("Pinocchio cflags: " ${PINO_CFLAGS})
  message("Pinocchio ldflags: " ${PINO_LDFLAGS})
  message("Pinocchio include dir at: " ${PINO_INCLUDE_DIRS})
  message("Pinocchio libraries dir at: " ${PINO_LIBRARY_DIRS})
  include_directories(SYSTEM ${PINO_INCLUDE_DIRS})
  link_directories(${PINO_LIBRARY_DIRS})
else()
  message(FATAL_ERROR "Cannot find pinocchio")
endif(PINO_FOUND)
# link pinocchio to the project

###########
## Build ##
###########
add_executable(base_controller_ src/base_controller.cpp)
target_link_libraries(base_controller_
    ${catkin_LIBRARIES}
    ${PROJECT_NAME}
    KortexApiCpp_without_protobuf
    KortexApiCpp
)
add_dependencies(base_controller_
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
