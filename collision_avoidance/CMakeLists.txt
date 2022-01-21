cmake_minimum_required(VERSION 3.0.2)
project(collision_avoidance)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  laser_geometry
)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )


catkin_package(
 # INCLUDE_DIRS include
 # LIBRARIES collision_avoidance
  CATKIN_DEPENDS roscpp std_msgs
 # DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)




add_executable(collision_avoider src/collision_avoider.cpp)
target_link_libraries(collision_avoider ${catkin_LIBRARIES})
add_dependencies(collision_avoider collision_avoidance_generate_messages_cpp)