cmake_minimum_required(VERSION 3.0.2)
project(hw1_BasicSearchAlgo)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Debug")		# change it to "Release" when needed

################################################
## Find all ROS dependencies ##
################################################
find_package (Eigen3 3.3.0 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
    geometry_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  	FILES
	graphSearch.srv
)

#add_message_files(
#	FILES
#
#)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

###################################
## Catkin specific configuration ##
###################################

catkin_package(
#    INCLUDE_DIRS 
#    LIBRARIES 
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime 
#    DEPENDS
)

###########
## Build ##
###########

include_directories(
	include
    ${catkin_INCLUDE_DIRS}
)


########## Building search algo node exe ##########
file(GLOB SOURCES "src/*.cpp")
#set(SOURCES
#		src/main.cpp
#		src/BFS.cpp	
#)
message(8888888886555555555////////////-------**********55--- ${SOURCES})
add_executable(hw1_basic_search_algo ${SOURCES})
#add_dependencies(hw1_basic_search_algo HW1_BasicSearchAlgo_generate_messages_cpp)
target_link_libraries(hw1_basic_search_algo ${catkin_LIBRARIES} Eigen3::Eigen)


#############
## Testing ##
#############

#############
## Install ##
#############
