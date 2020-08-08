cmake_minimum_required(VERSION 2.8.3)
project(nir_adjust)

#--------------------------2020/8/8/Mkam--------------------------------------

#set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Coveralls support
#option(ROS_CI_COVERALLS "Turn on coveralls support" OFF)
#option(ROS_CI_COVERALLS_UPLOAD "Upload the generated coveralls json" OFF)

#if($ENV{ROS_CI_COVERAGE})
#  set(CMAKE_BUILD_TYPE Debug)
#  set(ROS_CI_COVERALLS ON)
#  set(ROS_CI_COVERALLS_UPLOAD ON)
#endif()

#if(ROS_CI_COVERALLS)
#  include(Coveralls)
#  coveralls_turn_on_coverage()
#  message(STATUS "coveralls on")
#endif()

#------------------------------------------------------------------------------

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs 
  roscpp rospy 
  sensor_msgs 
  std_msgs 
  pcl_ros 
  roslib 
  pcl_conversions 
  dynamic_reconfigure 
  message_generation
)


 generate_messages(
   DEPENDENCIES
   geometry_msgs
   sensor_msgs
   std_msgs
 )



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nir_adjust
#  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  src/
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp src/nir_adjust.hpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencpp ${PCL_LIBRARIES} ${PROJECT_NAME}_gencfg)


#############
## Tests ##
#############


if (CATKIN_ENABLE_TESTING)
 find_package(rostest REQUIRED)
 find_package(PCL 1.2 REQUIRED)
 include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
 link_directories(${PCL_LIBRARY_DIRS})
 add_definitions(${PCL_DEFINITIONS})

 catkin_add_gtest(
   test_nir_adjust
   test/nir_adjust_test.cpp
 )
target_link_libraries(test_nir_adjust ${catkin_LIBRARIES} ${PCL_LIBRARIES})
endif()

#---------------------2020/8/8 Mkam--------------------------------
#if (ROS_CI_COVERALLS)
#  set(COV_FILES)
#  file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
#  list(APPEND COV_FILES ${SRCS})
#  file(GLOB_RECURSE HDRS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp)
#  list(APPEND COV_FILES ${HDRS})
#  coveralls_setup("${COV_FILES}" ${ROS_CI_COVERALLS_UPLOAD})
#endif()
#-------------------------------------------------------------------
