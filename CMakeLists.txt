cmake_minimum_required(VERSION 2.8.3)
project(nir_adjust)

#--------------------------2020/8/8/Mkam--------------------------------------
option(COVERALLS "Turn on coveralls support" OFF)
option(COVERALLS_UPLOAD "Upload the generated coveralls json" ON)

if (COVERALLS)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} 
	${PROJECT_SOURCE_DIR}/coveralls-cmake/cmake)

endif()

if (COVERALLS)
	include(Coveralls)
	coveralls_turn_on_coverage()
endif()

#---------------------2020/8/8 Mkam--------------------------------
if (COVERALLS)
  set(LIB_SRC)
  file(GLOB_RECURSE SRCS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
  list(APPEND LIB_SRC ${SRCS})
  file(GLOB_RECURSE HDRS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp)
  list(APPEND LIB_SRC ${HDRS})
endif()
#-------------------------------------------------------------------


if (COVERALLS)
	coveralls_setup(
        "${LIB_SRC}" 
        ${COVERALLS_UPLOAD}                 
        "${PROJECT_SOURCE_DIR}/coveralls-cmake/cmake")
endif()




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

)

###########
## Build ##
###########


include_directories(
  src/
  ${catkin_INCLUDE_DIRS}
)

## -------r01_nir_adjust_node-----------##
add_executable(${PROJECT_NAME}_r01_nir_adjust_node src/r01_nir_adjust_main.cpp src/r01_nir_adjust.hpp src/pointcloud_adjust_base.hpp)
target_link_libraries(${PROJECT_NAME}_r01_nir_adjust_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME}_r01_nir_adjust_node ${PROJECT_NAME}_gencpp ${PCL_LIBRARIES})
## -------suture_point_adjust_node-----------##
add_executable(${PROJECT_NAME}_suture_point_adjust_node src/suture_point_adjust_main.cpp src/suture_point_adjust.hpp src/pointcloud_adjust_base.hpp)
target_link_libraries(${PROJECT_NAME}_suture_point_adjust_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(${PROJECT_NAME}_suture_point_adjust_node ${PROJECT_NAME}_gencpp ${PCL_LIBRARIES})

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


