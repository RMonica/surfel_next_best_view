cmake_minimum_required(VERSION 2.8.3)
project(surfel_next_best_view)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  roscpp
  sensor_msgs
  actionlib
  std_msgs
  geometry_msgs
  actionlib_msgs
  init_fake_opengl_context
  surfel_next_best_view_msgs
  pluginlib
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES surfel_next_best_view
#  CATKIN_DEPENDS eigen_conversions roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  )

link_directories(${catkin_LIBRARY_DIRS})

add_executable(surfel_next_best_view 
               src/surfel_next_best_view_node.cpp
               src/surfel_renderer.cpp
               src/shaders/surfel_vertex_shader.cpp
               src/shaders/surfel_fragment_shader.cpp
	       )

add_dependencies(surfel_next_best_view ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(surfel_next_best_view
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
  )

add_executable(surfel_next_best_view_test src/surfel_test.cpp)

add_dependencies(surfel_next_best_view_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(surfel_next_best_view_test 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
  )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_surfel_next_best_view.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
