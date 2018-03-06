cmake_minimum_required(VERSION 2.8.3)
project(nav2d_registration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pluginlib
  geometry_msgs
  nav_msgs
  visualization_msgs
  message_generation
)

add_service_files(
  FILES
  RegService.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)
###################################
## catkin specific configuration ##
###################################

catkin_package(
  LIBRARIES Registration
  CATKIN_DEPENDS
    roscpp
	tf
	pluginlib
	geometry_msgs
	nav_msgs
	visualization_msgs
	message_runtime
	INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}/
  ${catkin_INCLUDE_DIRS}
)
add_library(Registration
  src/registration_common.cpp)

add_dependencies(Registration ${catkin_EXPORTED_TARGETS})

target_link_libraries(Registration ${catkin_LIBRARIES})


#############
## Install ##
#############

install(TARGETS Registration
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nav2d_registration.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
