cmake_minimum_required(VERSION 2.8.3)
project(simple_bgc)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp geometry_msgs serial dynamic_reconfigure
)

## Clone related code from git
include(ExternalProject)
ExternalProject_Add(sbgc-api
  GIT_REPOSITORY https://github.com/tongtybj/sbgc-api-examples.git
  PREFIX sbgc-api
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
  BUILD_IN_SOURCE
)

set(sbgc-api-library ${CMAKE_BINARY_DIR}/sbgc-api/src/sbgc-api/libraries/SBGC_lib)
include_directories(${sbgc-api-library})

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
generate_dynamic_reconfigure_options(
  cfg/Control.cfg
  )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES simple_bgc_driver
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(simple_bgc_driver
  src/simple_bgc_driver.cpp
  ${sbgc-api-library}/SBGC_cmd_helpers.cpp
  )
target_link_libraries(simple_bgc_driver ${catkin_LIBRARIES})
add_dependencies(simple_bgc_driver  ${PROJECT_NAME}_gencfg)

## Declare a C++ executable
add_executable(simple_bgc_node src/simple_bgc_node.cpp)
target_link_libraries(simple_bgc_node simple_bgc_driver ${catkin_LIBRARIES})
add_dependencies(simple_bgc_node  ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


# install(TARGETS simple_bgc simple_bgc_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_simple_bgc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
