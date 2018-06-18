cmake_minimum_required(VERSION 2.8.3)
project(hace_core)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        image_geometry
        moveit_visual_tools
        visualization_msgs
        sensor_msgs
        hace_msgs
        )

## System dependencies are found with CMake's conventions
find_package(OpenCV)
find_package(OpenPose REQUIRED)
find_package(Caffe REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp cv_bridge
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenPose_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/hace.cpp

        include/cv_convert.h
        include/RosWorkerOutput.h
        include/RosImageProducer.h
        include/RosDepthInput.h
        include/RosDepthOutput.h
        include/DepthDatum.h
        include/HumanProcessor.h


        src/cv_convert.cpp
        src/RosWorkerOutput.cpp
        src/RosDepthInput.cpp
        src/RosDepthOutput.cpp
        src/RosImageProducer.cpp
        src/HumanProcessor.cpp
        )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
#        ${PROJECT_NAME}_lib
        ${catkin_LIBRARIES}
        ${OpenPose_LIBS}
        ${Caffe_LIBS}
        ${OpenCV_LIBS}
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

