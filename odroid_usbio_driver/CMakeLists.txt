cmake_minimum_required(VERSION 2.8.3)
project(odroid_usbio_driver)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -Ofast -march=native -lusb-1.0 -lpthread -DDEBUG_PRINTF")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs)

find_package(OpenCV 2.4.8 REQUIRED)
#find_package(libusb-1.0 REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/src
	${catkin_INCLUDE_DIRS}
#	/usr/include/libusb-1.0/
)





######################################################################################################
#        FROM HERE: these are the correct newly organized libraries and executables                  #
######################################################################################################


##################################
#        CREATE camera drivers   #
##################################
add_executable(usbio src/hid-libusb.c src/main.cpp )
target_link_libraries(usbio libusb-1.0.so.0 ${catkin_LIBRARIES} )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES  
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS 
)

