cmake_minimum_required(VERSION 2.8.3)
project(ccny_my_camera_drivers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs image_transport cv_bridge camera_info_manager)
find_package(Boost REQUIRED COMPONENTS signals system)

find_package(OpenCV 2.4.8 REQUIRED)

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
  INCLUDE_DIRS include
  LIBRARIES camera_array
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${PROJECT_SOURCE_DIR}
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/ccny_mvVirtualDevice
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)


if(EXISTS ${PROJECT_SOURCE_DIR}/../mvIMPACT_Acquire/)
  message("-- mvIMPACT_Acquire drivers found")
  if(EXISTS ${PROJECT_SOURCE_DIR}/../mvIMPACT_Acquire/PLATFORM_ARM)
    message("-- for platform ARM.")
    set(MVIMPACT_LIBRARIES  ${PROJECT_SOURCE_DIR}/../mvIMPACT_Acquire/lib/armhf/libmvDeviceManager.so)
  else(EXISTS ${PROJECT_SOURCE_DIR}/../mvIMPACT_Acquire/PLATFORM_ARM)
    if(EXISTS ${PROJECT_SOURCE_DIR}/../mvIMPACT_Acquire/PLATFORM_x86_64)
      message("-- for platform x86_64.")
      set(MVIMPACT_LIBRARIES  ${PROJECT_SOURCE_DIR}/../mvIMPACT_Acquire/lib/x86_64/libmvDeviceManager.so)
    else(EXISTS ${PROJECT_SOURCE_DIR}/../mvIMPACT_Acquire/PLATFORM_x86_64)
      message("-- but no platform specified.")
      message(FATAL_ERROR "Please run the script\n${PROJECT_SOURCE_DIR}/../install_mvBlueFox_drivers.sh")
    endif(EXISTS ${PROJECT_SOURCE_DIR}/../mvIMPACT_Acquire/PLATFORM_x86_64)
  endif(EXISTS ${PROJECT_SOURCE_DIR}/../mvIMPACT_Acquire/PLATFORM_ARM)
else(EXISTS ${PROJECT_SOURCE_DIR}/../mvIMPACT_Acquire/)
  message(FATAL_ERROR "mvIMPACT_Acquire drivers not found.\nPlease run the script\n${PROJECT_SOURCE_DIR}/../install_mvBlueFox_drivers.sh")
endif(EXISTS ${PROJECT_SOURCE_DIR}/../mvIMPACT_Acquire/)



##################################
#        CREATE LIBRARY          #
##################################
add_library(mv_bluefox_driver_nodelet
		src/mv_bluefox_driver/nodelets.cpp
		src/mv_bluefox_driver/camera.cpp
)

target_link_libraries(mv_bluefox_driver_nodelet
        ${MVIMPACT_LIBRARIES}
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)

##################################
#        CREATE EXECUTABLE       #
##################################
add_executable(camera_node
		src/mv_bluefox_driver/camera_node.cpp
		src/mv_bluefox_driver/camera.cpp
)
target_link_libraries(camera_node
	mv_bluefox_driver_nodelet
	${MVIMPACT_LIBRARIES}
	${Boost_LIBRARIES}
	${catkin_LIBRARIES}
	${OpenCV_LIBRARIES}
)


#target_link_libraries(image_analyzer_user ${catkin_LIBRARIES} image_analyzer)

#add_executable(cmeans
#	src/FuzzyClustering/fuzzy_clustering.cpp
#	src/FuzzyClustering/cmeans.cpp
#)
#target_link_libraries(cmeans  ${catkin_LIBRARIES})


## Specify libraries to link a library or executable target against
# target_link_libraries(camera_array_node
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(camera_node
# opencv_features2d
# opencv_highgui
# opencv_ml
# opencv_contrib
# opencv_flann
# opencv_imgproc
# opencv_objdetect
# opencv_core
# opencv_gpu
# opencv_legacy
# opencv_video
# )





