cmake_minimum_required(VERSION 2.8.3)
project(camera_array_arm)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x -Ofast -march=native")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs image_transport sensor_msgs cv_bridge camera_info_manager image_proc stereo_image_proc)
find_package(Boost REQUIRED COMPONENTS signals system)

find_package(OpenCV 2.4.8 REQUIRED)
# message(FATAL_ERROR  ${OpenCV_LIBRARIES})
# message(FATAL_ERROR  ${OpenCV_INCLUDE_DIRS})
# SET(OpenCV_INCLUDE_DIRS /usr/include/opencv)

#SET(OPENCV_LIBRARIES opencv_features2d opencv_highgui opencv_ml opencv_contrib opencv_flann opencv_imgproc opencv_objdetect opencv_core opencv_gpu opencv_legacy opencv_video)

# opencv_videostab
# opencv_video
# opencv_superres
# opencv_stitching
# opencv_photo
# opencv_ocl
# opencv_objdetect
# opencv_ml
# opencv_legacy
# opencv_imgproc
# opencv_highgui
# opencv_gpu
# opencv_flann
# opencv_features2d
# opencv_core
# opencv_contrib
# opencv_calib3d


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
	${PROJECT_SOURCE_DIR}/include
	${PROJECT_SOURCE_DIR}/ccny_mvVirtualDevice
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(camera_array
#   src/${PROJECT_NAME}/camera_array.cpp
# )


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


























######################################################################################################
#        FROM HERE: these are the correct newly organized libraries and executables                  #
######################################################################################################


##################################
#        CREATE PID              #
##################################
add_library(PID src/PID/PID.cpp)


##################################
#        CREATE camera drivers   #
##################################
add_library(mv_bluefox_driver_nodelet_arm
# 		src/mv_bluefox_driver/nodelets.cpp
		src/mv_bluefox_driver/camera.cpp
		src/mv_bluefox_driver/MyCamera.cpp
)
target_link_libraries(mv_bluefox_driver_nodelet_arm PID ${MVIMPACT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} )

add_executable(camera_node_arm src/mv_bluefox_driver/camera_node.cpp )
target_link_libraries(camera_node_arm mv_bluefox_driver_nodelet_arm ${MVIMPACT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(my_camera_node_arm src/mv_bluefox_driver/my_camera_node.cpp )
target_link_libraries(my_camera_node_arm mv_bluefox_driver_nodelet_arm ${MVIMPACT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

##################################
#        CREATE StereoConverter  #
##################################
add_library(StereoConverter src/StereoConverter/StereoConverter.cpp)
target_link_libraries(StereoConverter ${MVIMPACT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(StereoConverterTest src/StereoConverter/StereoConverterTest.cpp)
target_link_libraries(StereoConverterTest StereoConverter PID)

##################################
#        CREATE CameraConverter  #
##################################
add_library(CameraConverter src/CameraConverter/CameraConverter.cpp)
target_link_libraries(CameraConverter ${MVIMPACT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(CameraConverterTest src/CameraConverter/CameraConverterTest.cpp)
target_link_libraries(CameraConverterTest CameraConverter PID)

##################################
#        CREATE ImageAnalyzer    #
##################################
add_library(ImageAnalyzer
src/ImageAnalyzer/detect_material_online.cpp
src/ImageAnalyzer/disparity_map.cpp
src/ImageAnalyzer/disparity_map_slide_images.cpp
# src/ImageAnalyzer/disparity_map_slide_images2.cpp
src/ImageAnalyzer/disparity_map_winner_take_all.cpp
src/ImageAnalyzer/disparity_map_clean_disparity.cpp
src/ImageAnalyzer/disparity_map_new_image.cpp
src/ImageAnalyzer/compute_ndvi_online.cpp
src/ImageAnalyzer/ndvi_clustering_parabola.cpp
src/ImageAnalyzer/ndvi_clustering_sigmoid.cpp
src/ImageAnalyzer/compute_pointcloud.cpp
src/ImageAnalyzer/utilities.cpp
)
target_link_libraries(ImageAnalyzer ${MVIMPACT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

add_executable(ImageAnalyzerTest src/ImageAnalyzer/ImageAnalyzerTest.cpp)
target_link_libraries(ImageAnalyzerTest ImageAnalyzer)

##################################
#        CREATE main executable  #
##################################
add_executable(main_new src/main/stereo_viewer_runningOutput.cpp)
target_link_libraries(main_new CameraConverter StereoConverter ImageAnalyzer PID)

##################################
#        CREATE new main executable  #
##################################
add_executable(ndvi_computer src/main/main.cpp)
target_link_libraries(ndvi_computer CameraConverter StereoConverter ImageAnalyzer PID mv_bluefox_driver_nodelet_arm)

##################################
#        CREATE new main executable  #
##################################
add_executable(calibrator src/main/calibration.cpp)
target_link_libraries(calibrator CameraConverter StereoConverter ImageAnalyzer PID mv_bluefox_driver_nodelet_arm)


##################################
#        CREATE new main executable  #
##################################
add_executable(test_new src/main/test.cpp)
target_link_libraries(test_new ${MVIMPACT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(test_new CameraConverter StereoConverter ImageAnalyzer PID)

##################################
#        CREATE new main executable  #
##################################
add_executable(test_var src/main/test_var.cpp)
target_link_libraries(test_var ${MVIMPACT_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
target_link_libraries(test_var CameraConverter StereoConverter ImageAnalyzer PID)

##################################
#        CREATE new main executable  #
##################################
#add_executable(mv_blue src/main/single_camera_main.cpp)
#target_link_libraries(ndvi_computer CameraConverter StereoConverter ImageAnalyzer PID mv_bluefox_driver_nodelet_arm)


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
  INCLUDE_DIRS include src
  LIBRARIES mv_bluefox_driver_nodelet_arm PID StereoConverter CameraConverter ImageAnalyzer 
  CATKIN_DEPENDS roscpp rospy std_msgs image_transport sensor_msgs cv_bridge camera_info_manager image_proc stereo_image_proc
  DEPENDS Boost
)

