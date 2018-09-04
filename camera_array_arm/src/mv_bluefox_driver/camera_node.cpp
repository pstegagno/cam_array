#include <ros/ros.h>
#include <nodelet/loader.h>

#include <camera_array_arm/camera.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "mv_camera");
  
  CameraArrayProject::PIDParameters pid_params(0, 0.0011,0.0010, 0.001, 0, 0, 0);

  std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA " << std::endl;
  std::string numcamera("25000698");

  mv_bluefox_driver::Camera camera(ros::NodeHandle(), ros::NodeHandle("~"), numcamera, pid_params);

  ros::spin();
  return 0;
}

