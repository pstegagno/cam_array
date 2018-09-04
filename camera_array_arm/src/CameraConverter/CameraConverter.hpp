

#ifndef CAMERA_CONVERTER_HPP
#define CAMERA_CONVERTER_HPP

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>

#include <cv.h>
#include <highgui.h>

#include <PID/PID.hpp>


using namespace cv;
using namespace ros;
using namespace std;


namespace CameraArrayProject{

void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata);

class CameraConverter
{
  
  ofstream myfile;
  
  unsigned long int timeLast;
  
  
  bool _saveframes;
  bool _showframes;
  int _framecounter;
  
  
  int id;

  bool _buttonPressed;
  
  string _imageName;
  string _windowName;
  int _imageCounter;
  
  //int _expTime;


  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber exposition_time_sub;
  ros::Publisher exp_time_equal_for_all_cameras;
 // image_transport::Publisher image_pub_;
  
  PID control;
  
  cv_bridge::CvImagePtr cv_ptr;
  bool img_received;
    
  bool image_busy;
  
public:
   
  bool copyImage_central(cv::Mat &out1);
  
  Mat img;
  
  int _expTime;
  
  void saveImage();
   
  void save_time_cb(const std_msgs::Int32ConstPtr& msg);
   
  CameraConverter(ros::NodeHandle nh_, std::string s_name, string  exp_time_topic, int idNum);

  ~CameraConverter();

  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  
  bool ready();
  
    void saveframes(){
    _saveframes = true;
  }
  
  void showframes(){
    _showframes = true;
  }

};

};// end namespace CameraArrayProject

#endif

