


#ifndef STEREO_CONVERTER_HPP
#define STEREO_CONVERTER_HPP



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


class StereoConverter
{
  
  ofstream myfile;
  ofstream myfile2;
  
  unsigned long int timeLast;
  unsigned long int timeLast2;
  
  bool _saveframes;
  bool _showframes;
  
  int _framecounter;
  int _framecounter2;
  
  int id, id2;

  bool _buttonPressed;
  bool _buttonPressed2;
  string _imageName;
  string _windowName;
  string _imageName2;
  string _windowName2;
  
  int _imageCounter;
  int _imageCounter2;
  
  double pixel_value;
  double pixel_value2;
  
  double weightMess = 1.0;//92642298225;
  
 // int _expTime;
  
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher exp_time_equal_for_all_cameras;
  image_transport::Subscriber image_sub2_;
  ros::Publisher exp_time_equal_for_all_cameras2;
  
  cv_bridge::CvImagePtr cv_ptr;
  cv_bridge::CvImagePtr cv_ptr2;
  bool img_received;
  bool img_received2;
  
  bool image_1_busy;
  bool image_2_busy;
  
  PID control;
  
public:
  
  bool copyImages(cv::Mat &out1, cv::Mat &out2);
  
  Mat img, img2; 
  
  int _expTime;
  
  void saveImage();
   
  void save_time_cb(const std_msgs::Int32ConstPtr& msg);
   
   // costrutture
  StereoConverter(ros::NodeHandle nh_, std::string s_name, string  exp_time_topic, int idNum, std::string s_name2, string  exp_time_topic2, int idNum2);

  ~StereoConverter();
  
  void controlExpTime(double weight);

  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  
  void imageCb2(const sensor_msgs::ImageConstPtr& msg);
  
  bool ready();
  
  void saveframes(){
    _saveframes = true;
  }
  
  void showframes(){
    _showframes = true;
  }
  
}; // end of class StereoConverter

}; // end namespace CameraArrayProject

#endif

