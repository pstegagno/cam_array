
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sstream>
#include <string>

#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/legacy/legacy.hpp"
#include <cv.h>
#include <highgui.h>
#include "image_analyzer.hpp"

#include "PID.hpp"

int numimage;

using namespace cv;
using namespace ros;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

static const std::string OPENCV_WINDOW = "spectralViewer";




void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata);


class StereoConverter
{
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
  
  int _expTime;
  


  
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Publisher exp_time_equal_for_all_cameras;
  image_transport::Subscriber image_sub2_;
  ros::Publisher exp_time_equal_for_all_cameras2;
  
 // image_transport::Publisher image_pub_;
  
    PID control;
  
public:
    Mat img, img2; 
  
  void saveImage(){
    cout << "StereoConverter :: saveImage 1" << endl;
    _buttonPressed=true;
    _buttonPressed2=true;
//     cout << "StereoConverter :: saveImage 2" << endl;
//     while (_buttonPressed || _buttonPressed){
//       usleep(10000);
//     }
//     cout << "StereoConverter :: saveImage 3" << endl;
  }
   
   
 void save_time_cb(const std_msgs::Int32ConstPtr& msg)

 {
   _expTime = msg->data;
 }
   
   // costrutture
  StereoConverter(ros::NodeHandle nh_, std::string s_name, string  exp_time_topic, int idNum, std::string s_name2, string  exp_time_topic2, int idNum2)
    : it_(nh_)
  {
    id = idNum;
    id2 = idNum2;

    // Subscribe to input video feed and publish output video feedtarget_link_libraries(camera_viewer  ${catkin_LIBRARIES})
    image_sub_ = it_.subscribe(s_name, 1, &StereoConverter::imageCb, this);
    //Publisher 
    exp_time_equal_for_all_cameras = nh_.advertise<std_msgs::Int32>(exp_time_topic, 1);



    // Subscribe to input video feed and publish output video feed
    image_sub2_ = it_.subscribe(s_name2, 1, &StereoConverter::imageCb2, this);
    //Publisher 
    exp_time_equal_for_all_cameras2 = nh_.advertise<std_msgs::Int32>(exp_time_topic2, 1);

    
    // setup cv window
    stringstream ss;
    ss << OPENCV_WINDOW << "_" << id;
    _windowName = ss.str();
    cv::namedWindow(_windowName);
    cv::setMouseCallback(_windowName, mouseCallBackFunc, NULL);
    
      stringstream ss2;
    ss2 << OPENCV_WINDOW << "_" << id2;
    _windowName2 = ss2.str();
    cv::namedWindow(_windowName2);
    cv::setMouseCallback(_windowName2, mouseCallBackFunc, NULL);

    stringstream sss;
    sss << "image_" << id << "_";
    _imageName = sss.str();
    _imageCounter=0;

    stringstream sss2;
    sss2<< "image_" << id2 << "_";
    _imageName2= sss2.str();
    _imageCounter2=0;
    
    _expTime=200;
// subscribe
    _buttonPressed = false;
    _buttonPressed2 = false;
    
    pixel_value = 0.0;
    pixel_value2 = 0.0;
  }

  ~StereoConverter()
  {
    cv::destroyWindow(_windowName);
  }
  
  
  
  
  void controlExpTime(){
    
    float mean_pixel_value, error;
    
    mean_pixel_value = (pixel_value + pixel_value2)/2.0;
    error= 128-mean_pixel_value;
//     std::cout<<" mean pixel value is " << mean_pixel_value << std::endl;    
    
    control.pid_update(error,  _expTime);
    _expTime =   _expTime + control.control;
    _expTime= std::max( _expTime,12);
    
    std_msgs::Int32 msg32;
    msg32.data = _expTime;
    exp_time_equal_for_all_cameras2.publish(msg32);
    exp_time_equal_for_all_cameras.publish(msg32);
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {

      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      if(!cv_ptr->image.empty())
      {
	cv::imshow(_windowName, cv_ptr->image);
	cv::waitKey(1);

	double sum_array=0;

	sum_array = sum(cv_ptr->image)[0];
	pixel_value=sum_array/(480 * 752);
	
	controlExpTime();

	if(_buttonPressed){
	 
	  cv_ptr->image.convertTo(img, CV_32F);

	  _buttonPressed=false;
	  
	  stringstream ss;
	  ss << "../imSaves/" << "_" << _imageCounter << _imageName  << _expTime << ".jpg";
	  ROS_INFO("Saved Image named: %s",ss.str().c_str());
	  cv::imwrite( ss.str().c_str(), cv_ptr->image );
	  _imageCounter++;
	}
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
  
  void imageCb2(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      if(!cv_ptr->image.empty())
      {
	cv::imshow(_windowName2, cv_ptr->image);
	cv::waitKey(1);

	double sum_array=0;

	sum_array = sum(cv_ptr->image)[0];
	pixel_value2=sum_array/(480 * 752);
	
	controlExpTime();
	       
	if(_buttonPressed2){
	 
	  cv_ptr->image.convertTo(img2, CV_32F);

	  _buttonPressed2=false;

	  stringstream ss2;
	  ss2 << "../imSaves/" << "_"<< _imageCounter2  << _imageName2  << _expTime << ".jpg";
	  cv::imwrite( ss2.str().c_str(), cv_ptr->image );
	  _imageCounter2++; 
	  ROS_INFO("Saved Image named: %s",ss2.str().c_str());
	  
	}
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
  
}; // end of class StereoConverter

class ImageConverter
{
  int id;

  bool _buttonPressed;
  
  string _imageName;
  string _windowName;
  int _imageCounter;
  
  int _expTime;


  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber exposition_time_sub;
  ros::Publisher exp_time_equal_for_all_cameras;
 // image_transport::Publisher image_pub_;
  
    PID control;
  
public:
   
 Mat img;
  
  void saveImage(){
    cout << "ImageConverter :: saveImage 1" << endl;
    _buttonPressed=true;
/*    cout << "ImageConverter :: saveImage 2" << endl;
    while (_buttonPressed){
      usleep(10000);
    }
    cout << "ImageConverter :: saveImage 3" << endl; */   
  }
   
   
 void save_time_cb(const std_msgs::Int32ConstPtr& msg)

 {
   _expTime = msg->data;
 }
   
   
  ImageConverter(ros::NodeHandle nh_, std::string s_name, string  exp_time_topic, int idNum)
    : it_(nh_)
  {
    id = idNum;
 


    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe(s_name, 1, &ImageConverter::imageCb, this);
    //Publisher 
    exp_time_equal_for_all_cameras = nh_.advertise<std_msgs::Int32>(exp_time_topic, 1);

    // setup cv window
    stringstream ss;
    ss << OPENCV_WINDOW << "_" << id;
    _windowName = ss.str();
    cv::namedWindow(_windowName);
    cv::setMouseCallback(_windowName, mouseCallBackFunc, NULL);


    stringstream sss;
    sss << "image_" << id << "_";
    _imageName = sss.str();
    _imageCounter=0;

    _expTime=200;
// subscribe
    _buttonPressed = false;

  }

  ~ImageConverter()
  {
    cv::destroyWindow(_windowName);
  }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
// 	ROS_INFO("A");
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//       	ROS_INFO("A1");
      if(!cv_ptr->image.empty())
      {
// 	ROS_INFO("B");
	cv::imshow(_windowName, cv_ptr->image);
	cv::waitKey(1);
	double pixel_value , sum_array=0, error;

	sum_array = sum(cv_ptr->image)[0];
	pixel_value=sum_array/(480 * 752);
	error= 128-pixel_value;
// 	ROS_INFO("C");

	control.pid_update(error,  _expTime);
	_expTime =   _expTime + control.control;
	_expTime= std::max( _expTime,12);
	
	std_msgs::Int32 msg32;
	msg32.data = _expTime;
	exp_time_equal_for_all_cameras.publish(msg32);
// 	ROS_INFO("D");
	
	if(_buttonPressed){
// 	  	ROS_INFO("E");

	  cv_ptr->image.convertTo(img, CV_32F);
	  _buttonPressed=false;
	  
	  stringstream ss;

	  ss << "../imSaves/" << "_"<< _imageCounter << _imageName  << _expTime << ".jpg";
	  
	  ROS_INFO("Saved Image named: %s",ss.str().c_str());

	  
	  cv::imwrite( ss.str().c_str(), cv_ptr->image );
	  _imageCounter++; 
	}
      }
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
};


StereoConverter* cam2;
ImageConverter* cam1;

ImageAnalyzer callFunction;


  void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata)
  {
    if  ( event == EVENT_LBUTTONDOWN )
    {

      cam1->saveImage();
      cam2->saveImage();

      cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
            
      Mat dsp, calibrated, clean;
      cout << cam2->img2.size() << endl;
      cout << cam2->img.size() << endl;
      callFunction.stereo(cam2->img2, cam2->img, 35, dsp, clean, calibrated);
      //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_MBUTTONDOWN )
    {
      //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if ( event == EVENT_MOUSEMOVE )
    {
      //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
    }
  }





int main(int argc, char **argv)
{
  numimage = 0;

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;



   cam2 = (StereoConverter*)new StereoConverter(n, "left/image_raw", "left/exposition_time", 1,"right/image_raw", "right/exposition_time", 3 );
   
   //cam1 = (ImageConverter*)new ImageConverter(n, "camera2/image_raw", "camera2/exposition_time", 2);

  ros::spin();

  return 0;

}
