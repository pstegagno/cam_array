
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


#include "PID.hpp"

int numimage;

using namespace cv;
using namespace ros;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

static const std::string OPENCV_WINDOW = "spectralViewer";



void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata);


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
    _buttonPressed=true;
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
//	         ROS_INFO("qui00");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if(!cv_ptr->image.empty())
    {
       cv::imshow(_windowName, cv_ptr->image);
       
       
       
       
       
       
       
         double coeff = 1.1, pixel_value , sum_array=0, error;


  sum_array = sum(cv_ptr->image)[0];
  pixel_value=sum_array/(480 * 752);
//   std::cout<<" pixel value is " << pixel_value << std::endl;
  error= 128-pixel_value;

   control.pid_update(error,  _expTime);
  _expTime =   _expTime + control.control;
  _expTime= std::max( _expTime,12);
  
  
  std_msgs::Int32 msg32;
  msg32.data = _expTime;
   exp_time_equal_for_all_cameras.publish(msg32);
       
  
  
  
       
       
       cv::waitKey(10);
       if(_buttonPressed){
	 
	 	  stringstream ss;
ROS_INFO("Saved Image named: %s",ss.str().c_str());
	  _buttonPressed=false;
	  
	  img =  cv_ptr->image;

	  ss << "../imSaves/" << _imageName << _imageCounter << "_" << _expTime << ".jpg";
	  
	  ROS_INFO("Saved Image named: %s",ss.str().c_str());

	  
	  cv::imwrite( ss.str().c_str(), cv_ptr->image );
	  _imageCounter++; 
ROS_INFO("Saved Image named: %s",ss.str().c_str());

       img = cv_ptr->image;
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



ImageConverter* cam1;






  void mouseCallBackFunc(int event, int x, int y, int flags, void* userdata)
  {
    if  ( event == EVENT_LBUTTONDOWN )
    {
      cam1->saveImage();
      
//       cam2->saveImage();
//       cam3->saveImage();
      cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
    else if  ( event == EVENT_RBUTTONDOWN )
    {
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

   cam1 = (ImageConverter*)new ImageConverter(n, "camera1/image_rect", "camera1/exposition_time", 3);
 

  ros::spin();

  return 0;
}
