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
#include "stereo_viewer_runningOutput.hpp"

#include <thread>

#include <fstream>
#include <iostream>
using namespace std;

int numimage;

using namespace cv;
using namespace ros;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

static const std::string OPENCV_WINDOW = "spectralViewer";

void StereoConverter::saveImage()
{
  cout << "StereoConverter :: saveImage 1" << endl;
  _buttonPressed=true;
  _buttonPressed2=true;
}
   
   
 void StereoConverter::save_time_cb(const std_msgs::Int32ConstPtr& msg)

 {
   _expTime = msg->data;
 }
   
   // costrutture
StereoConverter::StereoConverter(ros::NodeHandle nh_, std::string s_name, string  exp_time_topic, int idNum, std::string s_name2, string  exp_time_topic2, int idNum2)
  : it_(nh_)
{
  id = idNum;
  id2 = idNum2;
  
//       std::cout<<" A " << std::endl;    

  // Subscribe to input video feed and publish output video feedtarget_link_libraries(camera_viewer  ${catkin_LIBRARIES})
  image_sub_ = it_.subscribe(s_name, 1, &StereoConverter::imageCb, this);
  //Publisher 
  exp_time_equal_for_all_cameras = nh_.advertise<std_msgs::Int32>(exp_time_topic, 1);


//       std::cout<<" b " << std::endl;    

  // Subscribe to input video feed and publish output video feed
  image_sub2_ = it_.subscribe(s_name2, 1, &StereoConverter::imageCb2, this);
  //Publisher 
  exp_time_equal_for_all_cameras2 = nh_.advertise<std_msgs::Int32>(exp_time_topic2, 1);

//       std::cout<<" c " << std::endl;    
  
  // setup cv window
  stringstream ss;
  ss << OPENCV_WINDOW << "_" << id;
  _windowName = ss.str();
  cv::namedWindow(_windowName);
  cv::setMouseCallback(_windowName, mouseCallBackFunc, NULL);
//       std::cout<<" d " << std::endl;    
  
    stringstream ss2;
  ss2 << OPENCV_WINDOW << "_" << id2;
  _windowName2 = ss2.str();
  cv::namedWindow(_windowName2);
  cv::setMouseCallback(_windowName2, mouseCallBackFunc, NULL);
//       std::cout<<" e " << std::endl;    

  stringstream sss;
  sss << "image_" << id << "_";
  _imageName = sss.str();
  _imageCounter=0;
//       std::cout<<" f " << std::endl;    

  stringstream sss2;
  sss2<< "image_" << id2 << "_";
  _imageName2= sss2.str();
  _imageCounter2=0;
//       std::cout<<" g " << std::endl;    
  
  _expTime=200;
// subscribe
  _buttonPressed = false;
  _buttonPressed2 = false;
//       std::cout<<" h " << std::endl;    
  
  pixel_value = 0.0;
  pixel_value2 = 0.0;
//       std::cout<<" i " << std::endl;s
  img_received = false;
  img_received2 = false;
  
  image_1_busy = false;
  image_2_busy = false;
  
  _saveframes = false;
  _showframes = false;
  _framecounter = 0;
  _framecounter2 = 0;
  
  myfile.open ("../imSaves/camera1.txt");
  myfile2.open ("../imSaves/camera3.txt");
  
  struct timeval tv;
  gettimeofday(&tv,NULL);
  timeLast = 1000000 * tv.tv_sec + tv.tv_usec;
  timeLast2 = timeLast;
}


StereoConverter::~StereoConverter()
{
  cv::destroyWindow(_windowName);
}
  
  
  
  
void StereoConverter::controlExpTime(double weight){
  
  float mean_pixel_value, error;
  
  mean_pixel_value = (pixel_value + pixel_value2)/2.0;
//   std::cout << pixel_value << "  " <<  pixel_value2  << " " << weightMess << endl;
  
  error= 108-mean_pixel_value;
//     std::cout<<" mean pixel value is " << mean_pixel_value << std::endl;    
  
  control.pid_update(error,  _expTime);
  _expTime =   _expTime + control.control;
  _expTime= std::max( _expTime,12);
  
  if (pixel_value>0 && pixel_value2>0){
//     if (pixel_value - pixel_value2 > 0){
      weightMess -=0.0003*(pixel_value - pixel_value2);
//     }
//     else if (pixel_value - pixel_value2 < 0){
//       weightMess +=0.001;
//     }
    if (weightMess < 0.1){
      weightMess = 0.1;
    }
  }
  
  std_msgs::Int32 msg32;
  msg32.data = _expTime;
  exp_time_equal_for_all_cameras2.publish(msg32);
  msg32.data = (int)(((double)_expTime)*weight);
  exp_time_equal_for_all_cameras.publish(msg32);
}


void StereoConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
//   cv_bridge::CvImagePtr cv_ptr;
  try
  {
    
    while(image_1_busy){
      std::cout << "stuck here 6" << std::endl;
      usleep(1000);
    }
    image_1_busy = true;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    image_1_busy = false;

    if(!cv_ptr->image.empty())
    {
      img_received = true;
            
      
      if (_saveframes){
	
	if (_framecounter>0){
	  struct timeval tv;
	  gettimeofday(&tv,NULL);
	  unsigned long int timeEl = 1000000 * tv.tv_sec + tv.tv_usec;
	  myfile << "duration " <<  (double)(timeEl - timeLast)/1000000.0 <<  endl;
	  timeLast = timeEl;
	}
	
	stringstream ss;
	ss.fill('0');
	ss << "../imSaves/camera1_" <<  std::setw(10) << internal <<_framecounter << ".png";
	myfile << "file '" <<  ss.str().substr(11) << "'" << endl;
	
      // 	ROS_INFO("Saved Image named: %s",ss.str().c_str());
	cv::imwrite( ss.str().c_str(), cv_ptr->image );
	_framecounter++;
      }
      if (_showframes){
	cv::imshow(_windowName, cv_ptr->image);
	cv::waitKey(1);
      }
  
      double sum_array=0;
      sum_array = sum(cv_ptr->image)[0];
      pixel_value=sum_array/(480 * 752);
      
      
      controlExpTime(weightMess);
      
      
       	
/*
       VideoWriter video("capture.avi",CV_FOURCC('D','I','V','X'), 10, cvSize(640,480)); 
        // Get frame from capture
        capture >> image;
        // Save frame to video
        video << image;*/

      if(_buttonPressed){
      
	cv_ptr->image.convertTo(img, CV_32F);

	_buttonPressed=false;
	
	stringstream ss;
	ss << "../imSaves/" <<  _imageCounter << "_" << _imageName  << _expTime << ".png";
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
  
void StereoConverter::imageCb2(const sensor_msgs::ImageConstPtr& msg)
{
//   cv_bridge::CvImagePtr cv_ptr;
  try
  {
    while(image_2_busy){
      std::cout << "stuck here 5" << std::endl;
      usleep(1000);
    }
    image_2_busy = true;
    cv_ptr2 = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    image_2_busy = false;
    if(!cv_ptr2->image.empty())
    {
      img_received2 = true;
      
      if (_saveframes){
	
	if (_framecounter2>0){
	  struct timeval tv;
	  gettimeofday(&tv,NULL);
	  unsigned long int timeEl = 1000000 * tv.tv_sec + tv.tv_usec;
	  myfile2 << "duration " <<  (double)(timeEl - timeLast2)/1000000.0 <<  endl;
	  timeLast2 = timeEl;
	}

	
	stringstream ss;
	ss.fill('0');
	ss << "../imSaves/camera3_" <<  std::setw(10) << internal <<_framecounter2 << ".png";
// 	ROS_INFO("Saved Image named: %s",ss.str().c_str());
	myfile2 << "file '" <<  ss.str().substr(11) << "'" << endl;
	cv::imwrite( ss.str().c_str(), cv_ptr2->image );
	_framecounter2++;
      }
      if (_showframes){
	cv::imshow(_windowName2, cv_ptr2->image);
	cv::waitKey(1);
      }

      double sum_array=0;

      sum_array = sum(cv_ptr2->image)[0];
      pixel_value2=sum_array/(480 * 752);
      
      controlExpTime(weightMess);
	      
      if(_buttonPressed2){
	
	cv_ptr2->image.convertTo(img2, CV_32F);

	_buttonPressed2=false;

	stringstream ss2;
	ss2 << "../imSaves/" << _imageCounter2  << "_"<<  _imageName2  << _expTime << ".png";
	cv::imwrite( ss2.str().c_str(), cv_ptr2->image );
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


bool StereoConverter::copyImages(cv::Mat &out1, cv::Mat &out2){
  
  if ( img_received && img_received2 ) {
    
    while (image_1_busy){
      usleep(1000);
      std::cout << "stuck here 1" << std::endl;
    }
    image_1_busy = true;
    cv_ptr->image.copyTo(out1);
    img_received = false;
    image_1_busy = false;
    
    while (image_2_busy){
      std::cout << "stuck here 2" << std::endl;
      usleep(1000);
    }
    image_2_busy = true;
    cv_ptr2->image.copyTo(out2);
    img_received2 = false;
    image_2_busy = false;
    return true;
  }
  return false;
}

bool StereoConverter::ready(){
  return img_received && img_received2;
}

// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// begin of class ImageConverter
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  
  
void ImageConverter::saveImage(){
  cout << "ImageConverter :: saveImage 1" << endl;
  _buttonPressed=true;
}
   
   
void ImageConverter::save_time_cb(const std_msgs::Int32ConstPtr& msg)

{
  _expTime = msg->data;
}
   
   
ImageConverter::ImageConverter(ros::NodeHandle nh_, std::string s_name, string  exp_time_topic, int idNum)
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
  img_received = false;
  image_busy = false;
  
  _saveframes = false;
  _showframes = false;
  _framecounter = 0;  
  
  myfile.open ("../imSaves/camera2.txt");
  
  struct timeval tv;
  gettimeofday(&tv,NULL);
  timeLast = 1000000 * tv.tv_sec + tv.tv_usec;
}

ImageConverter::~ImageConverter()
{
  cv::destroyWindow(_windowName);
}


void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  //cv_bridge::CvImagePtr cv_ptr;
  try
  {
// 	ROS_INFO("A");
    while (image_busy){
      std::cout << "stuck here 3" << std::endl;
      usleep(1000);
    }
    image_busy = true;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
//       	ROS_INFO("A1");
    image_busy = false;
    
    if(!cv_ptr->image.empty())
    {
       
      img_received = true;
      
      
      
      if (_saveframes){
	
	if (_framecounter>0){
	  struct timeval tv;
	  gettimeofday(&tv,NULL);
	  unsigned long int timeEl = 1000000 * tv.tv_sec + tv.tv_usec;
	  myfile << "duration " <<  (double)(timeEl - timeLast)/1000000.0 <<  endl;
	  timeLast = timeEl;
	}

	
	stringstream ss;
	ss.fill('0');
	ss << "../imSaves/camera2_" <<  std::setw(10) << internal <<_framecounter << ".png";
	myfile << "file '" <<  ss.str().substr(11) << "'" << endl;
	cv::imwrite( ss.str().c_str(), cv_ptr->image );
	_framecounter++;
      }
      if (_showframes){
	cv::imshow(_windowName, cv_ptr->image);
	cv::waitKey(1);
      }
      
      
      double pixel_value , sum_array=0, error;

      sum_array = sum(cv_ptr->image)[0];
      pixel_value=sum_array/(480 * 752);
      error= 108-pixel_value;
// 	ROS_INFO("C");

      control.pid_update(error, _expTime);
      _expTime =   _expTime + 2*control.control;
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

	ss << "../imSaves/" <<  _imageCounter <<"_"<< _imageName  << _expTime << ".png";
	
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

bool ImageConverter::copyImage_central(cv::Mat &out1){
  
  if ( img_received ) {
    while (image_busy){
      std::cout << "stuck here 4" << std::endl;
      usleep(1000);
    }
    image_busy = true;
    cv_ptr->image.copyTo(out1);
    img_received = false;
    image_busy = false;
    return true;
  }
  return false;
}

bool ImageConverter::ready(){
  return img_received;
}


// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// begin of main stuff
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@



StereoConverter* cam2;
ImageConverter* cam1;



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
            
      Mat dsp, calibrated;
      cout << cam2->img2.size() << endl;
      cout << cam2->img.size() << endl;
//       callFunction.stereo(cam2->img2, cam2->img, 35, dsp, calibrated);
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

//  
// thread
//

void processImages(StereoConverter* stConv, ImageConverter* imConv, bool save, bool show)
{
  
  int counter = 0;
  cv::Mat imgR, imgL, imgRnew, imgLnew, imgC, imgCnew;
  cv::Mat dsp, calibrated, clean, clustering(480,752,CV_8UC3);
  ImageAnalyzer callFunction;
  float expTime_corection;
  
  string _dspWindowName("clustering");
  cv::namedWindow(_dspWindowName);
  
  string _dspWindowName1("Red_central");
  cv::namedWindow(_dspWindowName1);
  
  string _dspWindowName2("NDVI");
  cv::namedWindow(_dspWindowName2);
  
  while(!stConv->ready()  ||  !imConv->ready()) {
     usleep(100000);
  }

  int framecounter = 0;
  
  ofstream myfile;
  myfile.open ("../imSaves/clustering.txt");
  ofstream myfile1;
  myfile1.open ("../imSaves/ndvi_classified.txt");
  ofstream myfile2;
  myfile2.open ("../imSaves/img_calibrated.txt");
  ofstream myfile3;
  myfile3.open ("../imSaves/img_disparity.txt");
  ofstream myfile4;
  myfile4.open ("../imSaves/ndvi_1D.txt");
   
   
  struct timeval tv;
  gettimeofday(&tv,NULL);
  unsigned long time_first = 1000000 * tv.tv_sec + tv.tv_usec;
  unsigned long time_el = 0;
  unsigned long time_last = 0;

  while(1) {

   
//    Mat recostruction;
// 0.73 0.6 0.6 0.45
//    reprojectImageTo3D(clean_disparity, recostruction);
//    
//   namedWindow( "image", CV_WINDOW_AUTOSIZE );
//   imshow( "image", recostruction);
//   waitKey(0);

    
    if ( stConv->copyImages(imgR, imgL) && imConv->copyImage_central(imgC) ) {
      
      int diny, dinx;
      diny = imgL.rows/2;
      dinx = imgL.cols/2;
      Size size3(dinx, diny);
  
      cv::resize(imgL, imgLnew, size3);
      cv::resize(imgR, imgRnew, size3);
      cv::resize(imgC, imgCnew, size3);

    expTime_corection = (float)((imConv->_expTime))/(float)((stConv->_expTime));    
//     expTime_corection = (float)((stConv->_expTime))/(float)((imConv->_expTime));    
//       
//     cout << "R " << imgR.size() << " " << imgR.channels()  << endl;
    cout << "expTime_corection " << expTime_corection << endl;
//     cout << "L " << imgL.size() << " " << imgL.channels()  << endl;
//     cout << "C " << imgC.size() << " " << imgC.channels()  << endl;
    
    int  win_size  = 7;

    Disparity img(diny,dinx,win_size,win_size);
    
    Ndvi ndvi(dinx, diny);

//    callFunction.stereo(imgL, imgR, 50, dsp, clean, calibrated);
    callFunction.Detect_Material_online( expTime_corection, imgLnew, imgRnew, imgCnew, 9/2, 0.1, ndvi, img);
    
    
      
    Mat dsp8, dsp8res, dsp82;
//     Size size(imgL.cols, imgL.rows);;
    img.calibrated.convertTo(dsp8, CV_8U);
    img.clean.convertTo(dsp82, CV_8U);
//     cv::resize(dsp8, dsp8res, size);

//         cv::namedWindow("imgLnew");
//         cv::namedWindow("imgRnew");
//         cv::namedWindow("imgCnew");
	
//     cout << "3" << endl;
//     cv::imshow("imgLnew", imgLnew);
//     cv::waitKey(2);
//     
//     cv::imshow("imgRnew", imgRnew);
//     cv::waitKey(2);
//     
//     cv::imshow("imgCnew", imgCnew);
//     cv::waitKey(2);
    
    
    if (show){
      cv::imshow(_dspWindowName, ndvi.clustering  /*dsp8*/ /*ndvi.ndvi_classified*/);
      cv::waitKey(2);
      
//       cv::imshow(_dspWindowName1, dsp8);
//       cv::waitKey(2);
      
//       cv::imshow(_dspWindowName2, ndvi.ndvi);
//       cv::waitKey(2);
    }
      
    if (save){
      stringstream ss;
      ss.fill('0');
      ss << "../imSaves/clustering_" <<  std::setw(10) << internal << framecounter << ".png";
      cv::imwrite( ss.str().c_str(), ndvi.clustering );
      myfile << "file '" <<  ss.str().substr(11) << "'" << endl;
      
      
      stringstream ss1;
      ss1.fill('0');
      ss1 << "../imSaves/ndvi_classified_" <<  std::setw(10) << internal << framecounter << ".png";
      cv::imwrite( ss1.str().c_str(), ndvi.ndvi_classified );
      myfile1 << "file '" <<  ss1.str().substr(11) << "'" << endl;
      
      stringstream ss2;
      ss2.fill('0');
      ss2<< "../imSaves/img_calibrated_" <<  std::setw(10) << internal << framecounter << ".png";
      cv::imwrite( ss2.str().c_str(), dsp8 );
      myfile2 << "file '" <<  ss2.str().substr(11) << "'" << endl;
      
      stringstream ss3;
      ss3.fill('0');
      ss3 << "../imSaves/img_disparity_" <<  std::setw(10) << internal << framecounter << ".png";
      cv::imwrite( ss3.str().c_str(), dsp82*4.0 );
      myfile3 << "file '" <<  ss3.str().substr(11) << "'" << endl;
      
      
      stringstream ss4;
      ss4.fill('0');
      ss4 << "../imSaves/ndvi_1D_" <<  std::setw(10) << internal << framecounter << ".png";
      cv::imwrite( ss4.str().c_str(), (ndvi.ndvi+1.0)*128.0 );
      myfile4 << "file '" <<  ss4.str().substr(11) << "'" << endl;
      
      framecounter++;
    }
    counter++;
    
    
    gettimeofday(&tv,NULL);
    time_el = (1000000 * tv.tv_sec + tv.tv_usec) - time_first;
    std::cout << "freq " << (1000000.0*(double)counter)/time_el << std::endl;
//     std::cout << "time_el " << time_el << std::endl;
//     std::cout << "time_fi " << time_first << std::endl;
//     std::cout << "time_last " << time_last << std::endl;
    myfile << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
    myfile1 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
    myfile2 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
    myfile3 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
    myfile4 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
    
    time_last = time_el;

    
//       cv::waitKey(1);
//     cout << "4" << endl;
    }
  }
  myfile.close();
  myfile1.close();
  myfile2.close();
  myfile3.close();
  myfile4.close();
}




int main(int argc, char **argv)
{
//   XInitThreads();
  
  numimage = 0;

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  cam2 = (StereoConverter*)new StereoConverter(n, "my_stereo/left/image_rect", "my_stereo/left/exposition_time", 1,"my_stereo/right/image_rect", "my_stereo/right/exposition_time", 3 );
  cam1 = (ImageConverter*)new ImageConverter(n, "camera2/image_rect", "camera2/exposition_time", 2);

  
  bool _saveframesthread = false;
  bool _showframesthread = true;
 // cam2->saveframes();
//  cam2->showframes();
//   cam1->saveframes();
  cam1->showframes();
//   
  
  thread t1(processImages, cam2, cam1, _saveframesthread, _showframesthread);


  ros::spin();

  return 0;

}
