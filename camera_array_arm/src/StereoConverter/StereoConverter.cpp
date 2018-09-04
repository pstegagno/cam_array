#include <StereoConverter/StereoConverter.hpp>


using namespace std;

int numimage;

using namespace cv;
using namespace ros;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

static const std::string OPENCV_WINDOW = "spectralViewer";


void CameraArrayProject::StereoConverter::saveImage()
{
  cout << "StereoConverter :: saveImage 1" << endl;
  _buttonPressed=true;
  _buttonPressed2=true;
}
   
   
 void CameraArrayProject::StereoConverter::save_time_cb(const std_msgs::Int32ConstPtr& msg)

 {
   _expTime = msg->data;
 }
   
   // costrutture
CameraArrayProject::StereoConverter::StereoConverter(ros::NodeHandle nh_, std::string s_name, string  exp_time_topic, int idNum, std::string s_name2, string  exp_time_topic2, int idNum2)
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


CameraArrayProject::StereoConverter::~StereoConverter()
{
  cv::destroyWindow(_windowName);
}
  
  
  
  
void CameraArrayProject::StereoConverter::controlExpTime(double weight){
  
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


void CameraArrayProject::StereoConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
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
  
void CameraArrayProject::StereoConverter::imageCb2(const sensor_msgs::ImageConstPtr& msg)
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


bool CameraArrayProject::StereoConverter::copyImages(cv::Mat &out1, cv::Mat &out2){
  
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

bool CameraArrayProject::StereoConverter::ready(){
  return img_received && img_received2;
}




  void CameraArrayProject::mouseCallBackFunc(int event, int x, int y, int flags, void* userdata)
  {
  }

  