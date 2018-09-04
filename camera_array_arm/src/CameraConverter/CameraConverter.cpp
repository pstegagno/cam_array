#include <CameraConverter/CameraConverter.hpp>

using namespace std;

int numimage;

using namespace cv;
using namespace ros;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

static const std::string OPENCV_WINDOW = "spectralViewer";



// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// begin of class CameraConverter
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
  
  
void CameraArrayProject::CameraConverter::saveImage(){
  cout << "CameraConverter :: saveImage 1" << endl;
  _buttonPressed=true;
}
   
   
void CameraArrayProject::CameraConverter::save_time_cb(const std_msgs::Int32ConstPtr& msg)

{
  _expTime = msg->data;
}
   
   
CameraArrayProject::CameraConverter::CameraConverter(ros::NodeHandle nh_, std::string s_name, string  exp_time_topic, int idNum)
  : it_(nh_)
{
  id = idNum;

  // Subscribe to input video feed and publish output video feed
  image_sub_ = it_.subscribe(s_name, 1, &CameraConverter::imageCb, this);
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

CameraArrayProject::CameraConverter::~CameraConverter()
{
  cv::destroyWindow(_windowName);
}


void CameraArrayProject::CameraConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
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

bool CameraArrayProject::CameraConverter::copyImage_central(cv::Mat &out1){
  
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

bool CameraArrayProject::CameraConverter::ready(){
  return img_received;
}






void CameraArrayProject::mouseCallBackFunc(int event, int x, int y, int flags, void* userdata)
{

}


