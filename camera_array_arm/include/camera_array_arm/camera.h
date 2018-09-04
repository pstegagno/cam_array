#include <sstream>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <ccny_mvVirtualDevice/mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <std_msgs/Int32.h>
//#define DEVELOP

#include <PID/PID.hpp>

#define PRESS_A_KEY getchar();


namespace mv_bluefox_driver
{

//-----------------------------------------------------------------------------
class ThreadParameter {
	
	mvIMPACT::acquire::Device* m_pDev;
	
	volatile bool m_boTerminateThread;
	
public:
	ThreadParameter(mvIMPACT::acquire::Device* pDev) : m_pDev(pDev), m_boTerminateThread(false)
	{
	}
	
	mvIMPACT::acquire::Device* device(void) const
	{
		return m_pDev;
	}
	
	bool terminated(void) const
	{
		return m_boTerminateThread;
	}
	
	void terminateThread(void)
	{
		m_boTerminateThread = true;
		// close the camera
		m_pDev->close();
	}
};

  

class Camera
{
public:
  
  /// default constructor 
//   Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
  
  /// default constructor 
  Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh, std::string &numCamera, CameraArrayProject::PIDParameters);
  
  /// "fake" constructor that simulate a camera
  Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh, std::string &numCamera, CameraArrayProject::PIDParameters, bool simulateCamera);
  
  /// initialization of the parameters
  bool initParameters(ros::NodeHandle &_param_nh, std::string &numCamera);
  
  void sendInfo(sensor_msgs::ImagePtr &image, ros::Time time);
  void feedImages();
  
  /// default destructor
  ~Camera();
  
  /// Set exposure time.
  void set_exposure_time(int ep);
  
  /// Get exposure time.
  int get_exposure_time();
  
private:
  
  // this is the controller for the exposure time
  CameraArrayProject::PID exposureTimeController;
  
  // this method performs the control on the exposure time
  void control_exposure(sensor_msgs::ImagePtr image, double error);

  // exposure time
  int exposureTime;
  
// <<<<<<< HEAD
  
  // method to initialize one parameter that will be given to the camera
//   template <class type> bool initCameraParameter(ros::NodeHandle &_param_nh, std::string paramName, type &classInstance, type defaultValue, bool verbose = false)
// =======
  template <class type> bool initCameraParameter(ros::NodeHandle &_param_nh, std::string nmspace, std::string paramName, type &classInstance, type defaultValue, bool verbose = false)
// >>>>>>> 383baf18bc6db227f6ce00a16eae2f77286c8259
  {
    std::stringstream ss;
    ss << paramName << "_" << nmspace;
    std::string fullname(ss.str());
    type tempInstance;
    if (!_param_nh.getParam(fullname, tempInstance)){
      pnode.param(paramName, classInstance, defaultValue);
      if (verbose) std::cout << paramName << " set to default value: " << classInstance << std::endl;
      return false;
    } else {
      pnode.param(paramName, classInstance, tempInstance);
      if (verbose) std::cout << paramName << " set to launch file value: " << classInstance << std::endl;
      return true;
    }
  }
  
  
  // method to initialize one parameter that will not be given to the camera
  template <class type> bool initParameter(ros::NodeHandle &_param_nh, std::string paramName, type &classInstance, type defaultValue, bool verbose = false)
  {
    type tempInstance;
    if (!_param_nh.getParam(paramName, tempInstance)){
      pnode.param(paramName, classInstance, defaultValue);
      if (verbose) std::cout << paramName << " set to default value: " << classInstance << std::endl;
      return false;
    } else {
      pnode.param(paramName, classInstance, tempInstance);
      if (verbose) std::cout << paramName << " set to launch file value: " << classInstance << std::endl;
      return true;
    }
  }

  

//   ros::Publisher exp_time_pub;
  
  ros::Subscriber expose_us;
  
  ros::NodeHandle node, pnode;

  mvIMPACT::acquire::DeviceManager devMgr; // mvIMPACT Acquire device manager
  // establish access to the statistic properties
  mvIMPACT::acquire::Statistics *statistics;
  // create an interface to the device found
  mvIMPACT::acquire::FunctionInterface *fi;
 
  const mvIMPACT::acquire::Request* pRequest;

  ThreadParameter* threaded_device_;

  bool ok;
  boost::mutex img_buffer_mutex_lock_; ///< Thread lock on image buffer
//  unsigned char *img_frame_buffer_; ///< where data of images are stored

  int width_, height_;
  int fps_, skip_frames_, frames_to_skip;
  std::string device, frame;

  bool use_color_; ///< To indicate whether we want to use 3-channel (RGB) or 1-channel (grayscale) images
  bool auto_gain_; ///< To turn/on auto gain
  int expose_us_; ///< Exposure time
  std::string camera_calibration_url_;
  
  std::string num_camera;

  /* Old:
   image_transport::ImageTransport it;
   CameraInfoManager info_mgr;
   image_transport::Publisher pub;
   ros::Publisher info_pub;
   */

  image_transport::CameraPublisher camera_pub_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_;
  boost::thread image_thread;

  pthread_t* pHandles; // pThread handles // TODO: switch these from pThreads to boost
  pthread_attr_t* pAttrs; // TODO: convert to boost
  std::vector<ThreadParameter*> threadParams;
  unsigned int devCnt;
  int lastRequestNr;
  unsigned int request_cnt;
  int request_timeout_ms_; // USB 1.1 on an embedded system needs a large timeout for the first image

  bool initMVDevices(); ///< Initializes several mv device(s). Return true if succeeded // TODO: not being used
  bool initSingleMVDevice(); ///< Initializes a single mv device. Return true if succeeded
  void setCameraSize(mvIMPACT::acquire::SettingsBlueFOX &settings, int width, int height);  ///< Sets frame size, fixing AOI for now

  bool using_pthreads; ///< indicates whether code is using the pthreaded implementation
//  static void* liveThread( void* pData );
//  static unsigned int thread_func( void* pData ); ///< The actual Thread for the device
  bool grab(sensor_msgs::ImagePtr image); ///< Returns true if frame grabbing succeeded

  void exp_time(const std_msgs::Int32ConstPtr& msg);
  
};

} // end namespace mv_bluefox_driver 

