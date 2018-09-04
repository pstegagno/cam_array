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

// #include "camera.h"
#include <thread>
#include <cv.h>


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <camera_info_manager/camera_info_manager.h>

#include <image_geometry/pinhole_camera_model.h>



namespace mv_bluefox_driver
{



class MyCamera
{
public:

	/// default constructor 
	//   MyCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);

	/// default constructor 
	MyCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh, CameraArrayProject::PIDParameters, std::string nsp);

	/// "fake" constructor that simulate a camera
// 	MyCamera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh, CameraArrayProject::PIDParameters, std::string nsp, bool simulateMyCamera);

	/// initialization of the parameters
	bool initParameters(ros::NodeHandle _param_nh);

	void feedImages();

	/// default destructor
	~MyCamera();

	/// Set exposure time.
	inline void set_exposure_time(int exposure)
	{
		expose_us_ = exposure;
		settings->cameraSetting.expose_us.write(expose_us_);
	}

	/// Get exposure time.
	inline int get_exposure_time();
	
	/// Get the mean value of the image.
	inline int get_image_mean_value();
	
	/// Get the mean value of the image.
	inline cv::Mat* get_image(int &exposure, double &intensity){
		frame_in_use_outside = true;
		exposure = frame1_exposure;
		intensity = avgFrame1Intensity;
		return &frame1;
	}
	
	void release_image(){
		frame_in_use_outside = false;
	};
	
	bool frame_is_ready(){
		return frame_ready;
	}
	
	/// start a contonuous acquisition
	inline  void start_continuous_acquisition(){frames_to_grab = -1;}
	
	/// stop a continuous acquisition
	inline void stop_continuous_acquisition(){frames_to_grab = 0;}
	
	/// command to the camera to acquire one frame
	/// this method does not do anything if the camera is in continuous acquisition mode
	inline void acquire_one_frame(){
		frames_to_grab = 1;
		frame_ready = false;
	}
	
	inline void acquire_n_frames(int n){frames_to_grab = n;}

  
private:
  
	// this is the controller for the exposure time
	CameraArrayProject::PID exposureTimeController;
  
	// this method performs the control on the exposure time
	void control_exposure(sensor_msgs::ImagePtr image, double error);

  
  
	// method to initialize one parameter that will be given to the camera
	template <class type> bool initMyCameraParameter(ros::NodeHandle &_param_nh, std::string paramName, type &classInstance, type defaultValue, bool verbose = false)
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
	
	
	// method to initialize one parameter that will not be given to the camera
	template <class type> bool initParameter(ros::NodeHandle &_param_nh, std::string paramName, type &classInstance, type defaultValue, bool verbose = false)
	{
		std::string fullParamName(namesp);
		fullParamName.append(paramName);
		type tempInstance;
		if (!_param_nh.getParam(fullParamName, tempInstance)){
			pnode.param(fullParamName, classInstance, defaultValue);
			if (verbose) std::cout << fullParamName << " set to default value: " << classInstance << std::endl;
			return false;
		} else {
			pnode.param(paramName, classInstance, tempInstance);
			if (verbose) std::cout << fullParamName << " set to launch file value: " << classInstance << std::endl;
			return true;
		}
	}

	
	
	
	int frames_to_grab;
  

//   ros::Publisher exp_time_pub;
  
  ros::Subscriber expose_us;
  
  ros::NodeHandle node, pnode;
	
	
	
	
	// -------------------------------------------------------------------------
	// ---   SDK classes                                                     ---
	// -------------------------------------------------------------------------
	mvIMPACT::acquire::DeviceManager devMgr; // mvIMPACT Acquire device manager
	// establish access to the statistic properties
	mvIMPACT::acquire::Statistics *statistics;
	// create an interface to the device found
	const mvIMPACT::acquire::Request* pRequest;
	
	mvIMPACT::acquire::Device* pDev;
	
	mvIMPACT::acquire::FunctionInterface* fi;
	
	mvIMPACT::acquire::SettingsBlueFOX* settings; // Using the "Base" settings (default)

	mvIMPACT::acquire::ImageDestination* imgDst;

//   ThreadParameter* threaded_device_;
	
	// MY STUFFFFFFF!!!!!!!!!!!!!!!!!!!!!
	
	
	// stuff for rectification;
	image_geometry::PinholeCameraModel model_;
	

  bool ok;
  boost::mutex img_buffer_mutex_lock_; ///< Thread lock on image buffer
//  unsigned char *img_frame_buffer_; ///< where data of images are stored

	std::string namesp;
	
	// camera parameters
	int width_;
	int height_;
	int fps_;
// 	std::string device;
// 	std::string frame;
	int request_timeout_ms_; // USB 1.1 on an embedded system needs a large timeout for the first image

	bool use_color_; ///< To indicate whether we want to use 3-channel (RGB) or 1-channel (grayscale) images
	bool auto_gain_; ///< To turn/on auto gain
	int expose_us_; ///< Exposure time
	
	cv::Mat frame1;
	int frame1_exposure;
	double avgFrame1Intensity;
	// 	cv::Mat frame2;
	
// 	int last_received_frame;
	
// 	cv::Mat* output_frame;
// 	cv::Mat* grabed_frame;
	bool frame_ready;
	bool frame_in_use_outside;
	
	
	// functionality parameters of the class
	std::string camera_calibration_url_;
	std::string camera_serial_number;
	
	camera_info_manager::CameraInfoManager *cam_info_;

	
// 	boost::shared_ptr<camera_info_manager::CameraInfoManager> cam_info_;
// 	boost::thread image_thread;
	
	std::thread* feedImageThread;
	
	unsigned int devCnt;
	
// 	int lastRequestNr;
	
	int image_mean_value;
	
	
	// local methods
	bool initSingleMVDevice(); ///< Initializes a single mv device. Return true if succeeded
		
	bool grab(); ///< Returns true if frame grabbing succeeded
	
	void copyImageFromBuffer(cv::Mat *dest, const unsigned char* src);
	
// 	inline void set_exposure_time(int exposure);
	
	void setMyCameraSize(int w, int h);
	
};

} // end namespace mv_bluefox_driver 

