#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include <camera_array_arm/MyCamera.h>

#include <errno.h>
#include <math.h>



#include <cv.h>
#include <highgui.h>
#include<math.h>
#include<new>
#include <iterator>
#include <vector>
#include <algorithm>
#include <sys/time.h>



namespace mv_bluefox_driver
{
  

  
MyCamera::MyCamera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh, CameraArrayProject::PIDParameters pid_params, std::string nsp) :
  exposureTimeController(pid_params), node(_comm_nh), pnode(_param_nh)
{
	
	
	
	frame_in_use_outside = false;
	frame_ready = false;
	
	namesp = nsp;
	// read parameters from launch file
	initParameters(_param_nh);
	
// 	last_received_frame=0;
// 	last_frame=NULL;
	
	// initialize camera
	bool init_success = initSingleMVDevice();
	
	std::cout << "init_success " << init_success << std::endl;
	
	if (init_success)
	{
		cam_info_ = new camera_info_manager::CameraInfoManager(node, "mv_bluefox", camera_calibration_url_);
		// Update the camera model
		sensor_msgs::CameraInfo info_msg = cam_info_->getCameraInfo();
		model_.fromCameraInfo(info_msg);
		
// 		// Create cv::Mat views onto both buffers
// 		const cv::Mat image = cv_bridge::toCvShare(image_msg)->image;
// 		cv::Mat rect;
// 
// 		// Rectify and publish
// 		int interpolation;
// 		{
// 			boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
// 			interpolation = config_.interpolation;
// 		}
// 		model_.rectifyImage(image, rect, interpolation);
		
// 		expose_us = pnode.subscribe ( "exposure_time", 1, &MyCamera::exp_time, this);
		
		ok = true;
		frames_to_grab = 0;
		// start thread to read images
		
		feedImageThread = new std::thread(&MyCamera::feedImages, this);
// 		image_thread = boost::thread(boost::bind(&MyCamera::feedImages, this));
	}
	else
	{
		ROS_WARN("MatrixVision BlueFox could not be initialized!");
	}
}






// MyCamera::MyCamera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh, CameraArrayProject::PIDParameters pid_params, std::string nsp, bool simulateMyCamera) :
//   exposureTimeController(pid_params), node(_comm_nh), pnode(_param_nh)
// {
// 	namesp = nsp;
// 	// read parameters from launch file
// 	initParameters(_param_nh);
// 	
// 	// initialize camera
// 	bool init_success = false;
// 	
// 	if (init_success)
// 	{
// 		// exp_time_pub = pnode.advertise<std_msgs::Int32>("exposition_time", 1);
// 		expose_us = pnode.subscribe ( "exposition_time", 1, &MyCamera::exp_time, this);
// 		
// 		ok = true;
// 		// start thread to read images
// 		feedImageThread = new std::thread(&MyCamera::feedImages, this);
// 	}
// 	else
// 	{
// 		ROS_WARN("MatrixVision BlueFox could not be initialized!");
// 	}
// }




bool MyCamera::initParameters(ros::NodeHandle _param_nh){
	
	ok = false; // because device (camera) is threaded and there is no guaranteed it has been initialized
	
	bool verbose = true;
	
	initMyCameraParameter<bool>		(_param_nh, "auto_gain",		auto_gain_, true, verbose);
	initMyCameraParameter<bool>		(_param_nh, "use_color",		use_color_, false, verbose);
	initMyCameraParameter<int>		(_param_nh, "fps",			fps_, 30, verbose);
	initMyCameraParameter<int>		(_param_nh, "expose_us",		expose_us_, 200, verbose);
	initMyCameraParameter<int>		(_param_nh, "width",			width_, 752, verbose);
	initMyCameraParameter<int>		(_param_nh, "height",			height_, 480, verbose);
// 	initMyCameraParameter<std::string>	(_param_nh, "frame_id",			frame, std::string("camera"), verbose);
	initMyCameraParameter<int>		(_param_nh, "request_timeout_ms",	request_timeout_ms_, 8000, verbose);
	
	// get functionality (non camera) parameters
	initParameter<std::string>	(_param_nh, "serial_number",	camera_serial_number,	std::string("nodefault"), verbose);
	
// 	cv::Settings s;
	initParameter<std::string>	(_param_nh, "calibration_url_",	camera_calibration_url_,std::string(""), verbose);
	std::cout << "!!!! opening: \"" << camera_calibration_url_ << "\"" << std::endl;
// 	cv::FileStorage fs(camera_calibration_url_, cv::FileStorage::READ); // Read the settings
// 	if (!fs.isOpened())
// 	{
// 		std::cout << "Could not open the configuration file: \"" << camera_calibration_url_ << "\"" << std::endl;
// 		return -1;
// 	}
// 	cv::Mat myMat;
// 	fs["MY_MAT_NAME_IN_THE_XML"] >> myMat;
// 	fs.release();                                         // close Settings file

// 	if (!s.goodInput)
// 	{
// 		cout << "Invalid input detected. Application stopping. " << endl;
// 		return -1;
// 	}

	
	return true;
}


MyCamera::~MyCamera()
{
	ok = false;
	feedImageThread->join();
	
	// stop the thread again and close the camera
	std::cout << "Terminating live threads..." << std::endl;
// 	threaded_device_->terminateThread();
}





inline int MyCamera::get_exposure_time(){
	return expose_us_;
}

inline int MyCamera::get_image_mean_value(){
	return image_mean_value;
}



// this method performs the control on the exposure time
void MyCamera::control_exposure(sensor_msgs::ImagePtr image, double error){
	float meanImageValue = 100.0;//TODO put here actual value
	exposureTimeController.pid_update(error, meanImageValue);
}



bool MyCamera::initSingleMVDevice()
{
	unsigned int i = 0; // One single device, take the one that matches the device number listed in the options
	bool deviceFound = false;

	// get number of devices. If 0, error
	devCnt = devMgr.deviceCount();
	if (devCnt == 0)
	{
		std::cout << "No MATRIX VISION device found! Unable to continue!" << std::endl;
		return false;
	}
  else // if numeber of devices != 0 look for the one whose serial number matches the serial number provided in the options
	{
		std::cout << devCnt << " MATRIX VISION device found!" << std::endl;
		for (unsigned int ii=0; ii < devCnt; ii++){
			std::cout <<  " - " << devMgr[ii]->family.read() << "(" << devMgr[ii]->serial.read() << ")" << std::endl;
			if (camera_serial_number == devMgr[ii]->serial.read()){
				std::cout <<  " requested "  << camera_serial_number << " found!" << std::endl;
				i = ii;
				deviceFound = true;
			}
		}
	}
	if (!deviceFound){
		std::cout << "Requested MATRIX VISION device not found! Unable to continue!" << std::endl;
		return false;
	}
	
	// at this point the i-h device of devMgr is the required camera specified 
	// via its serial number so it is stored in the threadParams vector
// 	threaded_device_ = new ThreadParameter(devMgr[i]);
// 	threadParams.push_back(threaded_device_);
	std::cout << devMgr[i]->family.read() << "(" << devMgr[i]->serial.read() << ")" << std::endl;
	std::cout << "The single device will be opened and initialized next ..." << std::endl;
	
// 	device = devMgr[i];
	pDev = devMgr[i];
	try
	{
		pDev->open();
	}
	catch(const mvIMPACT::acquire::ImpactAcquireException& e )
	{
		// this e.g. might happen if the same device is already opened in another process...
		std::cout << "An error occurred while opening the device " << pDev->serial.read() << std::endl;
		std::cout << "Error code: " << e.getErrorCode() << " (" << e.getErrorCodeAsString() << ")." << std::endl;
		std::cout << "Press any key to end the application..." << std::endl;
		PRESS_A_KEY
		return false;
	}
	
	try
	{
		// create an interface to the device found
		fi = new mvIMPACT::acquire::FunctionInterface(pDev);
	}
	catch (const mvIMPACT::acquire::ImpactAcquireException& e)
	{
		// this e.g. might happen if the same device is already opened in another process...
		std::cout << "An error occurred while creating the function interface on device " << pDev->serial.read() << std::endl;
		std::cout << "Error code: " << e.getErrorCode() << " (" << e.getErrorCodeAsString() << ")." << std::endl;
		std::cout << "Press any key to end the application..." << std::endl;
// 				<< std::endl << "Press [ENTER] to end the application..." << std::endl;
		PRESS_A_KEY;
		return false;
	}

	// Set Properties
	settings = new mvIMPACT::acquire::SettingsBlueFOX(pDev); // Using the "Base" settings (default)

	setMyCameraSize(this->width_, this->height_);
	if(auto_gain_)
		settings->cameraSetting.gain_dB.write( 12.0 ); // Maximum gain
	else
		settings->cameraSetting.gain_dB.restoreDefault();
		 //Set expose_us
	settings->cameraSetting.expose_us.write(expose_us_);
	settings->cameraSetting.expose_us.write(expose_us_);
	

	imgDst = new mvIMPACT::acquire::ImageDestination(pDev);
	// TODO: width, height, etc. (defaulting now to highest resolution)
	if(use_color_)
	{
		// when using a byte pointer.
		imgDst->pixelFormat.write( idpfBGR888Packed );     // TODO: Lots of choices (allow more types)
		///  idpfBGR888Packed:
		// 3 bytes        3 bytes        3 bytes      etc.
		// R(1)G(1)B(1)   R(2)G(2)B(2)   R(3)G(3)B(3) etc.
		// ..........................................
		// ...........................   R(n)G(n)B(n)
	}
	else
	{
		// 8Bit/pixel destination image
		imgDst->pixelFormat.write( idpfMono8 );
	}
	return true;
}




void MyCamera::feedImages()
{
	
	frame1 = cv::Mat(height_, width_, CV_8UC1);
// 	frame2 = cv::Mat(height_, width_, CV_8UC1);
	
	ROS_INFO("Starting thread to grab frames");
	

	
	unsigned int pair_id = 0; // a sequence counter really.

	// prefill the capture queue. There can be more then 1 queue for some device, but for this sample
	// we will work with the default capture queue. If a device supports more then one capture or result
	// queue, this will be stated in the manual. If nothing is set about it, the device supports one
	// queue only. Request as many images as possible. If there are no more free requests 'DEV_NO_FREE_REQUEST_AVAILABLE'
	// will be returned by the driver.
	
	
	int result = DMR_NO_ERROR;
	mvIMPACT::acquire::SystemSettings ss(pDev);
	
	const int REQUEST_COUNT = ss.requestCount.read();
	for (int i = 0; i < REQUEST_COUNT; i++)
	{
		result = fi->imageRequestSingle();
		if (result != DMR_NO_ERROR)
		{
			std::cout << "Error while filling the request queue: " << ImpactAcquireException::getErrorCodeAsString(result) << std::endl;
		}
	}
	pRequest = NULL;
	if(grab()){
		pair_id++;
		if(pair_id%100 == 0){
			std::cout << pDev->serial.read()  << " acquired "<< pair_id << " frames." << std::endl;
		}
	}

	
// 	lastRequestNr = INVALID_ID;
	pRequest = NULL;
	
	frames_to_grab = 5;
	while (ok)
	{
		if(frames_to_grab>0 || frames_to_grab==-1){
// 			std::cout << pDev->serial.read()  << " b"<< std::endl;
			fi->imageRequestSingle();
			// grab image frame from buffer
			if(grab()){
				pair_id++;
				if(pair_id%100 == 0){
					std::cout << pDev->serial.read()  << " acquired "<< pair_id << " frames." << std::endl;
				}
			}
// 			frames_to_grab--;
			frames_to_grab = std::max(-1, --frames_to_grab);
		}
		else{
			usleep(100);
// 			std::cout << pDev->serial.read()  << " " << frames_to_grab << std::endl;
		}
	} // while ok

	// free the last potential locked request
// 	if (fi->isRequestNrValid(lastRequestNr))
// 		fi->imageRequestUnlock(lastRequestNr);

	// clear the request queue
	fi->imageRequestReset(0, 0);
}



/// ######################################################################################
/// ######################################################################################
/// ######################################################################################
/// ######################################################################################
bool MyCamera::grab()
{
	//lock and unlock through mvImpact API
	boost::mutex::scoped_lock l(img_buffer_mutex_lock_);

	const unsigned char *img_frame = NULL;
	bool status = false;
	int requestNr = INVALID_ID;
	// This next comment is valid once we have a display:
	// we always have to keep at least 2 images as the display module might want to repaint the image, thus we
	// can't free it unless we have a assigned the display to a new buffer.


	// wait for results from the default capture queue
	requestNr = fi->imageRequestWaitFor(request_timeout_ms_);
	if (fi->isRequestNrValid(requestNr))
	{
		pRequest = fi->getRequest(requestNr);
		if (pRequest->isOK())
		{
			
			frame1_exposure = expose_us_;
			// Image acquired here 
			img_frame = (const unsigned char*)pRequest->imageData.read();
			
			copyImageFromBuffer(&frame1, img_frame);
				
			model_.rectifyImage(frame1, frame1);
			
			avgFrame1Intensity = cv::mean( frame1 )(0);
			frame_ready = true;
			
// 			std::cout<<" pixel value is " << avgFrame1Intensity << std::endl;
			
// 			float error= 128-avgFrame1Intensity;
// 			exposureTimeController.pid_update(error, expose_us_);
// 			expose_us_ =  expose_us_ + exposureTimeController.control;
// 			settings->cameraSetting.expose_us.write(expose_us_);

				
// 				last_received_frame=1;
// 				last_frame=&frame1;
// 			}
// 			else if (last_received_frame==1){
// 					
// 				copyImageFromBuffer(&frame2, img_frame);
// 				
// 				last_received_frame=2;
// 				last_frame=&frame2;
// 			}
			status = true;
			if (fi->isRequestNrValid(requestNr))
			{
// 			this image has been saved thus the buffer is no longer needed...
				fi->imageRequestUnlock(requestNr);
			}
			
// 				std::cout  << "d" <<  std::endl;

			
// 				std::cout  << "e" <<  std::endl;
// 				std::cout << "Grabbed image" << std::endl; 
//std::cout<<" the size is " << sizeof(img_frame)<< std::endl;
//   double coeff = 1.1, pixel_value , sum=0, error;
//   //const unsigned char *sum;
//    ROS_INFO("grab");
//  // expose_us_ += 10;
//     for(int i = 0; i < 480*752; i++)
//     {   
// 	    sum=sum+img_frame[i];    
//     }
//   pixel_value=sum/(480 * 752);
//   std::cout<<" pixel value is " << pixel_value << std::endl;
//   //std::cout<<" the sum is " << sum<< std::endl;
//   error= 128-pixel_value;
//   control.pid_update(error, expose_us_);
//   expose_us_ =  expose_us_ + control.control;
//  //expose_us_ =  expose_us_ + coeff*error;
//   //std::cout<<" the expose_us_ is " << expose_us_ << std::endl;
//   expose_us_ = std::max(expose_us_,12);
//   pnode.param("expose_us", expose_us_, expose_us_);
			
			
// 				mvIMPACT::acquire::SettingsBlueFOX settings(pDev); // Using the "Base" settings (default)
// 				settings->cameraSetting.expose_us.write(expose_us_);
			
			//   std_msgs::Int32 expTimemsg;
			//   expTimemsg.data = expose_us_;
			//   exp_time_pub.publish(expTimemsg);
				

		}
		else
		{
			std::cerr << "Error: " << pRequest->requestResult.readS() << std::endl;
			status = false;
		}

		
// 		if (fi->isRequestNrValid(lastRequestNr))
// 		{
			// this image has been displayed thus the buffer is no longer needed...
// 			fi->imageRequestUnlock(lastRequestNr);
// 		}
		
// 		lastRequestNr = requestNr;
		// send a new image request into the capture queue
	}
	else
	{
		// If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
		// additional information under TDMR_ERROR in the interface reference (
		std::cout << "imageRequestWaitFor failed (" << requestNr << ", "
				<< ImpactAcquireException::getErrorCodeAsString(requestNr) << ", device "
				<< pDev->serial.read() << ")" << ", timeout value too small?" << std::endl;
		status = false;
	}
	return status;
}


void MyCamera::copyImageFromBuffer(cv::Mat *dest, const unsigned char* src){
	int limit = 3*height_*width_;
	int counter = 0;
	unsigned char* ptr = &(dest->at<uchar>(0, 0));
	while(counter!=limit)
	{
		*ptr = src[counter];
		ptr++;
		counter += 3;
	}
}



void MyCamera::setMyCameraSize(int w, int h)
{
	settings->cameraSetting.aoiWidth.write(w);
	settings->cameraSetting.aoiHeight.write(h);
	frame1 = cv::Mat(h,w,CV_8UC1);
// 	frame2 = cv::Mat(h,w,CV_8UC1);
}



} // end namespace mv_bluefox_driver





