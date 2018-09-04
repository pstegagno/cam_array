#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include <std_msgs/Int32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include <camera_array_arm/camera.h>

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
  

  
Camera::Camera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh, std::string &numCamera, CameraArrayProject::PIDParameters pid_params) :
  exposureTimeController(pid_params), node(_comm_nh), pnode(_param_nh)
{
  
  initParameters(_param_nh, numCamera);

  bool init_success = false;

  if(using_pthreads)
    init_success = initMVDevices();
  else
    init_success = initSingleMVDevice();

  if (init_success)
  {
//     exp_time_pub = pnode.advertise<std_msgs::Int32>("exposition_time", 1);
    image_transport::ImageTransport it(pnode);
    // advertise image streams and info streams
    //      pub = it.advertise("image_raw", 1);
    camera_pub_ = it.advertiseCamera("image_raw", 1); // Publish raw image and camera info
    //      cam_info_pub_ = node.advertise<CameraInfo>("camera_info", 1);
    // Camera Information and data
    cam_info_
        = boost::shared_ptr<camera_info_manager::CameraInfoManager>(
                                                                    new camera_info_manager::CameraInfoManager(
                                                                                                               pnode,
                                                                                                               "mv_bluefox", // The camera name
                                                                                                               camera_calibration_url_));
	
    expose_us = pnode.subscribe ( "exposition_time", 1, &Camera::exp_time, this);



    if (cam_info_->isCalibrated())
    {
      ROS_INFO("MV BlueFox camera has loaded calibration file");
    }

    ROS_INFO("Publishing images via \"image_transport\"");
    // %EndTag(PUB)%

    ok = true;
    image_thread = boost::thread(boost::bind(&Camera::feedImages, this));
  }
  else
  {
    ROS_WARN("MatrixVision BlueFox could not be initialized!");
  }
}






Camera::Camera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh, std::string &numCamera, CameraArrayProject::PIDParameters pid_params, bool simulateCamera) :
  exposureTimeController(pid_params), node(_comm_nh), pnode(_param_nh)
{
  initParameters(_param_nh, numCamera);

  bool init_success = false;
  if (init_success)
  {

    image_transport::ImageTransport it(pnode);
    // advertise image streams and info streams
    //      pub = it.advertise("image_raw", 1);
    camera_pub_ = it.advertiseCamera("image_raw", 1); // Publish raw image and camera info
    //      cam_info_pub_ = node.advertise<CameraInfo>("camera_info", 1);
    // Camera Information and data
    cam_info_
        = boost::shared_ptr<camera_info_manager::CameraInfoManager>(
                                                                    new camera_info_manager::CameraInfoManager(
                                                                                                               pnode,
                                                                                                               "mv_bluefox", // The camera name
                                                                                                               camera_calibration_url_));
	
    expose_us = pnode.subscribe ( "exposition_time", 1, &Camera::exp_time, this);



    if (cam_info_->isCalibrated())
    {
      ROS_INFO("MV BlueFox camera has loaded calibration file");
    }

    ROS_INFO("Publishing images via \"image_transport\"");
    // %EndTag(PUB)%

    ok = true;
    image_thread = boost::thread(boost::bind(&Camera::feedImages, this));
  }
  else
  {
    ROS_WARN("MatrixVision BlueFox could not be initialized!");
  }
}

bool Camera::initParameters(ros::NodeHandle &_param_nh, std::string &numCamera){
  ok = false; // because device (camera) is threaded and there is no guaranteed it has been initialized
  // TODO:fill out appropriate values (Parametrize the rest of device properties)

  num_camera = numCamera;



  exposureTime = 200;
  
  // Auto Gain "on"
//   bool _auto_gain;
//   if (!_param_nh.getParam("auto_gain", _auto_gain)){
//     pnode.param("auto_gain", auto_gain_, true);
//     std::cout << "!!!!! auto_gain set to default value: " << auto_gain_ << std::endl;
//   } else {
//     pnode.param("auto_gain", auto_gain_, _auto_gain);
//     std::cout << "!!!!! auto_gain set to value: " << auto_gain_ << std::endl;
//   }
  
  bool verbose = true;
  
  initCameraParameter<bool>(_param_nh, num_camera, "auto_gain", auto_gain_, true, verbose);
  initCameraParameter<bool>(_param_nh, num_camera, "use_color", use_color_, false, verbose);
  initCameraParameter<int>(_param_nh, num_camera, "fps", fps_, 30, verbose);
  initCameraParameter<int>(_param_nh, num_camera, "skip_frames", skip_frames_, 0, verbose);
  initCameraParameter<int>(_param_nh, num_camera, "expose_us", expose_us_, exposureTime, verbose);
  initCameraParameter<int>(_param_nh, num_camera, "width", width_, 752, verbose);
  initCameraParameter<int>(_param_nh, num_camera, "height", height_, 480, verbose);
  initCameraParameter<std::string>(_param_nh, num_camera, "frame_id", frame, std::string("camera"), verbose);
  initCameraParameter<int>(_param_nh, num_camera, "request_timeout_ms", request_timeout_ms_, 8000, verbose);

  frames_to_skip = 0;

//   pnode.param("use_color", use_color_, false);
//   pnode.param("fps", fps_, 30);
//   pnode.param("skip_frames", skip_frames_, 0);
//   pnode.param("expose_us", expose_us_, exposureTime);
//   pnode.param("width", width_, 752);
//   pnode.param("height", height_, 480);
//   pnode.param("frame_id", frame, std::string("camera"));
//   pnode.param("request_timeout_ms", request_timeout_ms_, 8000);

  // set up information manager
  if (!_param_nh.getParam("calibration_file", camera_calibration_url_))
  {
    camera_calibration_url_ = "";
  }
  std::cout << "calibration_file: " << camera_calibration_url_ << std::endl;
  //  pnode.getParam("camera_info_url", url);
  //  info_mgr.loadCameraInfo(url);

  std::string mode_str;
  pnode.getParam("camera_mode", mode_str);
  // TODO: allow to set color encoding modes

  // initialize the cameras and turn on the streamer
  using_pthreads = false;

  bool init_success = false;
  
  return true;
}


Camera::~Camera()
{
  ok = false;
  image_thread.join();

  // stop all threads again and close the camera
  std::cout << "Terminating live threads..." << std::endl;
  size_t vSize = threadParams.size();
  for (unsigned int k = 0; k < vSize; k++)
  {
    std::cout << "Terminating thread " << k << "." << std::endl;
    threadParams[k]->terminateThread(); // This closes each device
  }

  // wait until each live thread has terminated.
  if (using_pthreads)
  {
    for (unsigned int j = 0; j < devCnt; j++)
    {
      std::cout << "Waiting for thread " << j << " to terminate." << std::endl;
      pthread_join(pHandles[j], NULL);
    }
  }
  std::cout << "All capture threads terminated." << std::endl;

  // free resources
  for (unsigned int l = 0; l < vSize; l++)
  {
    delete threadParams[l];
    std::cout << "threaded parameter " << l << " removed." << std::endl;
  }

  if (using_pthreads)
  {
    delete[] pHandles;
    std::cout << "All thread handles removed." << std::endl;

    delete[] pAttrs;
    std::cout << "All thread atributes removed." << std::endl;
  }

}




  inline void Camera::set_exposure_time(int ep){
    exposureTime = ep;
  }
  
  inline int Camera::get_exposure_time(){
    return exposureTime;
  }

  void Camera::exp_time(const std_msgs::Int32ConstPtr& msg)
{
  exposureTime = msg->data;
}


  // this method performs the control on the exposure time
  void Camera::control_exposure(sensor_msgs::ImagePtr image, double error){
    float meanImageValue = 100.0;//TODO put here actual value
    exposureTimeController.pid_update(error, meanImageValue);
  }


/*
void* Camera::liveThread(void* pData)
{
  thread_func(pData);
  return NULL;
}
*/

bool Camera::initMVDevices()
{

  devCnt = devMgr.deviceCount();
  if (devCnt == 0)
  {
    std::cout << "No MATRIX VISION device found! Unable to continue!" << std::endl;
    return false;
  }
  pHandles = new pthread_t[devCnt];
  pAttrs = new pthread_attr_t[devCnt];
  // store all device infos in a vector
  // and start the execution of a 'live' thread for each device.

  for (unsigned int i = 0; i < devCnt; i++)
  {
    threadParams.push_back(new ThreadParameter(devMgr[i]));
    std::cout << devMgr[i]->family.read() << "(" << devMgr[i]->serial.read() << ")" << std::endl;
  }

  // start live threads
  for (unsigned int j = 0; j < devCnt; j++)
  {
    pthread_attr_init(&pAttrs[j]);
    // you can set the stack size like this: pthread_attr_setstacksize (&pAttrs[j], 1024*1024);
    // TODO: not doing multiple devices yet...because I'd rather use boost threads for now
//    pthread_create(&pHandles[j], &pAttrs[j], liveThread, (void *)threadParams[j]);
  }

  // now all threads will start running...
  std::cout << "The initialisation of the devices might take some time ..." << std::endl;
  return true;
}

void Camera::setCameraSize(mvIMPACT::acquire::SettingsBlueFOX& settings, int width, int height)
{
  settings.cameraSetting.aoiWidth.write( width );
  settings.cameraSetting.aoiHeight.write( height );
}

bool Camera::initSingleMVDevice()
{
  unsigned int i = 0; // One single device (take first available) TODO: allow for a choice via the helper
  bool deviceFound = false;

  devCnt = devMgr.deviceCount();
  if (devCnt == 0)
  {
    std::cout << "No MATRIX VISION device found! Unable to continue!" << std::endl;
    return false;
  }
  else {
    std::cout << devCnt << " MATRIX VISION device found!" << std::endl;
    for (unsigned int ii=0; ii < devCnt; ii++){
      std::cout <<  " - " << devMgr[ii]->family.read() << "(" << devMgr[ii]->serial.read() << ")" << std::endl;
      if (num_camera == devMgr[ii]->serial.read()){
	std::cout <<  " requested "  << num_camera << " found!" << std::endl;
	i = ii;
	deviceFound = true;
      }
    }
  }
  
  if (!deviceFound){
    std::cout << "Requested MATRIX VISION device not found! Unable to continue!" << std::endl;
    return false;
  }


  threaded_device_ = new ThreadParameter(devMgr[i]);

  threadParams.push_back(threaded_device_);

  std::cout << devMgr[i]->family.read() << "(" << devMgr[i]->serial.read() << ")" << std::endl;

  std::cout << "The single device will be opened and initialized next ..." << std::endl;


  try
  {
    threaded_device_->device()->open();
  }
  catch (const mvIMPACT::acquire::ImpactAcquireException& e)
  {
    // this e.g. might happen if the same device is already opened in another process...
    std::cout << "An error occurred while opening the device " << threaded_device_->device()->serial.read()
        << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << ")). Terminating thread."
        << std::endl << "Press [ENTER] to end the application..." << std::endl;
    PRESS_A_KEY;
    return false;
  }


  request_cnt = 0;
#ifdef DEVELOP
  try
   {
    // establish access to the statistic properties
      statistics = new mvIMPACT::acquire::Statistics(threaded_device_->device());
   }
   catch (const mvIMPACT::acquire::ImpactAcquireException& e)
   {
     // this e.g. might happen if the same device is already opened in another process...
     std::cout << "An error occurred while initializing the statistical information on device " << threaded_device_->device()->serial.read()
         << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << ")). Terminating thread."
         << std::endl << "Press [ENTER] to end the application..." << std::endl;
     PRESS_A_KEY;
     return false;
   }
#endif
  try
   {
      // create an interface to the device found
      fi = new mvIMPACT::acquire::FunctionInterface(threaded_device_->device());
   }
   catch (const mvIMPACT::acquire::ImpactAcquireException& e)
   {
     // this e.g. might happen if the same device is already opened in another process...
     std::cout << "An error occurred while creating the function interface on device " << threaded_device_->device()->serial.read()
         << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << ")). Terminating thread."
         << std::endl << "Press [ENTER] to end the application..." << std::endl;
     PRESS_A_KEY;
     return false;
   }

   // Set Properties
   mvIMPACT::acquire::SettingsBlueFOX settings(threaded_device_->device()); // Using the "Base" settings (default)

   setCameraSize(settings, this->width_, this->height_);

   if(auto_gain_)
     settings.cameraSetting.gain_dB.write( 12.0 ); // Maximum gain
   else
     settings.cameraSetting.gain_dB.restoreDefault();
		 //Set expose_us
	 settings.cameraSetting.expose_us.write(expose_us_);
//    setCameraSize(settings, this->width_, this->height_);

   mvIMPACT::acquire::ImageDestination imgDst(threaded_device_->device());
   // TODO: width, height, etc. (defaulting now to highest resolution)
   if(use_color_)
   {
     // when using a byte pointer.
     imgDst.pixelFormat.write( idpfBGR888Packed );     // TODO: Lots of choices (allow more types)
     ///  idpfBGR888Packed:
     // 3 bytes        3 bytes        3 bytes      etc.
     // R(1)G(1)B(1)   R(2)G(2)B(2)   R(3)G(3)B(3) etc.
     // ..........................................
     // ...........................   R(n)G(n)B(n)
   }
   else
   {
     // 8Bit/pixel destination image
     imgDst.pixelFormat.write( idpfMono8 );
   }



  return true;
}

/*
unsigned int Camera::thread_func(void* pData)
{
  ThreadParameter* pThreadParameter = reinterpret_cast<ThreadParameter*> (pData);

  unsigned int cnt = 0;

  try
  {
    pThreadParameter->device()->open();
  }
  catch (const mvIMPACT::acquire::ImpactAcquireException& e)
  {
    // this e.g. might happen if the same device is already opened in another process...
    std::cout << "An error occurred while opening the device " << pThreadParameter->device()->serial.read()
        << "(error code: " << e.getErrorCode() << "(" << e.getErrorCodeAsString() << ")). Terminating thread."
        << std::endl << "Press [ENTER] to end the application..." << std::endl;
    PRESS_A_KEY;
    return 0;
  }

  // establish access to the statistic properties
  mvIMPACT::acquire::Statistics statistics(pThreadParameter->device());
  // create an interface to the device found
  mvIMPACT::acquire::FunctionInterface fi(pThreadParameter->device());

  // prefill the capture queue. There can be more then 1 queue for some device, but for this sample
  // we will work with the default capture queue. If a device supports more then one capture or result
  // queue, this will be stated in the manual. If nothing is set about it, the device supports one
  // queue only. Request as many images as possible. If there are no more free requests 'DEV_NO_FREE_REQUEST_AVAILABLE'
  // will be returned by the driver.
  int result = DMR_NO_ERROR;
  mvIMPACT::acquire::SystemSettings ss(pThreadParameter->device());
  const int REQUEST_COUNT = ss.requestCount.read();
  for (int i = 0; i < REQUEST_COUNT; i++)
  {
    result = fi.imageRequestSingle();
    if (result != DMR_NO_ERROR)
    {
      std::cout << "Error while filling the request queue: " << ImpactAcquireException::getErrorCodeAsString(result)
          << std::endl;
    }
  }

  // run thread loop
  const mvIMPACT::acquire::Request* pRequest = 0;
  const unsigned int timeout_ms = 8000; // USB 1.1 on an embedded system needs a large timeout for the first image
  int requestNr = INVALID_ID;
  // This next comment is valid once we have a display:
  // we always have to keep at least 2 images as the display module might want to repaint the image, thus we
  // can't free it unless we have a assigned the display to a new buffer.
  int lastRequestNr = INVALID_ID;
  while (!pThreadParameter->terminated())
  {
    // wait for results from the default capture queue
    requestNr = fi.imageRequestWaitFor(timeout_ms);
    if (fi.isRequestNrValid(requestNr))
    {
      pRequest = fi.getRequest(requestNr);
      if (fi.isRequestOK(pRequest))
      {
#ifdef DEVELOP
        ++cnt;
        // here we can display some statistical information every 100th image
        if (cnt % 100 == 0)
        {
          std::cout << "Info from " << pThreadParameter->device()->serial.read() << ": " << statistics.framesPerSecond
              << ", " << statistics.errorCount << ", " << statistics.captureTime_s << ", " << statistics.frameCount
              << std::endl;
        }
#endif
        if(ok) // it's only ok when there is publisher in which to feed the images to
        {
        // TODO: (const unsigned char*)pRequest->imageData.read()
        }
      }
      else
      {
        std::cerr << "Error: " << pRequest->requestResult.readS() << std::endl;
      }

      if (fi.isRequestNrValid(lastRequestNr))
      {
        // this image has been displayed thus the buffer is no longer needed...
        fi.imageRequestUnlock(lastRequestNr);
      }
      lastRequestNr = requestNr;
      // send a new image request into the capture queue
      fi.imageRequestSingle();
    }
    else
    {
      // If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
      // additional information under TDMR_ERROR in the interface reference (
      std::cout << "imageRequestWaitFor failed (" << requestNr << ", "
          << ImpactAcquireException::getErrorCodeAsString(requestNr) << ", device "
          << pThreadParameter->device()->serial.read() << ")" << ", timeout value too small?" << std::endl;
    }
  }

  // free the last potential locked request
  if (fi.isRequestNrValid(requestNr))
  {
    fi.imageRequestUnlock(requestNr);
  }
  // clear the request queue
  fi.imageRequestReset(0, 0);
  return 0;
}
*/

void Camera::sendInfo(sensor_msgs::ImagePtr &image, ros::Time time)
{
  sensor_msgs::CameraInfoPtr cam_info(new sensor_msgs::CameraInfo(cam_info_->getCameraInfo()));
  //      CameraInfoPtr info(new CameraInfo(info_mgr.getCameraInfo()));

  // Throw out any CamInfo that's not calibrated to this camera mode
  if (cam_info->K[0] != 0.0 && (image->width != cam_info->width || image->height != cam_info->height))
  {
    cam_info.reset(new sensor_msgs::CameraInfo());
  }

  // If we don't have a calibration, set the image dimensions
  if (cam_info->K[0] == 0.0)
  {
    cam_info->width = image->width;
    cam_info->height = image->height;
  }

  cam_info->header.stamp = time;
  cam_info->header.frame_id = frame;

  // info_pub.publish(cam_info); // Not needed by itself. It rather gets published with image below
  camera_pub_.publish(image, cam_info);
}


void Camera::feedImages()
{
  
  
  
  ROS_INFO("frame");

  
  unsigned int pair_id = 0; // a sequence counter really.

  // prefill the capture queue. There can be more then 1 queue for some device, but for this sample
  // we will work with the default capture queue. If a device supports more then one capture or result
  // queue, this will be stated in the manual. If nothing is set about it, the device supports one
  // queue only. Request as many images as possible. If there are no more free requests 'DEV_NO_FREE_REQUEST_AVAILABLE'
  // will be returned by the driver.
  int result = DMR_NO_ERROR;
  mvIMPACT::acquire::SystemSettings ss(threaded_device_->device());
  const int REQUEST_COUNT = ss.requestCount.read();
  for (int i = 0; i < REQUEST_COUNT; i++)
  {
    result = fi->imageRequestSingle();
    if (result != DMR_NO_ERROR)
    {
      std::cout << "Error while filling the request queue: " << ImpactAcquireException::getErrorCodeAsString(result)
          << std::endl;
    }
  }

  lastRequestNr = INVALID_ID;
  pRequest = NULL;

  while (ok)
  {


//    if (camera_pub_.getNumSubscribers() > 0)
    {
      ros::Time capture_time = ros::Time::now();
      sensor_msgs::ImagePtr image(new sensor_msgs::Image);



            /* Read in every frame the camera generates, but only send each
             * (skip_frames_ + 1)th frame. It's set up this way just because
             * this is based on Stereo...
             */
      if (skip_frames_ == 0 || frames_to_skip == 0)
      {
        // grab image frame from buffer
        if (grab(image))
        {
          image->header.stamp = capture_time;
          image->header.seq = pair_id;
          image->header.frame_id = frame;
/// khkb

          // img_pub_.publish(image); // Not needed because we are using camera publisher instead (see sendInfo())

          sendInfo(image, capture_time);

          ++pair_id;
        }

        frames_to_skip = skip_frames_;   // TODO: just leave it like this for now
      }
      else
      {
        frames_to_skip--;
      }

      //        if (img_frame) cam->release(idx);

    } // if there are subscribers to camera topic
  } // while ok

  // free the last potential locked request
  if (fi->isRequestNrValid(lastRequestNr))
    fi->imageRequestUnlock(lastRequestNr);

  // clear the request queue
  fi->imageRequestReset(0, 0);
}



/// ######################################################################################
/// ######################################################################################
/// ######################################################################################
/// ######################################################################################
bool Camera::grab(sensor_msgs::ImagePtr image)
{
  
  // AGGIUNTO da noi per cambiare il tempo di esposizione durante l'esecuzione del programma
  
   
  
  
  
  
  //lock and unlock through mvImpact API
  boost::mutex::scoped_lock l(img_buffer_mutex_lock_);

  const unsigned char *img_frame = NULL;

  bool status;

  int requestNr = INVALID_ID;
  // This next comment is valid once we have a display:
  // we always have to keep at least 2 images as the display module might want to repaint the image, thus we
  // can't free it unless we have a assigned the display to a new buffer.

  if (!threaded_device_->terminated())
  {
    // wait for results from the default capture queue
    requestNr = fi->imageRequestWaitFor(request_timeout_ms_);
    if (fi->isRequestNrValid(requestNr))
    {
      pRequest = fi->getRequest(requestNr);
      if (fi->isRequestOK(pRequest))
      {
       int channels =  pRequest->imageChannelCount.read();

       image->height = pRequest->imageHeight.read();
       image->width = pRequest->imageWidth.read();
       image->step = channels * width_; // step is the line offset

       if(channels == 1)
         // Grayscale
       {
         image->encoding = sensor_msgs::image_encodings::MONO8;
       }
       else if (channels == 3)
       {
         image->encoding = sensor_msgs::image_encodings::RGB8; // TODO: Change encoding depending on property of camera (with more encodings)
       }

        image->data.resize(image->step * image->height);
	
	// Image acquired here 
        img_frame = (const unsigned char*)pRequest->imageData.read();
        memcpy(&image->data[0], img_frame, width_ * height_ * channels);
	
	//std::cout<<" the size is " << sizeof(img_frame)<< std::endl;


//   double coeff = 1.1, pixel_value , sum=0, error;
//   
//   //const unsigned char *sum;
//   
//    ROS_INFO("grab");
//  // exposureTime += 10;
//   
//   
//     for(int i = 0; i < 480*752; i++)
//     {   
// 	    sum=sum+img_frame[i];    
//     }
//   
//   pixel_value=sum/(480 * 752);
//   std::cout<<" pixel value is " << pixel_value << std::endl;
//   //std::cout<<" the sum is " << sum<< std::endl;
//   
//   error= 128-pixel_value;
// 
//   control.pid_update(error, exposureTime);
//   exposureTime =  exposureTime + control.control;
//  //exposureTime =  exposureTime + coeff*error;
//      
//   //std::cout<<" the exposureTime is " << exposureTime << std::endl;
//   exposureTime = std::max(exposureTime,12);


   

	
   
  pnode.param("expose_us", expose_us_, exposureTime);
  mvIMPACT::acquire::SettingsBlueFOX settings(threaded_device_->device()); // Using the "Base" settings (default)
  settings.cameraSetting.expose_us.write(expose_us_);
  
  
  
//   std_msgs::Int32 expTimemsg;
//   expTimemsg.data = expose_us_;
//   exp_time_pub.publish(expTimemsg);
	
	
	
	
	

        status = true;

#ifdef DEVELOP
        ++request_cnt;
        // here we can display some statistical information every 100th image
        if (request_cnt % 100 == 0)
        {
          std::cout << "Info from Device Serial #" << threaded_device_->device()->serial.read() << " FPS: " << statistics->framesPerSecond.read()
              << ", Error Count: " << statistics->errorCount.read() << ", Capture Time: " << statistics->captureTime_s.read() << " s, Frame Count: " << statistics->frameCount.read()
              << std::endl;
          ROS_INFO("%d Channels", channels);
        }
#endif
      }
      else
      {
        std::cerr << "Error: " << pRequest->requestResult.readS() << std::endl;
        status = false;
      }

      if (fi->isRequestNrValid(lastRequestNr))
      {
        // this image has been displayed thus the buffer is no longer needed...
        fi->imageRequestUnlock(lastRequestNr);
      }
      lastRequestNr = requestNr;
      // send a new image request into the capture queue
      fi->imageRequestSingle();
    }
    else
    {
      // If the error code is -2119(DEV_WAIT_FOR_REQUEST_FAILED), the documentation will provide
      // additional information under TDMR_ERROR in the interface reference (
      std::cout << "imageRequestWaitFor failed (" << requestNr << ", "
          << ImpactAcquireException::getErrorCodeAsString(requestNr) << ", device "
          << threaded_device_->device()->serial.read() << ")" << ", timeout value too small?" << std::endl;
      status = false;
    }
  }


  return status;
}




} // end namespace mv_bluefox_driver





