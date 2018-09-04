#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <ccny_mvVirtualDevice/mvIMPACT_CPP/mvIMPACT_acquire.h>
#include <std_msgs/Int32.h>
//#define DEVELOP
#define PRESS_A_KEY getchar();
namespace mv_bluefox_driver
{

//-----------------------------------------------------------------------------
class ThreadParameter
//-----------------------------------------------------------------------------
{
  mvIMPACT::acquire::Device* m_pDev;
  volatile bool m_boTerminateThread;
public:
  ThreadParameter(mvIMPACT::acquire::Device* pDev) :
    m_pDev(pDev), m_boTerminateThread(false)
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

  
class PID {
public:
    double windup_guard;
    double proportional_gain;
    double integral_gain;
    double derivative_gain;
    double prev_error;
    double int_error;
    double control; 
 
    PID(){
       windup_guard = 0;
       proportional_gain = 0.0011;
       integral_gain = 0.0010;
       derivative_gain = 0.001;
       prev_error = 0;
       int_error = 0;
       control = 0;
    }
      
    
    
    
// void pid_zeroize() {
//     // set prev and integrated error to zero
//     this->prev_error = 0;
//     this->int_error = 0;
// }
 
void pid_update( double curr_error)
{
    double diff;
    double p_term;
    double i_term;
    double d_term;
 
    // integration with windup guarding
    this->int_error += (curr_error);
    if (this->int_error < -(this->windup_guard))
        this->int_error = -(this->windup_guard);
    else if (this->int_error > this->windup_guard)
        this->int_error = this->windup_guard;
 
    // differentiation
    diff = ((curr_error - this->prev_error));
 
    // scaling
    p_term = (this->proportional_gain * curr_error);
    i_term = (this->integral_gain     * this->int_error);
    d_term = (this->derivative_gain   * diff);
 
    // summation of terms
    this->control = p_term + i_term + d_term;
 
    // save current error as previous error for next iteration
    this->prev_error = curr_error;
}




void pid_update( double curr_error, double curr_value)
{
    double diff;
    double p_term;
    double i_term;
    double d_term;
 
    // integration with windup guarding
    this->int_error += (curr_error);
    if (this->int_error < -(this->windup_guard))
        this->int_error = -(this->windup_guard);
    else if (this->int_error > this->windup_guard)
        this->int_error = this->windup_guard;
 
    // differentiation
    diff = ((curr_error - this->prev_error));
 
    // scaling
    p_term = (this->proportional_gain *curr_value* curr_error);
    i_term = (this->integral_gain     *curr_value* this->int_error);
    d_term = (this->derivative_gain   *curr_value* diff);
 
    // summation of terms
    this->control = p_term + i_term + d_term;
 
    // save current error as previous error for next iteration
    this->prev_error = curr_error;
    
    
}




};

class Camera
{
public:
  Camera(ros::NodeHandle comm_nh, ros::NodeHandle param_nh);
  void sendInfo(sensor_msgs::ImagePtr &image, ros::Time time);
  void feedImages();
  //void control_expositiion(sensor_msgs::ImagePtr image, float &error);
  ~Camera();

  PID control;
  
private:
  
  
  //parameters
  int expositionTime;

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
  int fps, skip_frames, frames_to_skip;
  std::string device, frame;
  bool rotate;
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

