// system includes
#include <thread>
#include <vector>

// external includes
#include <ros/ros.h>

// package includes
#include <camera_array_arm/MyCamera.h>
#include <StereoConverter/StereoConverter.hpp>
#include <CameraConverter/CameraConverter.hpp>
#include <ImageAnalyzer/ImageAnalyzer.hpp>
#include <std_msgs/Bool.h>

#include <sys/stat.h>

// #define DEBUG_MAIN
#define MAIN_SHOW_IMAGES 0


int numimage;

using namespace cv;
using namespace ros;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

using namespace CameraArrayProject;

static const std::string OPENCV_WINDOW = "spectralViewer";



#define ROWS 480
#define COLS 752




int diny = ROWS/2;
int dinx = COLS/2;
Size size3(dinx, diny);



int counter = 0;
int framecounter = 0;
bool camera_on;





int main(int argc, char **argv){
	
	int exp_number = 0;
	std::string _save_image_folder("/home/odroid/images_ndvi/");
	
	while (exp_number < 9999)
	{
		struct stat sb;
		exp_number++;
		stringstream save_folder;
		save_folder << _save_image_folder << std::setfill('0') << std::setw(4) << exp_number << "/";
		if (!(stat(save_folder.str().c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))){
			_save_image_folder = save_folder.str();
			break;
		}
	}
	mkdir(_save_image_folder.c_str(),  S_IRWXU);

	camera_on = true;
	
	cv::Mat* frameL = NULL;
	cv::Mat* frameR = NULL;
	cv::Mat* frameC = NULL;

  
	ros::init(argc, argv, "mv_camera");
		
	numimage = 0;

	ros::NodeHandle n;

	
	ros::NodeHandle pp("~");
	
	
	PIDParameters pid_params(0, 0.0011,0.0010, 0.001, 0, 0, 0);
	
	// this is the controller for the exposure time L and R cameras
	CameraArrayProject::PID exposureTimeControllerLR;
	CameraArrayProject::PID exposureTimeControllerR;
	// this is the controller for the exposure time C camera
	CameraArrayProject::PID exposureTimeControllerC;
	
	std::string nsp_camera_left, nsp_camera_central, nsp_camera_right; 
	pp.getParam("namespace_camL", nsp_camera_left);
	pp.getParam("namespace_camC", nsp_camera_central);
	pp.getParam("namespace_camR", nsp_camera_right);
	std::cout << "nsp_camera_left        " << nsp_camera_left << std::endl;
	std::cout << "nsp_camera_right       " << nsp_camera_right << std::endl;
	std::cout << "nsp_camera_central     " << nsp_camera_central << std::endl;


	mv_bluefox_driver::MyCamera camL(n, ros::NodeHandle("~"), pid_params, nsp_camera_left);
	
	mv_bluefox_driver::MyCamera camR(n, ros::NodeHandle("~"), pid_params, nsp_camera_right);

	mv_bluefox_driver::MyCamera camC(n, ros::NodeHandle("~"), pid_params, nsp_camera_central);

  
	
	int exposureL, exposureC, exposureR;
	double intensityL, intensityC, intensityR;
	
	
	ros::Time firstFrameTime = ros::Time::now();

	while(ros::ok()){
		ros::spinOnce();
		
		usleep(100);
		
		// frame aquisition
		camL.acquire_one_frame();
		camR.acquire_one_frame();
		camC.acquire_one_frame();
		bool camLready=false, camRready=false, camCready=false;
		
		ros::Time temp = ros::Time::now();
		while( (ros::Time::now() - temp).toSec()<0.5  && !camLready){
			camLready = camL.frame_is_ready();
			usleep(100);
		}
		if (camLready) {
			frameL = camL.get_image(exposureL, intensityL);
		}
		else {
			std::cout << ros::Time::now().toSec() << " Warning: frame not acquired - camLready " << camLready << std::endl;
		}

		temp = ros::Time::now();
		while( (ros::Time::now() - temp).toSec()<0.5  && !camRready){
			camRready = camR.frame_is_ready();
			usleep(100);
		}
		if (camRready) {
			frameR = camR.get_image(exposureR, intensityR);
		}
		else {
			std::cout << ros::Time::now().toSec() << " Warning: frame not acquired - camRready " << camRready << std::endl;
		}

		temp = ros::Time::now();
		while( (ros::Time::now() - temp).toSec()<0.5  && !camCready){
			camCready = camC.frame_is_ready();
			usleep(100);
		}
		if (camCready) {
			frameC = camC.get_image(exposureC, intensityC);
		}
		else {
			std::cout << ros::Time::now().toSec() << " Warning: frame not acquired - camCready " << camCready << std::endl;
		}

		if (!camCready || !camRready || !camLready){
			camL.release_image();
			camR.release_image();
			camC.release_image();
			continue;
		}
		

		
		// here goes exposure control for LR!
		double error= 100-intensityL;
		exposureTimeControllerLR.pid_update(error, exposureL);
		exposureL =  exposureL + exposureTimeControllerLR.control;
		camL.set_exposure_time(exposureL);
		error= 100-intensityR;
		exposureTimeControllerR.pid_update(error, exposureR);
		exposureR =  exposureR + exposureTimeControllerR.control;
		camR.set_exposure_time(exposureR);
		// here goes exposure control for C!
		error= 100-intensityC;
		exposureTimeControllerC.pid_update(error, exposureC);
		exposureC =  exposureC + exposureTimeControllerC.control;
		camC.set_exposure_time(exposureC);
		
		camL.release_image();
		camR.release_image();
		camC.release_image();
			
		
	}
	return 0;
}





