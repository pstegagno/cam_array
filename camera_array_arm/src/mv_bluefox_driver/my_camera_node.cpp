// system includes
#include <thread>
#include <vector>

// external includes
#include <ros/ros.h>

// package includes
#include <camera_array_arm/MyCamera.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


using namespace cv;
using namespace ros;
using namespace std;
//namespace enc = sensor_msgs::image_encodings;

using namespace CameraArrayProject;

static const std::string OPENCV_WINDOW = "spectralViewer";


#define ROWS 480
#define COLS 752


int diny = ROWS;
int dinx = COLS;
Size size3(dinx, diny);



int main(int argc, char **argv){
	
//	int exp_number = 0;
//	std::string _save_image_folder("/home/odroid/images_ndvi/");
//	
//	while (exp_number < 9999)
//	{
//		struct stat sb;
//		exp_number++;
//		stringstream save_folder;
//		save_folder << _save_image_folder << std::setfill('0') << std::setw(4) << exp_number << "/";
//		if (!(stat(save_folder.str().c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))){
//			_save_image_folder = save_folder.str();
//			break;
//		}
//	}
//	mkdir(_save_image_folder.c_str(),  S_IRWXU);
//
//	outputFile.open("/home/odroid/images_ndvi/outputFile.txt", std::fstream::out);

	int counter =0;
	
	cv::Mat* frameC;
  
	ros::init(argc, argv, "mv_camera");
		
	ros::NodeHandle n;
	
	ros::NodeHandle pp("~");

	PIDParameters pid_params(0, 0.0011,0.0010, 0.001, 0, 0, 0);
	
	// this is the controller for the exposure time L and R cameras
	CameraArrayProject::PID exposureTimeControllerLR;
	CameraArrayProject::PID exposureTimeControllerR;
	// this is the controller for the exposure time C camera
	CameraArrayProject::PID exposureTimeControllerC;
	
	std::string nsp_camera_left, nsp_camera_central, nsp_camera_right; 
	pp.getParam("namespace_camC", nsp_camera_central);
	std::cout << "nsp_camera_central     " << nsp_camera_central << std::endl;

	std::cout << "hi there, starting camera initalization" << std::endl;

	mv_bluefox_driver::MyCamera camC(n, ros::NodeHandle("~"), pid_params, nsp_camera_central);

	std::cout << "camera initalization ended" << std::endl;




	cv_bridge::CvImage img_bridge;
	sensor_msgs::Image img_msg; // >> message to be sent
	std_msgs::Header header; // empty header
	ros::Publisher pub_img = n.advertise<sensor_msgs::Image>("camera", 1);

	
	int exposureC;
	double intensityC;
	
	
	ros::Time firstFrameTime = ros::Time::now();
	std::cout << "starting camera loop" << std::endl;

	ros::Rate loop_rate(10);

	while(ros::ok()){
		ros::spinOnce();
		

		usleep(100);

		camC.acquire_one_frame();
		bool camCready=false;

	
		ros::Time temp = ros::Time::now();

		while( (ros::Time::now() - temp).toSec()<0.5  && !camCready){

			camCready = camC.frame_is_ready(); usleep(100);

			usleep(100);
		}


		if (camCready) { frameC = camC.get_image(exposureC, intensityC); }
		else { std::cout << ros::Time::now().toSec() << " Warning: frame not acquired - camCready " << camCready << std::endl;}


		if (!camCready ){ continue; }
	

		header.seq = counter; // user defined counter
		header.stamp = ros::Time::now(); // time
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, *frameC);
		img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
		pub_img.publish(img_msg);
		cv::waitKey(1);

		camC.release_image();


		double elapsed = (ros::Time::now()-firstFrameTime).toSec();
		counter++;
		if (counter%10==0)std::cout << counter << " " << elapsed << " " << counter/elapsed << std::endl;
		usleep(10000);


		// here goes exposure control for C!
		double error= 80-intensityC;
		exposureTimeControllerC.pid_update(error, exposureC);
		exposureC =  exposureC + exposureTimeControllerC.control;
		camC.set_exposure_time(exposureC);
		//	std::cout << "starting camera loop G " << exposureC << std::endl;

		loop_rate.sleep();
	}

	return 0;
}

