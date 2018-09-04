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



// cv::Mat imgR, imgL(ROWS,COLS,CV_8UC3), imgC;
cv::Mat imgRnew(size3, CV_8UC1), imgLnew(size3, CV_8UC1), imgCnew(size3, CV_8UC1);
cv::Mat dsp, calibrated, clean, clustering(ROWS,COLS,CV_8UC3);
Mat dsp8, calibrated8, clean8;
ImageAnalyzer* callFunction;
int counter = 0;
int framecounter = 0;
bool camera_on;

std::fstream outputFile;




int win_size  = 7;
Disparity img(diny,dinx,win_size,win_size);
Ndvi ndvi(dinx, diny);


void processImages(cv::Mat *imL, int expoL, cv::Mat *imR, int expoR, cv::Mat *imC, int expoC, bool save, bool show)
{

	
#ifdef DEBUG_MAIN
TIME_DEBUG_INIT();
#endif
	
	float expTime_correction;
	


//    Mat recostruction;
// 0.73 0.6 0.6 0.45
//    reprojectImageTo3D(clean_disparity, recostruction);
//    
//   namedWindow( "image", CV_WINDOW_AUTOSIZE );
//   imshow( "image", recostruction);
//   waitKey(0);

  
	cv::resize(*imL, imgLnew, size3);
	cv::resize(*imR, imgRnew, size3);
	cv::resize(*imC, imgCnew, size3);
			
	
//       
//     cout << "R " << imgR.size() << " " << imgR.channels()  << endl;
//     std::cout << "expTime_correction " << expTime_correction << std::endl;
//     cout << "L " << imgL.size() << " " << imgL.channels()  << endl;
//     cout << "C " << imgC.size() << " " << imgC.channels()  << endl;
	
#ifdef DEBUG_MAIN
TIME_DEBUG(main::processImages::resize_images);
#endif
	
	expTime_correction = (float)expoC/(float)expoL;
//    callFunction.stereo(imgL, imgR, 50, dsp, clean, calibrated);
	callFunction->detect_material_online( expTime_correction, imgLnew, imgRnew, imgCnew);
	
	
#ifdef DEBUG_MAIN
TIME_DEBUG(main::processImages::detect_material_online);
#endif

	
//     Mat dsp8, dsp8res, dsp82;
// //     Size size(imgL.cols, imgL.rows);;
// 		
// 
// 		
// //     cv::resize(dsp8, dsp8res, size);
// 
// //         cv::namedWindow("imgLnew");
// //         cv::namedWindow("imgRnew");
// //         cv::namedWindow("imgCnew");
// 	
// //     cout << "3" << endl;
// //     cv::imshow("imgLnew", imgLnew);
// //     cv::waitKey(2);
// //     
// //     cv::imshow("imgRnew", imgRnew);
// //     cv::waitKey(2);
// //     
// //     cv::imshow("imgCnew", imgCnew);
// //     cv::waitKey(2);
// 		
// // 			end = ros::Time::now();
// // 			std::cout << "aaa " << (end -begin).toSec() << std::endl;
    
    
	if (show){
		string _dspWindowName("clustering");
		cv::namedWindow(_dspWindowName);
		
		string _dspWindowName1("dsp");
		cv::namedWindow(_dspWindowName1);
		
		string _dspWindowName2("NDVI");
		cv::namedWindow(_dspWindowName2);
		
		string _dspWindowName3("calibrated");
		cv::namedWindow(_dspWindowName3);

		string _dspWindowName4("clean");
		cv::namedWindow(_dspWindowName4);
		
		img.calibrated.convertTo(calibrated8, CV_8U);
		img.clean.convertTo(clean8, CV_8U);
		callFunction->dsp1.convertTo(dsp8, CV_8U);
		
		cv::imshow(_dspWindowName, ndvi.clustering  /*dsp8*/ /*ndvi.ndvi_classified*/);
		cv::waitKey(2);
		
		cv::imshow(_dspWindowName1, dsp8*5);//img.disparity);
		cv::waitKey(2);
		
		cv::imshow(_dspWindowName2, ndvi.ndvi);
		cv::waitKey(2);
		
		cv::imshow(_dspWindowName3, calibrated8);//img.disparity);
		cv::waitKey(2);

		cv::imshow(_dspWindowName4, clean8*5);//img.disparity);
		cv::waitKey(2);
	}
      
//     if (save){
//       stringstream ss;
//       ss.fill('0');
//       ss << "../imSaves/clustering_" <<  std::setw(10) << internal << framecounter << ".png";
//       cv::imwrite( ss.str().c_str(), ndvi.clustering );
//       myfile << "file '" <<  ss.str().substr(11) << "'" << endl;
//       
//       
//       stringstream ss1;
//       ss1.fill('0');
//       ss1 << "../imSaves/ndvi_classified_" <<  std::setw(10) << internal << framecounter << ".png";
//       cv::imwrite( ss1.str().c_str(), ndvi.ndvi_classified );
//       myfile1 << "file '" <<  ss1.str().substr(11) << "'" << endl;
//       
//       stringstream ss2;
//       ss2.fill('0');
//       ss2<< "../imSaves/img_calibrated_" <<  std::setw(10) << internal << framecounter << ".png";
//       cv::imwrite( ss2.str().c_str(), dsp8 );
//       myfile2 << "file '" <<  ss2.str().substr(11) << "'" << endl;
//       
//       stringstream ss3;
//       ss3.fill('0');
//       ss3 << "../imSaves/img_disparity_" <<  std::setw(10) << internal << framecounter << ".png";
//       cv::imwrite( ss3.str().c_str(), dsp82*4.0 );
//       myfile3 << "file '" <<  ss3.str().substr(11) << "'" << endl;
//       
//       
//       stringstream ss4;
//       ss4.fill('0');
//       ss4 << "../imSaves/ndvi_1D_" <<  std::setw(10) << internal << framecounter << ".png";
//       cv::imwrite( ss4.str().c_str(), (ndvi.ndvi+1.0)*128.0 );
//       myfile4 << "file '" <<  ss4.str().substr(11) << "'" << endl;
//       
//       framecounter++;
//     }
	counter++;
    
    
//     gettimeofday(&tv,NULL);
//     time_el = (1000000 * tv.tv_sec + tv.tv_usec) - time_first;
//     std::cout << "freq " << (1000000.0*(double)counter)/time_el << std::endl;
//     std::cout << "time_el " << time_el << std::endl;
//     std::cout << "time_fi " << time_first << std::endl;
//     std::cout << "time_last " << time_last << std::endl;
//     myfile << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
//     myfile1 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
//     myfile2 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
//     myfile3 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
//     myfile4 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
    
//     time_last = time_el;

    
//       cv::waitKey(1);
//     cout << "4" << endl;
//   myfile.close();
//   myfile1.close();
//   myfile2.close();
//   myfile3.close();
//   myfile4.close();
	
	
	#ifdef DEBUG_MAIN
	TIME_DEBUG_END(main::processImages::showImages);
	#endif
}








void camera_on_callback(const std_msgs::Bool::ConstPtr& msg)
{
	outputFile << "I heard: " << msg->data << std::endl;
	outputFile.flush();
	
  ROS_INFO("I heard: [%d]", msg->data);
camera_on = msg->data;
}













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

	outputFile.open("/home/odroid/images_ndvi/outputFile.txt", std::fstream::out);
	outputFile << "hi there, waiting 15 seconds" << std::endl;
	outputFile.flush();

	sleep(15);

	outputFile << "hi there, waited 15 seconds" << std::endl;
	outputFile.flush();


	camera_on = false;
	
	cv::Mat* frameL = NULL;
	cv::Mat* frameR = NULL;
	cv::Mat* frameC = NULL;



#if MAIN_SHOW_IMAGES
	cv::namedWindow("camL", CV_WINDOW_AUTOSIZE); 
	cv::namedWindow("camR", CV_WINDOW_AUTOSIZE); 
	cv::namedWindow("camC", CV_WINDOW_AUTOSIZE); 
#endif
  
	ros::init(argc, argv, "mv_camera");
		
	numimage = 0;

	ros::NodeHandle n;

	
	ros::NodeHandle pp("~");

	ros::Subscriber sub = n.subscribe("/camera_on", 1000, camera_on_callback);
	
	
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

	outputFile << "hi there, starting camera initalization" << std::endl;
	outputFile.flush();

	mv_bluefox_driver::MyCamera camL(n, ros::NodeHandle("~"), pid_params, nsp_camera_left);
	
	mv_bluefox_driver::MyCamera camR(n, ros::NodeHandle("~"), pid_params, nsp_camera_right);

	mv_bluefox_driver::MyCamera camC(n, ros::NodeHandle("~"), pid_params, nsp_camera_central);

	outputFile << "camera initalization ended" << std::endl;
	outputFile.flush();
  
	int _dinx = dinx;
	int _diny = diny;
	int _max_disparity=25;
	int _blur_window_size = 9;
	int _step_size = 2;
	int _disparity_winner_take_all_tolerance = 4;
	int _win_size = 9;
	double _weight_CGRAD = 1.1;
	bool _use_threads_slide_disparity = true;
	bool _use_threads_ndvi = true;

	
	int _length = 9/2;
	float _beta = 0.1;
	
	callFunction = new ImageAnalyzer(_dinx, _diny, _max_disparity, _blur_window_size, _step_size, _disparity_winner_take_all_tolerance,
																	 _win_size, _weight_CGRAD, _use_threads_slide_disparity, _use_threads_ndvi, _length, _beta, _save_image_folder);
	
		
  bool _saveframesthread = false;
  bool _showframesthread = MAIN_SHOW_IMAGES;
	
	
// 	ros::Time begin, end;
	
	int exposureL, exposureC, exposureR;
	double intensityL, intensityC, intensityR;
	
	
	// ### ### ### ### ### ### END TIMING
// 	begin = ros::Time::now();
	// ### ### ### ### ### ### END TIMING
	
#ifdef DEBUG_MAIN
TIME_DEBUG_INIT();
#endif

	
	ros::Time firstFrameTime = ros::Time::now();
	outputFile << "starting loop" << std::endl;
	outputFile.flush();
	while(ros::ok()){
		ros::spinOnce();
	outputFile << "spinOnce" << std::endl;
	outputFile.flush();
		
		
#ifdef DEBUG_MAIN
TIME_DEBUG(main::restart_loop);
#endif
		
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
			continue;
		}
		

#ifdef DEBUG_MAIN
TIME_DEBUG(main::acquire_images);
#endif
		if(camera_on){
			
			
			processImages(frameL, exposureL, frameR, exposureR, frameC, exposureC, _saveframesthread, _showframesthread);
			
#ifdef DEBUG_MAIN
TIME_DEBUG(main::processImages());
#endif
#if MAIN_SHOW_IMAGES
			cv::imshow("camL",*frameL);
			cv::imshow("camR",*frameR);
			cv::imshow("camC",*frameC);
			cv::waitKey(20);
#endif		
			camL.release_image();
			camR.release_image();
			camC.release_image();
			
			double elapsed = (ros::Time::now()-firstFrameTime).toSec();
			std::cout << counter << " " << elapsed << " " << counter/elapsed << std::endl;
		}
		else{
			usleep(10000);
		}
		
		// here goes exposure control for LR!
		double error= 80-intensityL;
		exposureTimeControllerLR.pid_update(error, exposureL);
		exposureL =  exposureL + exposureTimeControllerLR.control;
		camL.set_exposure_time(exposureL);
		error= 80-intensityR;
		exposureTimeControllerR.pid_update(error, exposureR);
		exposureR =  exposureR + exposureTimeControllerR.control;
		camR.set_exposure_time(exposureR);
		// here goes exposure control for C!
		error= 80-intensityC;
		exposureTimeControllerC.pid_update(error, exposureC);
		exposureC =  exposureC + exposureTimeControllerC.control;
		camC.set_exposure_time(exposureC);
		
#ifdef DEBUG_MAIN
TIME_DEBUG(main::exposure_control);
std::cout << "exposure control      " << intensityL  << " " << intensityR  << " " << intensityC << std::endl;
#endif


#ifdef DEBUG_MAIN
TIME_DEBUG(main::plotImages);
#endif
	}
#ifdef DEBUG_MAIN
TIME_DEBUG_END(main::end);
#endif
	return 0;
}





