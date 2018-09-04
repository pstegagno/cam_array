// system includes
#include <thread>
#include <vector>
#include "../CameraConverter/CameraConverter.hpp"

// external includes
#include <ros/ros.h>


#include <cv.h>
#include <highgui.h>

#include <thread>

#include <ImageAnalyzer/ImageAnalyzer.hpp>



int numimage;

using namespace cv;
using namespace ros;
using namespace std;

#define DEBUG_MAIN




int main(int argc, char **argv){
	
	int dx = 376;
	int dy = 240;
	
// 	int dx = 1000;
// 	int dy = 600;


		ros::init(argc, argv, "mv_camera");
	ros::NodeHandle n;
	
	
	Mat imgL, imgR, imgC;
// 	std::cout << "a" << std::endl;
	imgL = imread("/home/paolo/telekyb_ws/src/cameraarrayproject/camera_array_arm/resources/data/im0.png", CV_LOAD_IMAGE_COLOR);   // Read the file
// 	std::cout << "b" << std::endl;
	imgR = imread("/home/paolo/telekyb_ws/src/cameraarrayproject/camera_array_arm/resources/data/im1.png", CV_LOAD_IMAGE_COLOR);   // Read the file
// // 	std::cout << "c" << std::endl;
	imgC = imread("/home/paolo/telekyb_ws/src/cameraarrayproject/camera_array_arm/resources/data/im1c.png", CV_LOAD_IMAGE_COLOR);   // Read the file
	
	Mat imgL32, imgR32, imgC32, imgLnew, imgRnew, imgCnew;
//   imgL.convertTo(imgL32, CV_32FC1);
//   imgR.convertTo(imgR32, CV_32FC1);
	
	cv::cvtColor(imgR, imgR32, CV_BGR2GRAY);
	cv::cvtColor(imgL, imgL32, CV_BGR2GRAY);
	cv::cvtColor(imgC, imgC32, CV_BGR2GRAY);
	
	cv::Size size3(dx,dy);
	
	cv::resize(imgR32, imgRnew, size3);
	cv::resize(imgL32, imgLnew, size3);
	cv::resize(imgC32, imgCnew, size3);
	
	


// int blockSize = 3;
// 	Point anchor( 1 ,1 );
// 	double delta = 0;
	
	
	int _dinx = dx;
	int _diny = dy;
	int _max_disparity=25;
	int _blur_window_size = 9;
	int _step_size = 1;
	int _disparity_winner_take_all_tolerance = 6;
	int _win_size = 11;
	double _weight_CGRAD = 1.1;
	bool _use_threads_slide_disparity = true;
	bool _use_threads_ndvi = true;
	
	int _length = 9/2;
	float _beta = 0.1;
	std::string _save_image_folder("/home/paolo/images_ndvi/");
	
	
	CameraArrayProject::Disparity img(_diny,_dinx,_win_size,_win_size);
	CameraArrayProject::Ndvi ndvi(_dinx, _diny);
	
	CameraArrayProject::ImageAnalyzer* callFunction = new CameraArrayProject::ImageAnalyzer(_dinx, _diny, _max_disparity, _blur_window_size, _step_size, _disparity_winner_take_all_tolerance,
																																													_win_size, _weight_CGRAD, _use_threads_slide_disparity, _use_threads_ndvi, _length, _beta, _save_image_folder);
		
// 	callFunction->disparity_map(imgLnew,  imgRnew, img.dsp, img.clean, img.calibrated );
	float expTime_correction = 0.6;
	
#ifdef DEBUG_MAIN
TIME_DEBUG_INIT();
#endif

	callFunction->detect_material_online( expTime_correction, imgLnew, imgRnew, imgCnew);
	
#ifdef DEBUG_MAIN
TIME_DEBUG_END(ImageAnalyzer::detect_material_online);
#endif
	
	
// 	vector<int> compression_params;
// 	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
// 	compression_params.push_back(0);
// 	imwrite("/home/paolo/telekyb_ws/src/cameraarrayproject/camera_array_arm/resources/data/im1c.png", img.calibrated, compression_params );
	
	int counter = 0;
	
	ros::Time timer = ros::Time::now();
	for (int i = 0; i<3; i++){
		

// 	callFunction->printFloatImg(imgLnew, "imgLnew", 100, false);
// // 	
// 	callFunction->printFloatImg(imgRnew, "imgRnew", 100, false);
// // 	
// 	callFunction->printFloatImg(callFunction->dsp1*5, "img.dsp", 100, false);
// // 	
// 	callFunction->printFloatImg(callFunction->data_structs.clean*5, "img.clean", 100, false);
// // 	
// 	callFunction->printFloatImg(callFunction->data_structs.calibrated, "img.calibrated", 100, false);
// // 	
// // 	
// 	callFunction->printFloatImg(callFunction->ndvi.clustering, "ndvi.clustering", 100, false);
// // 
// 	callFunction->printFloatImg(callFunction->ndvi.ndvi*5, "ndvi.ndvi", 100, false);
// 	
		callFunction->detect_material_online( expTime_correction, imgLnew, imgRnew, imgCnew);
		counter++;
		double elapsed = (ros::Time::now() - timer).toSec();
		cout << ((double)counter)/elapsed << std::endl;
		
		
// 		if (show){
// 		string _dspWindowName("clustering");
// 		cv::namedWindow(_dspWindowName);
// 		
// 		string _dspWindowName1("dsp");
// 		cv::namedWindow(_dspWindowName1);
// 		
// 		string _dspWindowName2("NDVI");
// 		cv::namedWindow(_dspWindowName2);
// 		
// 		string _dspWindowName3("calibrated");
// 		cv::namedWindow(_dspWindowName3);
// 
// 		string _dspWindowName4("clean");
// 		cv::namedWindow(_dspWindowName4);
// 		
// 		img.calibrated.convertTo(calibrated8, CV_8U);
// 		img.clean.convertTo(clean8, CV_8U);
// 		img.dsp.convertTo(dsp8, CV_8U);
// 		
// 		cv::imshow(_dspWindowName, ndvi.clustering  /*dsp8*/ /*ndvi.ndvi_classified*/);
// 		cv::waitKey(2);
// 		
// 		cv::imshow(_dspWindowName1, dsp8*5);//img.disparity);
// 		cv::waitKey(2);
// 		
// 		cv::imshow(_dspWindowName2, ndvi.ndvi);
// 		cv::waitKey(2);
// 		
// 		cv::imshow(_dspWindowName3, calibrated8);//img.disparity);
// 		cv::waitKey(2);
// 
// 		cv::imshow(_dspWindowName4, clean8*5);//img.disparity);
// 		cv::waitKey(2);
// 	}
	
	
	
	
	
	
	
	
	
	}
	callFunction->terminate();
	
	callFunction->ndvi_thread->join();

// 	sleep(1);
// 	sleep(5);



// 	cv::Mat testmat(3, 6,CV_32FC1 );
// 	cv::Mat outmatA(4, 7, CV_32FC1 ), outmatB(4, 7, CV_32FC1 );
// 	cv::Mat outmat1(3, 6, CV_32FC1 );
// 	cv::Mat outmat2(3, 6, CV_32FC1 );
// 	
// // 	callFunction.
// 	testmat.at<float>(0, 0) = 1;
// 	testmat.at<float>(0, 1) = 1;
// 	testmat.at<float>(0, 2) = 1;
// 	testmat.at<float>(0, 3) = 3;
// 	testmat.at<float>(0, 4) = 4;
// 	testmat.at<float>(0, 5) =-3;
// 	testmat.at<float>(1, 0) = 3;
// 	testmat.at<float>(1, 1) = 4;
// 	testmat.at<float>(1, 2) = 5;
// 	testmat.at<float>(1, 3) =-4;
// 	testmat.at<float>(1, 4) = 7;
// 	testmat.at<float>(1, 5) = 7;
// 	testmat.at<float>(2, 0) = 3;
// 	testmat.at<float>(2, 1) = 2;
// 	testmat.at<float>(2, 2) = 8;
// 	testmat.at<float>(2, 3) = 2;
// 	testmat.at<float>(2, 4) = 1;
// 	testmat.at<float>(2, 5) = 1;
// 	
// 	std::cout << testmat << std::endl;
// 
// callFunction->integrateImg(testmat, outmatA);
// 
// 
// 	std::cout << outmatA << std::endl;
// 	
// 	
// 			integral(testmat, outmatB, CV_32F);
// 			
// 			
// 	std::cout << outmatB << std::endl;
// 
// 			callFunction->blockImg(outmatA,outmat1);
// 	
// 		
// 			std::cout << outmat1 << std::endl;
// 
// 
// 	std::cout << "END" << std::endl;
// 
// cv::Mat h (blockSize, blockSize, CV_32FC1);
// 
// h = 1.0/(double)(blockSize*blockSize);
//  
// 
// 		Ptr<FilterEngine> fe1 = createLinearFilter(CV_32FC1, CV_32FC1, h, anchor, delta, BORDER_CONSTANT, BORDER_CONSTANT, Scalar(0));
// 		fe1->apply(testmat, outmat2);
// 
// 
// 
// 			std::cout << outmat2 << std::endl;

















/*









	
	
	for (int i = 0; i<1000; i++){
	for (int j = 0; j<1000; j++){
// 					for (int k = 0; k<10000; k++){

							cv::Mat a(100, 100, CV_8UC1 );
							a.setTo(Scalar(2));
							cv::Mat b(100, 100, CV_8UC1 );
							cv::Mat c(100, 100, CV_8UC1 );
							b = a;
							c=a+b;
// 					}
	}
	}
	
end = ros::Time::now();
std::cout << "char  " << (end-begin).toSec() << std::endl;	
begin = ros::Time::now();
	
	for (int i = 0; i<1000; i++){
			for (int j = 0; j<1000; j++){
// 			for (int k = 0; k<10000; k++){

							cv::Mat a(100, 100, CV_32FC1 );
							a.setTo(Scalar(2));
							cv::Mat b(100, 100, CV_32FC1 );
							cv::Mat c(100, 100, CV_32FC1 );
							b = a;
							c=a+b;

// 			}
			}
	}
	
end = ros::Time::now();
std::cout << "float " << (end-begin).toSec() << std::endl;	
	
	
sleep(10);	
	
	cv::Mat disparity(10, 10, CV_8UC1 );
	
	std::cout << disparity << std::endl;
	std::cout << disparity.type() << " " << disparity.channels() << std::endl;

	disparity.setTo(Scalar(0));
	
	std::cout << disparity << std::endl;
	std::cout << disparity.type() << " " << disparity.channels() << std::endl;
	
	sleep (10);
	
	
	
	
	
	
	cv::namedWindow( "Grabbed image");// Create a window for display.
	

	
	
	int w = 460, h = 460;
	cv::Mat f01, f02;
	cv::Mat f03, f04;
	cv::Mat f05, f06;
	
		std::cout << "a"  << std::endl;

// 	cv::Mat f07, f08;
// 	cv::Mat f09, f10;
// 	cv::Mat f11, f12;
// 	cv::Mat f13, f14;
// 	cv::Mat f15, f16;
// 	cv::Mat f17, f18;
// 	cv::Mat f19, f20;
	uchar* pippo;
	
	pippo = new uchar(w*h);
// 	[w*h];

		std::cout << "a1 " << w*h  << std::endl;
	
	f01 = cv::Mat(h,w,CV_8UC1);
	f03 = cv::Mat(h,w,CV_8UC1);
	f05 = cv::Mat(h,w,CV_8UC1);
	f02 = cv::Mat(h,w,CV_8UC1);
	f04 = cv::Mat(h,w,CV_8UC1);
	f06 = cv::Mat(h,w,CV_8UC1);
// 	f07 = cv::Mat(h,w,CV_8UC1);
// 	f09 = cv::Mat(h,w,CV_8UC1);
// 	f11 = cv::Mat(h,w,CV_8UC1);
// 	f13 = cv::Mat(h,w,CV_8UC1);
// 	f15 = cv::Mat(h,w,CV_8UC1);
// 	f17 = cv::Mat(h,w,CV_8UC1);
// 	f19 = cv::Mat(h,w,CV_8UC1);
	
		std::cout << "b"  << std::endl;
		
	begin = ros::Time::now();
// 	for (int i=1; i<10; i++){
		
		f02 = f01.clone();
		f04 = f03.clone();
// 		f06 = cv::Mat();
		
// 		memcpy ( f06.data, pippo, sizeof(uchar)*w*h );
	
		f06 = cv::Mat(h, w, CV_8UC1, *pippo );
// 		f08 = f07.clone();
// 		f10 = f09.clone();
// 		f12 = f11.clone();
// 		f14 = f13.clone();
// 		f16 = f15.clone();
// 		f18 = f17.clone();
// 		f20 = f19.clone();
		std::cout << "c"  << std::endl;
		
// 	}
	end = ros::Time::now();
	
	std::cout << (end.toSec()-begin.toSec())  << std::endl;
	
	
	
	unsigned char img_frame[20];
	img_frame[0] = 1;
	img_frame[1] = 2;
	img_frame[2] = 3;
	img_frame[3] = 4;
	img_frame[4] = 5;
	img_frame[5] = 6;
	img_frame[6] = 7;
	img_frame[7] = 8;
	img_frame[8] = 9;
	img_frame[9] = 10;
	img_frame[10] = 110;
	img_frame[11] = 120;
	img_frame[12] = 130;
	img_frame[13] = 140;
	img_frame[14] = 150;
	img_frame[15] = 160;
	img_frame[16] = 170;
	img_frame[17] = 180;
	img_frame[18] = 190;
	img_frame[19] = 200;
	
	
	
	cv::Mat frame1(4, 5, CV_8UC1, *img_frame);
	
	for(int i = 0; i<4; i++)
		for(int j = 0; j<5; j++)
			frame1.at<uchar>(i, j) = img_frame[i*5+j];
	
	std::cout << frame1 << std::endl;
	
	
	imshow( "Grabbed image", frame1 ); 
	cv::waitKey(30);


	sleep(20);*/

	return 0;
}





