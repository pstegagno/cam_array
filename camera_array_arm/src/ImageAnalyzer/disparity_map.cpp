


#include <ImageAnalyzer/ImageAnalyzer.hpp>
#include <thread>


using namespace cv;
using namespace std;

#define DEBUG_DISPARITY_MAP 1


void CameraArrayProject::ImageAnalyzer::disparity_map(Mat imgR, Mat imgL){

#if DEBUG_DISPARITY_MAP && DEBUG_TIME
TIME_DEBUG_INIT();
#endif
	
	if (imgR.type() != CV_32F || imgR.channels() != 1){
			imgR.convertTo(imgL, CV_32FC1);
	}
	if (imgL.type() != CV_32F || imgL.channels() != 1){
			imgL.convertTo(imgL, CV_32FC1);
	}
	
#if DEBUG_DISPARITY_MAP && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::disparity_map::init);
#endif
	
	disparity_map_compute_gradients(imgL,imgR, gradients.l_gradientX, gradients.r_gradientX, gradients.l_gradientXblured, gradients.r_gradientXblured);

#if DEBUG_DISPARITY_MAP && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::disparity_map::compute_gradients);
#endif

	
// 	std::thread local_thread(&CameraArrayProject::ImageAnalyzer::disparity_map_slide_images, this, imgL, imgR,
// 													 gradients.l_gradientXblured, gradients.r_gradientXblured,
// 													0, -max_disparity,
// 													std::ref(slide_lr.mindiff), std::ref(slide_lr.disparity),
// 													std::ref(slide_lr)); // pass by value

	if (use_threads_slide_images){
		imgL_th = imgL;
		imgR_th = imgR;
		slide_imgs_flag = true;
	} else {
		disparity_map_slide_images(imgL, imgR, gradients.l_gradientXblured, gradients.r_gradientXblured, 0, -max_disparity,
															 slide_lr.mindiff, slide_lr.disparity, slide_lr);
	}
	
	disparity_map_slide_images(imgR, imgL, gradients.r_gradientXblured, gradients.l_gradientXblured, 0, max_disparity,
														 slide_rl.mindiff, slide_rl.disparity, slide_rl);
	
	while (slide_imgs_flag){
		usleep(100);
	}
// 	local_thread.join();
	
// // // // // // #if DEBUG_DISPARITY_MAP && DEBUG_TIME
// // // // // // TIME_DEBUG(ImageAnalyzer::detect_material_online::disparity_map::slide_images);
// // // // // // #endif
// // // // // // 
// // // // // //   disparity_map_winner_take_all(slide_lr.disparity, slide_lr.mindiff, slide_rl.disparity, slide_rl.mindiff,  dsp);
// // // // // // 
// // // // // //   
// // // // // // #if DEBUG_DISPARITY_MAP && DEBUG_TIME
// // // // // // TIME_DEBUG(ImageAnalyzer::detect_material_online::disparity_map::winner_take_all);
// // // // // // #endif
// // // // // //   
// // // // // // 
// // // // // // 	disparity_map_clean_disparity(dsp, clean);
// // // // // //   
// // // // // // //   std::cout << "c3" << std::endl;
// // // // // // //    Mat recostruction;
// // // // // // // 
// // // // // // //    reprojectImageTo3D(clean_disparity, recostruction);
// // // // // // //    
// // // // // // //   namedWindow( "image", CV_WINDOW_AUTOSIZE );
// // // // // // //   imshow( "image", recostruction);
// // // // // // //   waitKey(0);
// // // // // // #if DEBUG_DISPARITY_MAP && DEBUG_TIME
// // // // // // TIME_DEBUG(ImageAnalyzer::detect_material_online::disparity_map::clean_disparity);
// // // // // // #endif
// // // // // //   
// // // // // // 	
// // // // // // // 	printFloatImg(data_structs.clean_left*5, "data_structs.clean_left", 10, false);
// // // // // // // 	printFloatImg(data_structs.clean_down*5, "data_structs.clean_down", 10, false);
// // // // // // 
// // // // // // //   printFloatImg(clean, "clean", 10, false);
// // // // // //   
// // // // // // 
// // // // // //  
// // // // // //   
// // // // // // //    std::cout << "d" << std::endl;
// // // // // //   disparity_map_new_image( imgR, imgL, clean, calibrated);
// // // // // // //    disparityMap_new_image( imgL32, imgR32, dsp, matrix_RL.calibrated);
// // // // // // //    std::cout << "e" << std::endl;
// // // // // // //   printFloatImg(matrix_LR.calibrated, "calibrated", 10, false);
// // // // // // //   printFloatImg(matrix_RL.calibrated, "calibrated dsp", 10, false);
// // // // // //   
// // // // // //    
// // // // // // //    calibrated = matrix_LR.calibrated;
// // // // // // 	 


#if DEBUG_DISPARITY_MAP && DEBUG_TIME
TIME_DEBUG_END(ImageAnalyzer::detect_material_online::disparity_map::slide);
#endif
//    std::cout << "e" << std::endl;
 /*  
/////////  depth Map to 3D reconstruction //////////////////
   
    Mat_<double> cameraMatrix1(3, 3);
    Mat_<double> cameraMatrix2(3, 3);
    Mat_<double> distCoeffs1(5, 1);
    Mat_<double> distCoeffs2(5, 1);
    Mat_<double> R(3, 3);
    Mat_<double> T(3, 1);
   
   cameraMatrix1  << 413.8092624895842, 0.0, 378.41608555363683, 0.0, 414.58118108226586, 249.63202413917932, 0.0, 0.0, 1.0;
   cameraMatrix2  << 406.9185578784797, 0.0, 364.50646541778684, 0.0, 407.83987911594886, 252.405401574727, 0.0, 0.0, 1.0;
   distCoeffs1 << -0.2762540642040707, 0.06446557458606815, -0.0009005274726036505, -0.0013979815025480776, 0.0;
   distCoeffs2 << -0.274577132715303, 0.06301081337644864, -0.0013816235334555647, -0.00013153495216207346, 0.0;
   R  << 1, 0, 0, 0, 1, 0, 0, 0, 1;
   T << -0.09694180375497984,0,0;

   cv::Mat R1,R2,P1,P2,Q;   // you're safe to leave OutpuArrays empty !
   cv::Size imgSize = dsp.size(); 

   cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imgSize, R, T, R1, R2, P1, P2, Q);

std::cout << "Q" << Q << std::endl;
   

   
   
   std::cout << "h" << std::endl;
   
   cv::FileStorage savefile("../imSaves/Image.xml", cv::FileStorage::WRITE);
   cv::Mat Image(dsp.rows, dsp.cols, CV_32FC3);
   
//    FileStorage fs; 
//    fs.open("Vocabulary.xml", FileStorage::WRITE); 
//    
//    fs << "ciao" << Image ;
//    fs.release();
//    
//    
   std::cout << "i" << std::endl;
   
   reprojectImageTo3D(clean, Image, Q);
   
   std::cout << "l" << std::endl;
   
   // Declare what you need

//cv::Mat someMatrixOfAnyType;

   
 //Write to file!
 savefile << "Image" << Image;
 savefile.release();

std::cout << "m" << std::endl;   

   namedWindow( "image", CV_WINDOW_AUTOSIZE );
   imshow( "image",  Image);
   waitKey(0);
     
  
   */


}



