#include <cv.h>
#include <highgui.h>
#include<math.h>
#include<new>
#include <iterator>
#include <vector>
#include <algorithm>
#include <sys/time.h>

#include "image_analyzer.hpp"


#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>
#include <string.h>
#include <iostream>
using namespace cv;
using namespace std;



void ImageAnalyzer::stereo(Mat imgR, Mat imgL, int maxs, Mat &dsp, Mat &clean, Mat &calibrated ){

  
  Mat imgR32, imgL32/*, clean(imgL.size(), imgL.type())*/;
   
  imgL.convertTo(imgL32, CV_32F);
  imgR.convertTo(imgR32, CV_32F);
  
  int  win_size  = 7; // size of window used when smoothing
  int  tolerance = 4; // how close R-L and L-R values need to be
  int  weight = 1.1; // weight on gradients opposed to color
  
  int diny, dinx;
  diny = imgR.rows;
  dinx = imgR.cols;
  
  std::cout << "a" << std::endl;

  Disparity matrix_LR(diny,dinx,win_size,win_size), matrix_RL(diny,dinx,win_size,win_size);
 
  slide_images(imgL32, imgR32,  win_size , 0, -maxs, weight, matrix_LR.mindiff, matrix_LR.disparity);
  
   std::cout << "b" << std::endl;
 
  //std::cout << << std::endl;
 // printFloatImg(matrix_LR.disparity, "1", 10, false);
  
  slide_images(imgR32, imgL32,  win_size , 0, maxs, weight, matrix_RL.mindiff, matrix_RL.disparity);
  
  

  //printFloatImg(matrix_RL.disparity, "2", 10, false);
  


  
   std::cout << "c" << std::endl;
  winner_take_all(matrix_LR.disparity,matrix_LR.mindiff, matrix_RL.disparity,matrix_RL.mindiff,  tolerance, dsp);


  //std::cout << "dsp" << dsp << std::endl;

  
//   printFloatImg(dsp, "pd1", 10, false);
   std::cout << "c1" << std::endl;

     struct timeval tv;
   gettimeofday(&tv,NULL);
   unsigned long time_first = 1000000 * tv.tv_sec + tv.tv_usec; 
   
   std::cout << "c2" << std::endl;
   clean_disparity(dsp, clean);
   
   std::cout << "c3" << std::endl;
//    Mat recostruction;
// 
//    reprojectImageTo3D(clean_disparity, recostruction);
//    
//   namedWindow( "image", CV_WINDOW_AUTOSIZE );
//   imshow( "image", recostruction);
//   waitKey(0);
      gettimeofday(&tv,NULL);
  unsigned long time_el = (1000000 * tv.tv_sec + tv.tv_usec) - time_first;
  std::cout << "time clean " << time_el << std::endl;
  
//   printFloatImg(clean, "clean", 10, false);
  

 
  
//    std::cout << "d" << std::endl;
  disparityMap_new_image( imgL32, imgR32, clean, matrix_LR.calibrated);
//    disparityMap_new_image( imgL32, imgR32, dsp, matrix_RL.calibrated);
//    std::cout << "e" << std::endl;
//   printFloatImg(matrix_LR.calibrated, "calibrated", 10, false);
//   printFloatImg(matrix_RL.calibrated, "calibrated dsp", 10, false);
  
  std::cout << "d" << std::endl;
   
   calibrated = matrix_LR.calibrated;

   std::cout << "e" << std::endl;
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



