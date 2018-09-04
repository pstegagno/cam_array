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
using namespace cv;


void ImageAnalyzer::Detect_Material_online( float expTime_corection, Mat imgL, Mat imgR, Mat imgM,int length, float beta, Ndvi &ndvi, Disparity &img)   
{
  
  Mat imgL32, imgR32, clean, image;
  
  imgL.convertTo(imgL32, CV_32F);
  imgR.convertTo(imgR32, CV_32F);  
  
  
//    int  win_size  = 7;
//    int diny, dinx;
//    diny = imgR.rows;
//    dinx = imgR.cols;
//    Disparity img(diny,dinx,win_size,win_size);
   
   //    Ndvi matrix(diny,dinx);
   
   std::cout << "1" << std::endl;
   
   stereo(imgL32,  imgR32 , 25, img.dsp, img.clean, img.calibrated );
   
   //depth_map(img.dsp) ;
   std::cout << "2" << std::endl;
   
   compute_NDVI_online(expTime_corection, imgM, img.calibrated, ndvi.ndvi);
   
   std::cout << "3" << std::endl;
   Ndvi_clustering_parabola(length, beta, ndvi.ndvi, ndvi);    
   
   std::cout << "4" << std::endl;
   
  // Ndvi_clustering_parabola(int length, float beta, Mat mat_sum, Ndvi &ndvi)
   
   
   
   
   //stereo(Mat imgR, Mat imgL, int maxs, Mat &dsp, Mat &clean, Mat &calibrated )
   
   

}