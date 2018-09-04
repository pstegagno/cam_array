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


void ImageAnalyzer::Detect_Material( Mat imgL, Mat imgR, Mat imgM, int length, float beta, Mat &clustering)   
{
  
  Mat imgL32, imgR32, clean;
  
  imgL.convertTo(imgL32, CV_32F);
  imgR.convertTo(imgR32, CV_32F);  
  
   int  win_size  = 7;
   int diny, dinx;
   diny = imgR.rows;
   dinx = imgR.cols;

   Disparity img(diny,dinx,win_size,win_size);
   
   Ndvi matrix(diny,dinx);
   
   
  stereo(imgL,  imgR , 50, img.dsp, clean, img.calibrated);
  
  compute_NDVI( imgM, img.calibrated, matrix.ndvi); 
  
  Ndvi_clustering_parabola(length, beta, matrix.ndvi, matrix); 
  
}