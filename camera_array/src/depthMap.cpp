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

void ImageAnalyzer::depth_map( Mat dsp, Disparity &depth) 

{ 
  
  Mat disparity;
  
  dsp.convertTo(disparity, CV_32F);
  
  
  float focal_length, baseline, coeff;
//   int i,j;
  int diny, dinx, win_size;
    
  diny = disparity.rows;
  dinx = disparity.cols;
  win_size  = 7;

 Disparity img(diny,dinx,win_size,win_size);

  std::cout << "a depth" << std::endl;
  focal_length = 450.471;
  baseline = 0.072;
  
  std::cout << "b depth" << std::endl;
  
  coeff = focal_length*baseline;
  
  std::cout << depth.depth.size() << std::endl;
  
  //img.depth = coeff/disparity;
  
  for(int i = 0; i < diny ; i++)
    {
      for(int j = 0; j < dinx ; j++)
      {
	depth.depth.ptr<float>(i)[j] = fmin(255.0, coeff/disparity.ptr<float>(i)[j]);
      }
    }
    
    
    
    
//   std::cout << "col" << img.depth <<std::endl;
//   std::cout << "row" << img.depth.rows <<std::endl; 
//   std::cout << "col" << img.depth.cols<<std::endl; 
  
  
  //std::cout << "col" << img.depth <<std::endl;
  
//   printFloatImg(img.depth, "calibrated", 10, false);
//   
//   namedWindow( "Material1", CV_WINDOW_AUTOSIZE );
//   imshow( "Material1", img.depth);
//   waitKey(0);
//   
  std::cout << "c depth" << std::endl;  

}