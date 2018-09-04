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

void ImageAnalyzer::disparityMap_new_image(Mat imgL, Mat imgR, Mat dsp, Mat &calibrated)
{
 
//   std::cout << "A" << std::endl;
  int diny, dinx;
  diny = imgR.rows;
  dinx = imgR.cols;
 
  int shift;
 
    for(int y = 0; y < diny; y++)
    {
      for(int x = 0; x < dinx; x++)
      {
	shift = (int)dsp.ptr<float>(y)[x]/2;
	
// 	if (shift>5 || shift<-5){
// 	  std::cout << y << " " << x << " " << shift << std::endl;
// 	}
	if (x<dinx/2 && dsp.ptr<float>(y)[x]==0) {
// 	  if (dsp.ptr<float>(y)[x]>0) {
//   std::cout << "A1" << std::endl;
	    calibrated.ptr<float>(y)[x]= imgL.ptr<float>(y)[x+shift]/*/2+imgR.ptr<float>(y)[x-shift]/2*/;
// 	  }
	}
	else if (x>=dinx/2 && dsp.ptr<float>(y)[x]==0) {
// 	  if (dsp.ptr<float>(y)[x]>0) {
//   std::cout << "A2" << std::endl;
	    calibrated.ptr<float>(y)[x]=/* imgL.ptr<float>(y)[x+shift]/2+*/imgR.ptr<float>(y)[x-shift]/*/2*/;
// 	  }
	}
	else {
//   std::cout << "A3" << std::endl;
	    calibrated.ptr<float>(y)[x]= imgL.ptr<float>(y)[x+shift]/2+imgR.ptr<float>(y)[x-shift]/2;
	}
      }
    }
//   printFloatImg(calibrated, "calibrated ins", 0, false);  
   
  
}
