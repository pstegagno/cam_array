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
//using namespace std;



void ImageAnalyzer::winner_take_all(Mat d1, Mat m1, Mat d2,Mat m2, int tolerance, Mat &output)
{
   int diny, dinx, win_size;
   diny = d1.rows;
   dinx = d1.cols;
   win_size = 7;
   
   
//       printFloatImg(d1, "d1",0, false);
//       printFloatImg(d2, "d2",0, false);
  
   Disparity matrix(diny,dinx,win_size,win_size);
   

    for(int y = 0; y < diny; y++)
    {
      for(int x = 0; x < dinx; x++)
      {
	
	if(abs(d1.ptr<float>(y)[x]-d2.ptr<float>(y)[x]) < tolerance && m1.ptr<float>(y)[x] <= m2.ptr<float>(y)[x])
	{
	  matrix.pixel_dsp.ptr<float>(y)[x] = d1.ptr<float>(y)[x];  
	}
	else if(abs(d1.ptr<float>(y)[x]-d2.ptr<float>(y)[x]) < tolerance && m1.ptr<float>(y)[x] > m2.ptr<float>(y)[x])
	{
	  matrix.pixel_dsp.ptr<float>(y)[x] = d2.ptr<float>(y)[x];  
	}
	else{
	  matrix.pixel_dsp.ptr<float>(y)[x] = 0.0;
	}
      }
    }
    
    translateImg(matrix.pixel_dsp, output, 5, 0);

   
   
}


