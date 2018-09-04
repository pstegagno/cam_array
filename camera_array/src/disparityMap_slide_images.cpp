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



void ImageAnalyzer::printFloatImg(Mat img, string windowName, int wait, bool toGray=false){
    Mat img8;
    img.convertTo(img8, CV_8U);
    
    if(toGray){
      Mat gray_image;
      cvtColor( img8, gray_image, CV_BGR2GRAY );
      namedWindow( windowName.c_str(), CV_WINDOW_AUTOSIZE );
      imshow( windowName.c_str(),  gray_image  );
      waitKey(wait);
    }
    else{
      namedWindow( windowName.c_str(), CV_WINDOW_AUTOSIZE );
      imshow( windowName.c_str(),  img8  );
      waitKey(wait);
    }
}


void ImageAnalyzer::fillEdgeImage(Mat edgesIn, Mat &filledEdgesOut) 
{
    Mat edgesNeg = edgesIn.clone();

    floodFill(edgesNeg, cv::Point(0,0), CV_RGB(255,255,255));
    bitwise_not(edgesNeg, edgesNeg);
    filledEdgesOut = (edgesNeg | edgesIn);

    return;
}



Mat ImageAnalyzer::translateImg(Mat img, Mat &imgout,int offsetx, int offsety)
    {

    Mat trans_mat = (Mat_<float>(2,3) << 1, 0, offsetx, 0, 1, offsety);
    warpAffine(img,imgout,trans_mat,img.size());
    return trans_mat;
    }
    
void ImageAnalyzer::slide_images(Mat imgL, Mat imgR, int win_size, int mins, int maxs, int weight, Mat &output1, Mat &output2)//, int weigth, Mat &disparity, Mat &mindiff)
{

  int diny, dinx;
 
  diny = imgR.rows;
  dinx = imgR.cols;
//   c = imgR.channels(); 
  
  Disparity matrixR(diny,dinx,win_size,win_size), matrixL(diny,dinx,win_size,win_size); 
  
 
  Sobel( imgR, matrixR.gradientX, CV_32FC1, 1, 0);
  Sobel( imgL, matrixL.gradientX, CV_32FC1, 1, 0);
  
//       printFloatImg(matrixR.gradientX, "matrixR.gradientX1", 10, false);
//       printFloatImg(matrixL.gradientX, "matrixL.gradientX1", 10, false);
  Mat a, b;
  cv::Size sizzz(9,9);
  GaussianBlur(matrixR.gradientX, a, sizzz, 10);
  GaussianBlur(matrixL.gradientX, b, sizzz, 10);
  matrixR.gradientX = a;
  matrixL.gradientX = b;
//   matrixR.gradientX = (matrixR.gradientX -20.0) *10.0 ;
//   matrixL.gradientX = matrixL.gradientX*3.0 -20;
//       printFloatImg(matrixR.gradientX, "matrixR.gradientX", 10, false);
//       printFloatImg(matrixL.gradientX, "matrixL.gradientX", 10, false);
      
  int step;
  if(maxs-mins < 0)
  {step = -1;}
  if(maxs-mins > 0)
  {step = 1;}

   Point anchor( 0 ,0 );
   double delta = 0;

  for(int i = mins; abs(i) < abs(maxs); i+=step )
  {
    // translating image
    translateImg(imgL,matrixL.shift, i,0);
    translateImg(matrixL.gradientX,matrixL.shift_gradient, i,0);
    

   
    matrixL.diff = abs(imgR - matrixL.shift);
    matrixL.gdiff = abs(matrixL.shift_gradient-matrixR.gradientX);
    
    // equivalent  to imfilter Matlab
    
    
//     std::cout << "1" << std::endl;
//     std::cout << matrixL.diff.size() << std::endl;
//     std::cout << matrixL.diff.type() << std::endl;
//     std::cout << matrixL.diff.channels() << std::endl;
//     std::cout << matrixR.h.size() << std::endl;
//     std::cout << matrixR.h.type() << std::endl;
//     std::cout << "1" << std::endl;
    
    Ptr<FilterEngine> fe1 = createLinearFilter(matrixL.diff.type(), matrixR.h.type(), matrixR.h, anchor,delta, BORDER_CONSTANT, BORDER_CONSTANT, Scalar(0));
//     std::cout << "2" << std::endl;
    fe1->apply(matrixL.diff, matrixL.CSAD);
    
//     std::cout << "3" << std::endl;
    Ptr<FilterEngine> fe2 = createLinearFilter(matrixL.gdiff.type(), matrixR.h.type(), matrixR.h, anchor,delta, BORDER_CONSTANT, BORDER_CONSTANT, Scalar(0));
//     std::cout << "4" << std::endl;
    fe2->apply(matrixL.gdiff, matrixL.CGRAD);
//     std::cout << "5" << std::endl;
    
    matrixL.d = matrixL.CSAD+weight*matrixL.CGRAD;
//       printFloatImg(matrixL.CSAD, "matrixL.CSAD", 10, false);
//       printFloatImg(matrixL.CGRAD, "matrixL.CGRAD", 10, false);
//       printFloatImg(matrixL.d, "matrixL.d", 10, false);
//     
    
    for(int y = 0; y < diny; y++)
    {
      for(int x = 0; x < dinx; x++)
      {
	if(matrixL.d.ptr<float>(y)[x] < matrixL.mindiff.ptr<float>(y)[x]) 
	{
	  matrixL.mindiff.ptr<float>(y)[x] = matrixL.d.ptr<float>(y)[x];  
	  matrixL.disparity.ptr<float>(y)[x] = abs(i);
	}
      }
    }

  }

  
  
  
 output1 =  matrixL.mindiff;
 output2 =  matrixL.disparity;
 
}
