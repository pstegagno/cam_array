#include <cv.h>
#include <highgui.h>
#include<math.h>
#include<new>
#include <iterator>
#include <vector>
#include <algorithm>
#include <sys/time.h>

#include "image_analyzer.hpp"



void ImageAnalyzer::compute_NDVI_online( float expTime_corection ,Mat NiarInfared, Mat Red, Mat &output1)          
{
  

    Mat redf, nirf;
    Red.convertTo(redf, CV_32FC1);
    NiarInfared.convertTo(nirf, CV_32FC1);
  
    double coef_areaR = 1.43*(double)expTime_corection; 
    double coef_areaNIR = 1; 
    
    // Normalization
    Mat nirf_sum, redf_sum; 
    nirf_sum = nirf*coef_areaNIR;
    redf_sum = redf*coef_areaR;
   
    // calculation of NVDI index
    Mat num_sum, den_sum, ndvi_sum;
    num_sum=nirf_sum-redf_sum;
    den_sum=nirf_sum+redf_sum;
    divide(num_sum, den_sum ,ndvi_sum); // divide elememt by element
    
    // BLUR
    Mat finalMat_sum, finalMat_sum_temp; 
    finalMat_sum_temp = (ndvi_sum+1.0)/2.0;
    cv::blur(finalMat_sum_temp, finalMat_sum, Size(5,5),Point(-1,-1));
    

    
    
    
    Mat image(NiarInfared.rows,NiarInfared.cols,CV_8UC3);
    
    for(int i = 0; i <ndvi_sum.rows ; i++)
    {
      for(int j = 0; j <ndvi_sum.cols ; j++)
      {
	
	if(  (ndvi_sum.ptr<float>(i)[j])  >  0.4) // dense vegetation dark green
	{
	image.at<cv::Vec3b>(i,j) = cv::Vec3b(0,100,0);
	} 
	if(0.2 < (ndvi_sum.ptr<float>(i)[j]) && (ndvi_sum.ptr<float>(i)[j]) <= 0.4)// shurb and grassland
	{
	  image.at<cv::Vec3b>(i,j)= cv::Vec3b(0,250,0);
	}
	if( ndvi_sum.ptr<float>(i)[j] <= 0.2  && ndvi_sum.ptr<float>(i)[j] > -0.1)// land
	{
	  image.at<cv::Vec3b>(i,j)= cv::Vec3b(20,70,139);
	}
	if(ndvi_sum.ptr<float>(i)[j]  <= -0.1)// water
	{
	  image.at<cv::Vec3b>(i,j)= cv::Vec3b(100,0,0);
	}
      }
    }
    
//         namedWindow( "ndvi", CV_WINDOW_AUTOSIZE );
//   imshow( "ndvi", image );
//   waitKey(10);
//     

   output1= finalMat_sum;
   //output2 = image; 
}



