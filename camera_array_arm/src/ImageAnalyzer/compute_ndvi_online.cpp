#include <cv.h>
#include <highgui.h>
#include<math.h>
#include<new>
#include <iterator>
#include <vector>
#include <algorithm>
#include <sys/time.h>

#include <ImageAnalyzer/ImageAnalyzer.hpp>

#define DEBUG_COMPUTE_NDVI_ONLINE 0

void CameraArrayProject::ImageAnalyzer::compute_ndvi_online( float expTime_corection ,Mat nearInfared, Mat red, Mat &output1)          
{
  
#if DEBUG_COMPUTE_NDVI_ONLINE && DEBUG_TIME
TIME_DEBUG_INIT();
#endif

	if (red.type() != CV_32FC1){
		std::cout  << "ERROR CameraArrayProject::ImageAnalyzer::compute_ndvi_online: red is not of type CV_32F" << std::endl;
		return;
	}
	if (red.channels() != 1){
		std::cout  << "ERROR CameraArrayProject::ImageAnalyzer::compute_ndvi_online: red is not of type CV_32FC1" << std::endl;
		return;
	}
	if (nearInfared.type() != CV_32FC1){
		std::cout  << "ERROR CameraArrayProject::ImageAnalyzer::compute_ndvi_online: nearInfared is not of type CV_32F" << std::endl;
		return;
	}
	if (nearInfared.channels() != 1){
		std::cout  << "ERROR CameraArrayProject::ImageAnalyzer::compute_ndvi_online: nearInfared is not of type CV_32FC1" << std::endl;
		return;
	}
	
		
// 	Mat redf, nirf;
// 	red.convertTo(data_structs.redf, CV_32FC1);
// 	nearInfared.convertTo(data_structs.nirf, CV_32FC1);

	double coef_areaR = 1.43*(double)expTime_corection; 
	double coef_areaNIR = 1; 
	
	Mat /*finalMat_sum,*/ finalMat_sum_temp(diny, dinx, CV_32FC1); 
	
	
	// compute the bit per bit ndvi index
	float nirf_sum, redf_sum;
	for(int i = 0; i <diny ; i++)
	{
		for(int j = 0; j <dinx ; j++)
		{
			// this condition is put to exclude from classification areas in which there is a strong source of light or too much shadow
			float pixel_sum = nearInfared.ptr<float>(i)[j] + red.ptr<float>(i)[j];
			if (pixel_sum>500 || pixel_sum < 15){
				finalMat_sum_temp.ptr<float>(i)[j] = 2.0;
			}
			else{
			// END OF this condition is put to exclude from classification areas in which there is a strong source of light or too much shadow
				
				// here we compute the typical ndvi index
				nirf_sum = nearInfared.ptr<float>(i)[j]*coef_areaNIR;
				redf_sum = red.ptr<float>(i)[j]*coef_areaR;
				data_structs.ndvi_sum.ptr<float>(i)[j] = (nirf_sum - redf_sum)/(nirf_sum + redf_sum);
				finalMat_sum_temp.ptr<float>(i)[j] = (data_structs.ndvi_sum.ptr<float>(i)[j]+1.0)/2.0;
			}
		}
	}

	// Normalization
// 	Mat nirf_sum, redf_sum; 
// 	data_structs.nirf_sum = nearInfared*coef_areaNIR;
// 	data_structs.redf_sum = red*coef_areaR;
// 
// 	// calculation of NVDI index
// 	data_structs.num_sum = data_structs.nirf_sum - data_structs.redf_sum;
// 	data_structs.den_sum = data_structs.nirf_sum + data_structs.redf_sum;
// 	divide(data_structs.num_sum, data_structs.den_sum ,data_structs.ndvi_sum); // divide elememt by element

	// BLUR
	cv::GaussianBlur(finalMat_sum_temp, output1, Size(3,3), 10);
// 	cv::blur(finalMat_sum_temp, output1, Size(3,3), Point(-1,-1));
// 	output1 = finalMat_sum_temp;

#if DEBUG_COMPUTE_NDVI_ONLINE && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::compute_ndvi_online::init);
#endif    
    
// 	float temp_value;
// 	Mat image(diny,dinx,CV_8UC3);
// 
// 	for(int i = 0; i <diny ; i++)
// 	{
// 		for(int j = 0; j <dinx ; j++)
// 		{
// 			temp_value = data_structs.ndvi_sum.ptr<float>(i)[j];
// 			if(  temp_value  >  0.4) // dense vegetation dark green
// 			{
// 				image.at<cv::Vec3b>(i,j) = data_structs.dark_green;
// 			} 
// 			else if(0.2 < temp_value && temp_value <= 0.4)// shurb and grassland
// 			{
// 				image.at<cv::Vec3b>(i,j)= data_structs.green;
// 			}
// 			else if( temp_value <= 0.2  && temp_value > -0.1)// land
// 			{
// 				image.at<cv::Vec3b>(i,j)= data_structs.brown;
// 			}
// 			else if(temp_value  <= -0.1)// water
// 			{
// 				image.at<cv::Vec3b>(i,j)= data_structs.blue;
// 			}
// 		}
// 	}
	 
#if DEBUG_COMPUTE_NDVI_ONLINE && DEBUG_TIME
TIME_DEBUG_END(ImageAnalyzer::detect_material_online::compute_ndvi_online::cycle);
#endif
}



