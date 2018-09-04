
#include <ImageAnalyzer/ImageAnalyzer.hpp>


using namespace cv;
//using namespace std;

#define TIME_DEBUG_SLIDE_IMAGES 1


void CameraArrayProject::ImageAnalyzer::translateImgAndComputeDiff2(Mat &img, Mat &img2, Mat &out, Mat &gimg, Mat &gimg2, Mat &gout, int offsetx, int offsety)
{
	int minx, maxx, zeroStart, zeroEnd;
	float *outptri;
	float *goutptri;
	float *imgptri;
	float *gimgptri;
	float *img2ptri;
	float *gimg2ptri;
	if(offsetx<0){
		minx = 0;
		maxx = dinx+offsetx;
		zeroStart = maxx;
		zeroEnd = dinx;
	}
	else {
		minx = offsetx;
		maxx = dinx;
		zeroStart = 0;
		zeroEnd = minx;
	}
	for (int i = 0; i < diny; i++){
		outptri = out.ptr<float>(i);
		goutptri = gout.ptr<float>(i);
		imgptri=img.ptr<float>(i);
		gimgptri=gimg.ptr<float>(i);
		img2ptri=img2.ptr<float>(i);
		gimg2ptri=gimg2.ptr<float>(i);
		for (int j = minx; j < maxx; j++){
			outptri[j] = abs(imgptri[j-offsetx] - img2ptri[j]);
			goutptri[j] = abs(gimgptri[j-offsetx] - gimg2ptri[j]);
		}
		for (int j = zeroStart; j<zeroEnd; j++){
			outptri[j] = /*abs*/(img2ptri[j]);
			goutptri[j] = abs(gimg2ptri[j]);
		}
	}
}









void CameraArrayProject::ImageAnalyzer::integrateImg2(Mat *img, Mat *imgout, Mat *gimg, Mat *gimgout)
{
	int dinx1 = dinx+1;
	int diny1 = diny+1;
	int im1, jm1;
	float* outptr;
	float* goutptr;
	float* imgoutkptrim1;
	float* gimgoutkptrim1;
	float* imgkptrim1;
	float* gimgkptrim1;
	
	for (int j = 0; j<dinx1; j++){
		for (int k = 0; k<max_disparity; k++){
			imgout[k].ptr<float>(0)[j]=0.0;
			gimgout[k].ptr<float>(0)[j]=0.0;
		}
	}
	for (int i = 1; i<diny1; i++){
		for (int k = 0; k<max_disparity; k++){
			im1 = i-1;
			outptr = imgout[k].ptr<float>(i);
			goutptr = gimgout[k].ptr<float>(i);
			imgoutkptrim1 = imgout[k].ptr<float>(im1);
			gimgoutkptrim1 = gimgout[k].ptr<float>(im1);
			imgkptrim1 = img[k].ptr<float>(im1);
			gimgkptrim1 = gimg[k].ptr<float>(im1);
			
			outptr[0]=0.0;
			gimgout[k].ptr<float>(i)[0]=0.0;
			for (int j = 1; j<dinx1; j++){
				jm1 = j-1;
				outptr[j]=	 imgoutkptrim1[j]+outptr[jm1]+imgkptrim1[jm1]-imgoutkptrim1[jm1];
				goutptr[j]=	 gimgoutkptrim1[j]+goutptr[jm1]+gimgkptrim1[jm1]-gimgoutkptrim1[jm1];
			}
		}
	}
}







void CameraArrayProject::ImageAnalyzer::blockImg2(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout, Mat &dout, Mat &mindiff, Mat &dispar, int i)
{
	float *doutptr, *mindiffptr, *disparptr;
	int maxy, miny, maxx, minx;
	int ray = data_structs.h.rows/2;
	float raysquare = data_structs.h.rows*data_structs.h.rows;
	float graysquare = raysquare/weight_CGRAD;

	for(int y = 0; y < diny; y++)
	{
		maxy = min(y+1+ray,diny);
		miny = max(y-ray,0);
		doutptr = dout.ptr<float>(y);
		mindiffptr = mindiff.ptr<float>(y);
		disparptr = dispar.ptr<float>(y);
		for(int x = 0; x < dinx; x++)
		{
			maxx = min(x+1+ray,dinx);
			minx = max(x-ray,0);
			doutptr[x] =			( img.ptr<float>(maxy)[maxx]
												- img.ptr<float>(miny)[maxx]
												- img.ptr<float>(maxy)[minx]
												+ img.ptr<float>(miny)[minx] )/raysquare
												+( gimg.ptr<float>(maxy)[maxx]
												- gimg.ptr<float>(miny)[maxx]
												- gimg.ptr<float>(maxy)[minx]
												+ gimg.ptr<float>(miny)[minx] )/graysquare;
			
			if(doutptr[x] < mindiffptr[x]) 
			{
				mindiffptr[x] = doutptr[x];
				disparptr[x] = i;
			}
		}
	}
}















void CameraArrayProject::ImageAnalyzer::disparity_map_slide_images2(Mat imgL, Mat imgR, int mins, int maxs, Mat &output1, Mat &output2)//, int weigth, Mat &disparity, Mat &mindiff)
{
	
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
ros::Time begin, end;
begin = ros::Time::now();
#endif

	
	
// 	data_structs.mindiff.setTo(Scalar(1000000.0));
// 	data_structs.disparity.setTo(Scalar(0.0));
	output1.setTo(Scalar(1000000.0));
	output2.setTo(Scalar(0.0));
	
	
	Sobel( imgR, data_structs.r_gradientX, CV_32FC1, 1, 0);
	Sobel( imgL, data_structs.l_gradientX, CV_32FC1, 1, 0);
	
//       printFloatImg(data_structs.r_gradientX, "data_structs.r_gradientX1", 10, false);
//       printFloatImg(data_structs.l_gradientX, "data_structs.l_gradientX1", 10, false);
	
	GaussianBlur(data_structs.r_gradientX, data_structs.r_gradientXblured, blur_window_size, 10);
	GaussianBlur(data_structs.l_gradientX, data_structs.l_gradientXblured, blur_window_size, 10);
	
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
end = ros::Time::now();
std::cout << "ImageAnalyzer::detect_material_online::disparity_map::slide_images::Sobel " << (end-begin).toSec() << std::endl;
#endif
	
//       printFloatImg(data_structs.r_gradientX, "data_structs.r_gradientX", 10, false);
//       printFloatImg(data_structs.l_gradientX, "data_structs.l_gradientX", 10, false);
	
	int step;
	if(maxs-mins < 0)  {step = +step_size;}
	if(maxs-mins >= 0)  {step = -step_size;}
	
	Point anchor( 0 ,0 );
	double delta = 0;
	
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
double cycle1=0.0;
double cycle2a=0.0;
double cycle2b=0.0;
double cycle3=0.0;
#endif
	
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
		begin = ros::Time::now();
#endif
	for(int k = mins; abs(k) < abs(maxs); k+=step )
	{
		int kabs = abs(k);
		// ###################################################
		// translate the left image and its gradient
		// also compute the difference of the result
		// wrt the right image and its gradient 
		// ###################################################
// 		translateImg(imgL,data_structs.shift, data_structs.l_gradientXblured,data_structs.shift_gradient, i,0);
// 		translateImg(data_structs.l_gradientXblured,data_structs.shift_gradient, i,0);
// 		data_structs.diff = abs(imgR - data_structs.shift);
// 		data_structs.gdiff = abs(data_structs.shift_gradient-data_structs.r_gradientXblured);
		translateImgAndComputeDiff2(imgL, imgR, data_structs.diffptr[kabs], data_structs.l_gradientXblured, data_structs.r_gradientXblured, data_structs.gdiffptr[kabs], k,0);
	}
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
end = ros::Time::now();
cycle1 = (end-begin).toSec();
#endif
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
		begin = ros::Time::now();
#endif
		
	// ###################################################
	// create the integral image of the diff and the gdiff
	// ###################################################
	
	integrateImg2(data_structs.diffptr, data_structs.int_diffptr, data_structs.gdiffptr, data_structs.int_gdiffptr);
	
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
end = ros::Time::now();
cycle2a = (end-begin).toSec();
begin = ros::Time::now();
#endif



	for(int k = mins; abs(k) < abs(maxs); k+=step )
	{
		int kabs = abs(k);
		
		
		// #################################################################################
		// compute the sum of the diff and the gdiff for each block using the integral image
		// also computes the weighted sum of the results
		// #################################################################################
		blockImg2(data_structs.int_diffptr[kabs], data_structs.CSADptr[kabs], data_structs.int_gdiffptr[kabs], data_structs.CGRADptr[kabs], data_structs.d, output1, output2, kabs);
		
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
end = ros::Time::now();
cycle2b += (end-begin).toSec();
// begin = ros::Time::now();
#endif
// 		float *data_structsdptr = data_structs.d.ptr<float>(0);
// 		float *data_structsmindiffptr = output1.ptr<float>(0);
// 		float *data_structsdisparityptr = output2.ptr<float>(0);
// // 		float *data_structsmindiffptr = data_structs.mindiff.ptr<float>(0);
// // 		float *data_structsdisparityptr = data_structs.disparity.ptr<float>(0);
// 		int iDisp = abs(i);
// 		int numPixels = dinx*diny;
// 		for(int y = 0; y < numPixels; y++)
// 		{
// 			if(data_structsdptr[y] < data_structsmindiffptr[y]) 
// 			{
// 				data_structsmindiffptr[y] = data_structsdptr[y];
// 				data_structsdisparityptr[y] = iDisp;
// 			}
// 		}

// 		for(int y = 0; y < diny; y++)
// 		{
// 			for(int x = 0; x < dinx; x++)
// 			{
// 				if(data_structs.d.ptr<float>(y)[x] < data_structs.mindiff.ptr<float>(y)[x]) 
// 				{
// 					data_structs.mindiff.ptr<float>(y)[x] = data_structs.d.ptr<float>(y)[x];  
// 					data_structs.disparity.ptr<float>(y)[x] = iDisp;
// 				}
// 			}
// 		}
// #ifdef DEBUG_TIME_PRINTS
// end = ros::Time::now();
// cycle3 += (end-begin).toSec();
// #endif
	}
	
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
std::cout << "ImageAnalyzer::detect_material_online::disparity_map::slide_images::translate " << cycle1 << std::endl;
std::cout << "ImageAnalyzer::detect_material_online::disparity_map::slide_images::integral  " << cycle2a << std::endl;
std::cout << "ImageAnalyzer::detect_material_online::disparity_map::slide_images::using_int " << cycle2b << std::endl;
// std::cout << "slide_images merging     " << cycle3 << std::endl;
#endif

// 		printFloatImg(data_structs.disparity, "data_structs.disparity", 100000, false);

	
// 	output1 =  data_structs.mindiff;
// 	output2 =  data_structs.disparity;
// 	data_structs.mindiff.setTo(Scalar(1000000.0));
// 	data_structs.disparity.setTo(Scalar(0.0));

}

