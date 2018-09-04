
#include <ImageAnalyzer/ImageAnalyzer.hpp>


using namespace cv;
//using namespace std;

#define TIME_DEBUG_SLIDE_IMAGES 1

// Mat CameraArrayProject::ImageAnalyzer::translateImg(Mat img, Mat &imgout,int offsetx, int offsety)
// {
//   Mat trans_mat = (Mat_<float>(2,3) << 1, 0, offsetx, 0, 1, offsety);
//   warpAffine(img,imgout,trans_mat,img.size());
//   return trans_mat;
// }

void CameraArrayProject::ImageAnalyzer::translateImg(Mat &img, Mat &imgout,int offsetx, int offsety)
{
	if(offsetx<0){
		int minx = 0;
		int maxx = dinx+offsetx;
		for (int i = 0; i < diny; i++){
			for (int j = minx; j < maxx; j++){
				imgout.ptr<float>(i)[j] = img.ptr<float>(i)[j-offsetx];
			}
			for (int j = maxx; j<dinx; j++){
				imgout.ptr<float>(i)[j] = 0.0;
			}
		}
	}
	if(offsetx>=0){
		int minx = offsetx;
		int maxx = dinx;
		for (int i = 0; i < diny; i++){
			for (int j = 0; j<minx; j++){
				imgout.ptr<float>(i)[j] = 0.0;
			}
			for (int j = minx; j < maxx; j++){
				imgout.ptr<float>(i)[j] = img.ptr<float>(i)[j-offsetx];
			}
		}
	}
}



void CameraArrayProject::ImageAnalyzer::translateImg(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout, int offsetx, int offsety)
{
	if(offsetx<0){
		int minx = 0;
		int maxx = dinx+offsetx;
		for (int i = 0; i < diny; i++){
			for (int j = minx; j < maxx; j++){
				imgout.ptr<float>(i)[j] = img.ptr<float>(i)[j-offsetx];
				gimgout.ptr<float>(i)[j] = gimg.ptr<float>(i)[j-offsetx];
			}
			for (int j = maxx; j<dinx; j++){
				imgout.ptr<float>(i)[j] = 0.0;
				gimgout.ptr<float>(i)[j] = 0.0;
			}
		}
	}
	if(offsetx>=0){
		int minx = offsetx;
		int maxx = dinx;
		for (int i = 0; i < diny; i++){
			for (int j = 0; j<minx; j++){
				imgout.ptr<float>(i)[j] = 0.0;
				gimgout.ptr<float>(i)[j] = 0.0;
			}
			for (int j = minx; j < maxx; j++){
				imgout.ptr<float>(i)[j] = img.ptr<float>(i)[j-offsetx];
				gimgout.ptr<float>(i)[j] = gimg.ptr<float>(i)[j-offsetx];
			}
		}
	}
}


void CameraArrayProject::ImageAnalyzer::translateImgAndComputeDiff(Mat &img, Mat &img2, Mat &out, Mat &gimg, Mat &gimg2, Mat &gout, int offsetx, int offsety)
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












void CameraArrayProject::ImageAnalyzer::integrateImg(Mat &img, Mat &imgout)
{
	int dinx1 = dinx+1;
	int diny1 = diny+1;
	int im1, jm1;
	float* outptr;
	
	for (int j = 0; j<dinx1; j++){
		imgout.ptr<float>(0)[j]=0.0;
	}
	for (int i = 1; i<diny1; i++){
		imgout.ptr<float>(i)[0]=0.0;
		im1 = i-1;
		outptr = imgout.ptr<float>(i);
		for (int j = 1; j<dinx1; j++){
			jm1 = j-1;
			outptr[j]=imgout.ptr<float>(im1)[j]+imgout.ptr<float>(i)[jm1]+img.ptr<float>(im1)[jm1]-imgout.ptr<float>(im1)[jm1];
		}
	}
}



void CameraArrayProject::ImageAnalyzer::integrateImg(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout)
{
	int dinx1 = dinx+1;
	int diny1 = diny+1;
	int im1, jm1;
	float* outptr;
	float* goutptr;
	
	for (int j = 0; j<dinx1; j++){
		imgout.ptr<float>(0)[j]=0.0;
		gimgout.ptr<float>(0)[j]=0.0;
	}
	for (int i = 1; i<diny1; i++){
		imgout.ptr<float>(i)[0]=0.0;
		gimgout.ptr<float>(i)[0]=0.0;
		im1 = i-1;
		outptr = imgout.ptr<float>(i);
		goutptr = gimgout.ptr<float>(i);
		for (int j = 1; j<dinx1; j++){
			jm1 = j-1;
			outptr[j]=	 imgout.ptr<float>(im1)[j]
									+imgout.ptr<float>(i)[jm1]
									+img.ptr<float>(im1)[jm1]
									-imgout.ptr<float>(im1)[jm1];
			goutptr[j]=	 gimgout.ptr<float>(im1)[j]
									+gimgout.ptr<float>(i)[jm1]
									+gimg.ptr<float>(im1)[jm1]
									-gimgout.ptr<float>(im1)[jm1];
		}
	}
}



void CameraArrayProject::ImageAnalyzer::blockImg(Mat &img, Mat &imgout, int bs)
{
	int ray = bs/2;
	float raysquare = bs*bs;
	// equivalent  to imfilter Matlab
	for(int y = 1; y < diny+1; y++)
	{
		int maxy = min(y+ray,diny);
		int miny = max(y-ray-1,0);
		float* outptr = imgout.ptr<float>(y-1);
		for(int x = 1; x < dinx+1; x++)
		{
			int maxx = min(x+ray,dinx);
			int minx = max(x-ray-1,0);
			
// 			std::cout << x-1 << " " << y-1;
// 			std::cout << " x: " << minx << " " << maxx << "  y: " << miny << " " << maxy << std::endl;
 			outptr[x-1] =			( img.ptr<float>(maxy)[maxx]
												- img.ptr<float>(miny)[maxx]
												- img.ptr<float>(maxy)[minx]
												+ img.ptr<float>(miny)[minx] )/raysquare;
		}
	}
}



void CameraArrayProject::ImageAnalyzer::blockImg(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout, Mat &dout, int bs)
{
	int dinx1 = dinx+1, diny1 = diny+1;
	float *outptr, *goutptr, *doutptr;
	int maxy, miny, maxx, minx;
	int ray = bs/2;
	float raysquare = bs*bs;
	float graysquare = raysquare/weight_CGRAD;
	// equivalent  to imfilter Matlab
	int ym1 = 0;
	int xm1;
	for(int y = 1; y < diny1; y++)
	{
		maxy = min(y+ray,diny);
		miny = max(ym1-ray,0);
		outptr = imgout.ptr<float>(ym1);
		goutptr = gimgout.ptr<float>(ym1);
		doutptr = dout.ptr<float>(ym1);
		xm1 = 0;
		for(int x = 1; x < dinx1; x++)
		{
			maxx = min(x+ray,dinx);
			minx = max(xm1-ray,0);
// 			std::cout << x-1 << " " << y-1;
// 			std::cout << " x: " << minx << " " << maxx << "  y: " << miny << " " << maxy << std::endl;
			outptr[xm1] =			( img.ptr<float>(maxy)[maxx]
												- img.ptr<float>(miny)[maxx]
												- img.ptr<float>(maxy)[minx]
												+ img.ptr<float>(miny)[minx] )/raysquare;
			goutptr[xm1] =		( gimg.ptr<float>(maxy)[maxx]
												- gimg.ptr<float>(miny)[maxx]
												- gimg.ptr<float>(maxy)[minx]
												+ gimg.ptr<float>(miny)[minx] )/graysquare;
			doutptr[xm1] = outptr[xm1]+goutptr[xm1];
			xm1 = x;
		}
		ym1 = y;
	}
}







void CameraArrayProject::ImageAnalyzer::blockImg(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout, Mat &mindiff, Mat &dispar, int i, int bs)
{
	float *mindiffptr, *disparptr;
	int maxy, miny, maxx, minx;
	int ray = bs/2;
	float raysquare = bs*bs;
	float graysquare = raysquare/weight_CGRAD;
	
	float doutfloat;

	for(int y = 0; y < diny; y++)
	{
		maxy = min(y+1+ray,diny);
		miny = max(y-ray,0);
// 		doutptr = dout.ptr<float>(y);
		mindiffptr = mindiff.ptr<float>(y);
		disparptr = dispar.ptr<float>(y);
		for(int x = 0; x < dinx; x++)
		{
			maxx = min(x+1+ray,dinx);
			minx = max(x-ray,0);
// 			doutptr[x] =			( img.ptr<float>(maxy)[maxx]
			doutfloat =			( img.ptr<float>(maxy)[maxx]
												- img.ptr<float>(miny)[maxx]
												- img.ptr<float>(maxy)[minx]
												+ img.ptr<float>(miny)[minx] )/raysquare
												+( gimg.ptr<float>(maxy)[maxx]
												- gimg.ptr<float>(miny)[maxx]
												- gimg.ptr<float>(maxy)[minx]
												+ gimg.ptr<float>(miny)[minx] )/graysquare;
			
// 			if(doutptr[x] < mindiffptr[x]) 
			if(doutfloat < mindiffptr[x]) 
			{
// 				mindiffptr[x] = doutptr[x];
				mindiffptr[x] = doutfloat ;
				disparptr[x] = i;
			}
		}
	}
}





void CameraArrayProject::ImageAnalyzer::disparity_map_compute_gradients(Mat imgL, Mat imgR, Mat &gimgL, Mat &gimgR, Mat &gimgLblured, Mat &gimgRblured)//, int weigth, Mat &disparity, Mat &mindiff)
{
	Sobel( imgR, gimgR, CV_32FC1, 1, 0);
	Sobel( imgL, gimgL, CV_32FC1, 1, 0);
	
	GaussianBlur(gimgR, gimgRblured, blur_window_size, 10);
	GaussianBlur(gimgL, gimgLblured, blur_window_size, 10);
}




void CameraArrayProject::ImageAnalyzer::threaded_disparity_map_slide_images(){
	while(1){
		if (slide_imgs_flag){
			disparity_map_slide_images(imgL_th, imgR_th, gradients.l_gradientXblured, gradients.r_gradientXblured, 0, -max_disparity,
														 slide_lr.mindiff, slide_lr.disparity, slide_lr);
			slide_imgs_flag = false;
		}
		usleep(500);
	}
}




void CameraArrayProject::ImageAnalyzer::disparity_map_slide_images(Mat imgL, Mat imgR, Mat gimgL, Mat gimgR, int mins, int maxs, Mat &output1, Mat &output2, SlideImgs &slide_imgs)
{
	
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
ros::Time begin, end;
begin = ros::Time::now();
#endif
	
	output1.setTo(Scalar(1000000.0));
	output2.setTo(Scalar(0.0));
	
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
end = ros::Time::now();
std::cout << "ImageAnalyzer::detect_material_online::disparity_map::slide_images::init " << (end-begin).toSec() << std::endl;
#endif
	
//       printFloatImg(data_structs.r_gradientX, "data_structs.r_gradientX", 10, false);
//       printFloatImg(data_structs.l_gradientX, "data_structs.l_gradientX", 10, false);
	
	int step;
	if(maxs-mins < 0)  {step = +step_size;}
	if(maxs-mins >= 0)  {step = -step_size;}
	
	
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
double cycle1=0.0;
double cycle2a=0.0;
double cycle2b=0.0;
#endif
	
	for(int i = mins; abs(i) < abs(maxs); i+=step )
	{
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
		begin = ros::Time::now();
#endif
		
		// ###################################################
		// translate the left image and its gradient
		// also compute the difference of the result
		// wrt the right image and its gradient 
		// ###################################################
// 		translateImg(imgL,data_structs.shift, data_structs.l_gradientXblured,data_structs.shift_gradient, i,0);
// 		translateImg(data_structs.l_gradientXblured,data_structs.shift_gradient, i,0);
// 		data_structs.diff = abs(imgR - data_structs.shift);
// 		data_structs.gdiff = abs(data_structs.shift_gradient-data_structs.r_gradientXblured);
		translateImgAndComputeDiff(imgL, imgR, slide_imgs.diff, gimgL, gimgR, slide_imgs.gdiff, i,0);
		
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
end = ros::Time::now();
cycle1 += (end-begin).toSec();
// 		std::cout << "cycle z " << i << " " << (end-begin).toSec() << std::endl;
// 		printFloatImg(data_structs.shift, "data_structs.shift", 10, false);
// 		printFloatImg(data_structs.shift_gradient, "data_structs.shift_gradient", 10, false);
// 		usleep(100000);
begin = ros::Time::now();
#endif
		
		// ###################################################
		// create the integral image of the diff and the gdiff
		// ###################################################
		
		integrateImg(slide_imgs.diff, slide_imgs.int_diff, slide_imgs.gdiff, slide_imgs.int_gdiff);
		
#if TIME_DEBUG_SLIDE_IMAGES && DEBUG_TIME
end = ros::Time::now();
cycle2a += (end-begin).toSec();
// 		printFloatImg(data_structs.int_diff, "data_structs.int_diff", 10, false);
// 		printFloatImg(data_structs.int_gdiff, "data_structs.int_gdiff", 10, false);
begin = ros::Time::now();
#endif
		
		
		// #################################################################################
		// compute the sum of the diff and the gdiff for each block using the integral image
		// also computes the weighted sum of the results
		// #################################################################################
		blockImg(slide_imgs.int_diff, slide_imgs.CSAD, slide_imgs.int_gdiff, slide_imgs.CGRAD, output1, output2, abs(i), slide_imgs.block_size);
		
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
std::cout << "ImageAnalyzer::detect_material_online::disparity_map::slide_images::using_int "  << cycle2b  << " " << maxs<< std::endl;
// std::cout << "slide_images merging     " << cycle3 << std::endl;
#endif

// 		printFloatImg(data_structs.disparity, "data_structs.disparity", 100000, false);

	
// 	output1 =  data_structs.mindiff;
// 	output2 =  data_structs.disparity;
// 	data_structs.mindiff.setTo(Scalar(1000000.0));
// 	data_structs.disparity.setTo(Scalar(0.0));

}

