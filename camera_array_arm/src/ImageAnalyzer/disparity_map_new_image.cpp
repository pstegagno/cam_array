
#include <ImageAnalyzer/ImageAnalyzer.hpp>


using namespace cv;




  

void CameraArrayProject::ImageAnalyzer::disparity_map_new_image(Mat imgL, Mat imgR, Mat dsp, Mat &calibrated)
{

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



