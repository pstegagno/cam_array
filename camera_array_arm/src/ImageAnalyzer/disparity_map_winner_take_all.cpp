
#include <ImageAnalyzer/ImageAnalyzer.hpp>



using namespace cv;
//using namespace std;



void CameraArrayProject::ImageAnalyzer::disparity_map_winner_take_all(Mat d1, Mat m1, Mat d2,Mat m2, Mat &output)
{
//       printFloatImg(d1, "d1",0, false);
//       printFloatImg(d2, "d2",0, false);
	
	bool difference_condition;
	

	for(int y = 0; y < diny; y++)
		{
		for(int x = 0; x < dinx; x++)
			{
				difference_condition = abs(d1.ptr<float>(y)[x]-d2.ptr<float>(y)[x])< disparity_winner_take_all_tolerance;
				if(difference_condition && m1.ptr<float>(y)[x] <= m2.ptr<float>(y)[x])
				{
					data_structs.pixel_dsp.ptr<float>(y)[x] = d1.ptr<float>(y)[x];  
				}
				else if(difference_condition && m1.ptr<float>(y)[x] > m2.ptr<float>(y)[x])
				{
					data_structs.pixel_dsp.ptr<float>(y)[x] = d2.ptr<float>(y)[x];  
				}
				else
				{
					if(d1.ptr<float>(y)[x] == 0){
						data_structs.pixel_dsp.ptr<float>(y)[x] = d2.ptr<float>(y)[x];  
					}
					else if(d2.ptr<float>(y)[x] == 0){
						data_structs.pixel_dsp.ptr<float>(y)[x] = d1.ptr<float>(y)[x];  
					}
					else {
						data_structs.pixel_dsp.ptr<float>(y)[x] = 0.0;
					}
				}
			}
		}
// 		output = data_structs.pixel_dsp;
	translateImg(data_structs.pixel_dsp, output, 5, 0);
}

