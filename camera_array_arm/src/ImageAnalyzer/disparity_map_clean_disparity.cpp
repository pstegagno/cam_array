
#include <ImageAnalyzer/ImageAnalyzer.hpp>

#include <thread> 

#define MIN_VALID_DSP 1.0

using namespace cv;
//using namespace std;

void CameraArrayProject::ImageAnalyzer::disparity_map_clean_disparity_thread(Mat &dsp){
		int start =-1, end = -1; 
	int start_value, end_value;
	float step_size;
	
	for( int y = 0; y < diny; y++)
	{
		for( int x = 0; x < dinx; x++)
		{ 
			if(dsp.ptr<float>(y)[x]>= MIN_VALID_DSP)
			{
				data_structs.clean_left.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
			}
			else if(start == -1 && dsp.ptr<float>(y)[x] == 0)
			{
				start = x;
				if (start == 0){
					start_value = 0;
				}
				else {
					start_value = dsp.ptr<float>(y)[x-1];
				}
// 				std::cout << "start = " << x << " start_value = " << start_value << std::endl;
			}
			if(start != -1 && dsp.ptr<float>(y)[x] >= MIN_VALID_DSP && end == -1)
			{
				end = x;
				end_value = dsp.ptr<float>(y)[x];
			}
			if(start !=-1 && end != -1)
			{
				step_size = ((float)(end_value - start_value))/((float)(end-start));
				if(start != 0){
					for (int i = start; i < end; i++)
					{
						data_structs.clean_left.ptr<float>(y)[i] = data_structs.clean_left.ptr<float>(y)[i-1] + step_size;
					}
				}
				else
				{
					for (int i = start; i < end; i++)
					{
						data_structs.clean_left.ptr<float>(y)[i] = end_value;
					}
				}
				end = -1;
				start = -1;
			}
			else if(start !=-1 && end == -1 && x == dinx-1)
			{
				step_size = ((float)(end_value - start_value))/((float)(end-start));
				for (int i = start; i < dinx; i++)
				{
					data_structs.clean_left.ptr<float>(y)[i] = start_value;
				}
				end = -1;
				start = -1;
			}
		}
		end = -1;
		start = -1;
	}
}

void CameraArrayProject::ImageAnalyzer::disparity_map_clean_disparity(Mat &dsp, Mat &dsp_clean){
  
	std::thread local_thread(&CameraArrayProject::ImageAnalyzer::disparity_map_clean_disparity_thread, this, std::ref(dsp)); // pass by value
//   Disparity data_structs(diny,dinx,win_size,win_size);

  // first method 
  /*
  for( int y = 1; y < diny; y++)
  {
    for( int x = 1; x < dinx; x++)
    { 
      xrigth = dsp.cols - x -1;
      yup = dsp.rows -y -1;
      
   // std::cout<< "y is " << yup <<  std::endl;
      
      if(dsp.ptr<float>(y)[x]<1.0 && dsp.ptr<float>(y)[x-1]>=1.0)
      {
      data_structs.clean_left.ptr<float>(y)[x] = dsp.ptr<float>(y)[x-1];	
      }
      if(dsp.ptr<float>(y)[x]<1.0 && dsp.ptr<float>(y)[x-1]<1.0)
      {
      data_structs.clean_left.ptr<float>(y)[x] = data_structs.clean_left.ptr<float>(y)[x-1];
      }


      if(dsp.ptr<float>(y)[xrigth]<1.0 && dsp.ptr<float>(y)[xrigth+1]>=1.0)
      {
      data_structs.clean_right.ptr<float>(y)[xrigth] = dsp.ptr<float>(y)[xrigth+1];	
      }
      if(dsp.ptr<float>(y)[xrigth]<1.0 && dsp.ptr<float>(y)[xrigth+1]<1.0)
      {
      data_structs.clean_right.ptr<float>(y)[xrigth] = data_structs.clean_right.ptr<float>(y)[xrigth+1];
      }

	if(dsp.ptr<float>(y)[x]<1.0 && dsp.ptr<float>(y-1)[x]>=1.0)
      {
      data_structs.clean_down.ptr<float>(y)[x] = dsp.ptr<float>(y-1)[x];	
      }
      if(dsp.ptr<float>(y)[x]<1.0 && dsp.ptr<float>(y-1)[x]<1.0)
      {
      data_structs.clean_down.ptr<float>(y)[x] = data_structs.clean_down.ptr<float>(y-1)[x];
      }

      
      if(dsp.ptr<float>(yup)[x]<1.0 && dsp.ptr<float>(yup+1)[x]>=1.0)
      {
      data_structs.clean_up.ptr<float>(yup)[x] = dsp.ptr<float>(yup+1)[x];	
      }
      if(dsp.ptr<float>(yup)[x]==0 && dsp.ptr<float>(yup+1)[x]==0)
      {
      data_structs.clean_up.ptr<float>(yup)[x] = data_structs.clean_up.ptr<float>(yup+1)[x];
      }

      
      // std::cout<<"\n the new one L "<< data_structs.clean_left <<std::endl;
//   std::cout<<"\n the new one R "<< data_structs.clean_right <<std::endl;
//   std::cout<<"\n the new one "<< data_structs.clean <<std::endl;
      if(dsp.ptr<float>(y)[x]>=1.0)
      {
	data_structs.clean_left.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
	data_structs.clean_right.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
	data_structs.clean_up.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
	data_structs.clean_down.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
      }
	
    }
    
  }
//   
  data_structs.clean = (data_structs.clean_left + data_structs.clean_right+ data_structs.clean_down +data_structs.clean_up)/4;
  dsp_clean = data_structs.clean; 
  */
// //   std::cout<<"\n the new one L "<< data_structs.clean_left <<std::endl;
//   std::cout<<"\n the new one R "<< data_structs.clean_right <<std::endl;
//   std::cout<<"\n the new one "<< data_structs.clean <<std::endl;
//   std::cout<<"\n the new one down  "<< data_structs.clean_down <<std::endl;
//   std::cout<<"\n the new one up "<< data_structs.clean_up <<std::endl;
  
 //printFloatImg(data_structs.clean, "clean no_gradient", 10, false);
  
  
// Second method  
  
	//gradient along x
	int start =-1, end = -1; 
	int start_value, end_value;
	float step_size;
	
// 	for( int y = 0; y < diny; y++)
// 	{
// 		for( int x = 0; x < dinx; x++)
// 		{ 
// 			if(dsp.ptr<float>(y)[x]>= 1.0)
// 			{
// 				data_structs.clean_left.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
// 			}
// 			else if(start == -1 && dsp.ptr<float>(y)[x] == 0)
// 			{
// 				start = x;
// 				if (start == 0){
// 					start_value = 0;
// 				}
// 				else {
// 					start_value = dsp.ptr<float>(y)[x-1];
// 				}
// // 				std::cout << "start = " << x << " start_value = " << start_value << std::endl;
// 			}
// 			if(start != -1 && dsp.ptr<float>(y)[x] >= 1.0 && end == -1)
// 			{
// 				end = x;
// 				end_value = dsp.ptr<float>(y)[x];
// 			}
// 			if(start !=-1 && end != -1)
// 			{
// 				step_size = ((float)(end_value - start_value))/((float)(end-start));
// 				if(start != 0){
// 					for (int i = start; i < end; i++)
// 					{
// 						data_structs.clean_left.ptr<float>(y)[i] = data_structs.clean_left.ptr<float>(y)[i-1] + step_size;
// 					}
// 				}
// 				else
// 				{
// 					for (int i = start; i < end; i++)
// 					{
// 						data_structs.clean_left.ptr<float>(y)[i] = end_value;
// 					}
// 				}
// 				end = -1;
// 				start = -1;
// 			}
// 			else if(start !=-1 && end == -1 && x == dinx-1)
// 			{
// 				step_size = ((float)(end_value - start_value))/((float)(end-start));
// 				for (int i = start; i < dinx; i++)
// 				{
// 					data_structs.clean_left.ptr<float>(y)[i] = start_value;
// 				}
// 				end = -1;
// 				start = -1;
// 			}
// 		}
// 		end = -1;
// 		start = -1;
// 	}
	
	
	for( int x = 0; x < dinx; x++) 
	{
		for( int y = 0; y  < diny; y++)
		{
			if(dsp.ptr<float>(y)[x]>= MIN_VALID_DSP)
			{
				data_structs.clean_down.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
			}
			else if(start == -1 && dsp.ptr<float>(y)[x] == 0)
			{
				start = y;
				if (start == 0){
					start_value = 0;
				}
				else {
					start_value = dsp.ptr<float>(y-1)[x];
				}
			}
			if(start != -1 && dsp.ptr<float>(y)[x] >= MIN_VALID_DSP && end == -1)
			{
				end = y;
				end_value = dsp.ptr<float>(y)[x];
			}
			//filling
			if(start !=-1 && end != -1)
			{
				step_size = ((float)(end_value - start_value))/((float)(end-start));
				if(start != 0){
					for (int i = start; i < end; i++)
					{
						data_structs.clean_down.ptr<float>(i)[x] = data_structs.clean_down.ptr<float>(i-1)[x] + step_size;
					}
				}
				else {
					for (int i = start; i < end; i++)
					{
						data_structs.clean_down.ptr<float>(i)[x] = end_value;
					}
				}
				end = -1;
				start = -1;
			}
			else if(start !=-1 && end == -1 && y == diny-1)
			{
				step_size = ((float)(end_value - start_value))/((float)(end-start));
				for (int i = start; i < diny; i++)
				{
					data_structs.clean_down.ptr<float>(i)[x] = start_value;
				}
				end = -1;
				start = -1;
			}
		}
		end = -1;
		start = -1;
	}
	
	local_thread.join();
	
// 	data_structs.clean = (data_structs.clean_left+ data_structs.clean_down)/2;
// 	dsp_clean = data_structs.clean; 
	dsp_clean = (data_structs.clean_left+ data_structs.clean_down)/2;
}
  
  
  
