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



void ImageAnalyzer::clean_disparity(Mat dsp, Mat &dsp_clean){
  
     std::cout << "c2.1" << std::endl;

  int diny, dinx, win_size;//, xrigth, yup;
    
  diny = dsp.rows;
  dinx = dsp.cols;
  win_size  = 7;
     std::cout << "c2.2" << std::endl;

  Disparity img(diny,dinx,win_size,win_size);

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
      img.clean_left.ptr<float>(y)[x] = dsp.ptr<float>(y)[x-1];	
      }
      if(dsp.ptr<float>(y)[x]<1.0 && dsp.ptr<float>(y)[x-1]<1.0)
      {
      img.clean_left.ptr<float>(y)[x] = img.clean_left.ptr<float>(y)[x-1];
      }


      if(dsp.ptr<float>(y)[xrigth]<1.0 && dsp.ptr<float>(y)[xrigth+1]>=1.0)
      {
      img.clean_right.ptr<float>(y)[xrigth] = dsp.ptr<float>(y)[xrigth+1];	
      }
      if(dsp.ptr<float>(y)[xrigth]<1.0 && dsp.ptr<float>(y)[xrigth+1]<1.0)
      {
      img.clean_right.ptr<float>(y)[xrigth] = img.clean_right.ptr<float>(y)[xrigth+1];
      }

	if(dsp.ptr<float>(y)[x]<1.0 && dsp.ptr<float>(y-1)[x]>=1.0)
      {
      img.clean_down.ptr<float>(y)[x] = dsp.ptr<float>(y-1)[x];	
      }
      if(dsp.ptr<float>(y)[x]<1.0 && dsp.ptr<float>(y-1)[x]<1.0)
      {
      img.clean_down.ptr<float>(y)[x] = img.clean_down.ptr<float>(y-1)[x];
      }

      
      if(dsp.ptr<float>(yup)[x]<1.0 && dsp.ptr<float>(yup+1)[x]>=1.0)
      {
      img.clean_up.ptr<float>(yup)[x] = dsp.ptr<float>(yup+1)[x];	
      }
      if(dsp.ptr<float>(yup)[x]==0 && dsp.ptr<float>(yup+1)[x]==0)
      {
      img.clean_up.ptr<float>(yup)[x] = img.clean_up.ptr<float>(yup+1)[x];
      }

      
      // std::cout<<"\n the new one L "<< img.clean_left <<std::endl;
//   std::cout<<"\n the new one R "<< img.clean_right <<std::endl;
//   std::cout<<"\n the new one "<< img.clean <<std::endl;
      if(dsp.ptr<float>(y)[x]>=1.0)
      {
	img.clean_left.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
	img.clean_right.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
	img.clean_up.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
	img.clean_down.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
      }
	
    }
    
  }
//   
  img.clean = (img.clean_left + img.clean_right+ img.clean_down +img.clean_up)/4;
  dsp_clean = img.clean; 
  */
// //   std::cout<<"\n the new one L "<< img.clean_left <<std::endl;
//   std::cout<<"\n the new one R "<< img.clean_right <<std::endl;
//   std::cout<<"\n the new one "<< img.clean <<std::endl;
//   std::cout<<"\n the new one down  "<< img.clean_down <<std::endl;
//   std::cout<<"\n the new one up "<< img.clean_up <<std::endl;
  
 //printFloatImg(img.clean, "clean no_gradient", 10, false);
  
  
// Second method  
  
  //gradient along x
  int start =-1, end = -1; 
  int start_value, end_value;
  float step_size;

     std::cout << "c2.3" << std::endl;
//   
   for( int y = 0; y < diny; y++)
  {
    for( int x = 0; x < dinx; x++)
    { 
      if(dsp.ptr<float>(y)[x]>= 1.0)
      {
	img.clean_left.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
       }
      if(start == -1 && dsp.ptr<float>(y)[x] == 0)
      {
	start = x;
	start_value = dsp.ptr<float>(y)[x-1];
      }
      if(start != -1 && dsp.ptr<float>(y)[x] >= 1.0 && end == -1)
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
		img.clean_left.ptr<float>(y)[i] = img.clean_left.ptr<float>(y)[i-1] + step_size;
	      }
	  }
	  else {
	      for (int i = start; i < end; i++)
		{
		  img.clean_left.ptr<float>(y)[i] = end_value;
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
	    img.clean_left.ptr<float>(y)[i] = start_value;
	  }
	end = -1;
	start = -1;
      }
    }
    
   end = -1;
   start = -1;
  }
  
     std::cout << "c2.4 "<<dinx << " " << diny << std::endl;
   //gradient along y

  for( int x = 0; x < dinx; x++) 
    {
//       std::cout << "c2.4 "<< x << std::endl;
      for( int y = 0; y  < diny; y++)
	{  
//       std::cout << "c2.4 "<< x << " " << y << std::endl;
	  if(dsp.ptr<float>(y)[x]>= 1.0)
	  {
//       std::cout << "c2.4 a" << std::endl;
	    img.clean_down.ptr<float>(y)[x] = dsp.ptr<float>(y)[x];
//       std::cout << "c2.4 b" << std::endl;
	  }
	  if(start == -1 && dsp.ptr<float>(y)[x] == 0)
	  {
//       std::cout << "c2.4 c" << std::endl;
	    start = y;
//       std::cout << "c2.4 c " << y-1 << std::endl;
	    start_value = dsp.ptr<float>(y/*-1*/)[x];
//       std::cout << "c2.4 d" << std::endl;
	  }
	  if(start != -1 && dsp.ptr<float>(y)[x] >= 1.0 && end == -1)
	  {
//       std::cout << "c2.4 e" << std::endl;
	    end = y;
	    end_value = dsp.ptr<float>(y)[x];
//       std::cout << "c2.4 f" << std::endl;
	  }
	  
	  //filling
	  
	    if(start !=-1 && end != -1)
	    {
	      step_size = ((float)(end_value - start_value))/((float)(end-start));
		if(start != 0){
		  for (int i = start; i < end; i++)
		    {
		      img.clean_down.ptr<float>(i)[x] = img.clean_down.ptr<float>(i-1)[x] + step_size;
		    }
		}
		else {
		    for (int i = start; i < end; i++)
		      {
			img.clean_down.ptr<float>(i)[x] = end_value;
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
		  img.clean_down.ptr<float>(i)[x] = start_value;
		}
	      end = -1;
	      start = -1;
	    }
	    }

      end = -1;
      start = -1;
	
      }
   
  
       std::cout << "c2.5" << std::endl;

  img.clean = (img.clean_left+ img.clean_down)/2;
  
       std::cout << "c2.6" << std::endl;

  
//   dsp_clean = dsp; 
  dsp_clean = img.clean; 
  
       std::cout << "c2.7" << std::endl;


//      std::cout<<"\n the dsp \n"<< dsp <<std::endl;
//      std::cout<<"\n img.clean_down \n"<< img.clean_down<<std::endl;
//      std::cout<<"\n img.clean_left \n"<< img.clean_left<<std::endl;
//      std::cout<<"\n img.clean  \n"<< img.clean <<std::endl;
  

  
}