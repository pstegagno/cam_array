
#include <cv.h>
#include <highgui.h>


#ifndef IMAGEANALYZER_STRUCTS_HPP_
#define IMAGEANALYZER_STRUCTS_HPP_



namespace cv {
class Mat;}




using namespace cv;


namespace CameraArrayProject{

class Ndvi
{
	public:
		Mat ndvi;
		Mat ndvi_classified;
		Mat clustering;
		
		Ndvi() {
			}
		
		Ndvi(int cols, int rows):
			ndvi(cols, rows, CV_8UC3 ),
			ndvi_classified(cols, rows, CV_8UC3 ), 
			clustering(cols, rows, CV_8UC3 )
			{
			}
			
};



class Gradients{
public:
		Mat r_gradientX;
		Mat r_gradientXblured;
		Mat l_gradientX;
		Mat l_gradientXblured;
		
		
		Gradients(){
		};
		
		Gradients(int rows, int cols):
			r_gradientX(rows, cols, CV_32FC1 ),
			r_gradientXblured(rows, cols, CV_32FC1 ),
			l_gradientX(rows, cols, CV_32FC1 ),
			l_gradientXblured(rows, cols, CV_32FC1 )
			{
			};
};




class SlideImgs
{
	public:
		
		Size size;
		int type;
		int block_size;
		
		Mat disparity;
		Mat mindiff;
		Mat diff;
		Mat gdiff;
		Mat int_diff;
		Mat int_gdiff;
		Mat CSAD;
		Mat CGRAD;
		
		SlideImgs(){
		}
		
		
		SlideImgs(int rows, int cols, int bs):
			size(cols, rows),
			disparity(rows, cols, CV_32FC1 ),
			mindiff(rows, cols, CV_32FC1 ),
			diff(rows, cols, CV_32FC1 ),
			gdiff(rows, cols, CV_32FC1 ),
			int_diff(rows+1, cols+1, CV_32FC1 ),
			int_gdiff(rows+1, cols+1, CV_32FC1 ),
			CSAD(rows, cols, CV_32FC1),
			CGRAD(rows, cols, CV_32FC1)
// 			d(rows, cols, CV_32FC1)
			{
				block_size = bs;
				type = CV_32FC1;
				mindiff  =+ 1000000.0;
			}
};



















  // Disparity Map 
  
class Disparity
{
	public:
		
		int type;
		
// 		Mat dsp;
		Mat pixel_dsp;
		Mat calibrated;
		Mat clean_left;
		Mat clean_down;
		Mat clean;
		
		Mat ndvi_sum;
		cv::Vec3b dark_green;
		cv::Vec3b green;
		cv::Vec3b brown;
		cv::Vec3b blue;
		cv::Vec3b black;
		
		
		Disparity(){
		}
		
		
		
		
	Disparity(int rows, int cols, int r, int c):
		pixel_dsp(rows, cols, CV_32FC1),
		calibrated(rows, cols, CV_32FC1),
		clean_left(rows, cols, CV_32FC1),
		clean_down(rows, cols, CV_32FC1),
		clean(rows, cols, CV_32FC1),
	 
		ndvi_sum(rows, cols, CV_32FC1),
		dark_green(0,100,0),
		green(0,250,0),
		brown(20,70,139),
		blue(100,0,0),
		black(0,0,0)
		{
		 
			type = CV_32FC1;
			
			clean_left = 0.0;
			clean_down = 0.0;
		}
		
		
};



class Distance
{
public:
  Mat initial;
  Mat spectral;
  Mat spectral_norm;
  Mat spatial;
  Mat u;
  Mat actual;
  Mat prob;
  Mat spatial_norm;
  Mat spectral_spatial;
  Mat sub;
  Mat set;
  Mat set_int;
	
	
	Distance()
  {
  }


   
  Distance(int rows, int cols):
	initial(rows, cols, CV_32FC1 ),
	spectral(rows, cols, CV_32FC1 ),
	spectral_norm(rows, cols, CV_32FC1 ),
	spatial(rows, cols, CV_32FC1 ),
	u(rows, cols, CV_32FC1 ),
	actual(rows, cols, CV_32FC1 ),
	prob(rows, cols, CV_32FC1 ) ,
	spatial_norm(rows, cols, CV_32FC1 ),
	spectral_spatial(rows, cols, CV_32FC1 ),
	set(rows, cols, CV_32FC1 )
	
  {
    set = 1.0/5.0;
    integral(set,set_int);
  }

    
  
};



   class M
  {
  public:
    
    float start, actual, num, den;
    float start_vect[3], actual_vect[3], num_vect[3];
		float a, b, c;
 
    
    M(){
      num_vect[0] = 0;
      num_vect[1] = 0;
      num_vect[2] = 0;
      den = 0.0;
      num = 0.0;
			a = 0.0;
			b = 0.0;
			c = 0.0;
    }
      
    void setStart(float st0, float st1, float st2){
      start_vect[0]=st0;
      start_vect[1]=st1;
      start_vect[2]=st2;
    }
    
    void setActual(float ac0, float ac1, float ac2){
      actual_vect[0]=ac0;
      actual_vect[1]=ac1;
      actual_vect[2]=ac2;
    }
    
    // takes as inputs the vertex of the parabola (x_v, y_v) and one point (x_p, y_p)
		void computeParabolaParameters(float x_v, float y_v, float x_p, float y_p){
			a=(y_p-y_v)/((x_p-x_v)*(x_p-x_v));
			c=y_v + a*x_v*x_v;
			b=-2*a*x_v;
		}
		
		float evaluateParabola(float value){
			return a*value*value+b*value+c;
		}


  };

  class Clustering
  {
  public:
    
    float spatial;
    float spectral;
    float specSpat;
    float initial;
    float spatial_norm;
    float u;
    
    float sumSpatial,  sumSpectral,  denomSpatialSpectral;
    

   
  };
  

}; // end namespace CameraArrayProject  
  
  
#endif
  