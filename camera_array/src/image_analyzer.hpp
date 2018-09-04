
#include <cv.h>
#include <highgui.h>


#ifndef IMAGEANALYZER_HPP_
#define IMAGEANALYZER_HPP_
#include <boost/iterator/iterator_concepts.hpp>

#include <vector>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <string>


#include <fstream>
#include <iostream>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>



#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/legacy/legacy.hpp"









namespace cv {
class Mat;}




using namespace cv;




class Ndvi
{
  public:
    Mat ndvi;
    Mat ndvi_classified;
    Mat clustering;
    
    Ndvi(int rows, int cols):
      ndvi(rows, cols, CV_8UC3 ),
      ndvi_classified(cols, rows, CV_8UC3 ), 
      clustering(cols, rows, CV_8UC3 )
      
      {
      }
      
};






  // Disparity Map 
  
  class Disparity
  {
    
    
  public:
    
    Size size;
    int type;

    
    Mat disparity;
    Mat mindiff;
    Mat h;
    Mat gradientX;
    Mat dsp;
    Mat diff;
    Mat shift, shift_gradient;
    Mat gdiff;
    Mat id;
    Mat pixel_dsp;
    Mat pd;
    Mat CSAD, CGRAD, d;
    Mat calibrated;
    Mat filled;
    Mat clean_left;
    Mat clean_right;
    Mat clean_up;
    Mat clean_down;
    Mat clean;
    Mat depth;

  Disparity(int rows, int cols, int r, int c):
	 size(cols, rows),
	 disparity(rows, cols, CV_32FC1 ),
	 mindiff(rows, cols, CV_32FC1 ),
	 h(r, c, CV_32FC1 ),
	 gradientX(rows, cols, CV_32FC1 ),
	 dsp(rows, cols, CV_32FC1),
	 diff(rows, cols, CV_32FC1 ),
	 shift(rows, cols, CV_32FC1 ),
	 shift_gradient(rows, cols, CV_32FC1 ),
	 gdiff(rows, cols, CV_32FC1 ),
	 id(rows, cols, CV_32FC1),
         pixel_dsp(rows, cols, CV_32FC1),
	 pd(rows, cols, CV_32FC1),
	 CSAD(rows, cols, CV_32FC1),
	 CGRAD(rows, cols, CV_32FC1),
	 d(rows, cols, CV_32FC1),
         calibrated(rows, cols, CV_32FC1),
	 filled(rows, cols, CV_32FC1),
	 clean_left(rows, cols, CV_32FC1),
	 clean_right(rows, cols, CV_32FC1),
	 clean_up(rows, cols, CV_32FC1),
	 clean_down(rows, cols, CV_32FC1),
	 clean(rows, cols, CV_32FC1),
	 depth(rows, cols, CV_32FC1)
	 
	 
	 
	/* gradientY(rows, cols, CV_32FC3 ),
	 gradientX(rows, cols, CV_32FC3 ),
	 dsp(rows, cols, CV_32FC3 ),
	 diff(rows, cols, CV_32FC3 ),
	 shift(rows, cols, CV_32FC3 ),
	 shiftX(rows, cols, CV_32FC3 ),
	 shiftY(rows, cols, CV_32FC3 ),
	 gdiffx(rows, cols, CV_32FC3 )	*/ 
	 {
	   
	   type = CV_32FC1;
	   
	   mindiff  =+ 1000000.0;
	   h = 1.0/(double)(r*r);
	   clean_left = 0.0;
	   clean_right = 0.0;
	   clean_down = 0.0;
	   clean_up = 0.0;
	 }
  };





class ImageAnalyzer{
private:

public:	
  
  
  void compute_NDVI( Mat NiarInfared, Mat Red, Mat &output1);
  
  void Ndvi_clustering_sigmoid( int length, float beta, Mat mat_sum,  Mat &output3);     
  
  void four_images(int length, float beta, Mat NiarInfared, Mat Red, Mat Green, Mat Blue,  Mat &output4);
  
  //void Ndvi_clustering_parabola(int length, float beta, Mat mat_sum,  Mat &output5, Mat &output6);     
  
  void Ndvi_clustering_parabola(int length, float beta, Mat mat_sum, Ndvi &ndvi);    
  
  void Ndvi_clustering_parabola_old(int length, float beta, Mat mat_sum,  Mat &output5);      
  
  void compute_NDVI_online( float expTime_corection ,Mat NiarInfared, Mat Red, Mat &output1); 
  

  
  
  float	spectral_probability(float dis0, float dis1, float dis2, float dis3);

  float spectral_distance(float pixel_green, float pixel_red, float pixel_nir, float cluster_center0, float cluster_center1, float cluster_center2);

  void eq_parabola(float value, float x_v, float y_v, float x_p, float y_p, float &parabola );

  void coordinates(int row,int col ,int  length, Mat &output_matrix1, Mat &output_matrix2, Mat &output_matrix3, Mat &output_matrix4);

 
  
  // Disparity Map Functions
  
 
  void slide_images(Mat imgL, Mat imgR, int win_size, int mins, int maxs, int weight, Mat &output1, Mat &output2);
  
  Mat  translateImg(Mat img, Mat &imgout,int offsetx, int offsety);
  
  void printFloatImg(Mat img, string windowName, int wait, bool toGray);
  
  void stereo(Mat imgR, Mat imgL, int maxs, Mat &dsp,  Mat &clean, Mat &calibrated );
  
  void winner_take_all(Mat d1, Mat m1, Mat d2, Mat m2, int tolerance, Mat &output);

  void disparityMap_new_image(Mat imgL, Mat imgR, Mat dsp, Mat &calibrated);
  
  void fillEdgeImage(Mat edgesIn, Mat &filledEdgesOut);
  
  void MultipleThreshold(Mat *src, Mat *dst, int *threshold, int layers);

  void clean_disparity(Mat dsp, Mat &dsp_clean);
  
  void _3Dreconstruction(Mat clean, std::stringstream &namefile );

  
  
  void depth_map( Mat dsp, Disparity &depth) ;
  
  
  void Detect_Material( Mat imgL, Mat imgR, Mat imgM, int length, float beta, Mat &clustering);   
  
  //void Detect_Material_online(float expTime_corection, Mat imgL, Mat imgR, Mat imgM, int length, float beta, Mat &clustering, Ndvi &ndvi, Disparity &img);
  
  void Detect_Material_online(float expTime_corection, Mat imgL, Mat imgR, Mat imgM,int length, float beta, Ndvi &ndvi, Disparity  &img);
  
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
    set =+ 1.0/4.0;
    //integral(set,prob);
  }

    
  
};



   class M
  {
  public:
    
    float start, actual, num, den;
    float start_vect[3], actual_vect[3], num_vect[3];
 
    
    M(){
      num_vect[1] = 0;
      num_vect[2] = 0;
      num_vect[3] = 0;
      den = 0.0;
      num = 0.0;
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
  

	
    
  
  
  
  
#endif
  