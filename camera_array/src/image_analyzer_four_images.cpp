#include <cv.h>
#include <highgui.h>
#include<math.h>
#include<new>
#include <iterator>
#include <vector>
#include <algorithm>
#include <sys/time.h>

#include "image_analyzer.hpp"

float ImageAnalyzer::spectral_distance(float pixel_green, float pixel_red, float pixel_nir, float cluster_center0, float cluster_center1, float cluster_center2){
  
	return sqrt((pixel_green-cluster_center0)*(pixel_green-cluster_center0)+(pixel_red-cluster_center1)*(pixel_red-cluster_center1)+(pixel_nir-cluster_center2)*(pixel_nir-cluster_center2));
	}
	

float ImageAnalyzer::spectral_probability(float dis0, float dis1, float dis2, float dis3){
  
         return ((1.0/dis0)/(1.0/dis0 + 1.0/dis1 + 1.0/dis2 + 1.0/dis3));
         }

         
void ImageAnalyzer::four_images(int length, float beta, Mat NiarInfared, Mat Red, Mat Green, Mat Blue,  Mat &output4)    
{
  M m0, m1, m2, m3;
  
  Clustering clustering0, clustering1, clustering2, clustering3, clust;

  
  double coef_areaB = 1.56; 
  double coef_areaG = 1.17; 
  double coef_areaR = 1.43; 
  double coef_areaNIR = 1; 

  
  Mat redf, nirf, greenf, bluef;
  Red.convertTo(redf, CV_32FC1);
  NiarInfared.convertTo(nirf, CV_32FC1);
  Green.convertTo(greenf, CV_32FC1);
  Blue.convertTo(bluef, CV_32FC1);
  
  Mat nirf_sum, redf_sum, greenf_sum, bluef_sum ,num_sum, den_sum;
 
  nirf_sum = nirf*coef_areaNIR;
  redf_sum = redf*coef_areaR;
  greenf_sum = greenf*coef_areaG;
  bluef_sum = bluef*coef_areaB;

  Mat Matrix1, Matrix2, Matrix3, Matrix1_N, Matrix2_N, Matrix3_N;
  
  divide(greenf_sum,bluef_sum ,Matrix1_N);
  divide(redf_sum,bluef_sum,Matrix2_N);
  divide(nirf_sum,bluef_sum ,Matrix3_N);
  cv::blur(Matrix1_N, Matrix1, Size(5,5),Point(-1,-1));
  cv::blur(Matrix2_N, Matrix2, Size(5,5),Point(-1,-1));
  cv::blur(Matrix3_N, Matrix3, Size(5,5),Point(-1,-1));
  
  
  
  Distance distance0(Matrix1.rows, Matrix1.cols), distance1(Matrix1.rows, Matrix1.cols), distance2(Matrix1.rows, Matrix1.cols), distance3(Matrix1.rows, Matrix1.cols);
  
  m0.setStart(13.0/13.0, 22.0/13.0, 48.0/13.0);
  m1.setStart(13.0/13.0, 22.0/13.0, 40.0/13.0);
  m2.setStart(23.0/19.0, 26.0/19.0, 33.0/19.0);
  m3.setStart(1.2,1,0.5);
  

  int max0jml, max0iml, minjmlcm1, minimlrm1;
  int subRpersubC=0, j3=0;
//   long int t1=0, t2=0,t3=0,t4=0,t5=0,t6=0,t7=0;

  
  
  for (int num_iter = 0; num_iter  < 5; num_iter ++)
  {
    
    for (int i = 0; i < Matrix1.rows; i++)//for (int i = 0; i < 1; i++)
    {
      for(int j = 0; j < Matrix1.cols; j++)//for(int j = 0; j < 1; j++)
      {
	//spectral distance

	//std::cout<< "1" << std::endl;
	 j3 = j*3;

         if (num_iter==0) 
	 {
	  clustering0.initial =  spectral_distance(Matrix1.ptr<float>(i)[j3], Matrix2.ptr<float>(i)[j3], Matrix3.ptr<float>(i)[j3], m0.start_vect[0], m0.start_vect[1], m0.start_vect[2]);
	  clustering1.initial =  spectral_distance(Matrix1.ptr<float>(i)[j3], Matrix2.ptr<float>(i)[j3], Matrix3.ptr<float>(i)[j3], m1.start_vect[0], m1.start_vect[1], m1.start_vect[2]);
	  clustering2.initial =  spectral_distance(Matrix1.ptr<float>(i)[j3], Matrix2.ptr<float>(i)[j3], Matrix3.ptr<float>(i)[j3], m2.start_vect[0], m2.start_vect[1], m2.start_vect[2]);
	  clustering3.initial =  spectral_distance(Matrix1.ptr<float>(i)[j3], Matrix2.ptr<float>(i)[j3], Matrix3.ptr<float>(i)[j3], m3.start_vect[0], m3.start_vect[1], m3.start_vect[2]);
	  
	  if (clustering0.initial==0.0)
	  {
	    clustering0.initial=0.0000000001;
	  }
	  if (clustering1.initial==0.0)
	  {
	    clustering1.initial=0.0000000001;
	  }  
	  if (clustering2.initial==0.0)
	  {
	    clustering2.initial=0.0000000001;
	  }
	
	  if (clustering3.initial==0.0)
	  {
	    clustering3.initial=0.0000000001;
	  }
	  
	 
	  distance0.spectral.ptr<float>(i)[j] = spectral_probability( clustering0.initial, clustering1.initial, clustering2.initial, clustering3.initial) ;
	  distance1.spectral.ptr<float>(i)[j] = spectral_probability( clustering1.initial, clustering0.initial, clustering2.initial, clustering3.initial) ;
          distance2.spectral.ptr<float>(i)[j] = spectral_probability( clustering2.initial, clustering1.initial, clustering0.initial, clustering3.initial) ;
	  distance3.spectral.ptr<float>(i)[j] = spectral_probability( clustering3.initial, clustering1.initial, clustering2.initial, clustering0.initial);
	
	 }
	 
	// estimation of u
	
	  
	  max0jml = max(0,j-length);
	  max0iml = max(0,i-length);
	  minjmlcm1 = min(j+length,Matrix1.cols-1);
	  minimlrm1 = min(i+length,Matrix1.rows-1);
	  
	  subRpersubC = (minimlrm1+1-max0iml)*(minjmlcm1+1-max0jml);
	  
	  int rowc_end = (minimlrm1+1) ;
	  int colc_end = (minjmlcm1+1); 
	  
	  
	  clustering0.u = (subRpersubC);
	  clustering1.u = (subRpersubC);
	  clustering2.u = (subRpersubC);
	  clustering3.u = (subRpersubC);
	  for (int rowc=max0iml; rowc< rowc_end ; rowc++)
	  {
	    for (int colc=max0jml; colc< colc_end; colc++)
	    {
	      clustering0.u -= distance0.prob.ptr<float>(rowc)[colc];
	      clustering1.u -= distance1.prob.ptr<float>(rowc)[colc];
	      clustering2.u -= distance2.prob.ptr<float>(rowc)[colc];
	      clustering3.u -= distance3.prob.ptr<float>(rowc)[colc];
	    }
	  }

	  	  
	  clustering0.spatial = exp(-beta*clustering0.u);
	  clustering1.spatial = exp(-beta*clustering1.u);
	  clustering2.spatial = exp(-beta*clustering2.u);
	  clustering3.spatial  = exp(-beta*clustering3.u);
	  clust.sumSpatial = clustering0.spatial + clustering1.spatial +  clustering2.spatial + clustering3.spatial;

	
	  
	  // Normalization
	  clustering0.spatial_norm = clustering0.spatial/clust.sumSpatial;
	  clustering1.spatial_norm = clustering1.spatial/clust.sumSpatial;
	  clustering2.spatial_norm = clustering2.spatial/clust.sumSpatial;
	  clustering3.spatial_norm = clustering3.spatial/clust.sumSpatial; 
	 
	  
	  // the joint spectral-spatial membership 
	  
	  clustering0.specSpat =  distance0.spectral.ptr<float>(i)[j]*clustering0.spatial_norm;
	  clustering1.specSpat =  distance1.spectral.ptr<float>(i)[j]*clustering1.spatial_norm;
	  clustering2.specSpat =  distance2.spectral.ptr<float>(i)[j]*clustering2.spatial_norm;
	  clustering3.specSpat =  distance3.spectral.ptr<float>(i)[j]*clustering3.spatial_norm;
	  
	  clust.denomSpatialSpectral = (clustering0.specSpat + clustering1.specSpat + clustering2.specSpat + clustering3.specSpat);
	  
	  distance0.spectral_spatial.ptr<float>(i)[j] = clustering0.specSpat / clust.denomSpatialSpectral;
	  distance1.spectral_spatial.ptr<float>(i)[j] = clustering1.specSpat / clust.denomSpatialSpectral;
	  distance2.spectral_spatial.ptr<float>(i)[j] = clustering2.specSpat / clust.denomSpatialSpectral;
	  distance3.spectral_spatial.ptr<float>(i)[j] = clustering3.specSpat / clust.denomSpatialSpectral;

	  m0.num_vect[0] =  m0.num_vect[0] + Matrix1.ptr<float>(i)[j3]*(clustering0.specSpat /  clust.denomSpatialSpectral);
	  m0.num_vect[1] =  m0.num_vect[1] + Matrix2.ptr<float>(i)[j3]*(clustering0.specSpat /  clust.denomSpatialSpectral);
	  m0.num_vect[2] =  m0.num_vect[2] + Matrix3.ptr<float>(i)[j3]*(clustering0.specSpat /  clust.denomSpatialSpectral);
	  
	  m1.num_vect[0] =  m1.num_vect[0] + Matrix1.ptr<float>(i)[j3]*(clustering1.specSpat /  clust.denomSpatialSpectral);
	  m1.num_vect[1] =  m1.num_vect[1] + Matrix2.ptr<float>(i)[j3]*(clustering1.specSpat /  clust.denomSpatialSpectral);
	  m1.num_vect[2] =  m1.num_vect[2] + Matrix3.ptr<float>(i)[j3]*(clustering1.specSpat / clust.denomSpatialSpectral);
	 
	  m2.num_vect[0] =  m2.num_vect[0] + Matrix1.ptr<float>(i)[j3]*(clustering2.specSpat /  clust.denomSpatialSpectral);
	  m2.num_vect[1] =  m2.num_vect[1] + Matrix2.ptr<float>(i)[j3]*(clustering2.specSpat  / clust.denomSpatialSpectral);
	  m2.num_vect[2] =  m2.num_vect[2] + Matrix3.ptr<float>(i)[j3]*(clustering2.specSpat  / clust.denomSpatialSpectral);
	  
	  m3.num_vect[0] =  m3.num_vect[0] + Matrix1.ptr<float>(i)[j3]*(clustering3.specSpat / clust.denomSpatialSpectral);
	  m3.num_vect[1] =  m3.num_vect[1] + Matrix2.ptr<float>(i)[j3]*(clustering3.specSpat / clust.denomSpatialSpectral);
	  m3.num_vect[2] =  m3.num_vect[2] + Matrix3.ptr<float>(i)[j3]*(clustering3.specSpat / clust.denomSpatialSpectral);

	  m0.den = m0.den + (clustering0.specSpat / clust.denomSpatialSpectral);
	  m1.den = m1.den + (clustering1.specSpat / clust.denomSpatialSpectral);
	  m2.den = m2.den + (clustering2.specSpat / clust.denomSpatialSpectral);
	  m3.den = m3.den + (clustering3.specSpat / clust.denomSpatialSpectral);
	 
      }
    }

        m0.start_vect[0]=m0.num_vect[0]/m0.den;
	m0.start_vect[1]=m0.num_vect[1]/m0.den; 
	m0.start_vect[2]=m0.num_vect[3]/m0.den;
	
	m1.start_vect[0]=m1.num_vect[0]/m1.den;
	m1.start_vect[1]=m1.num_vect[1]/m1.den;
	m1.start_vect[2]=m1.num_vect[2]/m1.den;
	
	m2.start_vect[0]=m2.num_vect[0]/m2.den;
	m2.start_vect[1]=m2.num_vect[1]/m2.den;
	m2.start_vect[2]=m2.num_vect[2]/m2.den;
	
	m3.start_vect[0]=m3.num_vect[0]/m3.den;
	m3.start_vect[1]=m3.num_vect[1]/m3.den;
	m3.start_vect[2]=m3.num_vect[2]/m3.den;
         
    distance0.spectral_spatial.copyTo(distance0.prob);
    distance1.spectral_spatial.copyTo(distance1.prob);
    distance2.spectral_spatial.copyTo(distance2.prob);
    distance3.spectral_spatial.copyTo(distance3.prob);
}



Mat image_clean( Matrix1.rows, Matrix1.cols,CV_8UC3);


for (int i = 0; i < Matrix1.rows; i++)
    {
      for(int j = 0; j < Matrix1.cols; j++)
      {

	  float a=distance0.spectral_spatial.ptr<float>(i)[j];
	  float b=distance1.spectral_spatial.ptr<float>(i)[j];
	  float c=distance2.spectral_spatial.ptr<float>(i)[j];
	  float d=distance3.spectral_spatial.ptr<float>(i)[j];


	  if (a>b && a>c && a>d)
	  {
	  image_clean.at<cv::Vec3b>(i,j) = cv::Vec3b(0,100,0);
	  } 
	  if (b>a && b>c && b>d)
	    {
	  image_clean.at<cv::Vec3b>(i,j)= cv::Vec3b(0,250,0);
	  }
	  if (c>a && c>b && c>d)
	    {
	  image_clean.at<cv::Vec3b>(i,j)= cv::Vec3b(20,70,139);
	  
	  }
	  if (d>a && d>b && d>c)
	    {
	  image_clean.at<cv::Vec3b>(i,j)= cv::Vec3b(100,0,0);
	    }
	}
     }
     
     output4 =  image_clean;
}
  


  
  