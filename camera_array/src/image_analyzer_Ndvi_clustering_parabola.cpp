#include <cv.h>
#include <highgui.h>
#include<math.h>
#include<new>
#include <iterator>
#include <vector>
#include <algorithm>
#include <sys/time.h>

#include "image_analyzer.hpp"




void  ImageAnalyzer::eq_parabola(float value, float x_v, float y_v, float x_p, float y_p, float &parabola )
{
  float a, b, c;

  a=(y_p-y_v)/((x_p-x_v)*(x_p-x_v));
  c=y_v + a*x_v*x_v;
  b=-2*a*x_v;

  parabola = a*value*value+b*value+c;
	  
	  
}



void ImageAnalyzer::Ndvi_clustering_parabola(int length, float beta, Mat mat_sum, Ndvi &ndvi)          
{
  
  double bias = -0.0;
  
  M m0, m1, m2, m3;
  
  double b1 = 0.73 +bias;
  double b2 = 0.62 +bias;//0.60;
  double b3 = 0.50 +bias;//0.45;
  
  
  m0.actual = 0.85 +bias;
  m1.actual = (b1+b2)/2.0 +bias;//0.665;
  m2.actual = (b2+b3)/2.0 +bias;//0.52;
  m3.actual = 0.40 +bias;//0.40;
  
Distance distance0(mat_sum.rows, mat_sum.cols), distance1(mat_sum.rows, mat_sum.cols), distance2(mat_sum.rows, mat_sum.cols), distance3(mat_sum.rows, mat_sum.cols);

//Ndvi ndvi(mat_sum.rows, mat_sum.cols);

integral(distance0.set,distance0.prob, CV_32F);
integral(distance1.set,distance1.prob, CV_32F);
integral(distance2.set,distance2.prob, CV_32F);
integral(distance3.set,distance3.prob, CV_32F);

Clustering clustering0, clustering1, clustering2, clustering3, clust;


//float sum_square;





//Mat ndvi.ndvi(mat_sum.rows,mat_sum.cols,CV_8UC3);
    
    for(int i = 0; i <mat_sum.rows ; i++)
    {
      for(int j = 0; j <mat_sum.cols ; j++)
      {
	
	if(  (mat_sum.ptr<float>(i)[j])  >  b1) // dense vegetation dark green
	{
	 ndvi.ndvi_classified.at<cv::Vec3b>(i,j) = cv::Vec3b(0,100,0);
	} 
	if(0.2 < (mat_sum.ptr<float>(i)[j]) && (mat_sum.ptr<float>(i)[j]) <= b1)// shurb and grassland
	{
	   ndvi.ndvi_classified.at<cv::Vec3b>(i,j)= cv::Vec3b(0,250,0);
	}
	if( mat_sum.ptr<float>(i)[j] <= b2  && mat_sum.ptr<float>(i)[j] > b3)// land
	{
	   ndvi.ndvi_classified.at<cv::Vec3b>(i,j)= cv::Vec3b(20,70,139);
	}
	if(mat_sum.ptr<float>(i)[j]  <= b3)// water
	{
	   ndvi.ndvi_classified.at<cv::Vec3b>(i,j)= cv::Vec3b(100,0,0);
	}
      }
    }
   // output6=image;

struct timeval tv, tv_tot;


long int t1=0, t2=0,t3=0,t4=0,t5=0;//,t6=0,t7=0;
//float straight_line1, straight_line2, straight_line3, straight_line4, straight_line5, straight_line6;

int imgRows = mat_sum.rows;
int imgCols = mat_sum.cols;
int lenght1=length+1;
int subRpersubC=0, j1, i1;
int upper_row,lower_row,left_col,rigth_col;


    // TODO eliminate this -> only for debug
    gettimeofday(&tv_tot,NULL);
    unsigned long cord_time_in_micros = 1000000 * tv_tot.tv_sec + tv_tot.tv_usec;
//coordinates( imgRows, imgCols, length , upper_row, lower_row, left_col, rigth_col);
   //coordinates(480/2 ,  752/2, 3/2 , upper_row, lower_row, left_col, rigth_col);
    gettimeofday(&tv,NULL);
    unsigned long new_cord_time_in_micros = (1000000 * tv.tv_sec + tv.tv_usec) - cord_time_in_micros;
    // TODO eliminate this -> only for debug
    
    std::cout<< " new_cord_time_in_micros "<< new_cord_time_in_micros <<std::endl;
	  



for (int num_iter = 0; num_iter  < 3; num_iter ++)
{

  // TODO eliminate this -> only for debug
  gettimeofday(&tv_tot,NULL);
  unsigned long time_in_micros = 1000000 * tv_tot.tv_sec + tv_tot.tv_usec;
  // TODO eliminate this -> only for debug
  
     
    for (int i = 0; i < mat_sum.rows; i++)
    {
	
      for(int j = 0; j < mat_sum.cols; j++)
      {
// 	  // FIXME
// 	  gettimeofday(&tv,NULL);	
// 	  unsigned long time_loc = 1000000 * tv.tv_sec + tv.tv_usec;
	  
	  i1= i+1;
	  j1= j+1;
	  
	  float value =mat_sum.ptr<float>(i)[j];
       
	  if (value<m3.actual){
	    clustering0.spectral = 0.01;
	    clustering1.spectral = 0.01;
	    clustering2.spectral = 0.01;
	    clustering3.spectral = 0.97;
	  }
	  else if (value>m0.actual){
	    clustering0.spectral = 0.97;
	    clustering1.spectral = 0.01;
	    clustering2.spectral = 0.01;
	    clustering3.spectral = 0.01;
	  }
	  else
	  {
	    eq_parabola(value, m0.actual, 1, b1, 0.5, clustering0.spectral );
	    clustering0.spectral = max(clustering0.spectral, (float)0.0000000001);
	    
	    eq_parabola(value, m1.actual, 1, b1, 0.5, clustering1.spectral );
	    clustering1.spectral = max(clustering1.spectral, (float)0.0000000001);
	    
	    eq_parabola(value, m2.actual, 1, b3, 0.5, clustering2.spectral );
	    clustering2.spectral = max(clustering2.spectral, (float)0.0000000001);
	    
	    eq_parabola(value, m3.actual,1 , b3, 0.5, clustering3.spectral );
	    clustering3.spectral = max(clustering3.spectral, (float)0.0000000001);

	  }
	  

	  
	  clust.sumSpectral =  clustering0.spectral +  clustering1.spectral +  clustering2.spectral +  clustering3.spectral;
	  distance0.spectral_norm.ptr<float>(i)[j] =  clustering0.spectral/clust.sumSpectral;
	  distance1.spectral_norm.ptr<float>(i)[j] =  clustering1.spectral/clust.sumSpectral;
	  distance2.spectral_norm.ptr<float>(i)[j] =  clustering2.spectral/clust.sumSpectral;
	  distance3.spectral_norm.ptr<float>(i)[j] =  clustering3.spectral/clust.sumSpectral;
	  
	  
// 	  // FIXME
// 	  gettimeofday(&tv,NULL);
// 	  t1 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
// 	  time_loc = (1000000 * tv.tv_sec + tv.tv_usec);
	  
	  upper_row = max(0, i1-lenght1);
	  lower_row = min(i1+length, imgRows);
	  left_col  = max(0,j1-lenght1);
	  rigth_col = min(j1+length, imgCols);
	  
	  
// 	  // FIXME
// 	  gettimeofday(&tv,NULL);
// 	  t2 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
// 	  time_loc = (1000000 * tv.tv_sec + tv.tv_usec);
	  
	  
	  subRpersubC = (lower_row - upper_row)*(rigth_col - left_col);
	  
	  clustering0.u = subRpersubC - distance0.prob.ptr<float>(upper_row)[left_col] - distance0.prob.ptr<float>(lower_row)[rigth_col] + distance0.prob.ptr<float>(lower_row)[left_col] + distance0.prob.ptr<float>(upper_row)[rigth_col]; 
	  clustering1.u = subRpersubC - distance1.prob.ptr<float>(upper_row)[left_col] - distance1.prob.ptr<float>(lower_row)[rigth_col] + distance1.prob.ptr<float>(lower_row)[left_col] + distance1.prob.ptr<float>(upper_row)[rigth_col]; 
	  clustering2.u = subRpersubC - distance2.prob.ptr<float>(upper_row)[left_col] - distance2.prob.ptr<float>(lower_row)[rigth_col] + distance2.prob.ptr<float>(lower_row)[left_col] + distance2.prob.ptr<float>(upper_row)[rigth_col]; 
          clustering3.u = subRpersubC - distance3.prob.ptr<float>(upper_row)[left_col] - distance3.prob.ptr<float>(lower_row)[rigth_col] + distance3.prob.ptr<float>(lower_row)[left_col] + distance3.prob.ptr<float>(upper_row)[rigth_col]; 

	  
	  clustering0.spatial = exp(-beta*clustering0.u);
	  clustering1.spatial = exp(-beta*clustering1.u);
	  clustering2.spatial = exp(-beta*clustering2.u);
	  clustering3.spatial = exp(-beta*clustering3.u);
	  
	  clust.sumSpatial = clustering0.spatial + clustering1.spatial +  clustering2.spatial + clustering3.spatial;// spatial0+spatial1+ spatial2+spatial3;
	  
// 	  // FIXME
// 	  gettimeofday(&tv,NULL);
// 	  t3 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
// 	  time_loc = (1000000 * tv.tv_sec + tv.tv_usec);

	  // Normalization
	  distance0.spatial_norm.ptr<float>(i)[j] = clustering0.spatial /clust.sumSpatial;
	  distance1.spatial_norm.ptr<float>(i)[j] = clustering1.spatial /clust.sumSpatial;
	  distance2.spatial_norm.ptr<float>(i)[j] = clustering2.spatial /clust.sumSpatial;  
	  distance3.spatial_norm.ptr<float>(i)[j] = clustering3.spatial /clust.sumSpatial; 

	  
	  clustering0.specSpat = distance0.spectral_norm.ptr<float>(i)[j]*distance0.spatial_norm.ptr<float>(i)[j];
	  clustering1.specSpat = distance1.spectral_norm.ptr<float>(i)[j]*distance1.spatial_norm.ptr<float>(i)[j];
	  clustering2.specSpat = distance2.spectral_norm.ptr<float>(i)[j]*distance2.spatial_norm.ptr<float>(i)[j];
	  clustering3.specSpat = distance3.spectral_norm.ptr<float>(i)[j]*distance3.spatial_norm.ptr<float>(i)[j];
	  
	  clust.denomSpatialSpectral = (clustering0.specSpat + clustering1.specSpat + clustering2.specSpat + clustering3.specSpat);
	  

	  
	  

/*	  // FIXME
	  gettimeofday(&tv,NULL);
	  t4 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
	  time_loc = (1000000 * tv.tv_sec + tv.tv_usec);*/	  
	  
	  
	  
	  
	  distance0.spectral_spatial.ptr<float>(i)[j] = clustering0.specSpat / clust.denomSpatialSpectral;
	  distance1.spectral_spatial.ptr<float>(i)[j] = clustering1.specSpat / clust.denomSpatialSpectral;
	  distance2.spectral_spatial.ptr<float>(i)[j] = clustering2.specSpat / clust.denomSpatialSpectral;
	  distance3.spectral_spatial.ptr<float>(i)[j] = clustering3.specSpat / clust.denomSpatialSpectral;
    
	  m0.num =  m0.num + mat_sum.ptr<float>(i)[j]*distance0.spectral_spatial.ptr<float>(i)[j];
	  m1.num =  m1.num + mat_sum.ptr<float>(i)[j]*distance1.spectral_spatial.ptr<float>(i)[j];
	  m2.num =  m2.num + mat_sum.ptr<float>(i)[j]*distance2.spectral_spatial.ptr<float>(i)[j];
	  m3.num =  m3.num + mat_sum.ptr<float>(i)[j]*distance3.spectral_spatial.ptr<float>(i)[j];
	  
	  m0.den = m0.den + distance0.spectral_spatial.ptr<float>(i)[j];
	  m1.den = m1.den + distance1.spectral_spatial.ptr<float>(i)[j];
	  m2.den = m2.den + distance2.spectral_spatial.ptr<float>(i)[j];
	  m3.den = m3.den + distance3.spectral_spatial.ptr<float>(i)[j];
	  
	  
// 	  // FIXME
// 	  gettimeofday(&tv,NULL);
// 	  t5 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
// 	  time_loc = (1000000 * tv.tv_sec + tv.tv_usec);
      }
    }

//     m0.actual=m0.num/m0.den;
//     m1.actual=m1.num/m1.den;
//     m2.actual=m2.num/m2.den;
//     m3.actual=m3.num/m3.den;
// 
//     b1 = (m0.actual+m1.actual)/2;
//     b2 = (m1.actual+m2.actual)/2;
//     b3 = (m2.actual+m3.actual)/2;

    
    integral(distance0.spectral_spatial,distance0.prob, CV_32F);
    integral(distance1.spectral_spatial,distance1.prob, CV_32F);
    integral(distance2.spectral_spatial,distance2.prob, CV_32F);
    integral(distance3.spectral_spatial,distance3.prob, CV_32F);

    // TODO eliminate this -> only for debug
    gettimeofday(&tv_tot,NULL);
    unsigned long elapsed = (1000000 * tv_tot.tv_sec + tv_tot.tv_usec) - time_in_micros;
    std::cout << "time_in_micros " << elapsed << std::endl;
    // TODO eliminate this -> only for debug
}


std::cout << t1 << " " << t2 << " " << t3 << " " << t4 << " "  << t5 << std::endl;

//Mat image_clean(mat_sum.rows,mat_sum.cols,CV_8UC3);

    for(int i = 0; i <mat_sum.rows ; i++)
    {
      for(int j = 0; j <mat_sum.cols ; j++)
      {
  
	  float a=distance0.spectral_spatial.ptr<float>(i)[j];
	  float b=distance1.spectral_spatial.ptr<float>(i)[j];
	  float c=distance2.spectral_spatial.ptr<float>(i)[j];
	  float d=distance3.spectral_spatial.ptr<float>(i)[j];


	  if (a>b && a>c && a>d)
	  {
	   ndvi.clustering.at<cv::Vec3b>(i,j) = cv::Vec3b(0,100,0);
	  } 
	  if (b>a && b>c && b>d)
	  {
	   ndvi.clustering.at<cv::Vec3b>(i,j)= cv::Vec3b(0,250,0);
	  }
	  if (c>a && c>b && c>d)
	  {
	   ndvi.clustering.at<cv::Vec3b>(i,j)= cv::Vec3b(20,70,139);
	  }
	  if (d>a && d>b && d>c)
	  {
	  ndvi.clustering.at<cv::Vec3b>(i,j)= cv::Vec3b(100,0,0);
	  }
      }
    }
    

  
  
}


 