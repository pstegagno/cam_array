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





void ImageAnalyzer::Ndvi_clustering_parabola(int length, float beta, Mat mat_sum,  Mat &output5)          
{
  
  M m0, m1, m2, m3;
//   m0.start = 0.85;
//   m1.start = 0.65;
//   m2.start = 0.52;
//   m3.start = 0.40;
  
  m0.actual = 0.85;
  m1.actual = 0.65;
  m2.actual= 0.52;
  m3.actual = 0.40;
Distance distance0(mat_sum.rows, mat_sum.cols), distance1(mat_sum.rows, mat_sum.cols), distance2(mat_sum.rows, mat_sum.cols), distance3(mat_sum.rows, mat_sum.cols);

//int a1 = 30, a2 = 60; //paramiters sigmoid 
double b1 = 0.73, b2 = 0.60, b3 = 0.45; // paramiters sigmoid

//std::vector<Rect> rois;
  

Clustering clustering0, clustering1, clustering2, clustering3, clust;

int max0jml, max0iml, minjmlcm1, minimlrm1;
// make_vector(5,480,752, rois);

struct timeval tv, tv_tot;

// Mat sub0, sub1, sub2, sub3;

long int t1=0, t2=0,t3=0,t4=0,t5=0,t6=0,t7=0;
float straight_line1, straight_line2, straight_line3, straight_line4, straight_line5, straight_line6;

for (int num_iter = 0; num_iter  < 10; num_iter ++)
{

  // TODO eliminate this -> only for debug
  gettimeofday(&tv_tot,NULL);
  unsigned long time_in_micros = 1000000 * tv_tot.tv_sec + tv_tot.tv_usec;
  // TODO eliminate this -> only for debug
  
  int subRpersubC=0, j3=0;

     
    for (int i = 0; i < mat_sum.rows; i++)
    {
	
      for(int j = 0; j < mat_sum.cols; j++)
      {
	  gettimeofday(&tv,NULL);	
	  unsigned long time_loc = 1000000 * tv.tv_sec + tv.tv_usec;
	  
	  
	  j3 = j*3;
	  float value =mat_sum.ptr<float>(i)[j3];
       
            //clustering0.spectral = 1/(1+exp(-30*(mat_sum.ptr<float>(i)[j3]-b1)));  
	  eq_parabola(value, m0.actual, 1, b1, 0.5, clustering0.spectral );//1/(1+exp(-a1*(mat_sum.ptr<float>(i)[j3]-b1)));  
	  clustering0.spectral = max(clustering0.spectral, (float)0.0000000001);
// 	  if(clustering0.spectral<=0)
// 	  {
// 	    clustering0.spectral=0.0000000001;
// 	  }
	  
	  eq_parabola(value, m1.actual, 1, b1, 0.5, clustering1.spectral );
	  clustering1.spectral = max(clustering1.spectral, (float)0.0000000001);
// 	  if(clustering1.spectral<=0)
// 	  {
// 	    clustering1.spectral=0.0000000001;
// 	  }
	  
	  eq_parabola(value, m2.actual, 1, b3, 0.5, clustering2.spectral );
	  clustering2.spectral = max(clustering2.spectral, (float)0.0000000001);
// 	  if(clustering2.spectral<=0)
// 	  {
// 	    clustering2.spectral=0.0000000001;
// 	  }
	  
	  
          eq_parabola(value, m3.actual,1 , b3, 0.5, clustering3.spectral );
	  clustering3.spectral = max(clustering3.spectral, (float)0.0000000001);
// 	  if(clustering3.spectral<=0)
// 	  {
// 	    clustering3.spectral=0.0000000001;
// 	  }
	   //clustering3.spectral = 1/(1+exp( 30*(mat_sum.ptr<float>(i)[j3]-b2)));
	  
	 // std::cout<<clustering0.spectral<<std::endl;
// 	  clustering1.spectral =  1/(1+exp(-a2*(mat_sum.ptr<float>(i)[j3]-b3)))-1/(1+exp(-a2*(mat_sum.ptr<float>(i)[j3]-b1)));
// 	  clustering2.spectral = 1/(1+exp(-a2*(mat_sum.ptr<float>(i)[j3]-b2)))-1/(1+exp(-a2*(mat_sum.ptr<float>(i)[j3]-b3)));
// 	  clustering3.spectral = 1/(1+exp( a1*(mat_sum.ptr<float>(i)[j3]-b2)));
	  
	  
	  clust.sumSpectral =  clustering0.spectral +  clustering1.spectral +  clustering2.spectral +  clustering3.spectral;
	  distance0.spectral_norm.ptr<float>(i)[j] =  clustering0.spectral/clust.sumSpectral;
	  distance1.spectral_norm.ptr<float>(i)[j] =  clustering1.spectral/clust.sumSpectral;
	  distance2.spectral_norm.ptr<float>(i)[j] =  clustering2.spectral/clust.sumSpectral;
	  distance3.spectral_norm.ptr<float>(i)[j] =  clustering3.spectral/clust.sumSpectral;
	
	  //std::cout<< "the limit is "<<(b2+fabs(b3-b2)*3/4)<<std::endl;
	  
	  

	  // estimation of u

	  
	  max0jml = max(0,j-length);
	  max0iml = max(0,i-length);
	  minjmlcm1 = min(j+length,mat_sum.cols-1);
	  minimlrm1 = min(i+length,mat_sum.rows-1);
	  

	  
	  gettimeofday(&tv,NULL);
	  t1 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
	  time_loc = (1000000 * tv.tv_sec + tv.tv_usec);

	 
	  Rect roi0(max0jml, max0iml, (minjmlcm1+1-max0jml), (minimlrm1+1-max0iml)  );
	  Mat sub0( distance0.prob, roi0);//rois.at(i*mat_sum.cols+j));
	  Mat sub1( distance1.prob, roi0);//rois.at(i*mat_sum.cols+j));
	  Mat sub2( distance2.prob, roi0);//rois.at(i*mat_sum.cols+j));
	  Mat sub3( distance3.prob, roi0);//rois.at(i*mat_sum.cols+j));
	  
	  subRpersubC = sub0.rows*sub0.cols;
	  
	  
	  gettimeofday(&tv,NULL);
	  t5 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
	  time_loc = (1000000 * tv.tv_sec + tv.tv_usec);
	  
	  
	  clustering0.u = (subRpersubC);
	  clustering1.u = (subRpersubC);
	  clustering2.u = (subRpersubC);
	  clustering3.u = (subRpersubC);
	  for (int rowc=0; rowc<sub0.rows ; rowc++)
	  {
	    for (int colc=0; colc<sub0.cols ; colc++)
	    {
	      clustering0.u -= sub0.ptr<float>(rowc)[colc];
	      clustering1.u -= sub1.ptr<float>(rowc)[colc];
	      clustering2.u -= sub2.ptr<float>(rowc)[colc];
	      clustering3.u -= sub3.ptr<float>(rowc)[colc];
	    }
	  }


	  gettimeofday(&tv,NULL);
	  t2 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
	  time_loc = (1000000 * tv.tv_sec + tv.tv_usec);

	  
	  
// 	  
	  // estimation of Pspatial
       
	  clustering0.spatial = exp(-beta*clustering0.u);
	  clustering1.spatial = exp(-beta*clustering1.u);
	  clustering2.spatial = exp(-beta*clustering2.u);
	  clustering3.spatial = exp(-beta*clustering3.u);
	  clust.sumSpatial = clustering0.spatial + clustering1.spatial +  clustering2.spatial + clustering3.spatial;// spatial0+spatial1+ spatial2+spatial3;
	

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
	  

	  
	  
	  
	  gettimeofday(&tv,NULL);
	  t3 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
	  time_loc = (1000000 * tv.tv_sec + tv.tv_usec);
	  
	  
	  
	  
	  
	  distance0.spectral_spatial.ptr<float>(i)[j] = clustering0.specSpat / clust.denomSpatialSpectral;
	  distance1.spectral_spatial.ptr<float>(i)[j] = clustering1.specSpat / clust.denomSpatialSpectral;
	  distance2.spectral_spatial.ptr<float>(i)[j] = clustering2.specSpat / clust.denomSpatialSpectral;
	  distance3.spectral_spatial.ptr<float>(i)[j] = clustering3.specSpat / clust.denomSpatialSpectral;
    
	  m0.num =  m0.num + mat_sum.ptr<float>(i)[j3]*distance0.spectral_spatial.ptr<float>(i)[j];
	  m1.num =  m1.num + mat_sum.ptr<float>(i)[j3]*distance1.spectral_spatial.ptr<float>(i)[j];
	  m2.num =  m2.num + mat_sum.ptr<float>(i)[j3]*distance2.spectral_spatial.ptr<float>(i)[j];
	  m3.num =  m3.num + mat_sum.ptr<float>(i)[j3]*distance3.spectral_spatial.ptr<float>(i)[j];
	  
	  m0.den = m0.den + distance0.spectral_spatial.ptr<float>(i)[j];
	  m1.den = m1.den + distance1.spectral_spatial.ptr<float>(i)[j];
	  m2.den = m2.den + distance2.spectral_spatial.ptr<float>(i)[j];
	  m3.den = m3.den + distance3.spectral_spatial.ptr<float>(i)[j];
	  
	  
	  
	  
	  gettimeofday(&tv,NULL);
	  t4 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
	  time_loc = (1000000 * tv.tv_sec + tv.tv_usec);
      }
    }

    m0.actual=m0.num/m0.den;
    m1.actual=m1.num/m1.den;
    m2.actual=m2.num/m2.den;
    m3.actual=m3.num/m3.den;

    b1 = (m0.actual+m1.actual)/2;
    b2 = (m1.actual+m2.actual)/2;
    b3 = (m2.actual+m3.actual)/2;


    distance0.spectral_spatial.copyTo(distance0.prob);
    distance1.spectral_spatial.copyTo(distance1.prob);
    distance2.spectral_spatial.copyTo(distance2.prob);
    distance3.spectral_spatial.copyTo(distance3.prob);
    
    // TODO eliminate this -> only for debug
    gettimeofday(&tv_tot,NULL);
    unsigned long elapsed = (1000000 * tv_tot.tv_sec + tv_tot.tv_usec) - time_in_micros;
    std::cout << "time_in_micros " << elapsed << std::endl;
    // TODO eliminate this -> only for debug
}


std::cout << t1 << " " << t2 << " " << t3 << " " << t4 << " "  << t5 << std::endl;

Mat image_clean(mat_sum.rows,mat_sum.cols,CV_8UC3);

for (int i = 0; i < image_clean.rows; i++)
    {
      for(int j = 0; j < image_clean.cols; j++)
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
  output5 = image_clean;
  
  
}







