#include <cv.h>
#include <highgui.h>
#include<math.h>
#include<new>
#include <iterator>
#include <vector>
#include <algorithm>

using namespace cv;

class Distance
{
public:
//    Mat initial[480][752];
  Mat initial;
  Mat spectral;
  Mat spatial;
  Mat u;
  Mat summatory;
  Mat actual;
  Mat prob;
  Mat spatial_norm;
  Mat spectral_spatial;
//   float** vec;
  
  
  Distance(int rows, int cols):
	initial(rows, cols, CV_32FC1 ),
	spectral(rows, cols, CV_32FC1 ),
	spatial(rows, cols, CV_32FC1 ),
	u(rows, cols, CV_32FC1 ),
	summatory(rows, cols, CV_32FC1 ),
	actual(rows, cols, CV_32FC1 ),
	prob(rows, cols, CV_32FC1 ) ,
	spatial_norm(rows, cols, CV_32FC1 ),
	spectral_spatial(rows, cols, CV_32FC1 )
  {
    
//     vec = new (std::nothrow) float*[rows];
//     for(int i = 0; i < rows; i++){
//       vec[i]=new (std::nothrow) float[cols];
//     }
    
    std::cout << "ENTRO QUI DENTRO " << rows << " " << cols << std::endl;
  }
};



int main( int argc, char** argv )
{
 
double coef_areaB = 1.56; 
double coef_areaG = 1.17; 
double coef_areaR = 1.43; 
double coef_areaNIR = 1; 

// double coef_maxB = 1.15;
// double coef_maxG = 1.06;
double coef_maxR = 1;
double coef_maxNIR = 1.55;


 Mat NiarInfared, Red, Green,Blue,num, den, nvdi;
 NiarInfared = imread("/home/cate/imSaves/image0.jpg" );
 Red = imread("/home/cate/imSaves/image2.jpg" );
 Green = imread("/home/cate/imSaves/image9.jpg" );
 Blue = imread("/home/cate/imSaves/image10.jpg" );

 // imshow("showImg", image_850)
 
 Mat redf, nirf, greenf, bluef;
 Red.convertTo(redf, CV_32FC1);
 NiarInfared.convertTo(nirf, CV_32FC1);
 Green.convertTo(greenf, CV_32FC1);
 Blue.convertTo(bluef, CV_32FC1);
 /*namedWindow( "NIR", CV_WINDOW_AUTOSIZE );
 namedWindow( "RED", CV_WINDOW_AUTOSIZE );
 
 imshow( "NIR",NiarInfared );
 imshow( "RED",Red );
 waitKey(0);*/
 //imshow( "Gray image", gray_image );

 //void imshow(const string& winname, InputArray mat) //
 
// cv::Size sNear = NiarInfared.size(),sRed = Red.size();
// int rowsN, colsN, rowsR,colsR;
// rowsN = sNear.height;
// colsN = sNear.width;
// rowsR = sRed.height;
// colsR = sRed.width;

Mat  value;
value = Red.row(1);

//no normalization

num=nirf-redf;
den=nirf+redf;
divide(num, den ,nvdi); // divide elememt by element

Mat finalMat;
finalMat = (nvdi+1.0)/2.0;

/*namedWindow( "ndvi", CV_WINDOW_AUTOSIZE );
imshow( "ndvi",finalMat );
waitKey(0);*/

// normalization area
Mat nirf_sum, redf_sum, greenf_sum, bluef_sum ,num_sum, den_sum, nvdi_sum; 
 
nirf_sum = nirf*coef_areaNIR;
redf_sum = redf*coef_areaR;
greenf_sum = greenf*coef_areaG;
bluef_sum = bluef*coef_areaB;

num_sum=nirf_sum-redf_sum;
den_sum=nirf_sum+redf_sum;
divide(num_sum, den_sum ,nvdi_sum); // divide elememt by element

Mat finalMat_area, finalMat_area_temp;
// finalMat_area = (nvdi_sum+1.0)/2.0;
finalMat_area_temp = (nvdi_sum+1.0)/2.0;
cv::blur(finalMat_area_temp, finalMat_area, Size(5,5),Point(-1,-1));

/*namedWindow( "ndvi_normalization_area", CV_WINDOW_AUTOSIZE );
imshow( "ndvi_normalization_area",finalMat_area );
waitKey(0); */

// Normalization MAX
Mat nirf_max, redf_max, den_max, num_max, nvdi_max; 

nirf_max = nirf*coef_maxNIR;
redf_max = redf*coef_maxR;
num_max=nirf_max-redf_max;
den_max=nirf_max+redf_max;
divide(num_max, den_max ,nvdi_max); // divide elememt by element

Mat finalMat_max_temp, finalMat_max;
// finalMat_max = (nvdi_max+1.0)/2.0;
finalMat_max_temp = (nvdi_max+1.0)/2.0;
cv::blur(finalMat_max_temp, finalMat_max, Size(15,15),Point(-1,-1));

namedWindow( "ndvi_normalization_max", CV_WINDOW_AUTOSIZE );
imshow( "ndvi_normalization_max", finalMat_max );
waitKey(0);


Mat ndvi_choice(nvdi_sum);
Mat image(480,752,CV_8UC3 );

std::cout << ndvi_choice.rows << "  "<< ndvi_choice.cols << std::endl;

for(int i = 0; i <ndvi_choice.rows ; i++)
{
  for(int j = 0; j <ndvi_choice.cols ; j++)
    
  {
    if(  (ndvi_choice.at<float>(i,j*3))  >  0.4) // dense vegetation dark green
    {
     image.at<cv::Vec3b>(i,j) = cv::Vec3b(0,100,0);
    }  
    if(0.2 < (ndvi_choice.at<float>(i,j*3)) && (ndvi_choice.at<float>(i,j*3)) <= 0.4)// shurb and grassland
    {
      image.at<cv::Vec3b>(i,j)= cv::Vec3b(0,250,0);
    }
    if( ndvi_choice.at<float>(i,j*3) <= 0.2  && ndvi_choice.at<float>(i,j*3) > -0.1)// land
    {
      image.at<cv::Vec3b>(i,j)= cv::Vec3b(20,70,139);
    }
    if(ndvi_choice.at<float>(i,j*3)  <= -0.1)// water
    {
      image.at<cv::Vec3b>(i,j)= cv::Vec3b(100,0,0);
    }
  }
}

  
namedWindow( "image", CV_WINDOW_AUTOSIZE );
imshow( "image", image );
waitKey(0);

// Clustering on the NDVI image

int num_class = 4; 
//double m_class0 = 0.6, m_class1 = 0.3, m_class2 = 0.1, m_class3 = -0.2; //initial values of cluster center
//double dist_class0, dist_class1, dist_class2, dist_class3; 


std::cout << "A" << std::endl;


Distance distance0(image.rows, image.cols);
std::cout << "B" << std::endl;
Distance distance1(image.rows, image.cols), distance2(image.rows, image.cols), distance3(image.rows, image.cols);
std::cout << "C" << std::endl;
Distance distance(image.rows, image.cols);

class M
{
public:
   float start;
   float actual;
   float num;
   float den;
};

M m0, m1, m2, m3;
m0.start = 0.85;
m1.start = 0.65;
m2.start = 0.52;
m3.start = 0.40;

m0.num=0.0; 
m1.num=0.0; 
m2.num=0.0; 
m3.num=0.0; 

m0.den=0.0;
m1.den=0.0;
m2.den=0.0;
m3.den=0.0;



distance0.prob =+ 1.0/((float)num_class);
distance1.prob =+ 1.0/((float)num_class);
distance2.prob =+ 1.0/((float)num_class);
distance3.prob =+ 1.0/((float)num_class);

//std::cout<<""<< distance0.spatial << std::endl;

for (int num_iter = 0; num_iter  < 3; num_iter ++)
{
  //if (num_iter == 0) //only for the first step
//   {
    for (int i = 0; i < image.rows; i++)//for (int i = 0; i < 1; i++)
    {
      for(int j = 0; j < image.cols; j++)//for(int j = 0; j < 1; j++)
      {
	//spectral distance
	
	  distance0.initial.at<float>(i,j) = sqrt((finalMat_area.at<float>(i,j*3)-m0.start)*(finalMat_area.at<float>(i,j*3)-m0.start)); // distance of pixel i,j from the center of the cluter num
	  distance1.initial.at<float>(i,j) = sqrt((finalMat_area.at<float>(i,j*3)-m1.start)*(finalMat_area.at<float>(i,j*3)-m1.start));
	  distance2.initial.at<float>(i,j) = sqrt((finalMat_area.at<float>(i,j*3)-m2.start)*(finalMat_area.at<float>(i,j*3)-m2.start));
	  distance3.initial.at<float>(i,j) = sqrt((finalMat_area.at<float>(i,j*3)-m3.start)*(finalMat_area.at<float>(i,j*3)-m3.start));
	  if (distance0.initial.at<float>(i,j)==0.0)
	  {
	    distance0.initial.at<float>(i,j)=0.0000000001;
	  }
	  if (distance1.initial.at<float>(i,j)==0.0)
	  {
	    distance1.initial.at<float>(i,j)=0.0000000001;
	  }
	  if (distance2.initial.at<float>(i,j)==0.0)
	  {
	    distance2.initial.at<float>(i,j)=0.0000000001;
	  }
	  if (distance3.initial.at<float>(i,j)==0.0)
	  {
	    distance3.initial.at<float>(i,j)=0.0000000001;
	  }
	  
	  
	  distance0.spectral.at<float>(i,j) = (1.0/distance0.initial.at<float>(i,j))/(1.0/distance0.initial.at<float>(i,j)+1.0/distance1.initial.at<float>(i,j)+1.0/distance2.initial.at<float>(i,j) + 1.0/distance3.initial.at<float>(i,j));
	  distance1.spectral.at<float>(i,j) = (1.0/distance1.initial.at<float>(i,j))/(1.0/distance0.initial.at<float>(i,j)+1.0/distance1.initial.at<float>(i,j)+1.0/distance2.initial.at<float>(i,j) + 1.0/distance3.initial.at<float>(i,j));
	  distance2.spectral.at<float>(i,j) = (1.0/distance2.initial.at<float>(i,j))/(1.0/distance0.initial.at<float>(i,j)+1.0/distance1.initial.at<float>(i,j)+1.0/distance2.initial.at<float>(i,j) + 1.0/distance3.initial.at<float>(i,j));
	  distance3.spectral.at<float>(i,j) = (1.0/distance3.initial.at<float>(i,j))/(1.0/distance0.initial.at<float>(i,j)+1.0/distance1.initial.at<float>(i,j)+1.0/distance2.initial.at<float>(i,j) + 1.0/distance3.initial.at<float>(i,j));
	  
	//spatial distance
	  
	   
       int window_width=3;
       int length = window_width/2;
       float beta = 0.2;
       
//        std::cout << max(0,i-length) << " " << max(0,j-length) << " " << (min(i+length,image.rows-1)+1-max(0,i-length)) << " " << (min(j+length,image.cols-1)+1-max(0,j-length)) << std::endl;
//        Rect roi(max(0,i-length),max(0,j-length), (min(i+length,image.rows-1)+1-max(0,i-length)), (min(j+length,image.cols-1)+1-max(0,j-length)));
      
       // estimation of u
       
       Rect roi0(max(0,j-length), max(0,i-length), (min(j+length,image.cols-1)+1-max(0,j-length)), (min(i+length,image.rows-1)+1-max(0,i-length))  );
       Mat sub0( distance0.prob, roi0);
       distance0.u.at<float>(i,j) = (sub0.rows*sub0.cols)-sum(sub0)(0);
       
       Rect roi1(max(0,j-length), max(0,i-length), (min(j+length,image.cols-1)+1-max(0,j-length)), (min(i+length,image.rows-1)+1-max(0,i-length))  );
       Mat sub1( distance1.prob, roi1);
       distance1.u.at<float>(i,j) = (sub1.rows*sub1.cols)-sum(sub1)(0) ;
       
       Rect roi2(max(0,j-length), max(0,i-length), (min(j+length,image.cols-1)+1-max(0,j-length)), (min(i+length,image.rows-1)+1-max(0,i-length))  );
       Mat sub2( distance2.prob, roi2);
       distance2.u.at<float>(i,j) = (sub2.rows*sub2.cols)-sum(sub2)(0) ;
       
       Rect roi3(max(0,j-length), max(0,i-length), (min(j+length,image.cols-1)+1-max(0,j-length)), (min(i+length,image.rows-1)+1-max(0,i-length))  );
       Mat sub3( distance0.prob, roi3);
       distance3.u.at<float>(i,j) = (sub3.rows*sub3.cols)-sum(sub3)(0) ;
//        std::cout << sum(sub1)(0) << std::endl;
       
       // estimation of Pspatial
       
       distance0.spatial.at<float>(i,j) = exp(-beta*distance0.u.at<float>(i,j));
       distance1.spatial.at<float>(i,j) = exp(-beta*distance1.u.at<float>(i,j));
       distance2.spatial.at<float>(i,j) = exp(-beta*distance2.u.at<float>(i,j));
       distance3.spatial.at<float>(i,j) = exp(-beta*distance3.u.at<float>(i,j));

       // Normalization
       distance0.spatial_norm.at<float>(i,j) = distance0.spatial.at<float>(i,j)/(distance0.spatial.at<float>(i,j)+distance1.spatial.at<float>(i,j)+ distance2.spatial.at<float>(i,j)+distance3.spatial.at<float>(i,j));
       distance1.spatial_norm.at<float>(i,j) = distance1.spatial.at<float>(i,j)/(distance0.spatial.at<float>(i,j)+distance1.spatial.at<float>(i,j)+ distance2.spatial.at<float>(i,j)+distance3.spatial.at<float>(i,j));
       distance2.spatial_norm.at<float>(i,j) = distance2.spatial.at<float>(i,j)/(distance0.spatial.at<float>(i,j)+distance1.spatial.at<float>(i,j)+ distance2.spatial.at<float>(i,j)+distance3.spatial.at<float>(i,j));
       distance3.spatial_norm.at<float>(i,j) = distance3.spatial.at<float>(i,j)/(distance0.spatial.at<float>(i,j)+distance1.spatial.at<float>(i,j)+ distance2.spatial.at<float>(i,j)+distance3.spatial.at<float>(i,j));
       
       
      // std::cout<<(distance0.spatial_norm.at<float>(i,j)+distance1.spatial_norm.at<float>(i,j)+ distance2.spatial_norm.at<float>(i,j)+distance3.spatial_norm.at<float>(i,j))<<std::endl;
       
      // the joint spectral-spatial membership 
        
	distance0.spectral_spatial.at<float>(i,j) = distance0.spectral.at<float>(i,j)*distance0.spatial_norm.at<float>(i,j) / (distance0.spectral.at<float>(i,j)*distance0.spatial_norm.at<float>(i,j) + distance1.spectral.at<float>(i,j)*distance1.spatial_norm.at<float>(i,j) + distance2.spectral.at<float>(i,j)*distance2.spatial_norm.at<float>(i,j) + distance3.spectral.at<float>(i,j)*distance3.spatial_norm.at<float>(i,j));
	distance1.spectral_spatial.at<float>(i,j) = distance1.spectral.at<float>(i,j)*distance1.spatial_norm.at<float>(i,j) / (distance0.spectral.at<float>(i,j)*distance0.spatial_norm.at<float>(i,j) + distance1.spectral.at<float>(i,j)*distance1.spatial_norm.at<float>(i,j) + distance2.spectral.at<float>(i,j)*distance2.spatial_norm.at<float>(i,j) + distance3.spectral.at<float>(i,j)*distance3.spatial_norm.at<float>(i,j));
	distance2.spectral_spatial.at<float>(i,j) = distance2.spectral.at<float>(i,j)*distance2.spatial_norm.at<float>(i,j) / (distance0.spectral.at<float>(i,j)*distance0.spatial_norm.at<float>(i,j) + distance1.spectral.at<float>(i,j)*distance1.spatial_norm.at<float>(i,j) + distance2.spectral.at<float>(i,j)*distance2.spatial_norm.at<float>(i,j) + distance3.spectral.at<float>(i,j)*distance3.spatial_norm.at<float>(i,j));
	distance3.spectral_spatial.at<float>(i,j) = distance3.spectral.at<float>(i,j)*distance3.spatial_norm.at<float>(i,j) / (distance0.spectral.at<float>(i,j)*distance0.spatial_norm.at<float>(i,j) + distance1.spectral.at<float>(i,j)*distance1.spatial_norm.at<float>(i,j) + distance2.spectral.at<float>(i,j)*distance2.spatial_norm.at<float>(i,j) + distance3.spectral.at<float>(i,j)*distance3.spatial_norm.at<float>(i,j));
       if (std::isnan(distance0.spatial_norm.at<float>(i,j)))
       {
	 std::cout << 0 << " "<< i << " " << j << " "<< finalMat_area.at<float>(i,j*3) << std::endl;
      }
       if (std::isnan(distance1.spectral_spatial.at<float>(i,j)))
       {
	 std::cout << 1 << " "<< i << " " << j << " "<< finalMat_area.at<float>(i,j*3) << std::endl;
      }
       if (std::isnan(distance2.spectral_spatial.at<float>(i,j)))
       {
	 std::cout << 2 << " "<< i << " " << j << " "<< finalMat_area.at<float>(i,j*3) << std::endl;
      }
       if (std::isnan(distance3.spectral_spatial.at<float>(i,j)))
       {
	 std::cout << 3 << " "<< i << " " << j << " " << finalMat_area.at<float>(i,j*3) << std::endl;
      }
	    
      //std::cout<< "final " << distance0.spectral_spatial.at<float>(i,j)<<std::endl;
      


	m0.num =  m0.num + finalMat_area.at<float>(i,j*3)*distance0.spectral_spatial.at<float>(i,j);
	m1.num =  m1.num + finalMat_area.at<float>(i,j*3)*distance1.spectral_spatial.at<float>(i,j);
	m2.num =  m2.num + finalMat_area.at<float>(i,j*3)*distance2.spectral_spatial.at<float>(i,j);
	m3.num =  m3.num + finalMat_area.at<float>(i,j*3)*distance3.spectral_spatial.at<float>(i,j);

// 	  std::cout<<"minore di zero " << nvdi_max.at<float>(i,j*3) << "  " << distance0.spectral.at<float>(i,j) << "  " << distance1.spectral.at<float>(i,j) << "  " << distance2.spectral.at<float>(i,j) << "  " << distance3.spectral.at<float>(i,j)<< std::endl;
	  
	m0.den = m0.den + distance0.spectral_spatial.at<float>(i,j);
	m1.den = m1.den + distance1.spectral_spatial.at<float>(i,j);
	m2.den = m2.den + distance2.spectral_spatial.at<float>(i,j);
	m3.den = m3.den + distance3.spectral_spatial.at<float>(i,j);
	
// 	std::cout<<"num "<<m0.num<<std::endl;
//         std::cout<<"den "<<m0.den<<std::endl;
      }
    }
/*    
namedWindow( "0", CV_WINDOW_AUTOSIZE );
imshow( "0", distance0.spectral_spatial );
namedWindow( "1", CV_WINDOW_AUTOSIZE );
imshow( "1", distance1.spectral_spatial );
namedWindow( "2", CV_WINDOW_AUTOSIZE );
imshow( "2", distance2.spectral_spatial );
namedWindow( "3", CV_WINDOW_AUTOSIZE );
imshow( "3", distance3.spectral_spatial );
waitKey(0);
std::cout<<""<<m0.num<<std::endl;
std::cout<<""<<m0.den<<std::endl;  */

        m0.actual=m0.num/m0.den;
	m1.actual=m1.num/m1.den;
	m2.actual=m2.num/m2.den;
	m3.actual=m3.num/m3.den;
    
    distance0.spectral_spatial.copyTo(distance0.prob);
    distance1.spectral_spatial.copyTo(distance1.prob);
    distance2.spectral_spatial.copyTo(distance2.prob);
    distance3.spectral_spatial.copyTo(distance3.prob);
//     distance1.prob=distance1.spectral_spatial;
//     distance2.prob=distance2.spectral_spatial;
//     distance3.prob=distance3.spectral_spatial;
	
	
	 
	std::cout<<" m0 \n" << m0.actual << std::endl; 
	std::cout<<" m1 \n" << m1.actual << std::endl;
	std::cout<<" m2 \n" << m2.actual << std::endl;
	std::cout<<" m3 \n" << m3.actual << std::endl;
	
// 	m0.start=m0.actual;
// 	m1.start=m1.actual;
// 	m2.start=m2.actual;
// 	m3.start=m3.actual;
//   }
}


Mat image_clean(480,752,CV_8UC3);

for (int i = 0; i < image.rows; i++)//for (int i = 0; i < 1; i++)
    {
      for(int j = 0; j < image.cols; j++)//for(int j = 0; j < 1; j++)
      {
  
	  float a=distance0.spectral_spatial.at<float>(i,j);
	  float b=distance1.spectral_spatial.at<float>(i,j);
	  float c=distance2.spectral_spatial.at<float>(i,j);
	  float d=distance3.spectral_spatial.at<float>(i,j);


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

namedWindow( "image_clean", CV_WINDOW_AUTOSIZE );
imshow( "image_clean", image_clean );
waitKey(0);

}

 /*            // false color NIR GREEN BLUE

Mat false_color(480,752,CV_8UC3);

//false_color.at<cv::Vec3b>(i,j) = cv::Vec3b(bluef_sum,greenf_sum,nirf_sum);

for(int i=0 ; i < false_color.rows; i++)
{
  for (int j=0; j < false_color.cols; j++)
  {
    float band_one, band_two, band_three;
  
    band_one = greenf_sum.at<float>(i,j*3); //bluef_sum.at<float>(i,j*3);
    band_two =  redf_sum.at<float>(i,j*3);//greenf_sum.at<float>(i,j*3);
    band_three = nirf_sum.at<float>(i,j*3);

    false_color.at<cv::Vec3b>(i,j)=cv::Vec3b(band_one,band_two,band_three);
  }
    
}

    
    

namedWindow( "false_color", CV_WINDOW_AUTOSIZE );
imshow( "false_color", false_color);
waitKey(0);


//std::cout<< "Matrix "<< image << std::endl;
//std::cout << "number rows of NIR are " <<  nvdi_max.rows << std::endl;
//td::cout << "number cols NIR are " << coln << std::endl;
//std::cout << "number rows RED are " << rowsR << std::endl;
//std::cout << "number cols RED are " << colsR << std::endl;
//std::cout << "the value" << NiarInfared << std::endl;
//std::cout << "the sum " << nnvdi<< std::endl;

*/
  
// m_one, m_two, m_three, m_zero}