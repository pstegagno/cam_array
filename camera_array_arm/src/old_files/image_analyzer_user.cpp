#include <ros/ros.h>
#include <cv.h>
#include "image_analyzer.hpp"
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

#include <highgui.h>


#include "PID.hpp"
#include "stereo_viewer_runningOutput.hpp"

#include <thread>




int main(int argc, char **argv)
{
  
  
  ImageAnalyzer picture, disparity_map;

  Mat NiarInfared, Red, Green, Blue, ndvi, ndvi_classified, ndvi_clustering, four_images_clustering , ndvi_parabola;
  //std::vector<Rect> rois;
  Mat NiarInfared_big, Red_big, Green_big, Blue_big, NiarInfared1, Red1, Green1, Blue1,ndvi1, ndvi_parabola1,  ndvi_classified1 ;
  Mat upper_row1, lower_row1, left_col1, rigth_col1;
  
  Mat imgR, imgL, imgR_big, imgL_big, imgC ,imgLnew, imgRnew, imgCnew, disparity1, mindiff1, disparity2, mindiff2, dsp, clean,  calibrated , cleanNew, reproject3d;
  Mat dsp_dilate, calibrated_dilate;
  NiarInfared_big = imread("/home/cate/imSaves/0/image6.jpg",CV_LOAD_IMAGE_GRAYSCALE );  
  Red_big = imread("/home/cate/imSaves/0/image8.jpg" , CV_LOAD_IMAGE_GRAYSCALE);
  Green_big = imread("/home/cate/imSaves/0/image9.jpg" );
  Blue_big = imread("/home/cate/imSaves/0/image10.jpg" );
  
/*  imgR = imread("/home/cate/Desktop/ObjReco/camera_code/disparity_map/stereo_modefilt/tsuR.jpg",CV_LOAD_IMAGE_GRAYSCALE);
  imgL = imread("/home/cate/Desktop/ObjReco/camera_code/disparity_map/stereo_modefilt/tsuL.jpg",CV_LOAD_IMAGE_GRAYSCALE);
 */


//   imgR = imread("/home/cate/imSaves/_4image_1_6518.jpg",CV_LOAD_IMAGE_GRAYSCALE);
//   imgL = imread("/home/cate/imSaves/_4image_3_6515.jpg",CV_LOAD_IMAGE_GRAYSCALE);
  
//   imgR = imread("/home/cate/imSaves/1/image_1_1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
//   imgL = imread("/home/cate/imSaves/1/image_2_1.jpg",CV_LOAD_IMAGE_GRAYSCALE);
  imgC = imread("/home/cate/imSaves/4_image_2_183608.jpg",CV_LOAD_IMAGE_GRAYSCALE);
  imgR_big = imread("/home/cate/imSaves/4_image_1_102674.jpg",CV_LOAD_IMAGE_GRAYSCALE);
  imgL_big = imread("/home/cate/imSaves/4_image_3_102534.jpg",CV_LOAD_IMAGE_GRAYSCALE);
//   clean = imread("/home/cate/imSaves/exp12/img_disparity/img_disparity_0000000136.jpg",CV_LOAD_IMAGE_GRAYSCALE);
  
  //img.depth= imread("/home/cate/imSaves/dsp_dilate.jpg",CV_LOAD_IMAGE_GRAYSCALE);
 Size size(752/2,480/2);
  cv::resize(imgR_big, imgR, size);
  cv::resize(imgL_big, imgL, size);
  cv::resize(imgC, imgCnew, size);

  //ImageAnalyzer::disparityMap_new_image(Mat imgL, Mat imgR, Mat dsp, Mat &calibrated)
// stereo(Mat imgR, Mat imgL, int maxs, Mat &dsp, Mat &clean, Mat &calibrated )
  
  //picture.stereo(imgL, imgR, 20, dsp,clean ,calibrated );
 
  Disparity img(480/2,752/2,7,7);
  img.depth= imread("/home/cate/imSaves/dsp_dilate.jpg",CV_LOAD_IMAGE_GRAYSCALE);
  img.depth.convertTo(img.dsp, CV_32FC1);
  //picture.stereo( imgR, imgL, 20 ,img.disparity, img.clean, img.filled);
  picture.printFloatImg( img.depth, "depth2", 0, false);
  
  
  picture.disparityMap_new_image(imgR, imgL, img.dsp, img.calibrated);
  std::cout << img.dsp.type() << std::endl;
  std::cout << imgL.type() << std::endl;
  std::cout << img.calibrated.type() << std::endl;
      
  
  //picture.printFloatImg( calibrated, "depth", 0, false);
  picture.printFloatImg( img.calibrated, "depth2", 0, false);
  
  //cv::imwrite("../imSaves/dsp.png", dsp);

//   namedWindow( "Material", CV_WINDOW_AUTOSIZE );
//    imshow( "Material",  img.calibrated);
//    waitKey(0);
  

   struct timeval tv;
  
   gettimeofday(&tv,NULL);
   unsigned long time_first = 1000000 * tv.tv_sec + tv.tv_usec; 
 
  unsigned long time_el = (1000000 * tv.tv_sec + tv.tv_usec) - time_first;
  std::cout << "time alltogether " << time_el << std::endl;
  
//   int diny = imgR.rows;
//   int dinx = imgR.cols;
//   int win_size  = 7;
// 
//  Disparity img(diny,dinx,win_size,win_size);
//  Ndvi matrix(dinx,diny);

 /*
  int framecounter = 0;
      
      for(int framecounter = 0; framecounter < 238; framecounter++)
     
      {
	
      stringstream dsp;
      dsp.fill('0');
      dsp << "../imSaves/exp40/img_disparity/img_disparity_" <<  std::setw(10) << internal << framecounter << ".png";
      std::cout << "Disparity  " << dsp.str()<< std::endl;
      
      clean = imread(dsp.str(),CV_LOAD_IMAGE_GRAYSCALE);
      
      stringstream file_xml;
      file_xml.fill('0');
      file_xml << "../imSaves/exp40/pointClaud/img_points_" <<  std::setw(10) << internal << framecounter << ".xml";
      std::cout << "file_xml  " << file_xml.str()<< std::endl;
      
      picture._3Dreconstruction(clean, file_xml);
      
      //std::cout << "clean  " << clean << std::endl;s
      
      //picture.printFloatImg( clean , "depth", 0, false);
      
      }*/
 
 
  //picture.printFloatImg( clean, "depth", 0, false);
  
//cv::imwrite("../catkin_ws/src/cameraarrayproject/camera_array/documents/paper/images/experiment/3d_reconstruction/clustering.png", matrix.clustering);

  
//   namedWindow( "Material", CV_WINDOW_AUTOSIZE );
//   imshow( "Material", ndvi);
//   waitKey(0);

}