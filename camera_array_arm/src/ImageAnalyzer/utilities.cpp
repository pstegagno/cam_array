
#include "ImageAnalyzer.hpp"

using namespace cv;



void CameraArrayProject::ImageAnalyzer::printFloatImg(Mat img, string windowName, int wait, bool toGray=false){
    Mat img8;
    img.convertTo(img8, CV_8U);
    
    if(toGray){
      Mat gray_image;
      cvtColor( img8, gray_image, CV_BGR2GRAY );
      namedWindow( windowName.c_str(), CV_WINDOW_AUTOSIZE );
      imshow( windowName.c_str(),  gray_image  );
      waitKey(wait);
    }
    else{
      namedWindow( windowName.c_str(), CV_WINDOW_AUTOSIZE );
      imshow( windowName.c_str(),  img8  );
      waitKey(wait);
    }
}


void CameraArrayProject::ImageAnalyzer::fillEdgeImage(Mat edgesIn, Mat &filledEdgesOut) 
{
    Mat edgesNeg = edgesIn.clone();

    floodFill(edgesNeg, cv::Point(0,0), CV_RGB(255,255,255));
    bitwise_not(edgesNeg, edgesNeg);
    filledEdgesOut = (edgesNeg | edgesIn);

    return;
}



