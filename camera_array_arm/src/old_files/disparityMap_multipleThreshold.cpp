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


void  ImageAnalyzer::MultipleThreshold(Mat *src, Mat *dst, int *threshold, int layers)
{
  //std::cout << "a"<< std::endl;
    if(src == 0 || dst == 0)
        return;
    if(layers <= 0 && src->cols != dst->cols && src->rows != dst->cols && src->channels() != dst->channels())
        return;
    for(int i = 0;i<layers-1; i++)
    {
        int temp = threshold[i];
        for(int j = i+1;j<layers-1;j++)
        {
            if(temp > threshold[j])
            {
                threshold[i] = threshold[j];
                threshold[j] = temp;
                temp = threshold[i];
            }
        }
    }
  //std::cout << "b"<< std::endl;
    int numLayer = 256/layers;
    for(int i=0;i<src->rows;i++)
    {
        for(int j =0;j<src->cols;j+= src->channels())
        {
            for(int k=0;k<src->channels();k++)
            {
// 	      std::cout << src->size() << std::endl;
// 	      std::cout << dst->size() << std::endl;
                int data = (int)src->at<float>(i, j);
                int dstData = (data/numLayer)*numLayer;
//  	      std::cout << data << " " << dstData << std::endl;
                dst->at<float>(i, j) = dstData;
            }
        }
    }
 // std::cout << "c"<< std::endl;
}










