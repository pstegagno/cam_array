

#include <ImageAnalyzer/ImageAnalyzer.hpp>


using namespace std;
using namespace CameraArrayProject;


int main(int argc, char **argv){
  
  float expTime_correction;
  Mat imgL, imgR, imgM;
  int length;
  float beta;
  Ndvi ndvi(10,10);
  Disparity img(10, 10, 10, 10);

  
  ImageAnalyzer im;
  
  im.detect_material_online(expTime_correction, imgL, imgR, imgM/*, length, beta, ndvi, img*/);
  
  
//   ros::init(argc, argv, "listener");
//   ros::NodeHandle n;
  
//   CameraConverter cam(n, "camera2/image_rect", "camera2/exposition_time", 2);
}


