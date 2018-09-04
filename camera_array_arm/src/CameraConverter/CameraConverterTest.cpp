

#include <CameraConverter/CameraConverter.hpp>


using namespace std;
using namespace CameraArrayProject;

int main(int argc, char **argv){
  
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  CameraConverter cam(n, "camera2/image_rect", "camera2/exposition_time", 2);
}


