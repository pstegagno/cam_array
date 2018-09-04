

#include <StereoConverter/StereoConverter.hpp>

using namespace CameraArrayProject;

using namespace std;

int main(int argc, char **argv){
  
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  StereoConverter stStereoConverter(n, "my_stereo/left/image_rect", "my_stereo/left/exposition_time", 1,"my_stereo/right/image_rect", "my_stereo/right/exposition_time", 3 );
}


