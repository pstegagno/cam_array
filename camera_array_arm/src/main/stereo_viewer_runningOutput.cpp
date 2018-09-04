
#include <StereoConverter/StereoConverter.hpp>
#include <CameraConverter/CameraConverter.hpp>
#include <ImageAnalyzer/ImageAnalyzer.hpp>

#include <thread>

using namespace std;

int numimage;

using namespace cv;
using namespace ros;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

using namespace CameraArrayProject;

static const std::string OPENCV_WINDOW = "spectralViewer";




// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
// begin of main stuff
// @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@



StereoConverter* cam2;
CameraConverter* cam1;




//  
// thread
//

void processImages(StereoConverter* stConv, CameraConverter* imConv, bool save, bool show)
{
  
  int counter = 0;
  cv::Mat imgR, imgL, imgRnew, imgLnew, imgC, imgCnew;
  cv::Mat dsp, calibrated, clean, clustering(480,752,CV_8UC3);
  ImageAnalyzer callFunction;
  float expTime_corection;
  
  string _dspWindowName("clustering");
  cv::namedWindow(_dspWindowName);
  
  string _dspWindowName1("Red_central");
  cv::namedWindow(_dspWindowName1);
  
  string _dspWindowName2("NDVI");
  cv::namedWindow(_dspWindowName2);
  
  while(!stConv->ready()  ||  !imConv->ready()) {
     usleep(100000);
  }

  int framecounter = 0;
  
  ofstream myfile;
  myfile.open ("../imSaves/clustering.txt");
  ofstream myfile1;
  myfile1.open ("../imSaves/ndvi_classified.txt");
  ofstream myfile2;
  myfile2.open ("../imSaves/img_calibrated.txt");
  ofstream myfile3;
  myfile3.open ("../imSaves/img_disparity.txt");
  ofstream myfile4;
  myfile4.open ("../imSaves/ndvi_1D.txt");
   
   
  struct timeval tv;
  gettimeofday(&tv,NULL);
  unsigned long time_first = 1000000 * tv.tv_sec + tv.tv_usec;
  unsigned long time_el = 0;
  unsigned long time_last = 0;

  while(1) {

   
//    Mat recostruction;
// 0.73 0.6 0.6 0.45
//    reprojectImageTo3D(clean_disparity, recostruction);
//    
//   namedWindow( "image", CV_WINDOW_AUTOSIZE );
//   imshow( "image", recostruction);
//   waitKey(0);

    
    if ( stConv->copyImages(imgR, imgL) && imConv->copyImage_central(imgC) ) {
      
      int diny, dinx;
      diny = imgL.rows/2;
      dinx = imgL.cols/2;
      Size size3(dinx, diny);
  
      cv::resize(imgL, imgLnew, size3);
      cv::resize(imgR, imgRnew, size3);
      cv::resize(imgC, imgCnew, size3);

    expTime_corection = (float)((imConv->_expTime))/(float)((stConv->_expTime));    
//     expTime_corection = (float)((stConv->_expTime))/(float)((imConv->_expTime));    
//       
//     cout << "R " << imgR.size() << " " << imgR.channels()  << endl;
    cout << "expTime_corection " << expTime_corection << endl;
//     cout << "L " << imgL.size() << " " << imgL.channels()  << endl;
//     cout << "C " << imgC.size() << " " << imgC.channels()  << endl;
    
    int  win_size  = 7;

    Disparity img(diny,dinx,win_size,win_size);
    
    Ndvi ndvi(dinx, diny);

//    callFunction.stereo(imgL, imgR, 50, dsp, clean, calibrated);
    callFunction.detect_material_online( expTime_corection, imgLnew, imgRnew, imgCnew/*, 9/2, 0.1, ndvi, img*/);
    
    
      
    Mat dsp8, dsp8res, dsp82;
//     Size size(imgL.cols, imgL.rows);;
    img.calibrated.convertTo(dsp8, CV_8U);
    img.clean.convertTo(dsp82, CV_8U);
//     cv::resize(dsp8, dsp8res, size);

//         cv::namedWindow("imgLnew");
//         cv::namedWindow("imgRnew");
//         cv::namedWindow("imgCnew");
	
//     cout << "3" << endl;
//     cv::imshow("imgLnew", imgLnew);
//     cv::waitKey(2);
//     
//     cv::imshow("imgRnew", imgRnew);
//     cv::waitKey(2);
//     
//     cv::imshow("imgCnew", imgCnew);
//     cv::waitKey(2);
    
    
    if (show){
      cv::imshow(_dspWindowName, ndvi.clustering  /*dsp8*/ /*ndvi.ndvi_classified*/);
      cv::waitKey(2);
      
//       cv::imshow(_dspWindowName1, dsp8);
//       cv::waitKey(2);
      
//       cv::imshow(_dspWindowName2, ndvi.ndvi);
//       cv::waitKey(2);
    }
      
    if (save){
      stringstream ss;
      ss.fill('0');
      ss << "../imSaves/clustering_" <<  std::setw(10) << internal << framecounter << ".png";
      cv::imwrite( ss.str().c_str(), ndvi.clustering );
      myfile << "file '" <<  ss.str().substr(11) << "'" << endl;
      
      
      stringstream ss1;
      ss1.fill('0');
      ss1 << "../imSaves/ndvi_classified_" <<  std::setw(10) << internal << framecounter << ".png";
      cv::imwrite( ss1.str().c_str(), ndvi.ndvi_classified );
      myfile1 << "file '" <<  ss1.str().substr(11) << "'" << endl;
      
      stringstream ss2;
      ss2.fill('0');
      ss2<< "../imSaves/img_calibrated_" <<  std::setw(10) << internal << framecounter << ".png";
      cv::imwrite( ss2.str().c_str(), dsp8 );
      myfile2 << "file '" <<  ss2.str().substr(11) << "'" << endl;
      
      stringstream ss3;
      ss3.fill('0');
      ss3 << "../imSaves/img_disparity_" <<  std::setw(10) << internal << framecounter << ".png";
      cv::imwrite( ss3.str().c_str(), dsp82*4.0 );
      myfile3 << "file '" <<  ss3.str().substr(11) << "'" << endl;
      
      
      stringstream ss4;
      ss4.fill('0');
      ss4 << "../imSaves/ndvi_1D_" <<  std::setw(10) << internal << framecounter << ".png";
      cv::imwrite( ss4.str().c_str(), (ndvi.ndvi+1.0)*128.0 );
      myfile4 << "file '" <<  ss4.str().substr(11) << "'" << endl;
      
      framecounter++;
    }
    counter++;
    
    
    gettimeofday(&tv,NULL);
    time_el = (1000000 * tv.tv_sec + tv.tv_usec) - time_first;
    std::cout << "freq " << (1000000.0*(double)counter)/time_el << std::endl;
//     std::cout << "time_el " << time_el << std::endl;
//     std::cout << "time_fi " << time_first << std::endl;
//     std::cout << "time_last " << time_last << std::endl;
    myfile << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
    myfile1 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
    myfile2 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
    myfile3 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
    myfile4 << "duration " <<  (double)(time_el - time_last)/1000000.0 <<  endl;
    
    time_last = time_el;

    
//       cv::waitKey(1);
//     cout << "4" << endl;
    }
  }
  myfile.close();
  myfile1.close();
  myfile2.close();
  myfile3.close();
  myfile4.close();
}




int main(int argc, char **argv)
{
//   XInitThreads();
  
  numimage = 0;

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
  cam2 = (StereoConverter*)new StereoConverter(n, "my_stereo/left/image_rect", "my_stereo/left/exposition_time", 1,"my_stereo/right/image_rect", "my_stereo/right/exposition_time", 3 );
  cam1 = (CameraConverter*)new CameraConverter(n, "camera2/image_rect", "camera2/exposition_time", 2);

  
  bool _saveframesthread = false;
  bool _showframesthread = true;
 // cam2->saveframes();
//  cam2->showframes();
//   cam1->saveframes();
  cam1->showframes();
//   
  
  thread t1(processImages, cam2, cam1, _saveframesthread, _showframesthread);


  ros::spin();

  return 0;

}
