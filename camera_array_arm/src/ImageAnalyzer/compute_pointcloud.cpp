
#include <ImageAnalyzer/ImageAnalyzer.hpp>


using namespace cv;
using namespace std;




void CameraArrayProject::ImageAnalyzer::compute_pointcloud(Mat clean, std::stringstream &namefile ){

/////////  stereo camera parameters //////////////////
   
    Mat_<double> cameraMatrix1(3, 3);
    Mat_<double> cameraMatrix2(3, 3);
    Mat_<double> distCoeffs1(5, 1);
    Mat_<double> distCoeffs2(5, 1);
    Mat_<double> R(3, 3);
    Mat_<double> T(3, 1);
   
    cameraMatrix1  << 413.8092624895842, 0.0, 378.41608555363683, 0.0, 414.58118108226586, 249.63202413917932, 0.0, 0.0, 1.0;
    cameraMatrix2  << 406.9185578784797, 0.0, 364.50646541778684, 0.0, 407.83987911594886, 252.405401574727, 0.0, 0.0, 1.0;
    distCoeffs1 << -0.2762540642040707, 0.06446557458606815, -0.0009005274726036505, -0.0013979815025480776, 0.0;
    distCoeffs2 << -0.274577132715303, 0.06301081337644864, -0.0013816235334555647, -0.00013153495216207346, 0.0;
    R  << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    T << -0.09694180375497984,0,0;

    cv::Mat R1,R2,P1,P2,Q;   // you're safe to leave OutpuArrays empty !
    cv::Size imgSize = clean.size(); 

    cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imgSize, R, T, R1, R2, P1, P2, Q);
    
    
    
     
      
       cv::FileStorage savefile( namefile.str(), cv::FileStorage::WRITE);
      //framecounter++;
//       cv::imwrite( ss.str().c_str(), ndvi.clustering );
//       myfile << "file '" <<  ss.str().substr(11) << "'" << endl;

    //std::cout << "Q" << Q << std::endl;

    //std::cout << "h" << std::endl;

    //cv::FileStorage savefile( namefile, cv::FileStorage::WRITE);
    cv::Mat Image(clean.rows, clean.cols, CV_32FC3);

    //std::cout << "i" << std::endl;

    reprojectImageTo3D(clean, Image, Q);

    std::cout << "l" << std::endl;

    // Declare what you need

    //cv::Mat someMatrixOfAnyType;


    savefile << "Image" << Image;
    savefile.release();

    std::cout << "m" << std::endl;   

   }
     
  
