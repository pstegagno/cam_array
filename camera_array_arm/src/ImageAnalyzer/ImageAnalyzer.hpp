


#ifndef IMAGEANALYZER_HPP_
#define IMAGEANALYZER_HPP_

// standard includes
#include <sys/time.h>

// opencv includes
#include <cv.h>
#include <highgui.h>

#include <ros/ros.h>

#include <ImageAnalyzer/ImageAnalyzer_structs.hpp>
#include <thread>
#include <fstream>


#define DEBUG_TIME 0 
#define SAVE_IMAGES 1



#define TIME_DEBUG_INIT() \
	ros::Time begin, end; \
	begin = ros::Time::now();
	
	
#define TIME_DEBUG(MSG) \
	end = ros::Time::now(); \
	std::cout << #MSG << " " << (end -begin).toSec() << std::endl; \
	begin = ros::Time::now();
	
	
#define TIME_DEBUG_END(MSG) \
	end = ros::Time::now(); \
	std::cout << #MSG << " " << (end -begin).toSec() << std::endl;
	



using namespace cv;

namespace CameraArrayProject{

class ImageAnalyzer{
private:
	
public:
	
	// general parameters
	int dinx; // number of columns of the image
	int diny; // number of rows of the image
	int num_pixels; // number of pixels of the image
	
	// parameters for disparity map calculation
	int max_disparity;
	cv::Size blur_window_size; // window for gradient blur
	int step_size;	// block matching is performed every step_size pixels
	int win_size; // size of the block for the block matching
	double weight_CGRAD; //weight of CGRAD with respect to CSAD in the disparity map calculation 
	
	int disparity_winner_take_all_tolerance; // how close R-L and L-R values need to be while reconstructing the central image
	bool use_threads_slide_images;
	
	bool use_threads_ndvi;
	
	Mat imgL32, imgR32, imgM32;
	float exposure_time_correction;
	
	std::string save_image_folder;
	std::fstream frame_time_file;
	std::stringstream frame_time_filename;


	
	
	
	
	
	
	
	
	
	
	Gradients gradients;
	// structurese for disparity map calculation
	SlideImgs slide_lr;
	SlideImgs slide_rl;
	cv::Mat imgL_th, imgR_th;
	std::thread *slide_imgs_thread;
	bool slide_imgs_flag;
	int slide_images_counter;
	
	
	
	Mat *dsp_slide_images;
	Mat *dsp_ndvi;
	Mat dsp1;
	Mat dsp2;
	
	
	
	
	
	int ndvi_counter;
	int length;
	float beta;
	std::thread *ndvi_thread;
	bool ndvi_flag;
	// structures for everything coming after slide_images
	Disparity data_structs;
	// structures for clustering of the ndvi 
	double bias;
	M m0;
	M m1;
	M m2;
	M m3;
	double b1;
	double b2;
	double b3;
	
	Distance distance0;
	Distance distance1;
	Distance distance2;
	Distance distance3;
	Distance distanceINV;
	
	Clustering clustering0;
	Clustering clustering1;
	Clustering clustering2;
	Clustering clustering3;
	Clustering clusteringINV;
	Clustering clust;
	
// 	int imgRows = mat_sum.rows;
// 	int imgCols = mat_sum.cols;
// 	int lenght1=length+1;
// 	int subRpersubC=0, j1, i1;
// 	int upper_row,lower_row,left_col,rigth_col;
// 	float c0s, c1s, c2s, c3s, csum;
// 	float d0spn, d1spn, d2spn, d3spn;

	Ndvi ndvi;



    vector<int> compression_params;
		
		
		
		bool continue_job;








	
public:
	
	ImageAnalyzer(){};

	
	ImageAnalyzer(int _dinx, int _diny, int _max_disparity, int _blur_window_size, int _step_size, int _disparity_winner_take_all_tolerance,
								int _win_size, double _weight_CGRAD, bool _use_threads_slide_images, bool _use_threads_ndvi,
							  int _length, float _beta, std::string _save_image_folder):
	dinx(_dinx),
	diny(_diny),
	num_pixels(_dinx*_diny),
	max_disparity(_max_disparity),
	blur_window_size(_blur_window_size,_blur_window_size),
	step_size(_step_size),
	win_size(_win_size),
	weight_CGRAD(_weight_CGRAD),
	disparity_winner_take_all_tolerance(_disparity_winner_take_all_tolerance),
	gradients(_diny, _dinx),
	slide_lr(_diny, _dinx, _win_size),
	slide_rl(_diny, _dinx, _win_size),
	dsp1(_diny, _dinx, CV_32FC1),
	dsp2(_diny, _dinx, CV_32FC1),
	data_structs(_diny, _dinx, _win_size, _win_size),
	
	m0(),m1(),m2(),m3(),
	distance0(_diny, _dinx),
	distance1(_diny, _dinx),
	distance2(_diny, _dinx),
	distance3(_diny, _dinx), 
	distanceINV(_diny, _dinx), 
	ndvi( _diny, _dinx)
	{
		
		
		continue_job = true;
		// where do we save the files? here
		save_image_folder = _save_image_folder;
		frame_time_filename << _save_image_folder << "times.txt";
		frame_time_file.open(frame_time_filename.str(), std::fstream::out);
		
		
		// other settings
		use_threads_slide_images = _use_threads_slide_images;
		use_threads_ndvi = _use_threads_ndvi;
		
		length = _length;
		beta = _beta;
		
		// initialization of the parameters for the ndvi clustering function
		bias = -0.0;
		
		b1 = 0.73 +bias;
		b2 = 0.60 +bias;//0.60;
		b3 = 0.45 +bias;//0.45;
		
		m0.actual = 0.85 +bias;
		m1.actual = (b1+b2)/2.0 +bias;//0.665;
		m2.actual = (b2+b3)/2.0 +bias;//0.52;
		m3.actual = 0.40 +bias;//0.40;
		
		slide_images_counter = 0;
		ndvi_counter = 0;
		
		dsp_slide_images = &dsp1;
		dsp_ndvi = &dsp2;
		
		// parameters to save images
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		
		slide_imgs_flag = false;
		if (use_threads_slide_images){
			slide_imgs_thread = new std::thread(&CameraArrayProject::ImageAnalyzer::threaded_disparity_map_slide_images, this);
		}
		
		
		ndvi_flag = false;
		if (use_threads_ndvi){
			ndvi_thread = new std::thread(&CameraArrayProject::ImageAnalyzer::threaded_ndvi_computation, this);
		}

		
	};
  
  
//   void compute_NDVI( Mat NiarInfared, Mat Red, Mat &output1);
    
  
//   void four_images(int length, float beta, Mat NiarInfared, Mat Red, Mat Green, Mat Blue,  Mat &output4);
  
//   void Ndvi_clustering_parabola_old(int length, float beta, Mat mat_sum,  Mat &output5);      
  
//   float	spectral_probability(float dis0, float dis1, float dis2, float dis3);

//   float spectral_distance(float pixel_green, float pixel_red, float pixel_nir, float cluster_center0, float cluster_center1, float cluster_center2);

//   void coordinates(int row,int col ,int  length, Mat &output_matrix1, Mat &output_matrix2, Mat &output_matrix3, Mat &output_matrix4);

  //! This function performs the full algorithm to extract and cluster disparity and ndvi
  void terminate(){
		continue_job = false;
	}
 
  
  
  //! This function performs the full algorithm to extract and cluster disparity and ndvi
  void detect_material_online(float expTime_corection, Mat imgL, Mat imgR, Mat imgM);

  
  
  // Disparity Map Functions
  //! This function computes the disparity map of two given images
  void disparity_map(Mat imgR, Mat imgL);
		
		void disparity_map_compute_gradients(Mat imgL, Mat imgR, Mat &gimgL, Mat &gimgR, Mat &gimgLblured, Mat &gimgRblured);
		
		//! Subroutine of disparity_map
		void disparity_map_slide_images(Mat imgL, Mat imgR, Mat gimgL, Mat gimgR, int mins, int maxs, Mat &output1, Mat &output2, SlideImgs &slide_imgs);
		void threaded_disparity_map_slide_images();
      
			//! Subroutine of disparity_map_slide_images
//       Mat  translateImg(Mat img, Mat &imgout,int offsetx, int offsety);
			void translateImg(Mat &img, Mat &imgout,int offsetx, int offsety);
			void translateImg(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout, int offsetx, int offsety);
			void translateImgAndComputeDiff(Mat &img, Mat &img2, Mat &out, Mat &gimg, Mat &gimg2, Mat &gout, int offsetx, int offsety);
			
			void integrateImg(Mat &img, Mat &imgout);
			void integrateImg(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout);
			
			void blockImg(Mat &img, Mat &imgout, int bs);
			void blockImg(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout, Mat &dout, int bs);
			void blockImg(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout, Mat &mindiff, Mat &dispar, int i, int bs);
			
    //! Subroutine of disparity_map
//     void disparity_map_slide_images2(Mat imgL, Mat imgR, int mins, int maxs, Mat &output1, Mat &output2);
      
      //! Subroutine of disparity_map_slide_images
//       Mat  translateImg(Mat img, Mat &imgout,int offsetx, int offsety);
//       void translateImg2(Mat &img, Mat &imgout,int offsetx, int offsety);
//       void translateImg2(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout, int offsetx, int offsety);
// 			void translateImgAndComputeDiff2(Mat &img, Mat &img2, Mat &out, Mat &gimg, Mat &gimg2, Mat &gout, int offsetx, int offsety);
// 			
// 			void integrateImg2(Mat &img, Mat &imgout);
// 			void integrateImg2(Mat *img, Mat *imgout, Mat *gimg, Mat *gimgout);
// 			
// 			void blockImg2(Mat &img, Mat &imgout);
// 			void blockImg2(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout, Mat &dout);
// 			void blockImg2(Mat &img, Mat &imgout, Mat &gimg, Mat &gimgout, Mat &dout, Mat &mindiff, Mat &dispar, int i);
			
		void threaded_ndvi_computation();

			
		//! Subroutine of disparity_map
		void disparity_map_winner_take_all(Mat d1, Mat m1, Mat d2, Mat m2, Mat &output);
		
		//! Subroutine of disparity_map
		void disparity_map_clean_disparity(Mat &dsp, Mat &dsp_clean);
			void disparity_map_clean_disparity_thread(Mat &dsp);
		
		/// Subroutine of disparity_map
		void disparity_map_new_image(Mat imgL, Mat imgR, Mat dsp, Mat &calibrated);
  
  
  
  
  
  
  //! This function computes the ndvi of two given images 
  void compute_ndvi_online( float expTime_corection ,Mat NiarInfared, Mat Red, Mat &output1); 
  
  
  
  
  
  //! This function performs fuzzy clustering on an ndvi image based on parabolic membership functions
  void ndvi_clustering_parabola(int length, float beta, Mat mat_sum); 
    //! Subroutine of ndvi_clustering_parabola
    void eq_parabola(float value, float x_v, float y_v, float x_p, float y_p, float &parabola );
  
  
  //! This function performs fuzzy clustering on an ndvi image based on sigmoid-shaped membership functions
  void ndvi_clustering_sigmoid( int length, float beta, Mat mat_sum,  Mat &output3);
  
  
  //! This function computes the pointcloud from a disparity map and saves it in a file
  void compute_pointcloud(Mat clean, std::stringstream &namefile );
  
  
  
  // Utilities
  /// print image on screen after converting to CV_8U
  void printFloatImg(Mat img, std::string windowName, int wait, bool toGray);
  
  /// fill edges of merged images
  void fillEdgeImage(Mat edgesIn, Mat &filledEdgesOut);
  


  

  
//   void MultipleThreshold(Mat *src, Mat *dst, int *threshold, int layers);

  

  
  
//   void depth_map( Mat dsp, Disparity &depth) ;
  
  
//   void detect_Material( Mat imgL, Mat imgR, Mat imgM, int length, float beta, Mat &clustering);   
  
  
  
};

  
}; // end namespace CameraArrayProject
  
  
  
#endif
  
