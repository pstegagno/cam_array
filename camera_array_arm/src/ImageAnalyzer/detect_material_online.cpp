
#include <ImageAnalyzer/ImageAnalyzer.hpp>

#include <ros/ros.h>

using namespace cv;


#define DEBUG_DETECT_MATERIAL_ONLINE 1


void CameraArrayProject::ImageAnalyzer::threaded_ndvi_computation(){
	
	while(continue_job){
		
		usleep(100);
		
		if(!ndvi_flag) continue;
		
		
#if SAVE_IMAGES
std::stringstream ss_dsp, ss_clean, ss_calibrated, ss_ndvi, ss_clustering, ss_ndvi_classified;
ss_dsp << save_image_folder << "ss_dsp" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
ss_clean << save_image_folder<<  "ss_clean" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
ss_calibrated << save_image_folder<< "ss_calibrated" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
ss_ndvi << save_image_folder<< "ss_ndvi" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
ss_clustering << save_image_folder<< "ss_clustering" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
ss_ndvi_classified << save_image_folder<< "ss_ndvi_classified" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
#endif
		
	
#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG_INIT();
#endif
	
		disparity_map_clean_disparity(*dsp_ndvi, data_structs.clean);
		
#if SAVE_IMAGES
imwrite(ss_dsp.str().c_str(), (*dsp_ndvi)*5.0, compression_params);
imwrite(ss_clean.str().c_str(), data_structs.clean*5.0, compression_params);
#endif
		
	
#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::disparity_map::clean_disparity);
#endif
	
		disparity_map_new_image( imgL32, imgR32, data_structs.clean, data_structs.calibrated);
		
#if SAVE_IMAGES
imwrite(ss_calibrated.str().c_str(), data_structs.calibrated, compression_params);
#endif

	
#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::disparity_map);
#endif
	
		compute_ndvi_online(exposure_time_correction, imgM32, data_structs.calibrated, ndvi.ndvi);
		
#if SAVE_IMAGES
imwrite(ss_ndvi.str().c_str(), (ndvi.ndvi)*255.0, compression_params);
#endif

	
#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::compute_ndvi_online);
#endif
	
		ndvi_clustering_parabola(length, beta, ndvi.ndvi);
		
#if SAVE_IMAGES
imwrite(ss_ndvi_classified.str().c_str(), ndvi.ndvi_classified, compression_params);
imwrite(ss_clustering.str().c_str(), ndvi.clustering, compression_params);
#endif		
	
#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG_END(ImageAnalyzer::detect_material_online::ndvi_clustering_parabola);
#endif
		ndvi_counter++;
		ndvi_flag = false;
	}
	
	frame_time_file.close();
}







void CameraArrayProject::ImageAnalyzer::detect_material_online( float expTime_correction, Mat imgL, Mat imgR, Mat imgM)   
{
	
#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG_INIT();
#endif


#if SAVE_IMAGES
	frame_time_file << slide_images_counter << " " << std::setprecision(30) << ros::Time::now().toSec() << " " << expTime_correction << '\n';
	frame_time_file.flush();
  
	std::stringstream ss_imgL, ss_imgR, ss_imgM;
	ss_imgL << save_image_folder << "ss_imgL" << std::setfill('0') << std::setw(10) << slide_images_counter << ".png";
	ss_imgR << save_image_folder << "ss_imgR" << std::setfill('0') << std::setw(10) << slide_images_counter << ".png";
	ss_imgM << save_image_folder << "ss_imgM" << std::setfill('0') << std::setw(10) << slide_images_counter << ".png";
	imwrite(ss_imgL.str().c_str(), imgL, compression_params);
	imwrite(ss_imgR.str().c_str(), imgR, compression_params);
	imwrite(ss_imgM.str().c_str(), imgM, compression_params);
#endif


	exposure_time_correction = expTime_correction;
  imgL.convertTo(imgL32, CV_32F);
  imgR.convertTo(imgR32, CV_32F);
	imgM.convertTo(imgM32, CV_32F);

#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::init);
#endif
	
	disparity_map(imgL32,  imgR32);
	
#if DEBUG_DISPARITY_MAP && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::disparity_map::disparity_map);
#endif
	
	disparity_map_winner_take_all(slide_lr.disparity, slide_lr.mindiff, slide_rl.disparity, slide_rl.mindiff,  *dsp_slide_images);
	slide_images_counter++;
	
#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::disparity_map::winner_take_all);
#endif
	
	if (use_threads_ndvi){
		
		while(ndvi_flag){
			usleep(100);
		}
		Mat* temp = dsp_slide_images;
		dsp_slide_images = dsp_ndvi;
		dsp_ndvi = temp;

		ndvi_flag = true;
		
	}
	else{
	
	
	disparity_map_clean_disparity(dsp1, data_structs.clean);
		
#if SAVE_IMAGES
std::stringstream ss_dsp, ss_clean, ss_calibrated, ss_ndvi, ss_clustering, ss_ndvi_classified;
ss_dsp << save_image_folder << "ss_dsp" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
ss_clean << save_image_folder<<  "ss_clean" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
ss_calibrated << save_image_folder<< "ss_calibrated" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
ss_ndvi << save_image_folder<< "ss_ndvi" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
ss_clustering << save_image_folder<< "ss_clustering" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
ss_ndvi_classified << save_image_folder<< "ss_ndvi_classified" << std::setfill('0') << std::setw(10) << ndvi_counter << ".png";
imwrite(ss_dsp.str().c_str(), (*dsp_ndvi)*5.0, compression_params);
imwrite(ss_clean.str().c_str(), data_structs.clean*5.0, compression_params);
#endif
#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::disparity_map::clean_disparity);
#endif

	
		disparity_map_new_image( imgL32, imgR32, data_structs.clean, data_structs.calibrated);
		
		
#if SAVE_IMAGES
imwrite(ss_calibrated.str().c_str(), data_structs.calibrated, compression_params);
#endif
#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::disparity_map);
#endif
		
		compute_ndvi_online(expTime_correction, imgM32, data_structs.calibrated, ndvi.ndvi);
		
#if SAVE_IMAGES
imwrite(ss_ndvi.str().c_str(), (ndvi.ndvi)*255.0, compression_params);
#endif
#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::compute_ndvi_online);
#endif
		
		ndvi_clustering_parabola(length, beta, ndvi.ndvi);
		
#if SAVE_IMAGES
imwrite(ss_ndvi_classified.str().c_str(), ndvi.ndvi_classified, compression_params);
imwrite(ss_clustering.str().c_str(), ndvi.clustering, compression_params);
#endif
		
		ndvi_counter++;
		
#if DEBUG_DETECT_MATERIAL_ONLINE && DEBUG_TIME
TIME_DEBUG_END(ImageAnalyzer::detect_material_online::ndvi_clustering_parabola);
#endif
	}
}


