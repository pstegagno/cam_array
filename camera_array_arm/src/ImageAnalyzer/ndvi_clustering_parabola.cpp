
#include <ImageAnalyzer/ImageAnalyzer.hpp>

#define DEBUG_NDVI_CLUSTERING_PARABOLA 1
#define DEBUG_NDVI_CLUSTERING_PARABOLA_DEEP 0

void  CameraArrayProject::ImageAnalyzer::eq_parabola(float value, float x_v, float y_v, float x_p, float y_p, float &parabola )
{
	float a, b, c;
	a=(y_p-y_v)/((x_p-x_v)*(x_p-x_v));
	c=y_v + a*x_v*x_v;
	b=-2*a*x_v;
	parabola = a*value*value+b*value+c;
}


float  /*CameraArrayProject::ImageAnalyzer::*/my_exp(float value)
{
	if (value<-9) return 1.0000e-10;
	if (value>0) return 1.0;
	float a, b, c, d;
	a=(0.1*value+1);
	b=a*a;
	c= b*b;
	d = c*c;
	return d*d*b;
}



void CameraArrayProject::ImageAnalyzer::ndvi_clustering_parabola(int length, float beta, Mat ndvi_in)          
{
	
#if DEBUG_NDVI_CLUSTERING_PARABOLA && DEBUG_TIME
TIME_DEBUG_INIT();
#endif
	
	for(int i = 0; i <ndvi_in.rows ; i++)
	{
		for(int j = 0; j <ndvi_in.cols ; j++)
		{
			float px_val = (ndvi_in.ptr<float>(i)[j]);
			if(  px_val  >  1.0) // invalid
			{
				ndvi.ndvi_classified.ptr<cv::Vec3b>(i)[j]= data_structs.black;
			}
			else if(  px_val  >  b1) // dense vegetation dark green
			{
				ndvi.ndvi_classified.ptr<cv::Vec3b>(i)[j] = data_structs.dark_green;
			}
			else if(/*px_val <= b1 &&*/ px_val > b2)// shurb and grassland
			{
				ndvi.ndvi_classified.ptr<cv::Vec3b>(i)[j]= data_structs.green;
			}
			else if( /*px_val <= b2  &&*/ px_val > b3)// land
			{
				ndvi.ndvi_classified.ptr<cv::Vec3b>(i)[j]= data_structs.brown;
			}
			else // water
			{
				ndvi.ndvi_classified.ptr<cv::Vec3b>(i)[j]= data_structs.blue;
			}
		}
	}
	
#if DEBUG_NDVI_CLUSTERING_PARABOLA && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::ndvi_clustering_parabola::create_ndvi_classified);
struct timeval tv, tv_tot;
long int t1=0, t2=0,t3=0,t4=0,t5=0, t6=0;//,t6=0,t7=0;
#endif
	
	int lenght1=length+1;
	int subRpersubC=0, j1, i1;
	float c0s, c1s, c2s, c3s, cINVs, csum;
	int upper_row,lower_row,left_col,rigth_col;
	float dINVspn, d0spn, d1spn, d2spn, d3spn;
	
// 	integral(distance0.set,distance0.prob, CV_32F);
// 	integral(distance1.set,distance1.prob, CV_32F);
// 	integral(distance2.set,distance2.prob, CV_32F);
// 	integral(distance3.set,distance3.prob, CV_32F);
	distanceINV.prob = distanceINV.set_int.clone();
	distance0.prob = distance0.set_int.clone();
	distance1.prob = distance1.set_int.clone();
	distance2.prob = distance2.set_int.clone();
	distance3.prob = distance3.set_int.clone();
	
	m0.computeParabolaParameters(m0.actual, 1, b1, 0.5);
	m1.computeParabolaParameters(m1.actual, 1, b1, 0.5);
	m2.computeParabolaParameters(m2.actual, 1, b3, 0.5);
	m3.computeParabolaParameters(m3.actual, 1, b3, 0.5);
	
#if DEBUG_NDVI_CLUSTERING_PARABOLA && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::ndvi_clustering_parabola::integral);
#endif 
	
// create probability that a pixel belongs to a certain class given only the ndvi values
	for (int i = 0; i < ndvi_in.rows; i++)
	{
		for(int j = 0; j < ndvi_in.cols; j++)
		{
			float value = ndvi_in.ptr<float>(i)[j];
				
			if (value>1.0){
				distanceINV.spectral_norm.ptr<float>(i)[j] = 0.96;
				distance0.spectral_norm.ptr<float>(i)[j] = 0.01;
				distance1.spectral_norm.ptr<float>(i)[j] = 0.01;
				distance2.spectral_norm.ptr<float>(i)[j] = 0.01;
				distance3.spectral_norm.ptr<float>(i)[j] = 0.01;
			}
			else if (value>m0.actual){
				distanceINV.spectral_norm.ptr<float>(i)[j] = 0.01;
				distance0.spectral_norm.ptr<float>(i)[j] = 0.96;
				distance1.spectral_norm.ptr<float>(i)[j] = 0.01;
				distance2.spectral_norm.ptr<float>(i)[j] = 0.01;
				distance3.spectral_norm.ptr<float>(i)[j] = 0.01;
			}
			else if (value<m3.actual){
				distanceINV.spectral_norm.ptr<float>(i)[j] = 0.01;
				distance0.spectral_norm.ptr<float>(i)[j] = 0.01;
				distance1.spectral_norm.ptr<float>(i)[j] = 0.01;
				distance2.spectral_norm.ptr<float>(i)[j] = 0.01;
				distance3.spectral_norm.ptr<float>(i)[j] = 0.96;
			}
			else
			{
				c0s = max(m0.evaluateParabola(value), (float)0.0000000001);
				c1s = max(m1.evaluateParabola(value), (float)0.0000000001);
				c2s = max(m2.evaluateParabola(value), (float)0.0000000001);
				c3s = max(m3.evaluateParabola(value), (float)0.0000000001);
				cINVs = 0.0000000001;
				
				csum =  c0s +  c1s +  c2s +  c3s + cINVs;
				distanceINV.spectral_norm.ptr<float>(i)[j] = cINVs/csum;
				distance0.spectral_norm.ptr<float>(i)[j] =  c0s/csum;
				distance1.spectral_norm.ptr<float>(i)[j] =  c1s/csum;
				distance2.spectral_norm.ptr<float>(i)[j] =  c2s/csum;
				distance3.spectral_norm.ptr<float>(i)[j] =  c3s/csum;
			}
		}
	}
	
#if DEBUG_NDVI_CLUSTERING_PARABOLA && DEBUG_TIME
TIME_DEBUG(ImageAnalyzer::detect_material_online::ndvi_clustering_parabola::small_cycle);
#endif
	
	for (int num_iter = 0; num_iter  < 3; num_iter ++)
	{
		for (int i = 0; i < ndvi_in.rows; i++)
		{
			for(int j = 0; j < ndvi_in.cols; j++)
			{
#if DEBUG_NDVI_CLUSTERING_PARABOLA_DEEP && DEBUG_TIME
gettimeofday(&tv,NULL);	
unsigned long time_loc = 1000000 * tv.tv_sec + tv.tv_usec;
#endif
				
				i1= i+1;
				j1= j+1;
				
				upper_row = max(0, i1-lenght1);
				lower_row = min(i1+length, diny);
				left_col  = max(0,j1-lenght1);
				rigth_col = min(j1+length, dinx);
				
				subRpersubC = (lower_row - upper_row)*(rigth_col - left_col);
				
				clusteringINV.u = subRpersubC - distanceINV.prob.ptr<float>(upper_row)[left_col] - distanceINV.prob.ptr<float>(lower_row)[rigth_col] + distanceINV.prob.ptr<float>(lower_row)[left_col] + distanceINV.prob.ptr<float>(upper_row)[rigth_col]; 
				clustering0.u = subRpersubC - distance0.prob.ptr<float>(upper_row)[left_col] - distance0.prob.ptr<float>(lower_row)[rigth_col] + distance0.prob.ptr<float>(lower_row)[left_col] + distance0.prob.ptr<float>(upper_row)[rigth_col]; 
				clustering1.u = subRpersubC - distance1.prob.ptr<float>(upper_row)[left_col] - distance1.prob.ptr<float>(lower_row)[rigth_col] + distance1.prob.ptr<float>(lower_row)[left_col] + distance1.prob.ptr<float>(upper_row)[rigth_col]; 
				clustering2.u = subRpersubC - distance2.prob.ptr<float>(upper_row)[left_col] - distance2.prob.ptr<float>(lower_row)[rigth_col] + distance2.prob.ptr<float>(lower_row)[left_col] + distance2.prob.ptr<float>(upper_row)[rigth_col]; 
				clustering3.u = subRpersubC - distance3.prob.ptr<float>(upper_row)[left_col] - distance3.prob.ptr<float>(lower_row)[rigth_col] + distance3.prob.ptr<float>(lower_row)[left_col] + distance3.prob.ptr<float>(upper_row)[rigth_col]; 
				
// 				std::cout << -beta*clustering0.u << " " << -beta*clustering1.u << " " << -beta*clustering2.u << " " << -beta*clustering3.u << " " ;
#if DEBUG_NDVI_CLUSTERING_PARABOLA_DEEP && DEBUG_TIME
gettimeofday(&tv,NULL);
t2 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
time_loc = (1000000 * tv.tv_sec + tv.tv_usec);
#endif
				
				clusteringINV.spatial = my_exp(-beta*clusteringINV.u);
				clustering0.spatial = my_exp(-beta*clustering0.u);
				clustering1.spatial = my_exp(-beta*clustering1.u);
				clustering2.spatial = my_exp(-beta*clustering2.u);
				clustering3.spatial = my_exp(-beta*clustering3.u);
				
#if DEBUG_NDVI_CLUSTERING_PARABOLA_DEEP && DEBUG_TIME
gettimeofday(&tv,NULL);
t3 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
time_loc = (1000000 * tv.tv_sec + tv.tv_usec);
#endif
				
				clust.sumSpatial = clustering0.spatial + clustering1.spatial +  clustering2.spatial + clustering3.spatial + clusteringINV.spatial;
				
				dINVspn = clusteringINV.spatial /clust.sumSpatial;
				d0spn = clustering0.spatial /clust.sumSpatial;
				d1spn = clustering1.spatial /clust.sumSpatial;
				d2spn = clustering2.spatial /clust.sumSpatial;  
				d3spn = clustering3.spatial /clust.sumSpatial; 
				
				clusteringINV.specSpat = distanceINV.spectral_norm.ptr<float>(i)[j]*dINVspn;
				clustering0.specSpat = distance0.spectral_norm.ptr<float>(i)[j]*d0spn;
				clustering1.specSpat = distance1.spectral_norm.ptr<float>(i)[j]*d1spn;
				clustering2.specSpat = distance2.spectral_norm.ptr<float>(i)[j]*d2spn;
				clustering3.specSpat = distance3.spectral_norm.ptr<float>(i)[j]*d3spn;
				
				clust.denomSpatialSpectral = (clustering0.specSpat + clustering1.specSpat + clustering2.specSpat + clustering3.specSpat + clusteringINV.specSpat);
				
				distanceINV.spectral_spatial.ptr<float>(i)[j] = clusteringINV.specSpat / clust.denomSpatialSpectral;
				distance0.spectral_spatial.ptr<float>(i)[j] = clustering0.specSpat / clust.denomSpatialSpectral;
				distance1.spectral_spatial.ptr<float>(i)[j] = clustering1.specSpat / clust.denomSpatialSpectral;
				distance2.spectral_spatial.ptr<float>(i)[j] = clustering2.specSpat / clust.denomSpatialSpectral;
				distance3.spectral_spatial.ptr<float>(i)[j] = clustering3.specSpat / clust.denomSpatialSpectral;
				
#if DEBUG_NDVI_CLUSTERING_PARABOLA_DEEP && DEBUG_TIME
gettimeofday(&tv,NULL);
t4 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
time_loc = (1000000 * tv.tv_sec + tv.tv_usec);	  
#endif
			}
		}
#if DEBUG_NDVI_CLUSTERING_PARABOLA_DEEP && DEBUG_TIME
gettimeofday(&tv,NULL);	
unsigned long time_loc = 1000000 * tv.tv_sec + tv.tv_usec;
#endif
		integral(distanceINV.spectral_spatial,distanceINV.prob, CV_32F);
		integral(distance0.spectral_spatial,distance0.prob, CV_32F);
		integral(distance1.spectral_spatial,distance1.prob, CV_32F);
		integral(distance2.spectral_spatial,distance2.prob, CV_32F);
		integral(distance3.spectral_spatial,distance3.prob, CV_32F);
#if DEBUG_NDVI_CLUSTERING_PARABOLA_DEEP && DEBUG_TIME
gettimeofday(&tv,NULL);
t6 += (1000000 * tv.tv_sec + tv.tv_usec) - time_loc;
time_loc = (1000000 * tv.tv_sec + tv.tv_usec);
#endif
	}

#if DEBUG_NDVI_CLUSTERING_PARABOLA && DEBUG_TIME
std::cout << t1 << " " << t2 << " " << t3 << " " << t4 << " "  << t5 << " " << t6<< std::endl;
TIME_DEBUG(ImageAnalyzer::detect_material_online::ndvi_clustering_parabola::bigcycle);
#endif

//Mat image_clean(ndvi_in.rows,ndvi_in.cols,CV_8UC3);

	for(int i = 0; i < diny ; i++)
	{
		for(int j = 0; j < dinx ; j++)
		{
			float I=distanceINV.spectral_spatial.ptr<float>(i)[j];
			float a=distance0.spectral_spatial.ptr<float>(i)[j];
			float b=distance1.spectral_spatial.ptr<float>(i)[j];
			float c=distance2.spectral_spatial.ptr<float>(i)[j];
			float d=distance3.spectral_spatial.ptr<float>(i)[j];
			if (I>a && I>b && I>c && I>d)
			{
				ndvi.clustering.ptr<cv::Vec3b>(i)[j] = data_structs.black;
			} 
			else if (a>b && a>c && a>d)
			{
				ndvi.clustering.ptr<cv::Vec3b>(i)[j] = data_structs.dark_green;
			} 
			else if (/*b>a &&*/ b>c && b>d)
			{
				ndvi.clustering.ptr<cv::Vec3b>(i)[j]= data_structs.green;
			}
			else if (/*c>a && c>b &&*/ c>d)
			{
				ndvi.clustering.ptr<cv::Vec3b>(i)[j]= data_structs.brown;
			}
			else /*if (d>a && d>b && d>c)*/
			{
				ndvi.clustering.ptr<cv::Vec3b>(i)[j]= data_structs.blue;
			}
		}
	}
	
#if DEBUG_NDVI_CLUSTERING_PARABOLA && DEBUG_TIME
TIME_DEBUG_END(ImageAnalyzer::detect_material_online::ndvi_clustering_parabola::create_clustering_image);
#endif
}



 