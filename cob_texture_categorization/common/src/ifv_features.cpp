#include <cob_texture_categorization/ifv_features.h>
#include <iostream>

IfvFeatures::IfvFeatures()
: descriptor_dimension_(128)
{
	gmm_ = 0;
}


IfvFeatures::~IfvFeatures()
{
	if (gmm_ != 0)
		vl_gmm_delete(gmm_);
}


void IfvFeatures::computeImprovedFisherVector(const std::string& image_filename, const double image_resize_factor, const int number_clusters, cv::Mat& fisher_vector_encoding)
{
	// load and prepare image
	cv::Mat original_image = cv::imread(image_filename);
	cv::Mat image, temp;
	cv::cvtColor(original_image, temp, CV_BGR2GRAY);
	if (image_resize_factor != 1.0)
		cv::resize(temp, image, cv::Size(), image_resize_factor, image_resize_factor, cv::INTER_AREA);
	else
		image = temp;

	// compute dense SIFT features at multiple scales
	cv::Mat dsift_features;
	computeDenseSIFTMultiscale(image, dsift_features);

	// conduct PCA on data to remove correlation (GMM is only employing diagonal covariance matrices)
	cv::Mat dsift_feature_pc_subspace;
	projectToPrincipalComponents(dsift_features, dsift_feature_pc_subspace);

	// compute Improved Fisher Vector
	vl_fisher_encode((void*)fisher_vector_encoding.ptr(), VL_TYPE_FLOAT, vl_gmm_get_means(gmm_), dsift_features.cols, number_clusters, vl_gmm_get_covariances(gmm_), vl_gmm_get_priors(gmm_), (void*)dsift_feature_pc_subspace.ptr(), dsift_feature_pc_subspace.rows, VL_FISHER_FLAG_IMPROVED);
}


void IfvFeatures::constructGenerativeModel(const std::vector<std::string>& image_filenames, const double image_resize_factor, const int feature_samples_per_image, const int number_clusters)
{
	cv::Mat feature_subset(image_filenames.size()*feature_samples_per_image, descriptor_dimension_, CV_32FC1);
	for (size_t i=0; i<image_filenames.size(); ++i)
	{
		// load image
		std::cout << image_filenames[i] << std::endl;
		cv::Mat original_image = cv::imread(image_filenames[i]);
		cv::Mat image, temp;
		cv::cvtColor(original_image, temp, CV_BGR2GRAY);
		if (image_resize_factor != 1.0)
			cv::resize(temp, image, cv::Size(), image_resize_factor, image_resize_factor, cv::INTER_AREA);
		else
			image = temp;

		// compute dense SIFT features at multiple scales
		cv::Mat features;
		computeDenseSIFTMultiscale(image, features);
		//std::cout << "features size: " << features.rows << ", " << features.cols << std::endl;

		// sample a subset of features used for constructing the GMM
		for (int sample_index=0; sample_index<feature_samples_per_image; ++sample_index)
		{
			std::set<int> drawn_features;
			while (true)
			{
				int random_feature_index = (int)(((double)rand()/(double)RAND_MAX)*features.rows);
				if ((drawn_features.find(random_feature_index) == drawn_features.end()) && sum(features.row(random_feature_index)).val[0] != 0)
				{
					// feature not yet sampled and not zero
					drawn_features.insert(random_feature_index);
					for (int j=0; j<features.cols; ++j)
						feature_subset.at<float>(i*feature_samples_per_image+sample_index,j) = features.at<float>(random_feature_index, j);
					break;
				}
			}
		}
	}

	// conduct PCA on data to remove correlation (GMM is only employing diagonal covariance matrices)
	generatePCA(feature_subset);
	cv::Mat feature_subset_pc_subspace;
	projectToPrincipalComponents(feature_subset, feature_subset_pc_subspace);
	std::cout << "feature_subset_pc_subspace size: " << feature_subset_pc_subspace.rows << ", " << feature_subset_pc_subspace.cols << std::endl;

	// generate the Gaussian Mixture Model
	generateGMM(feature_subset_pc_subspace, number_clusters);
}


void IfvFeatures::computeDenseSIFTMultiscale(const cv::Mat& image, cv::Mat& features)
{
	const double contrast_threshold = 0.005;
	int number_contrast_below_threshold = 0;

	//	std::vector<int> spatial_bin_sizes; // side length of spatial bins in pixels for each scale, order ascendingly!
//	spatial_bin_sizes.push_back(4);
//	spatial_bin_sizes.push_back(6);
//	spatial_bin_sizes.push_back(8);
//	spatial_bin_sizes.push_back(10);
//	spatial_bin_sizes.push_back(12);
//	spatial_bin_sizes.push_back(14);
//	spatial_bin_sizes.push_back(16);	// last element has to be the maximum element!
//	double magnification_factor = 6.0;
//	for (size_t scale_index=0; scale_index<spatial_bin_sizes.size(); ++scale_index)
//	{
//		// smooth image according to scale
//		double sigma = (double)spatial_bin_sizes[scale_index] / magnification_factor;
//		cv::Mat img;
//		image.convertTo(img, CV_32F, 1./255., 0);
//		float* image_ptr = (float*)img.ptr();
//		cv::Mat smoothed_image = cv::Mat::zeros(img.rows, img.cols, CV_32FC1);
//		float* smoothed_image_ptr = (float*)smoothed_image.ptr();
//		vl_imsmooth_f(smoothed_image_ptr, smoothed_image.cols, image_ptr, img.cols, img.rows, img.cols, sigma, sigma);

	const int images_per_octave = 3;
	cv::Mat image_float, octave_base_image;
	image.convertTo(image_float, CV_32F, 1./255., 0);
	octave_base_image = image_float;
	for (int scale=-1; scale<9; ++scale)
	{
		// smooth and resize image according to scale
		cv::Mat smoothed_image;
		if (scale != -1)
		{
			double sigma = pow(2., (double)((((scale-1)%images_per_octave) + 1.)/(double)images_per_octave));
			cv::GaussianBlur(octave_base_image, smoothed_image, cv::Size(0,0), sigma, sigma);
			if (scale % images_per_octave == 0 && scale > 0)
			{
				cv::Mat temp = smoothed_image;
				cv::resize(temp, smoothed_image, cv::Size(0,0), 0.5, 0.5, cv::INTER_AREA);
				octave_base_image = smoothed_image;
			}
		}
		else
			smoothed_image = image_float;
		float* smoothed_image_ptr = (float*)smoothed_image.ptr();

//		cv::imshow("gray", image_float);
//		cv::imshow("smoothed", smoothed_image);
//		cv::waitKey();

		// extract dense SIFT features
		//% Recall from VL_DSIFT() that the first descriptor for scale SIZE has
		//% center located at XC = XMIN + 3/2 SIZE (the Y coordinate is
		//% similar). It is convenient to align the descriptors at different
		//% scales so that they have the same geometric centers. For the
		//% maximum size we pick XMIN = 1 and we get centers starting from
		//% XC = 1 + 3/2 MAX(OPTS.SIZES). For any other scale we pick XMIN so
		//% that XMIN + 3/2 SIZE = 1 + 3/2 MAX(OPTS.SIZES).
		//%
		//% In pracrice, the offset must be integer ('bounds'), so the
		//% alignment works properly only if all OPTS.SZES are even or odd.
		//int offset = floor(1. + 3./2. * (spatial_bin_sizes[spatial_bin_sizes.size()-1] - spatial_bin_sizes[scale_index]));
		VlDsiftFilter* dsift = vl_dsift_new(smoothed_image.cols, smoothed_image.rows);
		vl_dsift_set_steps(dsift, 2, 2);
		VlDsiftDescriptorGeometry dsift_geometry;
		dsift_geometry.numBinT = 8; // number of orientation bins
		dsift_geometry.numBinX = 4;	// number of bins along X
		dsift_geometry.numBinY = 4;	// number of bins along Y
		dsift_geometry.binSizeX = 6;	//spatial_bin_sizes[scale_index];	// size of bins along X
		dsift_geometry.binSizeY = 6;	//spatial_bin_sizes[scale_index];	// size of bins along Y
		vl_dsift_set_geometry(dsift, &dsift_geometry);
		//vl_dsift_set_bounds(dsift, offset, offset, smoothed_image.cols, smoothed_image.rows);
		vl_dsift_set_bounds(dsift, 0, 0, smoothed_image.cols, smoothed_image.rows);
		vl_dsift_set_flat_window(dsift, true);
		vl_dsift_set_window_size(dsift, 1.5);
		vl_dsift_process(dsift, smoothed_image_ptr);
		int number_keypoints = vl_dsift_get_keypoint_num(dsift);
		//std::cout << "number_keypoints" << number_keypoints << std::endl;
		cv::Mat const_features = cv::Mat(number_keypoints, vl_dsift_get_descriptor_size(dsift), CV_32FC1, (void*)vl_dsift_get_descriptors(dsift));
		cv::Mat scale_features = const_features.clone();

		// remove low contrast descriptors
		VlDsiftKeypoint const * keypoints = vl_dsift_get_keypoints(dsift);
		for (int i=0; i<number_keypoints; ++i)
		{
			if (keypoints[i].norm < contrast_threshold)
			{
				++number_contrast_below_threshold;
				for (int j=0; j<scale_features.cols; ++j)
					scale_features.at<float>(i,j) = 0.f;
			}
		}

		features.push_back(scale_features);

		vl_dsift_delete(dsift);
	}

	std::cout << "Contrast below threshold at " << number_contrast_below_threshold << " out of " << features.rows << " feature descriptors." << std::endl;
}


void IfvFeatures::generatePCA(const cv::Mat& data)
{
	pca_(data, cv::noArray(), CV_PCA_DATA_AS_ROW);
}


void IfvFeatures::projectToPrincipalComponents(const cv::Mat& data, cv::Mat& mapping)
{
	pca_.project(data, mapping);
}


void IfvFeatures::generateGMM(const cv::Mat& feature_set, const int number_clusters)
{
	const int number_data = feature_set.rows;
	const int data_dimension = feature_set.cols;

	float* data = (float*)feature_set.ptr();

	// init with kmeans
	std::cout << "Starting kmeans clustering ..." << std::endl;
	// Use float data and the L2 distance for clustering
	VlKMeans* kmeans = vl_kmeans_new(VL_TYPE_FLOAT, VlDistanceL2);
	// Use Lloyd algorithm
	vl_kmeans_set_algorithm(kmeans, VlKMeansLloyd);
	// Initialize the cluster centers by randomly sampling the data
	vl_kmeans_init_centers_with_rand_data(kmeans, data, data_dimension, number_data, number_clusters);
	// Run at most 100 iterations of cluster refinement using Lloyd algorithm
	vl_kmeans_set_max_num_iterations(kmeans, 100);
	vl_kmeans_refine_centers(kmeans, data, number_data);

	// create a new instance of a GMM object for float data
	if (gmm_ != 0)
		vl_gmm_delete(gmm_);
	gmm_ = vl_gmm_new(VL_TYPE_FLOAT, data_dimension, number_clusters);
	// set the maximum number of EM iterations to 100
	vl_gmm_set_max_num_iterations(gmm_, 100);
	// set the initialization to kmeans selection
	vl_gmm_set_initialization(gmm_, VlGMMKMeans);
	vl_gmm_set_kmeans_init_object(gmm_, kmeans);
	// cluster the data, i.e. learn the GMM
	std::cout << "Starting GMM clustering ..." << std::endl;
	vl_gmm_cluster(gmm_, data, number_data);
	std::cout << "Clustering finished." << std::endl;

	vl_kmeans_delete(kmeans);
}


void IfvFeatures::saveGenerativeModel(const std::string& filename)
{
	int number_clusters = vl_gmm_get_num_clusters(gmm_);
	int data_dimension = vl_gmm_get_dimension(gmm_);
	cv::Mat gmm_means(number_clusters, data_dimension, CV_32FC1, (float*)vl_gmm_get_means(gmm_));		// number_clusters * data_dimension
	cv::Mat gmm_covariances(number_clusters, data_dimension, CV_32FC1, (float*)vl_gmm_get_covariances(gmm_));		// number_clusters * data_dimension
	cv::Mat gmm_priors(number_clusters, 1, CV_32FC1, (float*)vl_gmm_get_priors(gmm_));		// number_clusters

	//	Save PCA and GMM models
	cv::FileStorage fs(filename, cv::FileStorage::WRITE);
	fs << "pca_eigenvalues" << pca_.eigenvalues;
	fs << "pca_eigenvectors" << pca_.eigenvectors;
	fs << "pca_mean" << pca_.mean;
	fs << "gmm_means" << gmm_means;
	fs << "gmm_covariances" << gmm_covariances;
	fs << "gmm_priors" << gmm_priors;
	fs.release();

	std::cout << "Generative model (PCA, GMM) stored on disc." << std::endl;
}


void IfvFeatures::loadGenerativeModel(const std::string& filename)
{
	// load computed attributes, class labels and ground truth attributes
	cv::Mat gmm_means, gmm_covariances, gmm_priors;
	cv::FileStorage fs(filename, cv::FileStorage::READ);
	fs["pca_eigenvalues"] >> pca_.eigenvalues;
	fs["pca_eigenvectors"] >> pca_.eigenvectors;
	fs["pca_mean"] >> pca_.mean;
	fs["gmm_means"] >> gmm_means;
	fs["gmm_covariances"] >> gmm_covariances;
	fs["gmm_priors"] >> gmm_priors;
	fs.release();
	int number_clusters = gmm_means.rows;
	int data_dimension = gmm_means.cols;

	// create a new instance of a GMM object for float data
	if (gmm_ != 0)
		vl_gmm_delete(gmm_);
	gmm_ = vl_gmm_new(VL_TYPE_FLOAT, data_dimension, number_clusters);
	// set the initialization to custom selection
	vl_gmm_set_initialization(gmm_, VlGMMCustom);
	vl_gmm_set_means(gmm_, (void*)gmm_means.ptr());
	vl_gmm_set_covariances(gmm_, (void*)gmm_covariances.ptr());
	vl_gmm_set_priors(gmm_, (void*)gmm_priors.ptr());

	std::cout << "Generative model (PCA, GMM) loaded from disc." << std::endl;
}
