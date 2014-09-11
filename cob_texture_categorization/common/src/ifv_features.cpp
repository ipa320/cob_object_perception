#include <cob_texture_categorization/ifv_features.h>
#include <iostream>

IfvFeatures::IfvFeatures()
{
	gmm_ = 0;
}


IfvFeatures::~IfvFeatures()
{
	if (gmm_ != 0)
		vl_gmm_delete(gmm_);
}


void IfvFeatures::constructGenerativeModel(const std::vector<std::string>& image_filenames, const int feature_samples_per_image, const double image_resize_factor)
{
	cv::Mat feature_subset(image_filenames.size()*feature_samples_per_image, 128, CV_32FC1);
	for (size_t i=0; i<image_filenames.size(); ++i)
	{
		// load image
		cv::Mat img = cv::imread(image_filenames[i]);
		cv::Mat image, temp;
		cv::cvtColor(img, temp, CV_BGR2GRAY);
		if (image_resize_factor != 1.0)
			cv::resize(temp, image, cv::Size(), image_resize_factor, image_resize_factor, cv::INTER_AREA);
		else
			image = temp;

		// compute dense SIFT features at multiple scales
		cv::Mat features;
		computeDenseSIFTMultiscale(image, features);

		// sample a subset of features used for constructing the GMM
		for (int sample_index=0; sample_index<features.rows; ++sample_index)
		{
			std::set<int> drawn_features;
			while (true)
			{
				int random_feature_index = (int)(((double)rand()/(double)RAND_MAX)*features.rows);
				if (drawn_features.find(random_feature_index) == drawn_features.end())
				{
					// feature not yet sampled
					drawn_features.insert(random_feature_index);
					for (int j=0; j<128; ++j)
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

	// generate the Gaussian Mixture Model
	generateGMM(feature_subset_pc_subspace);
}


void IfvFeatures::computeDenseSIFTMultiscale(const cv::Mat& image, cv::Mat& features)
{

}


void IfvFeatures::generatePCA(const cv::Mat& data)
{
	pca_(data, cv::noArray(), CV_PCA_DATA_AS_ROW);
}


void IfvFeatures::projectToPrincipalComponents(const cv::Mat& data, cv::Mat& mapping)
{
	pca_.project(data, mapping);
}


void IfvFeatures::generateGMM(const cv::Mat& feature_set)
{
	const int number_data = 100*1281;
	const int data_dimension = 128;
	const int number_clusters = 256;

	float* data = new float[number_data*data_dimension];
	for (int c=0; c<number_data; ++c)
		for (int r=0; r<data_dimension; ++r)
			data[c*data_dimension + r] = (float)((float)rand()/(float)RAND_MAX);

	// init with kmeans
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
	std::cout << "Starting clustering ..." << std::endl;
	vl_gmm_cluster(gmm_, data, number_data);
	std::cout << "Clustering finished." << std::endl;
}


void IfvFeatures::saveGenerativeModel(const std::string& filename)
{
	pca_.eigenvalues;
	pca_.eigenvectors;
	pca_.mean;

	//gmm_.;
}


void IfvFeatures::loadGenerativeModel(const std::string& filename)
{

}
