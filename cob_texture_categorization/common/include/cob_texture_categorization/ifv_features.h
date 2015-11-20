extern "C"
{
	#include <vl/generic.h>
	#include <vl/gmm.h>
	#include <vl/kmeans.h>
	#include <vl/dsift.h>
	#include <vl/imopv.h>
	#include <vl/fisher.h>
}

#include <vector>
#include <set>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>

class IfvFeatures
{
public:

	IfvFeatures();
	~IfvFeatures();

	enum FeatureType {DENSE_MULTISCALE_SIFT = 0, RGB_PATCHES = 1, HSV_PATCHES = 2};

	void computeImprovedFisherVector(const std::string& image_filename, const double image_resize_factor, const int number_clusters, cv::Mat& fisher_vector_encoding, FeatureType feature_type);
	void computeImprovedFisherVector(const cv::Mat& original_image, const double image_resize_factor, const int number_clusters, cv::Mat& fisher_vector_encoding, FeatureType feature_type);

	void constructGenerativeModel(const std::vector<std::string>& image_filenames, const double image_resize_factor=1.0, const int feature_samples_per_image=1000, const int number_clusters = 256, FeatureType feature_type = DENSE_MULTISCALE_SIFT, const int pca_retained_components = 80);

	// computes dense SIFT features at multiple scales
	// features = matrix with one feature per row
	void computeDenseSIFTMultiscale(const cv::Mat& image, cv::Mat& features);

	// computes dense 3x3 RGB patch features at one scale
	// features = matrix with one feature per row
	void computeDenseRGBPatches(const cv::Mat& image, cv::Mat& features);

	// computes a pca on the data
	void generatePCA(const cv::Mat& data, const int pca_retained_component);

	// maps data to the principal components
	void projectToPrincipalComponents(const cv::Mat& data, cv::Mat& mapping);

	// trains a GMM model on the provided data
	void generateGMM(const cv::Mat& feature_set, const int number_clusters = 256);

	// save/load model parameters
	void saveGenerativeModel(const std::string& filename);
	void loadGenerativeModel(const std::string& filename);

	// get pointer to GMM
	VlGMM* getGMMModelPtr()
	{
		return gmm_;
	}

	int getFeatureDimension(FeatureType feature_type)
	{
		if (feature_type == DENSE_MULTISCALE_SIFT)
			return 128;
		else if (feature_type == RGB_PATCHES || feature_type == HSV_PATCHES)
			return 27;
		return -1;
	}

private:
	VlGMM* gmm_;

	cv::PCA pca_;
};
