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

	void computeImprovedFisherVector(const std::string& image_filename, const double image_resize_factor, const int number_clusters, cv::Mat& fisher_vector_encoding);

	void constructGenerativeModel(const std::vector<std::string>& image_filenames, const double image_resize_factor=1.0, const int feature_samples_per_image=1000, const int number_clusters = 256);

	// computes dense SIFT features at multiple scales
	// features = matrix with one feature per row
	void computeDenseSIFTMultiscale(const cv::Mat& image, cv::Mat& features);

	// computes a pca on the data
	void generatePCA(const cv::Mat& data);

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

private:
	VlGMM* gmm_;

	cv::PCA pca_;

	const int descriptor_dimension_;
};
