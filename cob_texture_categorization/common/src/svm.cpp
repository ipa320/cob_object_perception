#include <lbp.h>



svm::svm(cv::Mat image_in, int radius, int samples, bool hist_features, )
{

	cv::Mat testval = create_lbp_class(image_in, radius, samples, hist_features);
}

