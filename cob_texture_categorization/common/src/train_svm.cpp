#include "cob_texture_categorization/train_svm.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>


train_svm::train_svm()
{
}

void train_svm::run_training(std::string *trainingdata, std::string *traininglabel, double gam, double val, std::string *path_)
{

	std::string train_path = *path_ + "train_data.yml";
//	cv::FileStorage fs("/home/rmb-dh/Test_dataset/training_data.yml", cv::FileStorage::READ);
	cv::FileStorage fs(train_path, cv::FileStorage::READ);
	cv::Mat training_data;
	fs["train_data"] >> training_data;
//	std::cout<<training_data<<"Matrix"<<std::endl;

	std::string label_path = *path_ + "train_data_label.yml";
//	cv::FileStorage fsl("/home/rmb-dh/Test_dataset/train_data_respons.yml", cv::FileStorage::READ);
	cv::FileStorage fsl(label_path, cv::FileStorage::READ);
	cv::Mat training_label;
	fsl["train_label"] >> training_label;
//	std::cout<<training_label<<"Matrix"<<std::endl;

	// Set up SVM's parameters
#if CV_MAJOR_VERSION == 2
	CvSVMParams params;
	params.svm_type    = CvSVM::C_SVC;
	params.kernel_type = CvSVM::LINEAR;
	params.term_crit   = cv::TermCriteria(CV_TERMCRIT_ITER, 10000, 1e-8);
	params.gamma = 8.1;
	params.degree = gam;
	params.coef0 = 0.1;
	params.nu = 0.5;
	std::cout << gam<<"gamma val"<<std::endl;

	CvSVM SVM;
	SVM.train(training_data, training_label, cv::Mat(), cv::Mat(), params);

//	cv::FileStorage fsvm("/home/rmb-dh/Test_dataset/svm.yml", cv::FileStorage::WRITE);
//	fs << "SVM_Model" << SVM;

	SVM.save("datasetTextur/yamlfiles/svm.yml", "svm");
#else
	cv::Ptr<cv::ml::SVM> svm = cv::ml::SVM::create();
	svm->setType(cv::ml::SVM::C_SVC);
	svm->setKernel(cv::ml::SVM::LINEAR);
	svm->setDegree(gam);
	svm->setGamma(8.1);
	svm->setCoef0(0.1);
	svm->setC(1.0);
	svm->setNu(0.5);
	svm->setP(0.0);
	svm->setTermCriteria(cv::TermCriteria(CV_TERMCRIT_ITER, 10000, 1e-8));
	cv::Ptr<cv::ml::TrainData> train_data_and_labels = cv::ml::TrainData::create(training_data, cv::ml::ROW_SAMPLE, training_label);
	svm->train(train_data_and_labels);
	svm->save("datasetTextur/yamlfiles/svm.yml");
#endif

	std::cout<<"SVM training completed"<<std::endl;

}
