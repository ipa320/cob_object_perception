#include <cob_texture_categorization/texture_categorization.h>
#include <cob_texture_categorization/timer.h>


#include "cob_texture_categorization/create_lbp.h"
#include "cob_texture_categorization/splitandmerge.h"
#include "cob_texture_categorization/texture_features.h"
#include "cob_texture_categorization/compute_textures.h"
#include "cob_texture_categorization/depth_image.h"
#include "cob_texture_categorization/segment_trans.h"
#include "cob_texture_categorization/perspective_transformation.h"
#include "cob_texture_categorization/create_train_data.h"
#include "cob_texture_categorization/train_svm.h"
#include "cob_texture_categorization/predict_svm.h"
#include "cob_texture_categorization/color_parameter.h"
#include "cob_texture_categorization/train_ml.h"
#include "cob_texture_categorization/run_meanshift_test.h"
#include "cob_texture_categorization/attribute_learning.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//#include <pcl_ros/point_cloud.h>
//#include <pcl/impl/point_types.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

//#include <pcl_ros/point_cloud.h>


// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>



#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;




TextCategorizationNode::TextCategorizationNode(ros::NodeHandle nh) :
node_handle_(nh)
{
	// considered classes may limit the amount of classes used from a database for training and recognition
	considered_classes_.insert(0);
	considered_classes_.insert(1);
	considered_classes_.insert(2);
/*
//	considered_classes_.insert(2);
	considered_classes_.insert(3);
//	considered_classes_.insert(5);
//	considered_classes_.insert(9);
//	considered_classes_.insert(19);
	considered_classes_.insert(26);
	considered_classes_.insert(29);
	considered_classes_.insert(32);
	considered_classes_.insert(37);
//	considered_classes_.insert(38);
//	considered_classes_.insert(41);
//	considered_classes_.insert(44);
	considered_classes_.insert(46);		// smarties
//	considered_classes_.insert(52);
	considered_classes_.insert(53);
	considered_classes_.insert(54);
//	considered_classes_.insert(56);
*/


//	camera_matrix_received_ = false;

	// subscribers
	it_ = 0;
	sync_input_ = 0;

//	it_ = new image_transport::ImageTransport(node_handle_);
//	colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
//	pointcloud_sub_.subscribe(node_handle_, "pointcloud_in", 1);

//	sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >(30);
//	sync_input_->connectInput(colorimage_sub_, pointcloud_sub_);
//	sync_input_->registerCallback(boost::bind(&TextCategorizationNode::inputCallback, this, _1, _2));

	if (false)
	{
		// live processing
		std::string package_path = ros::package::getPath("cob_texture_categorization");
	//	std::string feature_files_path = package_path + "/common/files/data/cimpoi2014_rgb/scale0-05/"; // path to save data
		std::string feature_files_path = package_path + "/common/files/texture_generator/handcrafted/";
//		std::string feature_files_path = package_path + "/common/files/texture_generator/cimpoi2014_rgb/";
//		std::string gmm_filename = feature_files_path + "gmm_model.yml";
//		ifv_.loadGenerativeModel(gmm_filename);
		al_.load_SVMs(feature_files_path);
		ml_.load_mlp(feature_files_path);
		segmented_pointcloud_  = nh.subscribe("/surface_classification/segmented_pointcloud", 1, &TextCategorizationNode::segmented_pointcloud_callback, this);
	}
	else
	{
		// database tests
	//	attributeLearningDatabaseTestHandcrafted();
	//	attributeLearningDatabaseTestFarhadi();
		attributeLearningDatabaseTestCimpoi();
	//	attributeLearningGeneratedDatabaseTestHandcrafted();
	//	crossValidationVerbalClassDescription();
	}
}

void TextCategorizationNode::segmentationCallback(const std_msgs::String::ConstPtr& msg)
{
	std::cout<<"TEST OK!!!!!!!!!!!!!!!!!!"<<std::endl;
}



TextCategorizationNode::~TextCategorizationNode()
{
	if (it_ != 0)
		delete it_;
	if (sync_input_ != 0)
		delete sync_input_;
}

void TextCategorizationNode::init()
{
//		Display normals of transforamtion
//   	coordinatesystem = node_handle_.advertise<visualization_msgs::MarkerArray>("markertest", 1 );
////    cloudpub = node_handle_.advertise<PointCloud> ("points2", 1);
//    pub_cloud = node_handle_.advertise<sensor_msgs::PointCloud2> ("cloud", 1);
}


void TextCategorizationNode::attributeLearningDatabaseTestHandcrafted()
{
	// === using the hand crafted attributes
	std::string package_path = ros::package::getPath("cob_texture_categorization");
	std::string path_database = package_path + "/common/files/texture_database/";			// path to database			//	std::string path_database = "/media/rmb/SAMSUNG/rmb/datasetTextur/texture_database/";		// path to database
	std::string feature_files_path = package_path + "/common/files/data/handcrafted/"; 		// path to save data

	// compute 17 texture attributes on the ipa texture database
	create_train_data database_data;									// computes feature and label matrices of the provided database
//	database_data.compute_data_handcrafted(path_database, feature_files_path, "ipa_database.txt");
//	return;

	// attribute cross-validation
	cv::Mat base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix;
	create_train_data::DataHierarchyType data_hierarchy;
	train_ml ml;
	AttributeLearning al;
	std::cout << "Loading base features, attributes and class hierarchy from file ...\n";
	// option 1: pre-computed in MATLAB:
//	std::string data_file_name = feature_files_path + "ipa_database.txt";		// Pfad zu Speicherort der Featurevektoren
//	al.loadTextureDatabaseBaseFeatures(data_file_name, 16, 17, computed_attribute_matrix, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);
//	cv::Mat temp = ground_truth_attribute_matrix.clone();
//	ground_truth_attribute_matrix.create(temp.rows, temp.cols-1, temp.type());
//	for (int r=0; r<temp.rows; ++r)
//		for (int c=0; c<16; ++c)
//			ground_truth_attribute_matrix.at<float>(r,c) = temp.at<float>(r,c+(c<13 ? 0 : 1));
	// option 2: computed with this program
	database_data.load_texture_database_features(feature_files_path, base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix, data_hierarchy);
	std::cout << "Loading base features, attributes and class hierarchy from file finished.\n";

//	// experiments on gt data
//	std::string generated_attributes_file_name = feature_files_path + "ipa_database_generated_class_attributes.txt";
//	cv::Mat generated_attributes_16, generated_attributes_17, generated_attributes_class_label_matrix;
//	create_train_data::DataHierarchyType generated_attributes_data_hierarchy;
//	al.loadTextureDatabaseBaseFeatures(generated_attributes_file_name, 16, 17, generated_attributes_16, generated_attributes_17, generated_attributes_class_label_matrix, generated_attributes_data_hierarchy);
//	//ml.cross_validation(20, generated_attributes_17, generated_attributes_class_label_matrix, generated_attributes_data_hierarchy);		// use this version if training and test data shall be drawn from the same data matrix
//	srand(0);
//	cv::Mat mat_rand = ground_truth_attribute_matrix.clone();
////	for (int r=0; r<mat_rand.rows; ++r)
////		for (int c=0; c<mat_rand.cols; ++c)
////		{
////			double sign_factor = (rand() < RAND_MAX/2 ? -1 : 1);
////			mat_rand.at<float>(r,c) += sign_factor*(1.5 + 0.5*(1. - 2.*((double)rand()/(double)RAND_MAX)));
////			mat_rand.at<float>(r,c) = std::max(0.f, std::min(mat_rand.at<float>(r,c), (c==1 || c==2)? 10.f : 5.f));
////		}
////	cv::Mat gt_16(ground_truth_attribute_matrix.rows, 16, CV_32FC1);
////	for (int r=0; r<ground_truth_attribute_matrix.rows; ++r)
////		for (int c=0; c<ground_truth_attribute_matrix.cols; ++c)
////		{
////			if (c<13)
////				gt_16.at<float>(r,c) = ground_truth_attribute_matrix.at<float>(r,c);
////			else if (c>13)
////				gt_16.at<float>(r,c-1) = ground_truth_attribute_matrix.at<float>(r,c);
////		}
////	for (int c=0; c<57; ++c)
////	{
////		al.displayAttributes(generated_attributes_17, generated_attributes_data_hierarchy, c, false);
////		al.displayAttributes(mat_rand, data_hierarchy, c, true, true);
////		//cv::waitKey();
////	}
//	cv::Mat train_ = generated_attributes_17, labels_ = generated_attributes_class_label_matrix, training_data, labels;
//	for (int r=0; r<labels_.rows; ++r)
//		if (considered_classes_.find(labels_.at<float>(r)) != considered_classes_.end())
//		{
//			training_data.push_back(train_.row(r));
//			labels.push_back(labels_.row(r));
//		}
//	ml.train(training_data, labels);
//	ml.save_mlp(feature_files_path);
//	cv::Mat predictions;
//	ml.predict(mat_rand, class_label_matrix, predictions);
//	return;

	CrossValidationParams cvp(CrossValidationParams::LEAVE_OUT_ONE_OBJECT_PER_CLASS, 20, 57);
	//setSVMConfigurations(cvp, "attributes_handcrafted");
	cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::RBF, 0., 0.05, 0., 1., 0.9, 0.));

	std::vector< std::vector<int> > preselected_train_indices;
	std::vector<cv::Mat> attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices;
	////al.crossValidation(cvp, base_feature_matrix, ground_truth_attribute_matrix, data_hierarchy, true, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, false, computed_attribute_matrices);
	al.crossValidation(cvp, computed_attribute_matrix, ground_truth_attribute_matrix, data_hierarchy, true, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, true, computed_attribute_matrices);
	al.saveAttributeCrossValidationData(feature_files_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);

	// final classification: NN learned with labeled attribute data from the training set and tested with the predicted attributes
	cvp.ml_configurations_.clear();
	setSVMConfigurations(cvp, "classes_handcrafted");
//	al.loadAttributeCrossValidationData(feature_files_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);
//	ml.cross_validation(cvp.folds_, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);		// use this version if training and test data shall be drawn from the same data matrix
	ml.cross_validation(cvp, cv::Mat(), class_label_matrix, data_hierarchy, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);	// use this if test data is stored in a different matrix than training data, e.g. because training data comes from the labeled attributes and test data is computed attributes
}


void TextCategorizationNode::attributeLearningDatabaseTestFarhadi()
{
	// === using the farhadi attributes that are learned from base features
	std::string package_path = ros::package::getPath("cob_texture_categorization");
	std::string path_database = package_path + "/common/files/texture_database/";			// path to database
	std::string feature_files_path = package_path + "/common/files/data/farhadi2009/"; 		// path to save data
//	std::string feature_files_path = "/home/rbormann/git/care-o-bot-indigo/src/cob_object_perception/cob_texture_categorization/common/files/data/farhadi2009/features/ipa_texture_database/";
	std::string data_file_name = feature_files_path + "ipa_database.txt";		//Pfad zu Speicherort der Featurevektoren
//	std::string data_file_name = "/home/rbormann/git/care-o-bot-indigo/src/cob_object_perception/cob_texture_categorization/common/files/data/farhadi2009/features/ipa_texture_database/ipa_database.txt";		//Pfad zu Speicherort der Featurevektoren

	// attribute learning
	std::cout << "Loading base features, attributes and class hierarchy from file ...\n";
	AttributeLearning al;
	cv::Mat base_feature_matrix, ground_truth_attribute_matrix, class_label_matrix;
	create_train_data::DataHierarchyType data_hierarchy;
	al.loadTextureDatabaseBaseFeatures(data_file_name, 9688, 17, base_feature_matrix, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);
	std::cout << "Loading base features, attributes and class hierarchy from file finished.\n";

	CrossValidationParams cvp(CrossValidationParams::LEAVE_OUT_ONE_OBJECT_PER_CLASS, 20, 57);
	//setSVMConfigurations(cvp, "attributes_farhadi2009");
	cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::RBF, 0., 0.05, 0., 1., 0.9, 0.));

	std::vector< std::vector<int> > preselected_train_indices;
	std::vector<cv::Mat> attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices;
	al.crossValidation(cvp, base_feature_matrix, ground_truth_attribute_matrix, data_hierarchy, true, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, true, computed_attribute_matrices);
	al.saveAttributeCrossValidationData(feature_files_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);
	//al.loadAttributeCrossValidationData(feature_files_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);

	// final classification: NN learned with labeled attribute data from the training set and tested with the predicted attributes
	cvp.ml_configurations_.clear();
	setSVMConfigurations(cvp, "classes_farhadi2009");
//	//std::cout << "Loading labeled attribute features from file ...\n";
//	//al.loadTextureDatabaseLabeledAttributeFeatures(data_file_name, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);
//	//std::cout << "Loading labeled attribute features from file finished.\n";
	train_ml ml;
	//ml.cross_validation(cvp.folds_, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);		// use this version if training and test data shall be drawn from the same data matrix
	ml.cross_validation(cvp, cv::Mat(), class_label_matrix, data_hierarchy, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);	// use this if test data is stored in a different matrix than training data, e.g. because training data comes from the labeled attributes and test data is computed attributes
}


void TextCategorizationNode::attributeLearningDatabaseTestCimpoi()
{
	// === using the 47 texture attributes of Cimpoi
	std::string package_path = ros::package::getPath("cob_texture_categorization");
	std::string path_database = package_path + "/common/files/texture_database/";			// path to database
//	std::string path_database = "/media/rmb/SAMSUNG/rmb/datasetTextur/texture_database/";		// path to database
	std::string feature_files_path = package_path + "/common/files/data/cimpoi2014_rgb/"; 		// path to save data
//	std::string feature_files_path = "/home/rbormann/git/care-o-bot-indigo/src/cob_object_perception/cob_texture_categorization/common/files/data/cimpoi2014_rgb/scale0-05/"; // path to save data

	// compute 17 texture attributes on the ipa texture database
	create_train_data database_data;									// computes feature and label matrices of the provided database
//	database_data.compute_data_cimpoi(path_database, feature_files_path, "ipa_database.txt", 0, true, IfvFeatures::RGB_PATCHES);
//	return;

	// attribute cross-validation
	cv::Mat base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix;
	create_train_data::DataHierarchyType data_hierarchy;
	train_ml ml;
	AttributeLearning al;
	std::cout << "Loading base features, attributes and class hierarchy from file ...\n";
	database_data.load_texture_database_features(feature_files_path, base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix, data_hierarchy);
	std::cout << "Loading base features, attributes and class hierarchy from file finished.\n";

//	// train classifier with whole database
//	al.train(base_feature_matrix, ground_truth_attribute_matrix);
//	al.save_SVMs(feature_files_path);
//	//return;

	CrossValidationParams cvp(CrossValidationParams::LEAVE_OUT_ONE_OBJECT_PER_CLASS, 20, 57);
	setSVMConfigurations(cvp, "attributes_cimpoi2014_rgb");

	std::vector< std::vector<int> > preselected_train_indices;
	std::vector<cv::Mat> attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices;
	al.crossValidation(cvp, base_feature_matrix, ground_truth_attribute_matrix, data_hierarchy);//, true, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, true, computed_attribute_matrices);
	//////al.crossValidation(cvp, computed_attribute_matrix, ground_truth_attribute_matrix, data_hierarchy, true, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, false, computed_attribute_matrices);
	al.saveAttributeCrossValidationData(feature_files_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);
return;
	// final classification: NN learned with labeled attribute data from the training set and tested with the predicted attributes
	//ml.cross_validation(cvp.folds_, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);		// use this version if training and test data shall be drawn from the same data matrix
	al.loadAttributeCrossValidationData(feature_files_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);
	ml.cross_validation(cvp, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);	// use this if test data is stored in a different matrix than training data, e.g. because training data comes from the labeled attributes and test data is computed attributes
}


void TextCategorizationNode::attributeLearningGeneratedDatabaseTestHandcrafted()
{
	// === using the hand crafted attributes
	std::string package_path = ros::package::getPath("cob_texture_categorization");
	std::string path_database = package_path + "/common/files/data/texture_generator/";
//	std::string data_file_name = package_path + "/common/files/data/texture_generator/handcrafted/ipa_database_handcrafted.txt";		//Pfad zu Speicherort der Featurevektoren
	std::string feature_files_path = package_path + "/common/files/data/texture_generator/handcrafted/"; // path to save data

	// compute 17 texture attributes on the ipa texture database
	create_train_data database_data(1);									// computes feature and label matrices of the provided database
//	database_data.compute_data_handcrafted(path_database, feature_files_path, 1700);
//	database_data.compute_data_cimpoi(path_database, feature_files_path, 1700, 0, true, IfvFeatures::RGB_PATCHES);
//	return;

	// attribute cross-validation
	cv::Mat base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix;
	create_train_data::DataHierarchyType data_hierarchy;
	train_ml ml;
	AttributeLearning al;
	std::cout << "Loading base features, attributes and class hierarchy from file ...\n";
	// option 1: pre-computed in MATLAB:
//	al.loadTextureDatabaseBaseFeatures(data_file_name, 16, 17, computed_attribute_matrix, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);
//	cv::Mat temp = ground_truth_attribute_matrix.clone();
//	ground_truth_attribute_matrix.create(temp.rows, temp.cols-1, temp.type());
//	for (int r=0; r<temp.rows; ++r)
//		for (int c=0; c<16; ++c)
//			ground_truth_attribute_matrix.at<float>(r,c) = temp.at<float>(r,c+(c<13 ? 0 : 1));
	// option 2: computed with this program
	database_data.load_texture_database_features(feature_files_path, base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix, data_hierarchy);
	std::cout << "Loading base features, attributes and class hierarchy from file finished.\n";

//	// train classifier with whole database
//	al.train(computed_attribute_matrix, ground_truth_attribute_matrix);
//	al.save_SVMs(feature_files_path);
//	return;

	// generated class descriptions
	std::string generated_attributes_file_name = feature_files_path + "all_generated_attributes.txt";//"ipa_database_generated_class_attributes.txt";
	cv::Mat generated_attributes_16, generated_attributes_17, generated_attributes_class_label_matrix;
	create_train_data::DataHierarchyType generated_attributes_data_hierarchy;
	al.loadTextureDatabaseBaseFeatures(generated_attributes_file_name, 16, 17, generated_attributes_16, generated_attributes_17, generated_attributes_class_label_matrix, generated_attributes_data_hierarchy);
	cv::Mat train_ = generated_attributes_17, labels_ = generated_attributes_class_label_matrix, training_data, labels;
//	for (int r=0; r<labels_.rows; ++r)
//		if (considered_classes_.find(labels_.at<float>(r)) != considered_classes_.end())
//		{
//			training_data.push_back(train_.row(r));
//			labels.push_back(labels_.row(r));
//		}
	training_data = train_;
	labels = labels_;

	ml.train(training_data, labels);
	ml.save_mlp(feature_files_path);
//	cv::Mat predictions;
//	ml.predict(mat_rand, class_label_matrix, predictions);
	return;

	CrossValidationParams cvp(CrossValidationParams::LEAVE_OUT_ONE_OBJECT_PER_CLASS, 20, 57);
	std::vector< std::vector<int> > preselected_train_indices;
	std::vector<cv::Mat> attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices;
	////al.crossValidation(cvp, base_feature_matrix, ground_truth_attribute_matrix, data_hierarchy, true, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, false, computed_attribute_matrices);
	al.crossValidation(cvp, computed_attribute_matrix, ground_truth_attribute_matrix, data_hierarchy, true, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, true, computed_attribute_matrices);
	al.saveAttributeCrossValidationData(feature_files_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);

	// final classification: NN learned with labeled attribute data from the training set and tested with the predicted attributes
//	al.loadAttributeCrossValidationData(feature_files_path, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);
//	ml.cross_validation(cvp.folds_, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);		// use this version if training and test data shall be drawn from the same data matrix
	ml.cross_validation(cvp, cv::Mat(), class_label_matrix, data_hierarchy, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, computed_attribute_matrices);	// use this if test data is stored in a different matrix than training data, e.g. because training data comes from the labeled attributes and test data is computed attributes
}


void TextCategorizationNode::crossValidationVerbalClassDescription()
{
	enum Method {HANDCRAFTED_RAW, HANDCRAFTED_LEARNED, FARHADI, CIMPOI};
	Method method = FARHADI;

	std::string package_path = ros::package::getPath("cob_texture_categorization");
	std::string feature_files_path = "";	// path to save data
	if (method == HANDCRAFTED_RAW || method == HANDCRAFTED_LEARNED)
		feature_files_path = package_path + "/common/files/data/handcrafted/";
	else if (method == FARHADI)
		feature_files_path = package_path + "/common/files/data/farhadi2009/features/ipa_texture_database/";
	else if (method == CIMPOI)
		feature_files_path = package_path + "/common/files/data/cimpoi2014_rgb/";
	else
		return;

	// load base features and attribute labels
	std::cout << "Loading base features, attributes and class hierarchy from file ...\n";
	create_train_data database_data;
	AttributeLearning al;
	train_ml ml;
	cv::Mat base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix;
	create_train_data::DataHierarchyType data_hierarchy;
	if (method == FARHADI)
	{
		std::string data_file_name = feature_files_path + "ipa_database.txt";
		al.loadTextureDatabaseBaseFeatures(data_file_name, 9688, 17, base_feature_matrix, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);
	}
	else
	{
		// option 1: pre-computed in MATLAB:
//		al.loadTextureDatabaseBaseFeatures(data_file_name, 16, 17, computed_attribute_matrix, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);
//		cv::Mat temp = ground_truth_attribute_matrix.clone();
//		ground_truth_attribute_matrix.create(temp.rows, temp.cols-1, temp.type());
//		for (int r=0; r<temp.rows; ++r)
//			for (int c=0; c<16; ++c)
//				ground_truth_attribute_matrix.at<float>(r,c) = temp.at<float>(r,c+(c<13 ? 0 : 1));
		// option 2: computed with this program
		database_data.load_texture_database_features(feature_files_path, base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix, data_hierarchy);
	}
	std::cout << "Loading base features, attributes and class hierarchy from file finished.\n";

	// compute the attribute predictions for all cross validation cycles (i.e. train attribute classifiers leaving out the class of interest each time)
	CrossValidationParams cvp(CrossValidationParams::LEAVE_OUT_ONE_CLASS, 57, 57);
	std::vector<cv::Mat> computed_attribute_matrices;
//	if (method == HANDCRAFTED_LEARNED || method == HANDCRAFTED_RAW)
//		al.crossValidation(cvp, computed_attribute_matrix, ground_truth_attribute_matrix, data_hierarchy, computed_attribute_matrices);
//	else
//		al.crossValidation(cvp, base_feature_matrix, ground_truth_attribute_matrix, data_hierarchy, computed_attribute_matrices);
//	ml.save_computed_attribute_matrices(feature_files_path, computed_attribute_matrices);

	// do the cross validation on class prediction (train the texture category classifier with computed attributes or labeled attributes and
	// use the artificially generated samples for training the class of interest (test data is the computed or labeled attributes on the real image data)
	// load the generated attributes for all classes from file (generated from human verbal description)
	std::string generated_attributes_file_name = feature_files_path + "ipa_database_generated_class_attributes.txt";
	cv::Mat generated_attributes_16, generated_attributes_17, generated_attributes_class_label_matrix;
	create_train_data::DataHierarchyType generated_attributes_data_hierarchy;
	al.loadTextureDatabaseBaseFeatures(generated_attributes_file_name, 16, 17, generated_attributes_16, generated_attributes_17, generated_attributes_class_label_matrix, generated_attributes_data_hierarchy);
	// do the cross validation
	// todo: change file name to not overwrite 20fold data ml.save_computed_attribute_matrices(feature_files_path, computed_attribute_matrices);
	ml.load_computed_attribute_matrices(feature_files_path, computed_attribute_matrices);
	if (method == HANDCRAFTED_LEARNED || method == HANDCRAFTED_RAW)
		ml.cross_validation_with_generated_attributes(cvp.folds_, computed_attribute_matrices, class_label_matrix, data_hierarchy, generated_attributes_16, generated_attributes_class_label_matrix, generated_attributes_data_hierarchy);
	else
		ml.cross_validation_with_generated_attributes(cvp.folds_, computed_attribute_matrices, class_label_matrix, data_hierarchy, generated_attributes_17, generated_attributes_class_label_matrix, generated_attributes_data_hierarchy);
		//ml.cross_validation_with_generated_attributes(cvp.folds_, computed_attribute_matrices, class_label_matrix, data_hierarchy, ground_truth_attribute_matrix, class_label_matrix, data_hierarchy);
}

void TextCategorizationNode::setNNConfigurations(CrossValidationParams& cvp, const std::string& experiment_key)
{
	// NN
	if (experiment_key.compare("attributes_handcrafted")==0)
	{
		for (double param1=0.1; param1<0.91; param1+=0.1)
			cvp.ml_configurations_.push_back(MLParams(MLParams::NEURAL_NETWORK, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 100, 0.00001f, CvANN_MLP_TrainParams::BACKPROP, 0.1f, 0.1f, std::vector<int>(1, 5), CvANN_MLP::SIGMOID_SYM, param1, 1.0));
		for (double param1=0.1; param1<0.91; param1+=0.1)
			cvp.ml_configurations_.push_back(MLParams(MLParams::NEURAL_NETWORK, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 100, 0.00001f, CvANN_MLP_TrainParams::BACKPROP, 0.1f, 0.1f, std::vector<int>(1, 10), CvANN_MLP::SIGMOID_SYM, param1, 1.0));
		for (double param1=0.1; param1<0.91; param1+=0.1)
			cvp.ml_configurations_.push_back(MLParams(MLParams::NEURAL_NETWORK, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 100, 0.00001f, CvANN_MLP_TrainParams::BACKPROP, 0.1f, 0.1f, std::vector<int>(1, 20), CvANN_MLP::SIGMOID_SYM, param1, 1.0));
	}
	else if (experiment_key.compare("classes_handcrafted")==0)
	{
		for (int hidden_neurons = 100; hidden_neurons<401; hidden_neurons*=2)
			for (double param1=0.1; param1<0.91; param1+=0.1)
				cvp.ml_configurations_.push_back(MLParams(MLParams::NEURAL_NETWORK, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 400, 0.0001f, CvANN_MLP_TrainParams::BACKPROP, 0.1f, 0.1f, std::vector<int>(1, hidden_neurons), CvANN_MLP::SIGMOID_SYM, param1, 1.2));
	}
	else if (experiment_key.compare("classes_farhadi2009")==0)
	{
		for (int hidden_neurons = 50; hidden_neurons<401; hidden_neurons*=2)
			for (double param1=0.1; param1<0.91; param1+=0.1)
				cvp.ml_configurations_.push_back(MLParams(MLParams::NEURAL_NETWORK, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 400, 0.0001f, CvANN_MLP_TrainParams::BACKPROP, 0.1f, 0.1f, std::vector<int>(1, hidden_neurons), CvANN_MLP::SIGMOID_SYM, param1, 1.2));
	}
}

void TextCategorizationNode::setSVMConfigurations(CrossValidationParams& cvp, const std::string& experiment_key)
{
	// SVM
	if (experiment_key.compare("attributes_handcrafted")==0)
	{
		for (double nu=0.1; nu<0.91; nu+=0.1)
			cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::LINEAR, 0., 0.1, 0., 1., nu, 0.));
		std::vector<double> values; values.push_back(0.01); values.push_back(0.05); values.push_back(0.1); values.push_back(0.5); values.push_back(1.0);
		for (size_t gamma_index=0; gamma_index<values.size(); ++gamma_index)
			for (double nu=0.1; nu<0.91; nu+=0.1)
				cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::RBF, 0., values[gamma_index], 0., 1., nu, 0.));
	}
	else if (experiment_key.compare("attributes_farhadi2009")==0)
	{
//		for (double nu=0.4; nu<0.91; nu+=0.1)
//			cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::LINEAR, 0., 0.1, 0., 1., nu, 0.));
//		std::vector<double> values; values.push_back(0.01); values.push_back(0.05); //values.push_back(0.1); //values.push_back(0.5); values.push_back(1.0);
//		for (double nu=0.8; nu<0.91; nu+=0.1)
//			for (size_t gamma_index=0; gamma_index<values.size(); ++gamma_index)
//					cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::RBF, 0., values[gamma_index], 0., 1., nu, 0.));
		cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::RBF, 0., 0.01, 0., 1., 0.8, 0.));
		cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::RBF, 0., 0.05, 0., 1., 0.9, 0.));
		cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::RBF, 0., 0.01, 0., 1., 0.9, 0.));
		cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::RBF, 0., 0.05, 0., 1., 0.8, 0.));
	}
	else if (experiment_key.compare("attributes_cimpoi2014_sift")==0)
	{
		for (double nu=0.9; nu>0.1; nu-=0.1)
			cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::LINEAR, 0., 0.1, 0., 1., nu, 0.));
	}
	else if (experiment_key.compare("attributes_cimpoi2014_rgb")==0)
	{
//		for (double nu=0.2; nu<0.91; nu+=0.1)
//			cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::LINEAR, 0., 0.1, 0., 1., nu, 0.));
		for (double nu=0.9; nu>0.1; nu-=0.1)
			cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::NU_SVR, CvSVM::RBF, 0., 0.05, 0., 1., nu, 0.));

	}
	else if (experiment_key.compare("classes_handcrafted")==0)
	{
		std::vector<double> values; values.push_back(0.01); values.push_back(0.05); values.push_back(0.1); values.push_back(0.5); values.push_back(1.0);
		for (size_t C_index=0; C_index<values.size(); ++C_index)
			cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::C_SVC, CvSVM::LINEAR, 0., 0.2, 1., 10*values[C_index], 0., 0.));
		for (size_t gamma_index=0; gamma_index<values.size(); ++gamma_index)
			for (size_t C_index=0; C_index<values.size(); ++C_index)
				cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::C_SVC, CvSVM::RBF, 0., values[gamma_index], 1., 10*values[C_index], 0., 0.));
	}
	else if (experiment_key.compare("classes_farhadi2009")==0)
	{
		std::vector<double> C_values; C_values.push_back(0.1); C_values.push_back(0.5); C_values.push_back(1.0); C_values.push_back(5.0); C_values.push_back(10.0); C_values.push_back(50.0); C_values.push_back(100.0);
		std::vector<double> gamma_values; gamma_values.push_back(0.01); gamma_values.push_back(0.05); gamma_values.push_back(0.1); gamma_values.push_back(0.5); gamma_values.push_back(1.0);
		for (size_t C_index=0; C_index<C_values.size(); ++C_index)
			cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::C_SVC, CvSVM::LINEAR, 0., 0.2, 1., C_values[C_index], 0., 0.));
		for (size_t gamma_index=0; gamma_index<gamma_values.size(); ++gamma_index)
			for (size_t C_index=0; C_index<C_values.size(); ++C_index)
				cvp.ml_configurations_.push_back(MLParams(MLParams::SVM, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, FLT_EPSILON, CvSVM::C_SVC, CvSVM::RBF, 0., gamma_values[gamma_index], 1., C_values[C_index], 0., 0.));
	}
}


struct segment_position{
	int segment;
	cv::Point2f position;
};
void TextCategorizationNode::segmented_pointcloud_callback(const cob_surface_classification::SegmentedPointCloud2& segmented_pointcloud_msg)
{
	std::cout<<"Begin"<<std::endl;
	Timer timer;

	// parameters
	bool do_2d_segmentation = false;
	bool display_original_3d_segments = false;
	bool display_2d_segments = (do_2d_segmentation && false);

	cv::Mat segmentation_3d; // visualization of received 3d segmentation
	cv::Mat segmentation_2d; // visualization of additional 2d segmentation on 3d segments

	const cv::Size source_image_size(segmented_pointcloud_msg.pointcloud.width, segmented_pointcloud_msg.pointcloud.height);
	std::cout << source_image_size.width << "x" << source_image_size.height << "p image." << std::endl;

	// display original 3d segmentation
	if (display_original_3d_segments == true)
	{
		segmentation_3d = cv::Mat::zeros(source_image_size.height,source_image_size.width,CV_8UC3);
		for(unsigned int i=0; i<segmented_pointcloud_msg.clusters.size();i++)
		{
			int r = rand() % 50;
			int g = rand() % 256;
			int b = rand() % 256;
			for(unsigned int j=0; j<segmented_pointcloud_msg.clusters[i].array.size();j++)
			{
				int x = segmented_pointcloud_msg.clusters[i].array[j]%source_image_size.width;
				int y = segmented_pointcloud_msg.clusters[i].array[j]/source_image_size.width;
				segmentation_3d.at<cv::Vec3b>(y,x)[0]=b;
				segmentation_3d.at<cv::Vec3b>(y,x)[1]=g;
				segmentation_3d.at<cv::Vec3b>(y,x)[2]=r;
			}
		}
	}

	// convert point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromROSMsg(segmented_pointcloud_msg.pointcloud, *cloud);

	// restore original image
	cv::Mat orig_img;
	orig_img = cv::Mat::zeros(source_image_size.height, source_image_size.width, CV_8UC3);
	for(unsigned int i=0; i<segmented_pointcloud_msg.clusters.size();i++)
	{
		for(unsigned int j=0; j<segmented_pointcloud_msg.clusters[i].array.size();j++)
		{
			int x = segmented_pointcloud_msg.clusters[i].array[j]%source_image_size.width;
			int y = segmented_pointcloud_msg.clusters[i].array[j]/source_image_size.width;
			orig_img.at<cv::Vec3b>(y,x)[0]=cloud->points[segmented_pointcloud_msg.clusters[i].array[j]].b;
			orig_img.at<cv::Vec3b>(y,x)[1]=cloud->points[segmented_pointcloud_msg.clusters[i].array[j]].g;
			orig_img.at<cv::Vec3b>(y,x)[2]=cloud->points[segmented_pointcloud_msg.clusters[i].array[j]].r;
		}
	}

	// 2d segmentation
	cv::Mat orig_img_draw;
	std::vector<cv::Mat> segment_vec, retransformed_segment;
	if (do_2d_segmentation == true)
	{
		cv::Mat segment_img;
		cv::Mat depth(source_image_size.height, source_image_size.width, CV_32F);
		std::vector<float> plane_coeff;
		visualization_msgs::MarkerArray marker;
		cv::Mat undefined_cluster;				// contains all small segments
		undefined_cluster = cv::Mat::zeros(source_image_size.height, source_image_size.width, CV_8UC3);
		cv::Mat seg_whole;						// image to show all segments of depthsegmentation
		seg_whole = cv::Mat::zeros(source_image_size.height,source_image_size.width,CV_8UC3);
		//std::vector<cv::Mat> segment_edges;
		//cv::Mat test=cv::Mat::zeros(source_image_size.height,source_image_size.width,CV_8UC3);

		std::vector<segment_position> seg_pos_vec;

		orig_img_draw = orig_img.clone();
		int count = 1;
		std::vector<cv::Point2f> schwerepunkt;
		std::cout<<"Transform"<<std::endl;
		////Transform Segments of Depthsegmentation if possible
		std::vector<cv::Mat> segment_img_vec;
		std::vector<cv::Mat> H_vec;
		for(unsigned int i=0; i<segmented_pointcloud_msg.clusters.size();i++)
		{
			schwerepunkt.clear();
			if(segmented_pointcloud_msg.clusters[i].array.size()>1500)//750
			{
				segment_img = cv::Mat::zeros(source_image_size.height, source_image_size.width, CV_8UC3);
				cv::Mat binary_img = cv::Mat::zeros(source_image_size.height, source_image_size.width, CV_32F);
				pcl::IndicesPtr indices_ptr(new std::vector<int>(segmented_pointcloud_msg.clusters[i].array.size()));
				for(unsigned int j=0; j<segmented_pointcloud_msg.clusters[i].array.size();j++)
				{
					const int point_index = segmented_pointcloud_msg.clusters[i].array[j];
					(*indices_ptr)[j] = point_index;
					binary_img.at<float>(point_index/source_image_size.width, point_index%source_image_size.width)=255;
				}

				////dilate binary image for reducing of artefacts
				int dilation_size = 1;
				int dilation_type = cv::MORPH_RECT;//MORPH_RECT MORPH_CROSS  MORPH_ELLIPSE;
				cv::Mat element = cv::getStructuringElement( dilation_type, cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
															   cv::Point( dilation_size, dilation_size ) );
				cv::dilate( binary_img, binary_img, element );

				////Create new image out of dilated binary image
	//			int r1 = rand() % 50;
	//			int g1 = rand() % 256;
	//			int b1 = rand() % 256;

				for(int iy=0;iy<source_image_size.height;iy++)
				{
					for(int ix=0;ix<source_image_size.width;ix++)
					{
						if(binary_img.at<int>(iy,ix)>0)
						{
							segment_img.at<cv::Vec3b>(iy,ix)=orig_img_draw.at<cv::Vec3b>(iy,ix);
//							segment_img.at<cv::Vec3b>(iy,ix)[1]=orig_img_draw.at<cv::Vec3b>(iy,ix)[1];
//							segment_img.at<cv::Vec3b>(iy,ix)[2]=orig_img_draw.at<cv::Vec3b>(iy,ix)[2];
							schwerepunkt.push_back(cv::Point(ix,iy));

	//						if(orig_img_draw.at<cv::Vec3b>(iy,ix)[0]!=0 || orig_img_draw.at<cv::Vec3b>(iy,ix)[1]!=0 || orig_img_draw.at<cv::Vec3b>(iy,ix)[2]!=0 ||((
	//							orig_img_draw.at<cv::Vec3b>(iy,ix-1)[0]!=0 && orig_img_draw.at<cv::Vec3b>(iy,ix-1)[1]!=0 && orig_img_draw.at<cv::Vec3b>(iy,ix-1)[2]!=0 &&
	//							orig_img_draw.at<cv::Vec3b>(iy,ix+1)[0]!=0 && orig_img_draw.at<cv::Vec3b>(iy,ix+1)[1]!=0 && orig_img_draw.at<cv::Vec3b>(iy,ix+1)[2]!=0) ||(
	//							orig_img_draw.at<cv::Vec3b>(iy-1,ix)[0]!=0 && orig_img_draw.at<cv::Vec3b>(iy-1,ix)[1]!=0 && orig_img_draw.at<cv::Vec3b>(iy-1,ix)[2]!=0 &&
	//							orig_img_draw.at<cv::Vec3b>(iy+1,ix)[0]!=0 && orig_img_draw.at<cv::Vec3b>(iy+1,ix)[1]!=0 && orig_img_draw.at<cv::Vec3b>(iy+1,ix)[2]!=0)||
	//							(orig_img_draw.at<cv::Vec3b>(iy,ix-2)[0]!=0 && orig_img_draw.at<cv::Vec3b>(iy,ix-2)[1]!=0 && orig_img_draw.at<cv::Vec3b>(iy,ix-2)[2]!=0 &&
	//							orig_img_draw.at<cv::Vec3b>(iy,ix+2)[0]!=0 && orig_img_draw.at<cv::Vec3b>(iy,ix+2)[1]!=0 && orig_img_draw.at<cv::Vec3b>(iy,ix+2)[2]!=0) ||(
	//							orig_img_draw.at<cv::Vec3b>(iy-2,ix)[0]!=0 && orig_img_draw.at<cv::Vec3b>(iy-2,ix)[1]!=0 && orig_img_draw.at<cv::Vec3b>(iy-2,ix)[2]!=0 &&
	//							orig_img_draw.at<cv::Vec3b>(iy+2,ix)[0]!=0 && orig_img_draw.at<cv::Vec3b>(iy+2,ix)[1]!=0 && orig_img_draw.at<cv::Vec3b>(iy+2,ix)[2]!=0)
	//									))
	//						{
	//							test.at<cv::Vec3b>(iy,ix)[0]=b1;
	//							test.at<cv::Vec3b>(iy,ix)[1]=g1;
	//							test.at<cv::Vec3b>(iy,ix)[2]=r1;
	//						}
						}
					}
				}
				segment_img_vec.push_back(segment_img.clone());
				cv::Mat edges = cv::Mat::zeros(source_image_size.height,source_image_size.width, CV_8UC1);
				////Create data for visualistation
				binary_img.convertTo(binary_img,CV_8U,255.0/(255));
				int nonZero = countNonZero(binary_img);
				cv::Canny(binary_img, edges , 254, 255, 3);
				std::vector<std::vector<cv::Point> > contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::findContours(edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
				cv::Mat drawing = cv::Mat::zeros( edges.size(), CV_8UC3 );
				//segment_edges.push_back(edges);
				std::vector<cv::Point> used_contours;
				for(unsigned int cont1=0;cont1<contours.size();cont1++)
				{
					for(unsigned int cont2=0; cont2<contours[cont1].size();cont2++)
					{
						if((int)contours[cont1].size()>40)
						{
							used_contours.push_back(contours[cont1][cont2]);
						}
					}
				}

				bool usefull_3D_data = true;

				///  Get the mass centers:
				cv::Point2f mc;
				/// Get min rect around contours
				cv::RotatedRect rec;
	//			imwrite( "/home/rmb-dh/Pictures/minArea1.jpg", segment_img );
				rec =  minAreaRect(schwerepunkt);

				mc = rec.center;
				cv::Point2f rect_points[4]; rec.points( rect_points );
				double size_of_rec = rec.size.width * rec.size.height;
				double fuellgrad = (nonZero)/size_of_rec;
	//			for( int j = 0; j < 4; j++ )
	//				cv::line( segment_img, rect_points[j], rect_points[(j+1)%4], cvScalar(255,0,0), 1, 8 );
				if(fuellgrad<0.3)
				{
					usefull_3D_data =false;
				}
				else
				{
					////Drawn Lines in orig image and rect in contours image
					for(unsigned int ci = 0; ci< contours.size(); ci++ )
					{
						if((int)contours[ci].size()>30)
						{
							  cv::Scalar color = cv::Scalar(0,0,200);
							  cv::drawContours( orig_img, contours, ci, color, 2, 8, hierarchy, 0, cv::Point() );
						 }
					}
	//				for( int j = 0; j < 4; j++ )
	//				cv::line( orig_img, rect_points[j], rect_points[(j+1)%4], cvScalar(255,0,0), 1, 8 );

					if(fuellgrad<=0.3)
					{
						struct segment_position pos;
						pos.position = mc;
						pos.segment = count;
						seg_pos_vec.push_back(pos);
						count++;
					}
					else
					{
						int index_bigcont=0;
						for(unsigned int bigcont=1;bigcont<contours.size();bigcont++)
						{
							if(contours[index_bigcont].size()<contours[bigcont].size())
							{
								index_bigcont=bigcont;
							}
						}
	//					int pos_bigcont = floor(contours[index_bigcont].size()/2);
						struct segment_position pos;
						pos.position = mc;//contours[index_bigcont][pos_bigcont];
						pos.segment = count;
						seg_pos_vec.push_back(pos);
						count++;
					 }
				}

				if(usefull_3D_data)
				{
					////Compute Transformation and save transformed image in segment_vec
					PerspectiveTransformation p_transform;
					cv::Mat H;
					p_transform.normalize_perspective(segment_img, cloud, plane_coeff, H, 1000., indices_ptr);

					cv::Mat newimg = segment_img.clone();
					segment_vec.push_back(newimg);
					cv::Mat H_new = H.clone();
					H_vec.push_back(H_new);
				}
				else
				{
					for(unsigned int j=0; j<segmented_pointcloud_msg.clusters[i].array.size();j++)
						{
							int x = segmented_pointcloud_msg.clusters[i].array[j]%source_image_size.width;
							int y = segmented_pointcloud_msg.clusters[i].array[j]/source_image_size.width;
							undefined_cluster.at<cv::Vec3b>(y,x)[0]=(*cloud).points[segmented_pointcloud_msg.clusters[i].array[j]].b;
							undefined_cluster.at<cv::Vec3b>(y,x)[1]=(*cloud).points[segmented_pointcloud_msg.clusters[i].array[j]].g;
							undefined_cluster.at<cv::Vec3b>(y,x)[2]=(*cloud).points[segmented_pointcloud_msg.clusters[i].array[j]].r;
						}
				}
			}
			else
			{
				for(unsigned int m=0; m<segmented_pointcloud_msg.clusters[i].array.size();m++)
				{
					int x = segmented_pointcloud_msg.clusters[i].array[m]%source_image_size.width;
					int y = segmented_pointcloud_msg.clusters[i].array[m]/source_image_size.width;
					undefined_cluster.at<cv::Vec3b>(y,x)[0]=(*cloud).points[segmented_pointcloud_msg.clusters[i].array[m]].b;
					undefined_cluster.at<cv::Vec3b>(y,x)[1]=(*cloud).points[segmented_pointcloud_msg.clusters[i].array[m]].g;
					undefined_cluster.at<cv::Vec3b>(y,x)[2]=(*cloud).points[segmented_pointcloud_msg.clusters[i].array[m]].r;
				}
			}
		}
		//Add unsegmented areas as one image to vector of segmented images
		cv::Mat work_segment;
		cvtColor( undefined_cluster, work_segment, CV_BGR2GRAY );
		//int nonZeros = cv::countNonZero(work_segment);
		if(true)//(nonZeros)/(source_image_size.width*source_image_size.height)>0.2)
		{

			segment_vec.push_back(undefined_cluster);
			cv::Mat zero;
			H_vec.push_back(zero);
			struct segment_position pos;
			pos.position = cv::Point2f(10,10);
			pos.segment = count;
			seg_pos_vec.push_back(pos);
		}

	//	for(size_t i=0;i<segment_vec.size();i++)
	//	{
	//		cv::imshow("Ausgabe tiefenbild", segment_vec[i]);
	//		cv::waitKey();
	//	}

		std::vector<cv::Mat> segment_copy = segment_vec;

		std::cout<<"Segment reduction"<<std::endl;
		////Reduce segment on necessary area
		std::vector<bool> optimized, swap_vec_bool;
		std::vector<cv::RotatedRect> rect_saved;
		std::vector<double> angle_saved;
		rect_saved.resize(segment_vec.size());
		angle_saved.resize(segment_vec.size());
		swap_vec_bool.resize(segment_vec.size());
		if(segment_vec.size()>1)
		{
			for(unsigned int i=0;i<segment_vec.size()-1;i++)
			{
				std::vector <cv::Point> seg_points;
				cv::Mat work_segment = segment_vec[i];
				for(int pi=0; pi<source_image_size.height;pi++)
				{
					for(int pj=0; pj<source_image_size.width;pj++)
					{
						if(work_segment.at<cv::Vec3b>(pi,pj)[0]!=0 || work_segment.at<cv::Vec3b>(pi,pj)[1]!=0 || work_segment.at<cv::Vec3b>(pi,pj)[2]!=0)
						{
							seg_points.push_back(cv::Point(pj,pi));
						}
					}
				}
				cv::RotatedRect rec;
				if(seg_points.size()>3)
				{
					rec =  minAreaRect(seg_points);
				}
				cv::Mat M, rotated, new_segment;
				// get angle and size from the bounding box
				float angle = rec.angle;
				cv::Size rect_size = rec.size;
				if (rec.angle < -45.)
				{
					angle += 90.0;
					std::swap(rect_size.width, rect_size.height);
					swap_vec_bool[i]=true;
				}else{
					swap_vec_bool[i]=false;
				}
				bool use_segment = false;
				if(segment_vec[i].cols>100 && segment_vec[i].rows>100 && rec.size.width>0 && rec.size.height>0)
				{
					M = cv::getRotationMatrix2D(rec.center, angle, 1.0);
					rect_saved[i]=rec;
					cv::warpAffine(work_segment, rotated, M, work_segment.size(), cv::INTER_CUBIC);
					cv::getRectSubPix(rotated, rect_size, rec.center, new_segment);
					use_segment=true;
				}
				if(use_segment&& !new_segment.empty())
				{
					segment_vec[i] = new_segment;
					angle_saved[i] = angle;
					optimized.push_back(true);
				}else{
					optimized.push_back(false);
				}
			}
		}


		for(size_t i=0;i<segment_vec.size();i++)
		{
			std::ostringstream outStream;
			outStream << i;
			std::string num;
			num  = outStream.str();

	//		std::string filename = "/home/rmb-dh/evaluation/segments" +num+".jpg";
			std::string filename = "segmentation/segments_3d_" +num+".png";
			cv::imwrite(filename, segment_vec[i] );
		}



	//	int size = segment_vec.size();
	//	for(int i=0; i<size; i++)
	//	{
	//
	//		if(swap_vec_bool[i])
	//		{
	//			double swap_val= rect_saved2[i].size.width;
	//			rect_saved2[i].size.width = rect_saved[i].size.height;
	//			rect_saved2[i].size.height = swap_val;
	//			rect_saved2[i].angle -= 90.0;
	//			std::cout<<"swap x/y"<<std::endl;
	////		swap_val = rect_saved[i].center.x;
	////		rect_saved[i].center.x= rect_saved[i].center.y;
	////		rect_saved[i].center.y= swap_val;
	//		}
	//
	//		if(optimized[i])
	//		{
	//			cv::Mat rotated, M;
	//			cv::Mat new_segment = cv::Mat::zeros(source_image_size.height,source_image_size.width,CV_8UC3);
	//
	//
	//					M = cv::getRotationMatrix2D(cv::Point2f(rect_saved2[i].size.width/2,rect_saved2[i].size.height/2), -angle_saved[i], 1.0);
	//					cv::warpAffine(segment_vec[i], new_segment, M, orig_img.size(), cv::INTER_CUBIC);
	//
	//					cv::imshow("first",segment_vec[i]);
	//					cv::imshow("second",new_segment);
	//
	//					int x_off=0,y_off=0;
	//					x_off= round(rect_saved2[i].center.x-(rect_saved2[i].size.width/2));
	//					y_off = round(rect_saved2[i].center.y-(rect_saved2[i].size.height/2));
	////				if(swap_vec_bool[i]){
	////					int swap_val = x_off;
	////					x_off = y_off;
	////					y_off = swap_val;
	////				}
	//
	//					cv::Mat offset_new_segment = cv::Mat::zeros(source_image_size.height,source_image_size.width,CV_8UC3);
	//					std::cout<<rect_saved[i].center<<"centercoo  "<<rect_saved[i].size<<"size"<<std::endl;
	//
	//					for(int n=0;n<source_image_size.width;n++)
	//					{
	//						for(int m=0;m<source_image_size.height;m++)
	//						{
	//							if(m+y_off<source_image_size.height && n+x_off<source_image_size.width && m+y_off>=0 && n+x_off>=0 ){
	//								offset_new_segment.at<cv::Vec3b>(m+y_off,n+x_off)[0]=new_segment.at<cv::Vec3b>(m,n)[0];
	//								offset_new_segment.at<cv::Vec3b>(m+y_off,n+x_off)[1]=new_segment.at<cv::Vec3b>(m,n)[1];
	//								offset_new_segment.at<cv::Vec3b>(m+y_off,n+x_off)[2]=new_segment.at<cv::Vec3b>(m,n)[2];
	//							}
	//						}
	//					}
	//					std::cout<<"debug2"<<std::endl;
	//					cv::imshow("new",offset_new_segment);
	//					cv::imshow("after trans", segment_copy[i]);
	//					std::cout<<"debug3"<<std::endl;
	//					cv::Mat testmat = cv::Mat::zeros(source_image_size.height,source_image_size.width,CV_8UC3);
	//					std::cout<<H_vec[i]<<std::endl;
	//					if(H_vec[i].rows==3 && H_vec[i].cols==3){
	//						cv::warpPerspective(offset_new_segment, testmat, H_vec[i], testmat.size());
	//					}else{
	//						testmat = offset_new_segment;
	//					}
	//
	//					std::cout<<"debug4"<<std::endl;
	//					cv::imshow("transformed",testmat);
	//					cv::waitKey(1000000);
	//					for(int n=0;n<source_image_size.width;n++)
	//					{
	//						for(int m=0;m<source_image_size.height;m++)
	//						{
	//							if(testmat.at<cv::Vec3b>(m,n)[0]!=0 && testmat.at<cv::Vec3b>(m,n)[1]!=0 && testmat.at<cv::Vec3b>(m,n)[2]!=0 ){
	//								orig_img.at<cv::Vec3b>(m,n)[0]=testmat.at<cv::Vec3b>(m,n)[0];
	//								orig_img.at<cv::Vec3b>(m,n)[1]=testmat.at<cv::Vec3b>(m,n)[1];
	//								orig_img.at<cv::Vec3b>(m,n)[2]=testmat.at<cv::Vec3b>(m,n)[2]+100;
	//							}
	//						}
	//					}
	//					}else{
	//						cv::Mat testmat = cv::Mat::zeros(source_image_size.height,source_image_size.width,CV_8UC3);
	//						cv::warpPerspective(segment_vec[i], testmat, H_vec[i], testmat.size());
	//						for(int n=0;n<source_image_size.width;n++)
	//											{
	//												for(int m=0;m<source_image_size.height;m++)
	//												{
	//													if(testmat.at<cv::Vec3b>(m,n)[0]!=0 && testmat.at<cv::Vec3b>(m,n)[1]!=0 && testmat.at<cv::Vec3b>(m,n)[2]!=0 ){
	//														orig_img.at<cv::Vec3b>(m,n)[0]=testmat.at<cv::Vec3b>(m,n)[0];
	//														orig_img.at<cv::Vec3b>(m,n)[1]=testmat.at<cv::Vec3b>(m,n)[1];
	//														orig_img.at<cv::Vec3b>(m,n)[2]=testmat.at<cv::Vec3b>(m,n)[2]+100;
	//													}
	//												}
	//											}
	//					}
	//		cv::imshow("orig", orig_img);
	//		cv::waitKey(10000);
	//
	//	}


	//	//split and merge orig img
	//	std::vector<cv::Mat> retransformed_segment;
	//	splitandmerge seg_step_two = splitandmerge();
	//	cv::Mat splitwork = orig_img_draw.clone();
	//	seg_step_two.categorize(splitwork, &retransformed_segment, 1);
	//
	//	for(int i=0;i<segment_vec.size();i++)
	//	{
	//		cv::imshow("sm", segment_vec[i]);
	//		cv::waitKey(10000);
	//	}


		int countsegment=0;

		std::cout<<"Split and Merge"<<std::endl;
		////Segment with split and merge
		std::vector<cv::Mat> swap_vec, newvec, newvectest;
		for(size_t i=0; i<segment_vec.size(); i++)
		{
			swap_vec.clear();
			if(segment_vec[i].rows >100 && segment_vec[i].cols>100)
			{
				splitandmerge seg_step_two = splitandmerge();
				if(i != (segment_vec.size()-1))
				{
					seg_step_two.categorize(segment_vec[i], &swap_vec, 1);
				}else{
					seg_step_two.categorize(segment_vec[i], &swap_vec, 2);
				}

			}else{
				swap_vec.push_back(segment_vec[i]);
			}
			countsegment += swap_vec.size();
	//			if(swap_vec.size()>=2)
	//			{
					if(swap_vec_bool[i])
					{
						double swap_val= rect_saved[i].size.width;
						rect_saved[i].size.width = rect_saved[i].size.height;
						rect_saved[i].size.height = swap_val;
						rect_saved[i].angle -= 90.0;
					}

					for(unsigned int pos=0;pos<swap_vec.size();pos++)
					{
						if(optimized[i])
						{
							cv::Mat rotated, M;
							cv::Mat new_segment = cv::Mat::zeros(source_image_size.height,source_image_size.width,CV_8UC3);
							M = cv::getRotationMatrix2D(cv::Point2f(rect_saved[i].size.width/2,rect_saved[i].size.height/2), -angle_saved[i], 1.0);
							cv::warpAffine(swap_vec[pos], new_segment, M, orig_img.size(), cv::INTER_CUBIC);

							int x_off=0,y_off=0;
							x_off= round(rect_saved[i].center.x-(rect_saved[i].size.width/2));
							y_off = round(rect_saved[i].center.y-(rect_saved[i].size.height/2));

							cv::Mat offset_new_segment = cv::Mat::zeros(source_image_size.height,source_image_size.width,CV_8UC3);
							for(int n=0;n<source_image_size.width;n++)
							{
								for(int m=0;m<source_image_size.height;m++)
								{
									if(m+y_off<source_image_size.height && n+x_off<source_image_size.width && m+y_off>=0 && n+x_off>=0 ){
										offset_new_segment.at<cv::Vec3b>(m+y_off,n+x_off)[0]=new_segment.at<cv::Vec3b>(m,n)[0];
										offset_new_segment.at<cv::Vec3b>(m+y_off,n+x_off)[1]=new_segment.at<cv::Vec3b>(m,n)[1];
										offset_new_segment.at<cv::Vec3b>(m+y_off,n+x_off)[2]=new_segment.at<cv::Vec3b>(m,n)[2];
									}
								}
							}

							cv::Mat original_image_plane = cv::Mat::zeros(source_image_size.height,source_image_size.width,CV_8UC3);
							if(H_vec[i].rows==3 && H_vec[i].cols==3){
								cv::warpPerspective(offset_new_segment, original_image_plane, H_vec[i], original_image_plane.size());
							}else{
								original_image_plane = offset_new_segment;
							}

							std::vector <cv::Point> seg_points;
							cv::Mat work_segment = swap_vec[pos];
							for(int pi=0; pi<swap_vec[pos].rows;pi++)
							{
								for(int pj=0; pj<swap_vec[pos].cols;pj++)
								{
									if(work_segment.at<cv::Vec3b>(pi,pj)[0]!=0 || work_segment.at<cv::Vec3b>(pi,pj)[1]!=0 || work_segment.at<cv::Vec3b>(pi,pj)[2]!=0)
									{
										seg_points.push_back(cv::Point(pj,pi));
									}
								}
							}
							cv::RotatedRect rec;
							if(seg_points.size()>3)
							{
								rec =  minAreaRect(seg_points);
							}
							// get angle and size from the bounding box
							float angle = rec.angle;
							cv::Size rect_size = rec.size;
							if (rec.angle < -45.)
							{
								angle += 90.0;
								std::swap(rect_size.width, rect_size.height);
							}
							if(segment_vec[i].cols>100 && segment_vec[i].rows>100 && rec.size.width>0 && rec.size.height>0)
							{
								M = cv::getRotationMatrix2D(rec.center, angle, 1.0);
								cv::warpAffine(work_segment, rotated, M, work_segment.size(), cv::INTER_CUBIC);
								cv::getRectSubPix(rotated, rect_size, rec.center, new_segment);
								newvectest.push_back(new_segment);

							}else{
								newvectest.push_back(swap_vec[pos]);
							}

							newvec.push_back(swap_vec[pos]);
							retransformed_segment.push_back(original_image_plane);

						}else{
							cv::Mat transform = cv::Mat::zeros(source_image_size.height,source_image_size.width,CV_8UC3);
							if(H_vec[i].rows==3 && H_vec[i].cols==3)
							{
								cv::warpPerspective(swap_vec[i], transform, H_vec[i], transform.size());
								retransformed_segment.push_back(transform);
							}else{
								retransformed_segment.push_back(swap_vec[pos]);
							}
							newvec.push_back(swap_vec[pos]);

						}
					}
	//			}else
	//			{
	//				newvec.push_back(segment_vec[i]);
	//				if(optimized[i])
	//				{
	//					cv::Mat transform;
	//					if(H_vec[i].rows==3 && H_vec[i].cols==3)
	//					{
	//						cv::warpPerspective(segment_vec[i], transform, H_vec[i], orig_img.size());
	//						retransformed_segment.push_back(transform);
	//					}
	//				}else
	//				{
	//					retransformed_segment.push_back(segment_vec[i]);
	//				}
	//
	//				std::cout<<"not transformed 1"<<std::endl;
	//			}
	//		}else{
	//			newvec.push_back(segment_vec[i]);
	//			if(optimized[i])
	//			{
	//				cv::Mat transform;
	//				if(H_vec[i].rows==3 && H_vec[i].cols==3)
	//				{
	//					cv::warpPerspective(segment_vec[i], transform, H_vec[i], orig_img.size());
	//					retransformed_segment.push_back(transform);
	//				}
	//			}else
	//			{
	//				retransformed_segment.push_back(segment_vec[i]);
	//			}
	//			std::cout<<"not transformed 2"<<std::endl;
	//		}
		}

		std::cout<<"countsegment: "<<countsegment << "\tretransseg: "<< retransformed_segment.size()<<"\tnewsize: "<<newvec.size()<<"\tsegment_vec size: "<<segment_vec.size()<<std::endl;

		// visualisation of 2d segmentation
		if (display_2d_segments == true)
		{
			segmentation_2d = cv::Mat::zeros(orig_img_draw.rows, orig_img_draw.cols, CV_8UC3);
			for(size_t i=0;i<retransformed_segment.size();i++)
			{
				int r=0,b=0,g=0;
				if(i%3==0)
					r = rand() % 50 + 100;
				if(i%3==1)
					b = rand() % 50 + 100;
				if(i%3==2)
					g = rand() % 50 + 100;
				cv::Vec3b segment_color(rand()%255, rand()%255, rand()%255);

				for(int m=0;m<source_image_size.height;m++)
				{
					for(int n=0;n<source_image_size.width;n++)
					{
						if(retransformed_segment[i].at<cv::Vec3b>(m,n)[0]!=0 || retransformed_segment[i].at<cv::Vec3b>(m,n)[1]!=0 || retransformed_segment[i].at<cv::Vec3b>(m,n)[2]!=0 )
						{
							orig_img_draw.at<cv::Vec3b>(m,n)[0]= retransformed_segment[i].at<cv::Vec3b>(m,n)[0]+r;
							orig_img_draw.at<cv::Vec3b>(m,n)[1]= retransformed_segment[i].at<cv::Vec3b>(m,n)[1]+g;
							orig_img_draw.at<cv::Vec3b>(m,n)[2]= retransformed_segment[i].at<cv::Vec3b>(m,n)[2]+b;
							segmentation_2d.at<cv::Vec3b>(m,n) = segment_color;
						}
					}
				}
				cv::imshow("Segments",orig_img_draw);
		//		if(newvec.size()>i)
		//		{
		//			imshow("newvec", newvec[i]);
		//		}
		//		if(newvectest.size()>i)
		//		{
		//			imshow("newvectest", newvectest[i]);
		//		}
		//		if(segment_vec.size()>i)
		//		{
					//imshow("segment_vec", retransformed_segment[i]);
					std::ostringstream outStream;
					outStream << i;
					std::string num;
					num  = outStream.str();
					//std::string filename = "segmentation/transformed" +num+".png";
					//cv::imwrite(filename, retransformed_segment[i]);
		//		}
				cv::waitKey(10);
			}
		}
		std::cout<<"Segmentation done"<<std::endl;
		//cv::imwrite("/home/rmb-dh/evaluation/Segmented.jpg", orig_img_draw );
		//cv::imwrite("segmentation/segmented_original_image.png", orig_img_draw );

		// create smaller patches for retransformed_segment
		std::vector<cv::Mat> retransformed_segment_cut(retransformed_segment.size());
		for (size_t i=0; i<retransformed_segment.size(); ++i)
		{
			cv::Rect bounding_box(retransformed_segment[i].cols, retransformed_segment[i].rows, 0, 0);  //min x,y ; max x,y
			for (int r=0; r<retransformed_segment[i].rows; ++r)
				for (int c=0; c<retransformed_segment[i].cols; ++c)
				{
					cv::Vec3b val = retransformed_segment[i].at<cv::Vec3b>(r,c);
					if (val != cv::Vec3b(0,0,0))
					{
						bounding_box.x = std::min(bounding_box.x, c);
						bounding_box.width = std::max(bounding_box.width, c);
						bounding_box.y = std::min(bounding_box.y, r);
						bounding_box.height = std::max(bounding_box.height, r);
					}
				}
			bounding_box.width -= bounding_box.x;
			bounding_box.height -= bounding_box.y;
			retransformed_segment_cut[i] = retransformed_segment[i](bounding_box);
	//		cv::imshow("cut", retransformed_segment[i]);
	//		cv::waitKey();
		}
		for(size_t i=0;i<retransformed_segment_cut.size();i++)
		{
			std::ostringstream outStream;
			outStream << i;
			//std::string filename = "segmentation/segments_2d_" +outStream.str()+".png";
			//cv::imwrite(filename, retransformed_segment_cut[i] );
		}
	}
	else
	{
		// segmentation just takes the 3d segments received
		for(size_t i=0; i<segmented_pointcloud_msg.clusters.size(); ++i)
		{
			size_t cluster_size = segmented_pointcloud_msg.clusters[i].array.size();
			if(cluster_size > 1500)//750
			{
				// create color image of segment
				cv::Mat segment_img = cv::Mat::zeros(source_image_size.height, source_image_size.width, CV_8UC3);
				pcl::IndicesPtr indices_ptr(new std::vector<int>(cluster_size));
				for(size_t j=0; j<cluster_size; ++j)
				{
					int point_index = segmented_pointcloud_msg.clusters[i].array[j];
					int x = point_index%source_image_size.width;
					int y = point_index/source_image_size.width;

					(*indices_ptr)[j] = point_index;

					pcl::PointXYZRGB& point = cloud->points[point_index];
					segment_img.at<cv::Vec3b>(y,x) = cv::Vec3b(point.b, point.g, point.r);
				}

				// find contours and bounding box
				cv::Mat gray_img;
				cv::cvtColor(segment_img, gray_img, CV_BGR2GRAY);
				std::vector<std::vector<cv::Point> > contours;
				std::vector<cv::Vec4i> hierarchy;
				cv::findContours(gray_img, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
				std::vector<cv::Point> all_segment_points;
				for (size_t k=0; k<contours.size(); ++k)
					all_segment_points.insert(all_segment_points.end(), contours[k].begin(), contours[k].end());
				cv::Rect bounding_box;
				if (all_segment_points.size()>0)
					bounding_box = cv::boundingRect(all_segment_points);

				// only accept compact segments
				if (cluster_size > 0.2*bounding_box.area())
				{
					cv::drawContours(orig_img, contours, -1, CV_RGB(0,0,255), 2, 8, hierarchy);
					retransformed_segment.push_back(segment_img);
					//cv::Mat seg = segment_img(bounding_box);

					// normalize the viewpoint and scale resolution
					PerspectiveTransformation p_transform;
					cv::Mat H;
					std::vector<float> plane_coeff;
					const double normalized_resolution = 1000.;
					cv::Mat seg = segment_img.clone();
					p_transform.normalize_perspective(seg, cloud, plane_coeff, H, normalized_resolution, indices_ptr);
					segment_vec.push_back(seg);
				}
			}
		}
	}

	//Get Position of Segments for naming
	std::vector<cv::Point> segment_center(retransformed_segment.size());
	//std::vector< std::vector <cv::Point> > seg_points(retransformed_segment.size());
	for(size_t i=0; i<retransformed_segment.size();i++)
	{
		double mean_x=0., mean_y=0.;
		int count = 0;
		for(int m=0;m<retransformed_segment[i].rows;m++)
		{
			for(int n=0;n<retransformed_segment[i].cols;n++)
			{
				cv::Vec3b& val = retransformed_segment[i].at<cv::Vec3b>(m,n);
				if(val[0]!=0 || val[1]!=0 || val[2]!=0)
				{
					//seg_points[i].push_back(cv::Point(n,m));
					mean_x += n;
					mean_y += m;
					count++;
				}
			}
		}
		segment_center[i] = cv::Point2f(mean_x/(double)count, mean_y/(double)count);
//		cv::RotatedRect rec;
//		if(seg_points[i].size()>3)
//		{
//			rec =  minAreaRect(seg_points[i]);
//			if(rec.size.height>30 && rec.size.width>30)
//				segment_center[i] = cv::Point2f(rec.center.x, rec.center.y);
//		}
	}


	// Compute Features of Segments
	std::vector<cv::Mat>& images = segment_vec; //retransformed_segment_cut;		// segment_vec;

//	double t1 = 20;
//	double t2 = 60;
//	int aperture_size = 3;
//	cv::Mat edges, image_gray;
//	cv::cvtColor(orig_img, image_gray, CV_BGR2GRAY);
//	uchar key = 0;
//	while (key != 'q')
//	{
//		cv::Canny(image_gray, edges, t1, t2, aperture_size);
//		cv::imshow("canny", edges);
//		std::cout << "t1=" << t1 << "\tt2=" << t2 << "\taperture_size=" << aperture_size << std::endl;
//		key = cv::waitKey();
//		if (key=='v')
//			t1 = std::max(10., std::min(300., t1-10.));
//		if (key=='b')
//			t1 = std::max(10., std::min(300., t1+10.));
//		if (key=='n')
//			t2 = std::max(10., std::min(300., t2-10.));
//		if (key=='m')
//			t2 = std::max(10., std::min(300., t2+10.));
//		if (key=='x')
//			aperture_size = std::max(3, std::min(31, aperture_size-2));
//		if (key=='c')
//			aperture_size = std::max(3, std::min(31, aperture_size+2));
//	}

	// todo: choose

	//    a) handcrafted
	std::cout<<"Compute features"<<std::endl;
	std::vector<struct feature_results> segment_features;
	struct feature_results results;
	for(unsigned int i=0;i<images.size();i++)
	{
		//imwrite( "/home/rmb-dh/Pictures/features.jpg", segment_vec[i] );
		cv::Mat img_seg = images[i];
		color_parameter color = color_parameter();
		color.get_color_parameter_new(img_seg, &results);
		texture_features textur = texture_features();
		cv::Mat dummy(1,100,CV_32F);
		textur.compute_texture_features(img_seg, results, &dummy);
		segment_features.push_back(results);
	}
	// Create attribute matrix for classification
	cv::Mat base_attribute_mat = cv::Mat::zeros(segment_features.size(), 17, CV_32FC1);
	for(unsigned int sample_index=0;sample_index<segment_features.size();sample_index++)
	{
		results = segment_features[sample_index];
		base_attribute_mat.at<float>(sample_index, 0) = results.colorfulness; // 3: colorfulness
		base_attribute_mat.at<float>(sample_index, 1) = results.dom_color; // 4: dominant color
		base_attribute_mat.at<float>(sample_index, 2) = results.dom_color2; // 5: dominant color2
		base_attribute_mat.at<float>(sample_index, 3) = results.v_mean; //6: v_mean
		base_attribute_mat.at<float>(sample_index, 4) = results.v_std; // 7: v_std
		base_attribute_mat.at<float>(sample_index, 5) = results.s_mean; // 8: s_mean
		base_attribute_mat.at<float>(sample_index, 6) = results.s_std; // 9: s_std
		base_attribute_mat.at<float>(sample_index, 7) = results.avg_size; // 10: average primitive size
		base_attribute_mat.at<float>(sample_index, 8) = results.prim_num; // 11: number of primitives
		base_attribute_mat.at<float>(sample_index, 9) = results.prim_strength; // 12: strength of primitives
		base_attribute_mat.at<float>(sample_index, 10) = results.prim_regularity; // 13: regularity of primitives
		base_attribute_mat.at<float>(sample_index, 11) = results.contrast; // 14: contrast:
		base_attribute_mat.at<float>(sample_index, 12) = results.line_likeness; // 15: line-likeness
		//	Nicht implementiert	    	feature_mat.at<float>(count,13) = results.roughness; // 16: 3D roughness
		base_attribute_mat.at<float>(sample_index, 13) = results.roughness;
		base_attribute_mat.at<float>(sample_index, 14) = results.direct_reg; // 17: directionality/regularity
		base_attribute_mat.at<float>(sample_index, 15) = results.lined; // 18: lined
		base_attribute_mat.at<float>(sample_index, 16) = results.checked; // 19: checked
	}
	//       compute attributes
	cv::Mat attribute_mat = base_attribute_mat;
	//al_.predict(base_attribute_mat, attribute_mat);

//	//    b) CIMPOI
//	//       load base features
//	const IfvFeatures::FeatureType feature_type = IfvFeatures::RGB_PATCHES;
//	const int number_gaussian_centers = 256;
//	const double image_resize_factor = 1.0;
//	cv::Mat feature_mat = cv::Mat::zeros(images.size(), 2*ifv_.getFeatureDimension(feature_type)*number_gaussian_centers, CV_32FC1);
//	for(unsigned int i=0;i<images.size();i++)
//	{
//		cv::Mat base_feature = feature_mat.row(i);
//		ifv_.computeImprovedFisherVector(images[i], image_resize_factor, number_gaussian_centers, base_feature, feature_type);
//	}
//	//       compute attributes
//	cv::Mat attribute_mat;
//	al_.predict(feature_mat, attribute_mat);

	// display attribute_mat
	for (int i=1; i<18; ++i)
		std::cout << "\t" << i;
	std::cout << std::endl;
	for (int r=0; r<attribute_mat.rows; ++r)
	{
		std::cout << r << ":\t" << std::setprecision(2);
		for (int c=0; c<attribute_mat.cols; ++c)
			std::cout << attribute_mat.at<float>(r,c) << "\t";
		std::cout << r << std::setprecision(5) << std::endl;
	}


	//	Run classification with SVM
	std::cout<<"Run classification"<<std::endl;
	cv::Mat prediction_results;
//	CvSVM SVM;
//	SVM.load("/home/rmb-dh/datasetTextur/yamlfiles/svm.yml", "svm");
//	SVM.predict(feature_mat,prediction_results);
	cv::Mat labels = cv::Mat::zeros(attribute_mat.rows, 1, CV_32FC1);
	ml_.predict(attribute_mat, labels, prediction_results);

	////Write Segment type
	create_train_data get_classes = create_train_data();
	std::vector<std::string> classes; //= get_classes.get_texture_classes(); Chives, Chocolate, Grapes, Kiwi, Lemon, Lime, Pineapple, Smarties, Tiles, Tomato,
	classes.push_back("Chives"); classes.push_back("Chocolate"); classes.push_back("Grapes"); classes.push_back("Kiwi"); classes.push_back("Lemon"); classes.push_back("Lime"); classes.push_back("Smarties"); classes.push_back("Tiles"); classes.push_back("Tomato"); classes.push_back("Varnished"); classes.push_back("Wood");
	std::string s;
	for(int i=0;i<prediction_results.rows;i++)
	{
		std::stringstream ss;
		ss << i << ". ";
		s.clear();
		s = ss.str() + classes[prediction_results.at<float>(i,0)];
		std::cout << s << std::endl;
		if(segment_center[i].x!=0 && segment_center[i].y!=0)
		{
			putText(orig_img, s, segment_center[i], cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
			if (display_original_3d_segments == true)
				putText(segmentation_3d, s, segment_center[i], cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
			if (display_2d_segments == true)
			{
				putText(segmentation_2d,s, segment_center[i], cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
				putText(orig_img_draw, s, segment_center[i], cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(0,0,255), 1, CV_AA);
			}
		}
	}
	cv::imshow("orig", orig_img);
	if (display_original_3d_segments == true)
		cv::imshow("seg 3d", segmentation_3d);
	if (display_2d_segments == true)
	{
		cv::imshow("seg 2d", segmentation_2d);
		cv::imshow("orig no lines", orig_img_draw);
	}
	//cv::imshow("seg2", test);

	std::cout << "Processing time: " << timer.getElapsedTimeInMilliSec() << "ms" << std::endl;
	cv::waitKey();
}

/*
void TextCategorizationNode::inputCallbackNoCam()
{


	//Test Split and merge
	std::vector<cv::Mat> swap_vec;
	cv::Mat test1 = cv::imread("/home/rmb-dh/obst.jpg"); //TEST

	splitandmerge seg_step_two = splitandmerge();
	seg_step_two.categorize(test1, &swap_vec, 0);


	for(unsigned int ne=0;ne<swap_vec.size();ne++)
	{
		cv::imshow("orig",swap_vec[ne]);
		cv::waitKey(10000);
	}


	//	cv::imshow("swap", swap_vec[j]);
	//	cv::waitKey(100000);




	//Computes trainingdata for training of klassification method. uses texture database
	//Saves data in file to hardcoded path

//	std::string path_traindata = "/media/SAMSUNG/rmb/datasetTextur/A_Klassification_Data/train_data/";			//Pfad zu Trainingsdaten
//	std::string path_testdata = "/media/SAMSUNG/rmb/datasetTextur/A_Klassification_Data/test_data/";			//Pfad zu Testdaten
	std::string path_database = "/media/SAMSUNG/rmb/datasetTextur/texture_database/";							// path to database
	std::string path_save_location = "/media/SAMSUNG/rmb/datasetTextur/data/handcrafted/";		//Pfad zu Speicherort der Featurevektoren

//	create_train_data testdata = create_train_data();									// Berechnet den Featurevektor und den einen Labelvektor zum Testen
//	testdata.compute_data(path_testdata, path_save_location, 146, 2);
//
//	create_train_data trainingdata = create_train_data();									// Berechnet den Featurevektor und den einen Labelvektor zum Trainieren
//	trainingdata.compute_data(path_traindata, path_save_location, 1135, 1);

//	create_train_data database_data = create_train_data();									// computes feature and label matrices of the provided database
//	database_data.compute_data(path_database, path_save_location, 1281);

	//Train and predict with NN
//	train_ml ml;
	//double gam =0;																		// Trainiert anhand des Trainingsvektors, testet anhand des Testvektors und gibt Ergebnis aus
	//ml.run_ml(gam, &path_save_location);
//	cv::Mat base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix;
//	create_train_data::DataHierarchyType data_hierarchy;
//	create_train_data database_data;
//	database_data.load_texture_database_features(path_save_location, base_feature_matrix, ground_truth_attribute_matrix, computed_attribute_matrix, class_label_matrix, data_hierarchy);
//	ml.cross_validation(10, computed_attribute_matrix, class_label_matrix, data_hierarchy);


	//Train and predict with SVM
																							//Einfaches Training und Auswertung mit SVM
//			train_svm traintest = train_svm();
//			traintest.run_training(&path_traindata,&path_testdata, gam, 0, &path_save_location);
//
//			predict_svm prediction = predict_svm();
//			prediction.run_prediction(&path_save_location);




		//	TEST SVM																		//Training und Auswertung der SVM mit unterschiedlichen Gamma werten
//		for(double gam = 0.01; gam<=5; gam=gam+0.2)
//		{
//		//	for(double val = 0.1; val<=10;val=val+0.5)
//		//	{
//			std::string data = "/home/rmb-dh/Test_dataset/training_data.yml";
//			std::string label = "/home/rmb-dh/Test_dataset/train_data_respons.yml";
//			train_svm traintest = train_svm();
//			traintest.run_training(&path_traindata,&path_testdata, gam, 0, &path_save_location);
//
//			predict_svm prediction = predict_svm();
//			prediction.run_prediction(&path_save_location);
//		//	}
//		}



//-----------------------------------Ab hier alter Code -----------------------------------

//	ROS_INFO("Input Callback No Cam");
//	compute_textures test = compute_textures();
//	test.compute_textures_all();
//	test.compute_textures_one();
//	test.compute_test_data();

//	cv::Mat image = cv::imread("/home/rmb-dh/datasetTextur/Texture_database/Wood/Wood_10_01.JPG");
//	cv::Mat image = cv::imread("/home/rmb-dh/datasetTextur/Texture_database/Cracker/Cracker_06_01.JPG");
//	cv::Mat image = cv::imread("/home/rmb-dh/datasetTextur/Texture_database/Styrofoam/Styrofoam_04_01.JPG");
//
//		struct feature_results results;
//	    		struct color_vals color_results;
//	    		color_parameter color = color_parameter();
//	    		color.get_color_parameter(image, &results);
//
//	    		std::cout<<results.colorfulness<<"colorfullness"<<std::endl;
//	    		std::cout<<results.dom_color<<"dom color"<<std::endl;
//	    		std::cout<<results.dom_color2<<"dom color2"<<std::endl;
//
//	    		cv::waitKey(100);











////	TEST K-NEIGHBOR
//	for(double gam = 5; gam<=5; gam=gam+1)
//	{
//	train_kneighbor kn = train_kneighbor();
//	kn.run_training(gam);
//	std::cout<< "Used val:"<<gam<<std::endl;
//}

//	cv::waitKey(10);
}
*/

/// callback for the incoming  data stream
void TextCategorizationNode::inputCallback(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
	ROS_INFO("Input Callback");
	// convert color image to cv::Mat
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);
}

/// Converts a color image message to cv::Mat format.
bool TextCategorizationNode::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
{
	try
	{
		image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ObjectCategorization: cv_bridge exception: %s", e.what());
		return false;
	}
	image = image_ptr->image;

	return true;
}

unsigned long TextCategorizationNode::ProjectXYZ(double x, double y, double z, int& u, int& v)
{
	cv::Mat XYZ(4, 1, CV_64FC1);
	cv::Mat UVW(3, 1, CV_64FC1);

	x *= 1000;
	y *= 1000;
	z *= 1000;

	double* d_ptr = XYZ.ptr<double>(0);
	d_ptr[0] = x;
	d_ptr[1] = y;
	d_ptr[2] = z;
	d_ptr[3] = 1.;

	UVW = color_camera_matrix_ * XYZ;

	d_ptr = UVW.ptr<double>(0);
	double du = d_ptr[0];
	double dv = d_ptr[1];
	double dw = d_ptr[2];

	u = cvRound(du/dw);
	v = cvRound(dv/dw);

	return 1;
}

void TextCategorizationNode::calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg)
{
	if (camera_matrix_received_ == false)
	{
		//	pointcloud_height_ = calibration_msg->height;
		//	pointcloud_width_ = calibration_msg->width;
		cv::Mat temp(3,4,CV_64FC1);
		for (int i=0; i<12; i++)
			temp.at<double>(i/4,i%4) = calibration_msg->P.at(i);
		//		std::cout << "projection_matrix: [";
		//		for (int v=0; v<3; v++)
		//			for (int u=0; u<4; u++)
		//				std::cout << temp.at<double>(v,u) << " ";
		//		std::cout << "]" << std::endl;
		color_camera_matrix_ = temp;
		camera_matrix_received_ = true;
	}
}


int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "texture_categorization");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of Object
	TextCategorizationNode texture_categorization(nh);
	texture_categorization.init();

//	ros::Subscriber segmented_pointcloud_;
//	segmented_pointcloud_ = nh.subscribe("/surface_classification/segmented_pointcloud", 1, TextCategorizationNode::segmentationCallback);

	ros::spin();

	return (0);
}
