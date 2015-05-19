#include "cob_texture_categorization/attribute_learning.h"

#include <fstream>
#include "stdint.h"

#include "ml.h"
#include "highgui.h"


void AttributeLearning::loadTextureDatabaseBaseFeatures(const std::string& filename, const int feature_number, cv::Mat& feature_matrix, cv::Mat& ground_truth_attribute_matrix, cv::Mat& class_label_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy, const std::string& database_identifier)
{
	// load feature vectors and corresponding labels computed on database and class-object-sample hierarchy
	//const int attribute_number = 17;			// label = attributes
	//const int feature_number = 9688;		// feature = base feature
	//const int total_sample_number = 1281;

	int attribute_number = 0;
	if (database_identifier.compare("ipa") == 0)
		attribute_number = 17;
	else if (database_identifier.compare("dtd") == 0)
		attribute_number = 47;
	else
		std::cout << "Error: create_train_data::load_filenames_gt_attributes: unsupported mode selected.";

	feature_matrix = cv::Mat();		//create(total_sample_number, feature_number, CV_32FC1);
	ground_truth_attribute_matrix = cv::Mat();		//.create(total_sample_number, attribute_number, CV_32FC1);
	class_label_matrix = cv::Mat();		//.create(total_sample_number, 1, CV_32FC1);
	int sample_index = 0;
	std::ifstream file(filename.c_str(), std::ios::in);
	if (file.is_open() == true)
	{
		unsigned int class_number = 0;
		file >> class_number;
		data_sample_hierarchy.resize(class_number);
		for (unsigned int i=0; i<class_number; ++i)
		{
			std::string class_name;
			file >> class_name;
			unsigned int object_number=0;
			file >> object_number;
			data_sample_hierarchy[i].resize(object_number);
			for (unsigned int j=0; j<object_number; ++j)
			{
				unsigned int sample_number=0;
				file >> sample_number;
				data_sample_hierarchy[i][j].resize(sample_number);
				cv::Mat feature_matrix_row(1, feature_number, CV_32FC1);
				cv::Mat ground_truth_attribute_matrix_row(1, attribute_number, CV_32FC1);
				cv::Mat class_label_matrix_row(1, 1, CV_32FC1);
				for (unsigned int k=0; k<sample_number; ++k)
				{
					// filename
					std::string image_filename;
					file >> image_filename;
					// read data
					while (true)
					{
						std::string tag;
						file >> tag;
						if (database_identifier.compare("ipa") == 0)
						{
							if (tag.compare("labels_ipa17:") == 0)
							{
								for (int l=0; l<attribute_number; ++l)
									file >> ground_truth_attribute_matrix_row.at<float>(0, l);	// ground truth attribute vector
							}
							else if (tag.compare("base_farhadi9688:") == 0)
							{
								for (int f=0; f<feature_number; ++f)
									file >> feature_matrix_row.at<float>(0, f);		// base feature vector
								break;
							}
							else
							{
								std::cout << "Error: create_train_data::load_filenames_gt_attributes: Unknown tag found." << std::endl;
								getchar();
								break;
							}
						}
						else if (database_identifier.compare("dtd") == 0)
						{
							if (tag.compare("labels_cimpoi47:") == 0)
							{
								for (int l=0; l<attribute_number; ++l)
									file >> ground_truth_attribute_matrix_row.at<float>(0, l);	// ground truth attribute vector
								break;
							}
							else
							{
								std::cout << "Error: AttributeLearning::loadTextureDatabaseBaseFeatures: Unknown tag found." << std::endl;
								getchar();
								break;
							}
						}
						else
							std::cout << "Error: create_train_data::load_filenames_gt_attributes: unsupported mode selected.";
					}
					class_label_matrix_row.at<float>(0, 0) = (float)i;
					data_sample_hierarchy[i][j][k] = sample_index;				// class label (index)
					feature_matrix.push_back(feature_matrix_row);
					ground_truth_attribute_matrix.push_back(ground_truth_attribute_matrix_row);
					class_label_matrix.push_back(class_label_matrix_row);
					++sample_index;
				}
			}
		}
	}
	else
	{
		std::cout << "Error: could not open file " << filename << std::endl;
	}

	std::cout << "feature_matrix: " << feature_matrix.rows << ", " << feature_matrix.cols << "\tground_truth_attribute_matrix: " << ground_truth_attribute_matrix.rows << ", " << ground_truth_attribute_matrix.cols << "\tclass_label_matrix: " << class_label_matrix.rows << ", " << class_label_matrix.cols << std::endl;

	file.close();
}


void AttributeLearning::loadTextureDatabaseLabeledAttributeFeatures(std::string filename, cv::Mat& ground_truth_attribute_matrix, cv::Mat& class_label_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy)
{
	// load attribute vectors and corresponding class labels computed on database and class-object-sample hierarchy
	const int label_number = 1;		// label = class label
	const int attribute_number = 17;	// feature = attribute
	const int total_sample_number = 1281;
	ground_truth_attribute_matrix.create(total_sample_number, attribute_number, CV_32FC1);
	class_label_matrix.create(total_sample_number, label_number, CV_32FC1);
	int sample_index = 0;
	std::ifstream file(filename.c_str(), std::ios::in);
	if (file.is_open() == true)
	{
		unsigned int class_number = 0;
		file >> class_number;
		data_sample_hierarchy.resize(class_number);
		for (unsigned int i=0; i<class_number; ++i)
		{
			std::string class_name;
			file >> class_name;
			unsigned int object_number=0;
			file >> object_number;
			data_sample_hierarchy[i].resize(object_number);
			for (unsigned int j=0; j<object_number; ++j)
			{
				unsigned int sample_number=0;
				file >> sample_number;
				data_sample_hierarchy[i][j].resize(sample_number);
				for (unsigned int k=0; k<sample_number; ++k)
				{
					class_label_matrix.at<float>(sample_index, 0) = i;
					for (int f=0; f<attribute_number; ++f)
						file >> ground_truth_attribute_matrix.at<float>(sample_index, f);
					file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');		// go to next line with base features
					file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');		// skip line with base features
					data_sample_hierarchy[i][j][k] = sample_index;
					++sample_index;
				}
			}
		}
	}
	else
	{
		std::cout << "Error: could not open file " << filename << std::endl;
	}
	file.close();
}

void AttributeLearning::saveAttributeCrossValidationData(std::string path, const std::vector< std::vector<int> >& preselected_train_indices, const std::vector<cv::Mat>& attribute_matrix_test_data, const std::vector<cv::Mat>& class_label_matrix_test_data, const std::vector<cv::Mat>& computed_attribute_matrices)
{
	// convert nested vector to vector of cv::Mat
	std::vector<cv::Mat> preselected_train_indices_mat(preselected_train_indices.size());
	for (unsigned int i=0; i<preselected_train_indices.size(); ++i)
	{
		preselected_train_indices_mat[i].create(preselected_train_indices[i].size(), 1, CV_32SC1);
		for (unsigned int j=0; j<preselected_train_indices[i].size(); ++j)
			preselected_train_indices_mat[i].at<int>(j,0) = preselected_train_indices[i][j];
	}

	// save data;
	std::string data = "ipa_database_attribute_cv_data.yml";
	std::string path_data = path + data;
	std::cout << "Saving data to file " << path_data << " ... ";
	cv::FileStorage fs(path_data, cv::FileStorage::WRITE);
	if (fs.isOpened() == true)
	{
		fs << "preselected_train_indices" << preselected_train_indices_mat;
		fs << "attribute_matrix_test_data" << attribute_matrix_test_data;
		fs << "class_label_matrix_test_data" << class_label_matrix_test_data;
		fs << "computed_attribute_matrices" << computed_attribute_matrices;
	}
	else
		std::cout << "Error: could not open file '" << path_data << "' for writing."<< std::endl;
	fs.release();
	std::cout << "done." << std::endl;
}

void AttributeLearning::loadAttributeCrossValidationData(std::string path, std::vector< std::vector<int> >& preselected_train_indices, std::vector<cv::Mat>& attribute_matrix_test_data, std::vector<cv::Mat>& class_label_matrix_test_data, std::vector<cv::Mat>& computed_attribute_matrices)
{
	// load data;
	std::vector<cv::Mat> preselected_train_indices_mat;
	std::string data = "ipa_database_attribute_cv_data.yml";
	std::string path_data = path + data;
	std::cout << "Loading data from file " << path_data << " ... ";
	cv::FileStorage fs(path_data, cv::FileStorage::READ);
	if (fs.isOpened() == true)
	{
		fs["preselected_train_indices"] >> preselected_train_indices_mat;
		fs["attribute_matrix_test_data"] >> attribute_matrix_test_data;
		fs["class_label_matrix_test_data"] >> class_label_matrix_test_data;
		fs["computed_attribute_matrices"] >> computed_attribute_matrices;
	}
	else
		std::cout << "Error: could not open file '" << path_data << "' for reading."<< std::endl;
	fs.release();

	// convert nested vector to vector of cv::Mat
	preselected_train_indices.resize(preselected_train_indices_mat.size());
	for (unsigned int i=0; i<preselected_train_indices_mat.size(); ++i)
	{
		preselected_train_indices[i].resize(preselected_train_indices_mat[i].rows);
		for (int j=0; j<preselected_train_indices_mat[i].rows; ++j)
			preselected_train_indices[i][j] = preselected_train_indices_mat[i].at<int>(j,0);
	}

	std::cout << "done." << std::endl;
}


void AttributeLearning::crossValidation(const CrossValidationParams& cross_validation_params, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy)
{
	cv::Mat class_label_matrix;
	std::vector< std::vector<int> > preselected_train_indices;
	std::vector<cv::Mat> attribute_matrix_test_data, class_label_matrix_test_data;
	std::vector<cv::Mat> computed_attribute_matrices;
	crossValidation(cross_validation_params, feature_matrix, attribute_matrix, data_sample_hierarchy, false, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, false, computed_attribute_matrices);
}

void AttributeLearning::crossValidation(const CrossValidationParams& cross_validation_params, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, std::vector<cv::Mat>& computed_attribute_matrices)
{
	cv::Mat class_label_matrix;
	std::vector< std::vector<int> > preselected_train_indices;
	std::vector<cv::Mat> attribute_matrix_test_data, class_label_matrix_test_data;
	crossValidation(cross_validation_params, feature_matrix, attribute_matrix, data_sample_hierarchy, false, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, true, computed_attribute_matrices);
}

void AttributeLearning::crossValidation(const CrossValidationParams& cross_validation_params, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy,
		bool return_set_data, const cv::Mat& class_label_matrix, std::vector< std::vector<int> >& preselected_train_indices, std::vector<cv::Mat>& attribute_matrix_test_data, std::vector<cv::Mat>& class_label_matrix_test_data,
		bool return_computed_attribute_matrices, std::vector<cv::Mat>& computed_attribute_matrices)
{
	// iterate over (possibly multiple) machine learning techniques or configurations
	for (size_t ml_configuration_index = 0; ml_configuration_index<cross_validation_params.ml_configurations_.size(); ++ml_configuration_index)
	{
		const MLParams& ml_params = cross_validation_params.ml_configurations_[ml_configuration_index];

		// vectors for evaluation data, each vector entry is meant to count the statistics for one attribute
		std::vector<double> sumAbsErrors(attribute_matrix.cols, 0.0);
		std::vector< std::vector<double> > absErrors(attribute_matrix.cols);	// first index = attribute index
		std::vector<int> numberSamples(attribute_matrix.cols, 0);
		std::vector<int> below05(attribute_matrix.cols, 0), below1(attribute_matrix.cols, 0);	// number of attributes estimated with less absolute error than 0.5, 1.0
		std::stringstream screen_output, output_summary;

		if (return_set_data == true)
		{
			preselected_train_indices.resize(cross_validation_params.folds_);
			attribute_matrix_test_data.resize(cross_validation_params.folds_);
			class_label_matrix_test_data.resize(cross_validation_params.folds_);
		}

		if (return_computed_attribute_matrices == true)
		{
			computed_attribute_matrices.resize(cross_validation_params.folds_);
			for (size_t i=0; i<computed_attribute_matrices.size(); ++i)
				computed_attribute_matrices[i].create(attribute_matrix.rows, attribute_matrix.cols, attribute_matrix.type());
		}

		create_train_data data_object;
		std::vector<std::string> texture_classes = data_object.get_texture_classes();
		std::cout << "\n\n" << cross_validation_params.configurationToString() << std::endl << ml_params.configurationToString() << std::endl;

		srand(0);	// random seed --> keep reproducible
		for (unsigned int fold=0; fold<cross_validation_params.folds_; ++fold)
		{
			std::cout << "\n=== fold " << fold+1 << " ===" << std::endl;		screen_output << "=== fold " << fold+1 << " ===" << std::endl;

			// === distribute data into training and test set ===
			std::vector<int> train_indices, test_indices;
			if (cross_validation_params.cross_validation_mode_ == CrossValidationParams::LEAVE_OUT_ONE_OBJECT_PER_CLASS)
			{
				// select one object per class for testing
				for (unsigned int class_index=0; class_index<data_sample_hierarchy.size(); ++class_index)
				{
					int object_number = data_sample_hierarchy[class_index].size();
					int test_object = (int)(object_number * (double)rand()/((double)RAND_MAX+1.0));
		//			std::cout << "object_number=" << object_number << "   test_object=" << test_object << std::endl;
					for (int object_index=0; object_index<object_number; ++object_index)
					{
						if (object_index == test_object)
							for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
								{
									test_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
									//std::cout << data_sample_hierarchy[class_index][object_index][s] << "\t";
									screen_output << data_sample_hierarchy[class_index][object_index][s] << "\t";
								}
						else
							for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
								train_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
					}
				}
			}
			else if (cross_validation_params.cross_validation_mode_ == CrossValidationParams::LEAVE_OUT_ONE_CLASS)
			{
				// select one class for testing and train attribute classifiers with remaining classes
				for (unsigned int class_index=0; class_index<data_sample_hierarchy.size(); ++class_index)
				{
					if (class_index == fold)
					{
						int object_number = data_sample_hierarchy[class_index].size();
						for (int object_index=0; object_index<object_number; ++object_index)
							for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
								{
									test_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
									//std::cout << data_sample_hierarchy[class_index][object_index][s] << "\t";
									screen_output << data_sample_hierarchy[class_index][object_index][s] << "\t";
								}
					}
					else
					{
						int object_number = data_sample_hierarchy[class_index].size();
						for (int object_index=0; object_index<object_number; ++object_index)
							for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
								train_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
					}
				}
			}
			else
				std::cout << "Error: chosen cross_validation_mode is unknown." << std::endl;
			//std::cout << std::endl;
			screen_output << std::endl;
			if (return_set_data==true)
				preselected_train_indices[fold] = train_indices;
			assert((int)(test_indices.size() + train_indices.size()) == feature_matrix.rows);

			// create training and test data matrices
			cv::Mat training_data(train_indices.size(), feature_matrix.cols, feature_matrix.type());
			cv::Mat training_labels(train_indices.size(), 1, attribute_matrix.type());
			cv::Mat test_data(test_indices.size(), feature_matrix.cols, feature_matrix.type());
			cv::Mat test_labels(test_indices.size(), 1, attribute_matrix.type());
			for (unsigned int r=0; r<train_indices.size(); ++r)
				for (int c=0; c<feature_matrix.cols; ++c)
					training_data.at<float>(r,c) = feature_matrix.at<float>(train_indices[r],c);
			for (unsigned int r=0; r<test_indices.size(); ++r)
				for (int c=0; c<feature_matrix.cols; ++c)
					test_data.at<float>(r,c) = feature_matrix.at<float>(test_indices[r],c);
			if (return_set_data==true)
			{
				attribute_matrix_test_data[fold].create(test_indices.size(), attribute_matrix.cols, attribute_matrix.type());
				class_label_matrix_test_data[fold].create(test_indices.size(), 1, CV_32FC1);
			}

			// train and evaluate classifier for each attribute with the given training set
			for (int attribute_index=0; attribute_index<attribute_matrix.cols; ++attribute_index)
			{
				std::cout << "--- attribute " << attribute_index+1 << " ---" << std::endl;		screen_output << "--- attribute " << attribute_index+1 << " ---" << std::endl;

				// create training and test label matrices
				const double feature_scaling_factor = (attribute_index==1 || attribute_index==2) ? 12.0 : 5.0;
				for (unsigned int r=0; r<train_indices.size(); ++r)
					training_labels.at<float>(r) = attribute_matrix.at<float>(train_indices[r], attribute_index)/feature_scaling_factor;
				for (unsigned int r=0; r<test_indices.size(); ++r)
					test_labels.at<float>(r) = attribute_matrix.at<float>(test_indices[r], attribute_index)/feature_scaling_factor;

				// === train classifier ===

//				// K-Nearest-Neighbor
//				cv::Mat test;
//				CvKNearest knn(training_data, training_label, cv::Mat(), false, 32);
//				knn.train(training_data,training_label,test,false,32,false );
//
//				cv::Mat result;
//				cv::Mat neighbor_results;
//				cv::Mat dist;
//				knn.find_nearest(train_data, 1, &result,0,&neighbor_results, &dist);
//				// End K-Nearest-Neighbor
//
//				// Randomtree
//				const float regression_accuracy = 0.0001f;		// per tree
//				const float forest_accuracy = 0.01f;		// oob error of forest
//				const int max_tree_depth = 10;
//				const int max_num_of_trees_in_the_forest = 25;
//				const int nactive_vars = 100;	// size of randomly chosen feature subset, best splits are sought within this subset
//				cv::Mat var_type = cv::Mat::ones(1, training_data.cols+1, CV_8UC1) * CV_VAR_NUMERICAL;
//				CvRTParams tree_params(max_tree_depth, (int)(0.01*training_data.rows), regression_accuracy, false, 10, 0, false, nactive_vars, max_num_of_trees_in_the_forest, forest_accuracy, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS);
//				CvRTrees rtree;
//				rtree.train(training_data, CV_ROW_SAMPLE, training_labels, cv::Mat(), cv::Mat(), var_type, cv::Mat(), tree_params);
//				std::cout << "Random forest training finished with " << rtree.get_tree_count() << " trees." << std::endl;
//				// End Randomtree
//
//				// Decision Tree
//				CvDTreeParams params = CvDTreeParams(25, // max depth
//														 5, // min sample count
//														 0, // regression accuracy: N/A here
//														 false, // compute surrogate split, no missing data
//														 15, // max number of categories (use sub-optimal algorithm for larger numbers)
//														 15, // the number of cross-validation folds
//														 false, // use 1SE rule => smaller tree
//														 false, // throw away the pruned tree branches
//														 NULL // the array of priors
//														);
//				CvDTree dtree;
//				dtree.train(training_data, CV_ROW_SAMPLE, training_label, cv::Mat(), cv::Mat(),cv::Mat(),cv::Mat(), params);
//
//				CvDTreeNode *node;
//
//				cv::Mat result(train_data.rows,1,CV_32FC1);
//					for(int i=0;i<train_data.rows;i++)
//					{
//						cv::Mat inputvec(1,train_data.cols, CV_32FC1);
//						for(int j=0;j<train_data.cols;j++)
//						{
//							inputvec.at<float>(0,j)=train_data.at<float>(i,j);
//						}
//						node = dtree.predict(inputvec);
//						result.at<float>(i,0)= (*node).class_idx;
//					}
//				// End Dicision Tree
//
//				// BayesClassifier
//				CvNormalBayesClassifier bayesmod;
//				bayesmod.train(training_data, training_label, cv::Mat(), cv::Mat());
//				cv::Mat result(train_data.rows,1,CV_32FC1);
//				bayesmod.predict(train_data, &result);
//				// End Bayes Classifier
//
//				// Boosting -- not implemented for regression at 28.06.2014
//				CvBoost boost;
//				cv::Mat var_type = cv::Mat::ones(1, training_data.cols+1, CV_8UC1) * CV_VAR_NUMERICAL;
//				CvBoostParams boost_params(CvBoost::GENTLE, 100, 0.95, 1, false, 0);
//				boost.train(training_data, CV_ROW_SAMPLE, training_labels, cv::Mat(), cv::Mat(), var_type, cv::Mat(), boost_params, false);
//				// End Boosting

				CvSVM svm;
				CvANN_MLP mlp;
				if (ml_params.classification_method_ == MLParams::SVM)
				{	// SVM
					svm.train(training_data, training_labels, cv::Mat(), cv::Mat(), ml_params.svm_params_);
				}

				else if (ml_params.classification_method_ == MLParams::NEURAL_NETWORK)
				{	//	Neural Network
					cv::Mat layers = cv::Mat(2+ml_params.nn_hidden_layers_.size(), 1, CV_32SC1);
					layers.row(0) = cv::Scalar(training_data.cols);
					for (size_t k=0; k<ml_params.nn_hidden_layers_.size(); ++k)
						layers.row(k+1) = cv::Scalar(ml_params.nn_hidden_layers_[k]);
					layers.row(ml_params.nn_hidden_layers_.size()+1) = cv::Scalar(1);

					mlp.create(layers, ml_params.nn_activation_function_, ml_params.nn_activation_function_param1_, ml_params.nn_activation_function_param2_);
					int iterations = mlp.train(training_data, training_labels, cv::Mat(), cv::Mat(), ml_params.nn_params_);
					std::cout << "Neural network training completed after " << iterations << " iterations." << std::endl;		screen_output << "Neural network training completed after " << iterations << " iterations." << std::endl;
				}

				// === apply ml classifier to predict test set ===
				double sumAbsError = 0.;
				int numberTestSamples = 0;
				int below05_ctr = 0, below1_ctr = 0;
				for (int r = 0; r < test_data.rows ; ++r)
				{
					cv::Mat response(1, 1, CV_32FC1);
					cv::Mat sample = test_data.row(r);
					if (ml_params.classification_method_ == MLParams::SVM)
						response.at<float>(0,0) = svm.predict(sample);	// SVM
					else if (ml_params.classification_method_ == MLParams::NEURAL_NETWORK)
						mlp.predict(sample, response);		// neural network
					//response.at<float>(0,0) = rtree.predict(sample);		// random tree
					//response.at<float>(0,0) = sample.at<float>(0, attribute_index) / feature_scaling_factor;		// direct relation: feature=attribute

					if (return_set_data == true)
					{
						attribute_matrix_test_data[fold].at<float>(r,attribute_index) = response.at<float>(0,0) * feature_scaling_factor;
						class_label_matrix_test_data[fold].at<float>(r, 0) = class_label_matrix.at<float>(test_indices[r],0);
					}

					float resp = std::max(0.f, response.at<float>(0,0)) * feature_scaling_factor;
					float lab = test_labels.at<float>(r, 0) * feature_scaling_factor;
					float absdiff = fabs(resp - lab);
					if (attribute_index == 1 || attribute_index == 2)
					{
						if (lab == 0.f && resp > 0.5f)
							absdiff = 1.01f;				// if no dominant color was labeled but a color detected, set maximum distance
						else
							absdiff = std::min(absdiff, 1.01f);		// cut distances above 1 to treat color distances equally then (any other wrong color shall be similarly wrong)
					}
					absErrors[attribute_index].push_back(absdiff);
					sumAbsError += absdiff;
					++numberTestSamples;
					if (absdiff < 1.f)
					{
						++below1_ctr;
						if (absdiff < 0.5f)
							++below05_ctr;
					}

					screen_output << "value: " << test_labels.at<float>(r, 0)*feature_scaling_factor << "\t predicted: " << response.at<float>(0,0)*feature_scaling_factor << "\t abs difference: " << absdiff << std::endl;
				}

				sumAbsErrors[attribute_index] += sumAbsError;
				numberSamples[attribute_index] += numberTestSamples;
				below05[attribute_index] += below05_ctr;
				below1[attribute_index] += below1_ctr;
				std::cout << "mean abs error: " << sumAbsError/(double)numberTestSamples << "\t\t<0.5: " << 100*below05_ctr/(double)numberTestSamples << "%\t\t<1.0: " << 100*below1_ctr/(double)numberTestSamples << "%" << std::endl;
				screen_output << "mean abs error: " << sumAbsError/(double)numberTestSamples << "\t\t<0.5: " << 100*below05_ctr/(double)numberTestSamples << "%\t\t<1.0: " << 100*below1_ctr/(double)numberTestSamples << "%" << std::endl;

				if (return_computed_attribute_matrices == true)
				{
					// compute predicted attribute value for each sample from feature_matrix
					for (int sample_index=0; sample_index<feature_matrix.rows; ++sample_index)
					{
						cv::Mat response(1, 1, attribute_matrix.type());
						cv::Mat sample = feature_matrix.row(sample_index);
						if (ml_params.classification_method_ == MLParams::SVM)
							response.at<float>(0,0) = svm.predict(sample);	// SVM
						else if (ml_params.classification_method_ == MLParams::NEURAL_NETWORK)
							mlp.predict(sample, response);		// neural network
						//response.at<float>(0,0) = rtree.predict(sample);		// random tree
						//response.at<float>(0,0) = sample.at<float>(0, attribute_index) / feature_scaling_factor;		// direct relation: feature=attribute
						computed_attribute_matrices[fold].at<float>(sample_index, attribute_index) = response.at<float>(0,0)*feature_scaling_factor;
					}
				}
			}
		}

		std::cout << "=== Total result over " << cross_validation_params.folds_ << "-fold cross validation ===" << std::endl;		output_summary << "=== Total result over " << cross_validation_params.folds_ << "-fold cross validation ===" << std::endl;
		double total_mean = 0., total_below05 = 0., total_below1 = 0.;
		for (int attribute_index=0; attribute_index<attribute_matrix.cols; ++attribute_index)
		{
			double n = numberSamples[attribute_index];
			double mean = sumAbsErrors[attribute_index]/n;
			total_mean += mean;
			if ((size_t)numberSamples[attribute_index] != absErrors[attribute_index].size())
				std::cout << "Warning: (numberSamples[attribute_index] == absErrors[attribute_index].size()) does not hold for (attribute_index+1): " << attribute_index+1 << std::endl;
			double stddev = 0.0;
			for (size_t j=0; j<absErrors[attribute_index].size(); ++j)
				stddev += (absErrors[attribute_index][j] - mean) * (absErrors[attribute_index][j] - mean);
			stddev /= (double)(absErrors[attribute_index].size() - 1.);
			stddev = sqrt(stddev);
			std::cout << "Attribute " << attribute_index+1 << ":\tmean abs error: " << mean << " \t+/- " << stddev << "\t\t<0.5: " << 100*below05[attribute_index]/n << "%\t\t<1.0: " << 100*below1[attribute_index]/n << "%" << std::endl;
			output_summary << "Attribute " << attribute_index+1 << ":\tmean abs error: " << mean << " \t+/- " << stddev << "\t\t<0.5: " << 100*below05[attribute_index]/n << "%\t\t<1.0: " << 100*below1[attribute_index]/n << "%" << std::endl;
			total_below05 += 100*below05[attribute_index]/n;
			total_below1 += 100*below1[attribute_index]/n;
		}
		std::cout << "total          \tmean abs error: " << total_mean/(double)attribute_matrix.cols << "\t           \t\t<0.5: " << total_below05/(double)attribute_matrix.cols << "%\t\t<1.0: " << total_below1/(double)attribute_matrix.cols << "%" << std::endl;
		output_summary << "total          \tmean abs error: " << total_mean/(double)attribute_matrix.cols << "\t           \t\t<0.5: " << total_below05/(double)attribute_matrix.cols << "%\t\t<1.0: " << total_below1/(double)attribute_matrix.cols << "%\n\n" << std::endl;

		// write screen outputs to file
		std::stringstream logfilename;
		logfilename << "texture_categorization/screen_output_attribute_learning_" << ml_configuration_index << ".txt";
		std::ofstream file(logfilename.str().c_str(), std::ios::out);
		if (file.is_open() == true)
			file << cross_validation_params.configurationToString() << std::endl << ml_params.configurationToString() << std::endl << output_summary.str() << std::endl << screen_output.str();
		else
			std::cout << "Error: could not write screen output to file " << logfilename.str() << "." << std::endl;
		file.close();

		// write summary to file
		std::stringstream summary_filename;
		summary_filename << "texture_categorization/screen_output_attribute_learning_summary.txt";
		file.open(summary_filename.str().c_str(), std::ios::app);
		if (file.is_open() == true)
			file << cross_validation_params.configurationToString() << std::endl << ml_params.configurationToString() << std::endl << output_summary.str();
		else
			std::cout << "Error: could not write summary to file " << summary_filename.str() << "." << std::endl;
		file.close();
	}
}


void AttributeLearning::train(const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix)
{
	svm_.clear();
	svm_.reserve(attribute_matrix.cols);
	for (int attribute_index=0; attribute_index<attribute_matrix.cols; ++attribute_index)
	{
		std::cout << "--- attribute " << attribute_index+1 << " ---" << std::endl;

		// create label vector
		cv::Mat training_labels(attribute_matrix.rows, 1, CV_32FC1);
		//const double feature_scaling_factor = (attribute_index==1 || attribute_index==2) ? 2.0 : 1.0;
		for (int r=0; r<attribute_matrix.rows; ++r)
			training_labels.at<float>(r) = attribute_matrix.at<float>(r, attribute_index); // /feature_scaling_factor;

		// SVM
		CvTermCriteria criteria;
		criteria.max_iter = 1000000;	// 1000
		criteria.epsilon  = FLT_EPSILON; // FLT_EPSILON
		criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
		CvSVMParams svm_params(CvSVM::NU_SVR, CvSVM::RBF, 0., 0.1, 0., 1.0, 0.4, 0., 0, criteria);		// RBF, 0.0, 0.1, 0.0, 1.0, 0.4, 0.
		if (attribute_index == 1 || attribute_index == 2)
			svm_params.svm_type = CvSVM::NU_SVC;
		svm_.push_back(boost::shared_ptr<CvSVM>(new CvSVM()));
		svm_[attribute_index]->train(feature_matrix, training_labels, cv::Mat(), cv::Mat(), svm_params);
	}
}


void AttributeLearning::predict(const cv::Mat& feature_data, cv::Mat& predicted_labels)
{
	predicted_labels.create(feature_data.rows, svm_.size(), CV_32FC1);
	for (size_t attribute_index=0; attribute_index<svm_.size(); ++attribute_index)
	{
		//const double feature_scaling_factor = (attribute_index==1 || attribute_index==2) ? 2.0 : 1.0;
		const double feature_scaling_factor = 1.0;
		for (int r = 0; r < feature_data.rows ; ++r)
		{
			cv::Mat sample = feature_data.row(r);
			predicted_labels.at<float>(r,attribute_index) = feature_scaling_factor*svm_[attribute_index]->predict(sample);	// SVM
		}
	}
}


void AttributeLearning::save_SVMs(std::string path)
{
	for (size_t i=0; i<svm_.size(); ++i)
	{
		std::stringstream ss;
		ss << path << "attribute_svm_" << i << ".yml";
		svm_[i]->save(ss.str().c_str(), "svm");
	}
}


void AttributeLearning::load_SVMs(std::string path)
{
	svm_.clear();
	const int attribute_number = 17;
	svm_.resize(attribute_number);
	for (size_t i=0; i<svm_.size(); ++i)
	{
		std::stringstream ss;
		ss << path << "attribute_svm_" << i << ".yml";
		svm_[i]->load(ss.str().c_str(), "svm");
	}
}


void AttributeLearning::displayAttributes(const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, int display_class, bool update, bool store_on_disk)
{
	// prepare display
	create_train_data ctd;
	std::vector<std::string> texture_classes = ctd.get_texture_classes();
	// scale at: height 50 - 250 -> 10 - 0
	const int column_spacing = 60;
	if (update == false)
	{
		attribute_display_mat_plot_counter_ = 0;
		attribute_display_mat_ = cv::Mat(320, (attribute_matrix.cols+2)*column_spacing, CV_8UC3);
		attribute_display_mat_.setTo(cv::Scalar(255,255,255));
		for (int a=1; a<=attribute_matrix.cols; ++a)
		{
			std::stringstream ss;
			ss << a;
			cv::putText(attribute_display_mat_, ss.str(), cv::Point(a*column_spacing-10, 300), cv::FONT_HERSHEY_SIMPLEX, 1., CV_RGB(0,0,0), 1);
		}
		for (int i=0; i<=10; ++i)
			cv::line(attribute_display_mat_, cv::Point(column_spacing/2+20, 250-20*i), cv::Point(attribute_display_mat_.cols-column_spacing/2, 250-20*i), CV_RGB(0,0,0), (i==1 || i==5 || i==10)? 2 : 1);
		cv::putText(attribute_display_mat_, "0", cv::Point(2, 250+10), cv::FONT_HERSHEY_SIMPLEX, 1., CV_RGB(0,0,0), 1);
		cv::putText(attribute_display_mat_, "5", cv::Point(2, 250+10-5*20), cv::FONT_HERSHEY_SIMPLEX, 1., CV_RGB(0,0,0), 1);
		cv::putText(attribute_display_mat_, "10", cv::Point(2, 250+10-10*20), cv::FONT_HERSHEY_SIMPLEX, 1., CV_RGB(0,0,0), 1);
		std::stringstream ss;
		ss << "class " << texture_classes[display_class];
		cv::putText(attribute_display_mat_, ss.str(), cv::Point(attribute_display_mat_.cols/2-50, 30), cv::FONT_HERSHEY_SIMPLEX, 1., CV_RGB(0,0,0), 1);
	}
	else
		++attribute_display_mat_plot_counter_;
	cv::Scalar data_point_color;
	if (attribute_display_mat_plot_counter_ == 0)
		data_point_color = CV_RGB(0,-1,-1);
	else if (attribute_display_mat_plot_counter_ == 1)
		data_point_color = CV_RGB(-1,0,-1);
	else if (attribute_display_mat_plot_counter_ == 2)
		data_point_color = CV_RGB(-1,-1,0);
	else
		data_point_color = CV_RGB(255*rand()/(double)RAND_MAX, 255*rand()/(double)RAND_MAX, 255*rand()/(double)RAND_MAX);

	// select data samples for current class
	std::vector<int> sample_indices;
	for (unsigned int class_index=0; class_index<data_sample_hierarchy.size(); ++class_index)
	{
		if ((int)class_index == display_class)
		{
			int object_number = data_sample_hierarchy[class_index].size();
			for (int object_index=0; object_index<object_number; ++object_index)
				for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
					sample_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
		}
	}

	// fill data into display
	data_point_color *= 255./(double)sample_indices.size();
	for (size_t i=0; i<sample_indices.size(); ++i)
		for (int a=0; a<attribute_matrix.cols; ++a)
		{
			cv::Point draw_location((a+1)*column_spacing + 8*attribute_display_mat_plot_counter_, 250-20*attribute_matrix.at<float>(sample_indices[i],a));
			cv::Vec3b old_color = attribute_display_mat_.at<cv::Vec3b>(draw_location);
			if (old_color == cv::Vec3b::zeros())
			{
				old_color.val[0] = 255;
				old_color.val[1] = 255;
				old_color.val[2] = 255;
			}
			cv::Scalar new_color(old_color.val[0]+data_point_color.val[0], old_color.val[1]+data_point_color.val[1], old_color.val[2]+data_point_color.val[2]);
			cv::circle(attribute_display_mat_, draw_location, 2, new_color, 2);
		}

	if (store_on_disk == false)
	{
		cv::imshow("Attribute distribution", attribute_display_mat_);
		cv::moveWindow("Attribute distribution", 0, 0);
		cv::waitKey(10);
	}
	else
	{
		std::stringstream ss;
		ss << "attribute_distribution/" << texture_classes[display_class] << "_" << display_class << ".png";
		cv::imwrite(ss.str(), attribute_display_mat_);
	}
}


size_t AttributeLearning::sampleIndexFromFilename(const std::string& filename, const std::vector<std::string>& indexed_filenames)
{
	size_t i=0;
	for (; i<indexed_filenames.size(); ++i)
		if (indexed_filenames[i].compare(filename)==0)
			return i;
	return SIZE_MAX;
}

void AttributeLearning::loadDTDDatabaseCrossValidationSet(const std::string& path_to_cross_validation_sets, const std::string& set_type, const std::vector<std::string>& image_filenames, const int fold,
		const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, cv::Mat& feature_matrix_set, cv::Mat& attribute_matrix_set)
{
	// read out set filenames and map them to sample numbers (= line indices of the feature_matrix)
	std::vector<int> set_indices;
	std::stringstream set_filename;
	set_filename << path_to_cross_validation_sets << set_type << fold+1 << ".txt";
	std::ifstream file(set_filename.str().c_str(), std::ios::in);
	if (file.is_open()==true)
	{
		while (file.eof()==false)
		{
			std::string filename;
			file >> filename;
			size_t sample_index = sampleIndexFromFilename(filename, image_filenames);
			if (sample_index == SIZE_MAX)
				break;
			set_indices.push_back(sample_index);
		}
		file.close();
	}
	else
		std::cout << "Error: AttributeLearning::loadDTDDatabaseCrossValidationSet: could not open file " << set_filename.str() << std::endl;

	// create the set feature and attributes matrices
	feature_matrix_set = cv::Mat((int)set_indices.size(), feature_matrix.cols, feature_matrix.type());
	attribute_matrix_set = cv::Mat((int)set_indices.size(), attribute_matrix.cols, attribute_matrix.type());
	for (size_t set_index=0; set_index<set_indices.size(); ++set_index)
	{
		for (int j=0; j<feature_matrix.cols; ++j)
			feature_matrix_set.at<float>(set_index, j) = feature_matrix.at<float>(set_indices[set_index], j);
		for (int j=0; j<attribute_matrix.cols; ++j)
			attribute_matrix_set.at<float>(set_index, j) = attribute_matrix.at<float>(set_indices[set_index], j);
	}
}

void AttributeLearning::loadDTDDatabaseCrossValidationSets(const std::string& path_to_cross_validation_sets, const std::vector<std::string>& image_filenames, const int fold,
		const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, cv::Mat& feature_matrix_train, cv::Mat& attribute_matrix_train,
		cv::Mat& feature_matrix_validation, cv::Mat& attribute_matrix_validation, cv::Mat& feature_matrix_test, cv::Mat& attribute_matrix_test)
{
	loadDTDDatabaseCrossValidationSet(path_to_cross_validation_sets, "train", image_filenames, fold, feature_matrix, attribute_matrix, feature_matrix_train, attribute_matrix_train);
	loadDTDDatabaseCrossValidationSet(path_to_cross_validation_sets, "val", image_filenames, fold, feature_matrix, attribute_matrix, feature_matrix_validation, attribute_matrix_validation);
	loadDTDDatabaseCrossValidationSet(path_to_cross_validation_sets, "test", image_filenames, fold, feature_matrix, attribute_matrix, feature_matrix_test, attribute_matrix_test);
}


void AttributeLearning::crossValidationDTD(const CrossValidationParams& cross_validation_params, const std::string& path_to_cross_validation_sets, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, const std::vector<std::string>& image_filenames)
{
	std::stringstream screen_output, output_summary;
	srand(0);	// random seed --> keep reproducible
	// train and evaluate classifier for each attribute with the given training set
	for (int attribute_index=0; attribute_index<attribute_matrix.cols; ++attribute_index)
	{
		// iterate over (possibly multiple) machine learning techniques or configurations
		for (size_t ml_configuration_index = 0; ml_configuration_index<cross_validation_params.ml_configurations_.size(); ++ml_configuration_index)
		{
			const MLParams& ml_params = cross_validation_params.ml_configurations_[ml_configuration_index];
			std::cout << "Attribute " << attribute_index+1 << ":\nML params: " << ml_params.configurationToString() << "\n";

			for (unsigned int fold=0; fold<cross_validation_params.folds_; ++fold)
			{
				std::cout << "Attribute " << attribute_index+1 << ": fold " << fold << ":\n";

				// obtain official train/val/test splits
				cv::Mat feature_matrix_train, feature_matrix_validation, feature_matrix_test;
				cv::Mat attribute_matrix_train, attribute_matrix_validation, attribute_matrix_test;
				loadDTDDatabaseCrossValidationSets(path_to_cross_validation_sets, image_filenames, fold, feature_matrix, attribute_matrix, feature_matrix_train, attribute_matrix_train, feature_matrix_validation, attribute_matrix_validation, feature_matrix_test, attribute_matrix_test);

				// train classifier
				CvSVM svm;
				CvANN_MLP mlp;
				if (ml_params.classification_method_ == MLParams::SVM)
				{	// SVM
					svm.train(feature_matrix_train, attribute_matrix_train.col(attribute_index), cv::Mat(), cv::Mat(), ml_params.svm_params_);
				}
				else if (ml_params.classification_method_ == MLParams::NEURAL_NETWORK)
				{	//	Neural Network
					cv::Mat layers = cv::Mat(2+ml_params.nn_hidden_layers_.size(), 1, CV_32SC1);
					layers.row(0) = cv::Scalar(feature_matrix_train.cols);
					for (size_t k=0; k<ml_params.nn_hidden_layers_.size(); ++k)
						layers.row(k+1) = cv::Scalar(ml_params.nn_hidden_layers_[k]);
					layers.row(ml_params.nn_hidden_layers_.size()+1) = cv::Scalar(1);

					mlp.create(layers, ml_params.nn_activation_function_, ml_params.nn_activation_function_param1_, ml_params.nn_activation_function_param2_);
					int iterations = mlp.train(feature_matrix_train, attribute_matrix_train.col(attribute_index), cv::Mat(), cv::Mat(), ml_params.nn_params_);
					std::cout << "Neural network training completed after " << iterations << " iterations." << std::endl;		screen_output << "Neural network training completed after " << iterations << " iterations." << std::endl;
				}

				// validate classification performance
				for (int r=0; r<feature_matrix_validation.rows ; ++r)
				{
					cv::Mat response(1, 1, CV_32FC1);
					cv::Mat sample = feature_matrix_validation.row(r);
					if (ml_params.classification_method_ == MLParams::SVM)
						response.at<float>(0,0) = svm.predict(sample);	// SVM
					else if (ml_params.classification_method_ == MLParams::NEURAL_NETWORK)
						mlp.predict(sample, response);		// neural network
				}
			} // folds
		} // ml_configurations
	} // attribute_index
}
