#include "cob_texture_categorization/attribute_learning.h"

#include <fstream>

#include "ml.h"
#include "highgui.h"


void AttributeLearning::loadTextureDatabaseBaseFeatures(std::string filename, const int feature_number, const int attribute_number, cv::Mat& feature_matrix, cv::Mat& ground_truth_attribute_matrix, cv::Mat& class_label_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy)
{
	// load feature vectors and corresponding labels computed on database and class-object-sample hierarchy
	//const int attribute_number = 17;			// label = attributes
	//const int feature_number = 9688;		// feature = base feature
	//const int total_sample_number = 1281;
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
					for (int l=0; l<attribute_number; ++l)
						file >> ground_truth_attribute_matrix_row.at<float>(0, l);	// attribute vector
					for (int f=0; f<feature_number; ++f)
						file >> feature_matrix_row.at<float>(0, f);		// base feature vector
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


void AttributeLearning::crossValidation(unsigned int folds, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, CrossValidationMode cross_validation_mode)
{
	cv::Mat class_label_matrix;
	std::vector< std::vector<int> > preselected_train_indices;
	std::vector<cv::Mat> attribute_matrix_test_data, class_label_matrix_test_data;
	std::vector<cv::Mat> computed_attribute_matrices;
	crossValidation(folds, feature_matrix, attribute_matrix, data_sample_hierarchy, cross_validation_mode, false, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, false, computed_attribute_matrices);
}

void AttributeLearning::crossValidation(unsigned int folds, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, CrossValidationMode cross_validation_mode, std::vector<cv::Mat>& computed_attribute_matrices)
{
	cv::Mat class_label_matrix;
	std::vector< std::vector<int> > preselected_train_indices;
	std::vector<cv::Mat> attribute_matrix_test_data, class_label_matrix_test_data;
	crossValidation(folds, feature_matrix, attribute_matrix, data_sample_hierarchy, cross_validation_mode, false, class_label_matrix, preselected_train_indices, attribute_matrix_test_data, class_label_matrix_test_data, true, computed_attribute_matrices);
}

void AttributeLearning::crossValidation(unsigned int folds, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy, CrossValidationMode cross_validation_mode,
		bool return_set_data, const cv::Mat& class_label_matrix, std::vector< std::vector<int> >& preselected_train_indices, std::vector<cv::Mat>& attribute_matrix_test_data, std::vector<cv::Mat>& class_label_matrix_test_data,
		bool return_computed_attribute_matrices, std::vector<cv::Mat>& computed_attribute_matrices)
{
	// vectors for evaluation data, each vector entry is meant to count the statistics for one attribute
	std::vector<double> sumAbsErrors(attribute_matrix.cols, 0.0);
	std::vector< std::vector<double> > absErrors(attribute_matrix.cols);	// first index = attribute index
	std::vector<int> numberSamples(attribute_matrix.cols, 0);
	std::vector<int> below05(attribute_matrix.cols, 0), below1(attribute_matrix.cols, 0);	// number of attributes estimated with less absolute error than 0.5, 1.0
	std::stringstream screen_output;

	if (return_set_data == true)
	{
		preselected_train_indices.resize(folds);
		attribute_matrix_test_data.resize(folds);
		class_label_matrix_test_data.resize(folds);
	}

	if (return_computed_attribute_matrices == true)
	{
		computed_attribute_matrices.resize(folds);
		for (size_t i=0; i<computed_attribute_matrices.size(); ++i)
			computed_attribute_matrices[i].create(attribute_matrix.rows, attribute_matrix.cols, attribute_matrix.type());
	}

	create_train_data data_object;
	std::vector<std::string> texture_classes = data_object.get_texture_classes();

	srand(0);	// random seed --> keep reproducible
	for (unsigned int fold=0; fold<folds; ++fold)
	{
		std::cout << "\n=== fold " << fold+1 << " ===" << std::endl;		screen_output << "=== fold " << fold+1 << " ===" << std::endl;

		// === distribute data into training and test set ===
		std::vector<int> train_indices, test_indices;
		if (cross_validation_mode == LEAVE_OUT_ONE_OBJECT_PER_CLASS)
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
		else if (cross_validation_mode == LEAVE_OUT_ONE_CLASS)
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
			const double feature_scaling_factor = (attribute_index==1 || attribute_index==2) ? 2.0 : 1.0;
			for (unsigned int r=0; r<train_indices.size(); ++r)
				training_labels.at<float>(r) = attribute_matrix.at<float>(train_indices[r], attribute_index)/feature_scaling_factor;
			for (unsigned int r=0; r<test_indices.size(); ++r)
				test_labels.at<float>(r) = attribute_matrix.at<float>(test_indices[r], attribute_index)/feature_scaling_factor;

			// === train classifier ===

//			// K-Nearest-Neighbor
//			cv::Mat test;
//			CvKNearest knn(training_data, training_label, cv::Mat(), false, 32);
//			knn.train(training_data,training_label,test,false,32,false );
//
//			cv::Mat result;
//			cv::Mat neighbor_results;
//			cv::Mat dist;
//			knn.find_nearest(train_data, 1, &result,0,&neighbor_results, &dist);
//			// End K-Nearest-Neighbor

//			// Randomtree
//			const float regression_accuracy = 0.0001f;		// per tree
//			const float forest_accuracy = 0.01f;		// oob error of forest
//			const int max_tree_depth = 10;
//			const int max_num_of_trees_in_the_forest = 25;
//			const int nactive_vars = 100;	// size of randomly chosen feature subset, best splits are sought within this subset
//			cv::Mat var_type = cv::Mat::ones(1, training_data.cols+1, CV_8UC1) * CV_VAR_NUMERICAL;
//			CvRTParams tree_params(max_tree_depth, (int)(0.01*training_data.rows), regression_accuracy, false, 10, 0, false, nactive_vars, max_num_of_trees_in_the_forest, forest_accuracy, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS);
//			CvRTrees rtree;
//			rtree.train(training_data, CV_ROW_SAMPLE, training_labels, cv::Mat(), cv::Mat(), var_type, cv::Mat(), tree_params);
//			std::cout << "Random forest training finished with " << rtree.get_tree_count() << " trees." << std::endl;
//			// End Randomtree

//			// Decision Tree
//			CvDTreeParams params = CvDTreeParams(25, // max depth
//													 5, // min sample count
//													 0, // regression accuracy: N/A here
//													 false, // compute surrogate split, no missing data
//													 15, // max number of categories (use sub-optimal algorithm for larger numbers)
//													 15, // the number of cross-validation folds
//													 false, // use 1SE rule => smaller tree
//													 false, // throw away the pruned tree branches
//													 NULL // the array of priors
//													);
//			CvDTree dtree;
//			dtree.train(training_data, CV_ROW_SAMPLE, training_label, cv::Mat(), cv::Mat(),cv::Mat(),cv::Mat(), params);
//
//			CvDTreeNode *node;
//
//			cv::Mat result(train_data.rows,1,CV_32FC1);
//				for(int i=0;i<train_data.rows;i++)
//				{
//					cv::Mat inputvec(1,train_data.cols, CV_32FC1);
//					for(int j=0;j<train_data.cols;j++)
//					{
//						inputvec.at<float>(0,j)=train_data.at<float>(i,j);
//					}
//					node = dtree.predict(inputvec);
//					result.at<float>(i,0)= (*node).class_idx;
//				}
//			// End Dicision Tree

//			// BayesClassifier
//			CvNormalBayesClassifier bayesmod;
//			bayesmod.train(training_data, training_label, cv::Mat(), cv::Mat());
//			cv::Mat result(train_data.rows,1,CV_32FC1);
//			bayesmod.predict(train_data, &result);
//			// End Bayes Classifier

//			// Boosting -- not implemented for regression at 28.06.2014
//			CvBoost boost;
//			cv::Mat var_type = cv::Mat::ones(1, training_data.cols+1, CV_8UC1) * CV_VAR_NUMERICAL;
//			CvBoostParams boost_params(CvBoost::GENTLE, 100, 0.95, 1, false, 0);
//			boost.train(training_data, CV_ROW_SAMPLE, training_labels, cv::Mat(), cv::Mat(), var_type, cv::Mat(), boost_params, false);
//			// End Boosting

			// SVM
			CvSVM svm;
			CvTermCriteria criteria;
			criteria.max_iter = 1000;//1000;	// 1000
			criteria.epsilon  = FLT_EPSILON; // FLT_EPSILON
			criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
			CvSVMParams svm_params(CvSVM::NU_SVR, CvSVM::LINEAR, 0., 0.1, 0., 1.0, 0.4, 0., 0, criteria);		// RBF, 0.0, 0.1, 0.0, 1.0, 0.4, 0.
			svm.train(training_data, training_labels, cv::Mat(), cv::Mat(), svm_params);

//			//	Neural Network
//			cv::Mat input;
//			training_data.convertTo(input, CV_32F);
//			cv::Mat output=cv::Mat::zeros(training_data.rows, 1, CV_32FC1);
//			cv::Mat labels;
//			training_labels.convertTo(labels, CV_32F);
//			for(int i=0; i<training_data.rows; ++i)
//				output.at<float>(i,0) = labels.at<float>(i,0);
//
//			cv::Mat layers = cv::Mat(3,1,CV_32SC1);
//
//			layers.row(0) = cv::Scalar(training_data.cols);
//			layers.row(1) = cv::Scalar(10);
//			layers.row(2) = cv::Scalar(1);
//
//			CvANN_MLP mlp;
//			CvANN_MLP_TrainParams params;
//			CvTermCriteria criteria;
//			criteria.max_iter = 100;//100;
//			criteria.epsilon  = 0.00001f; // farhadi:0.0001f, handcrafted:0.00001f;
//			criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
//
//			params.train_method    = CvANN_MLP_TrainParams::BACKPROP;
//			params.bp_dw_scale     = 0.1f;
//			params.bp_moment_scale = 0.1f;
//			params.term_crit       = criteria;
//
//			//double alpha = (attribute_index==1 || attribute_index==2) ? 0.2 : 0.4;
//			mlp.create(layers,CvANN_MLP::SIGMOID_SYM, /*alpha*/ 0.6, 1.0);			// 0.4, except for dominant/sec. dom. color: 0.2
//			int iterations = mlp.train(input, output, cv::Mat(), cv::Mat(), params);
//			std::cout << "Neural network training completed after " << iterations << " iterations." << std::endl;		screen_output << "Neural network training completed after " << iterations << " iterations." << std::endl;

			// === apply ml classifier to predict test set ===
			double sumAbsError = 0.;
			int numberTestSamples = 0;
			int below05_ctr = 0, below1_ctr = 0;
			for (int r = 0; r < test_data.rows ; ++r)
			{
				cv::Mat response(1, 1, CV_32FC1);
				cv::Mat sample = test_data.row(r);
				//mlp.predict(sample, response);		// neural network
				//response.at<float>(0,0) = rtree.predict(sample);		// random tree
				//response.at<float>(0,0) = sample.at<float>(0, attribute_index) / feature_scaling_factor;		// direct relation: feature=attribute
				response.at<float>(0,0) = svm.predict(sample);	// SVM

				if (return_set_data == true)
				{
					attribute_matrix_test_data[fold].at<float>(r,attribute_index) = response.at<float>(0,0) * feature_scaling_factor;
					class_label_matrix_test_data[fold].at<float>(r, 0) = class_label_matrix.at<float>(test_indices[r],0);
				}

				float resp = std::max(0.f, response.at<float>(0,0));
				float lab = test_labels.at<float>(r, 0);
				float absdiff = fabs(resp - lab) * feature_scaling_factor;
				if (attribute_index == 1 || attribute_index == 2)
				{
					float absdiff2 = fabs(std::min(resp, lab)+9 - std::max(resp, lab)) * feature_scaling_factor;
					absdiff = std::min(absdiff, absdiff2);
					absdiff *= 4./9.;
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

				screen_output << "value: " << test_labels.at<float>(r, 0) << "\t predicted: " << response.at<float>(0,0)*feature_scaling_factor << "\t abs difference: " << absdiff << std::endl;
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
					//mlp.predict(sample, response);		// neural network
					//response.at<float>(0,0) = rtree.predict(sample);		// random tree
					//response.at<float>(0,0) = sample.at<float>(0, attribute_index) / feature_scaling_factor;		// direct relation: feature=attribute
					response.at<float>(0,0) = svm.predict(sample);	// SVM
					computed_attribute_matrices[fold].at<float>(sample_index, attribute_index) = response.at<float>(0,0)*feature_scaling_factor;
				}
			}
		}
		// End Neural Network
	}

	std::cout << "=== Total result over " << folds << "-fold cross validation ===" << std::endl;		screen_output << "=== Total result over " << folds << "-fold cross validation ===" << std::endl;
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
		screen_output << "Attribute " << attribute_index+1 << ":\tmean abs error: " << mean << " \t+/- " << stddev << "\t\t<0.5: " << 100*below05[attribute_index]/n << "%\t\t<1.0: " << 100*below1[attribute_index]/n << "%" << std::endl;
		total_below05 += 100*below05[attribute_index]/n;
		total_below1 += 100*below1[attribute_index]/n;
	}
	std::cout << "total          \tmean abs error: " << total_mean/(double)attribute_matrix.cols << "\t           \t\t<0.5: " << total_below05/(double)attribute_matrix.cols << "%\t\t<1.0: " << total_below1/(double)attribute_matrix.cols << "%" << std::endl;
	screen_output << "total          \tmean abs error: " << total_mean/(double)attribute_matrix.cols << "\t           \t\t<0.5: " << total_below05/(double)attribute_matrix.cols << "%\t\t<1.0: " << total_below1/(double)attribute_matrix.cols << "%" << std::endl;

	// write screen outputs to file
	std::ofstream file("screen_output_attribute_learning.txt", std::ios::out);
	if (file.is_open() == true)
		file << screen_output.str();
	else
		std::cout << "Error: could not write screen output to file.";
	file.close();
}
