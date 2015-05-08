#include "cob_texture_categorization/train_ml.h"

#include <stdlib.h>
#include <float.h>
#include <stdio.h>
#include <ctype.h>
#include <fstream>
#include <map>

#include <cv.h>
#include "ml.h"
#include "highgui.h"

#include "cob_texture_categorization/attribute_learning.h"

train_ml::train_ml()
{
}


void train_ml::cross_validation(int folds, const cv::Mat& feature_matrix, const cv::Mat& label_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy,
			const std::vector< std::vector<int> >& preselected_train_indices, const std::vector<cv::Mat>& feature_matrix_test_data, const std::vector<cv::Mat>& label_matrix_test_data,
			const std::vector<cv::Mat>& feature_matrices)
{
	const int number_classes = 57;
	std::vector<int> true_predictions, false_predictions, true_predictions_rank(number_classes, 0);
	create_train_data data_object;
	std::vector<std::string> texture_classes = data_object.get_texture_classes();
	std::stringstream screen_output;

	bool use_preselected_set_distribution = (preselected_train_indices.size()==(size_t)folds && feature_matrix_test_data.size()==(size_t)folds && label_matrix_test_data.size()==(size_t)folds && feature_matrices.size()==(size_t)folds);
	if (use_preselected_set_distribution == true)
	{
		std::cout << "Using the provided pre-selected sets for training and testing." << std::endl;		screen_output << "Using the provided pre-selected sets for training and testing." << std::endl;
	}
	else
	{
		std::cout << "Computing the individual training and testing sets for each fold." << std::endl;		screen_output << "Computing the individual training and testing sets for each fold." << std::endl;
	}

	srand(0);	// random seed --> keep reproducible
	for (int fold=0; fold<folds; ++fold)
	{
		std::cout << "=== fold " << fold+1 << " ===" << std::endl;		screen_output << "=== fold " << fold+1 << " ===" << std::endl;

		// === distribute data into training and test set ===
		std::vector<int> train_indices, test_indices;
		cv::Mat feature_matrix_reference = feature_matrix;
		if (use_preselected_set_distribution==true)
		{
			// just take the provided training indices for this fold
			feature_matrix_reference = feature_matrices[fold];
			if (feature_matrix_test_data[fold].empty()==true || feature_matrix_test_data[fold].cols!=feature_matrix_reference.cols || label_matrix_test_data[fold].empty()==true || label_matrix_test_data[fold].cols!=label_matrix.cols || feature_matrix_test_data[fold].rows!=label_matrix_test_data[fold].rows)
				std::cout << "Error: provided pre-computed test data and label matrices are not suitable." << std::endl;
			train_indices = preselected_train_indices[fold];
			test_indices.resize(feature_matrix_test_data[fold].rows);
		}
		else
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
							test_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
					else
						for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
							train_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
				}
			}
		}
		assert(test_indices.size() + train_indices.size() == (unsigned int)feature_matrix_reference.rows);

		// create training and test data matrices
		cv::Mat training_data(train_indices.size(), feature_matrix_reference.cols, feature_matrix_reference.type());
		cv::Mat training_labels(train_indices.size(), 1, label_matrix.type());
		cv::Mat test_data(test_indices.size(), feature_matrix_reference.cols, feature_matrix_reference.type());
		cv::Mat test_labels(test_indices.size(), 1, label_matrix.type());
		for (unsigned int r=0; r<train_indices.size(); ++r)
		{
			for (int c=0; c<feature_matrix_reference.cols; ++c)
				training_data.at<float>(r,c) = feature_matrix_reference.at<float>(train_indices[r],c);
			training_labels.at<float>(r) = label_matrix.at<float>(train_indices[r]);
		}
		if (use_preselected_set_distribution==true)
		{
			test_data = feature_matrix_test_data[fold];
			test_labels = label_matrix_test_data[fold];
		}
		else
		{
			for (unsigned int r=0; r<test_indices.size(); ++r)
			{
				for (int c=0; c<feature_matrix_reference.cols; ++c)
					test_data.at<float>(r,c) = feature_matrix_reference.at<float>(test_indices[r],c);
				test_labels.at<float>(r) = label_matrix.at<float>(test_indices[r]);
			}
		}

		// === train ml classifier ===

//	std::cout<<"Value:"<<test_data_label.at<float>(3,0)<<std::endl;
//	K-Nearest-Neighbor

//	cv::Mat test;
//	CvKNearest knn(training_data, training_label, cv::Mat(), false, 32);
//	knn.train(training_data,training_label,test,false,32,false );
//
//	cv::Mat result;
//	cv::Mat neighbor_results;
//	cv::Mat dist;
//	knn.find_nearest(train_data, 1, &result,0,&neighbor_results, &dist);

// 	End K-Nearest-Neighbor

//	Decision Tree

//    CvDTreeParams params = CvDTreeParams(25, // max depth
//                                             5, // min sample count
//                                             0, // regression accuracy: N/A here
//                                             false, // compute surrogate split, no missing data
//                                             15, // max number of categories (use sub-optimal algorithm for larger numbers)
//                                             15, // the number of cross-validation folds
//                                             false, // use 1SE rule => smaller tree
//                                             false, // throw away the pruned tree branches
//                                             NULL // the array of priors
//                                            );
//    CvDTree dtree;
//    dtree.train(training_data, CV_ROW_SAMPLE, training_label, cv::Mat(), cv::Mat(),cv::Mat(),cv::Mat(), params);
//
//    CvDTreeNode *node;
//
//    cv::Mat result(train_data.rows,1,CV_32FC1);
//    	for(int i=0;i<train_data.rows;i++)
//    	{
//    		cv::Mat inputvec(1,train_data.cols, CV_32FC1);
//    		for(int j=0;j<train_data.cols;j++)
//    		{
//    			inputvec.at<float>(0,j)=train_data.at<float>(i,j);
//    		}
//    		node = dtree.predict(inputvec);
//    		result.at<float>(i,0)= (*node).class_idx;
//    	}

//	End Dicision Tree

//		// BayesClassifier
//		CvNormalBayesClassifier bayesmod;
//		bayesmod.train(training_data, training_labels);

//		// Random trees
//		float* priors = 0;
//		//CvRTParams rtree_params(78, 6, 0.0, false,28, priors, false,23,28,1.0, 0 );
//		CvRTParams rtree_params(100, 0.0005*training_data.rows, 0.f, false, number_classes, priors, false, training_data.cols, 100, FLT_EPSILON, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS);
//		cv::Mat var_type = cv::Mat::ones(training_data.cols+1, 1, CV_8U) * CV_VAR_NUMERICAL;
//		var_type.at<int>(training_data.cols) = CV_VAR_CATEGORICAL;
//		CvRTrees rtree;
//		rtree.train(training_data, CV_ROW_SAMPLE, training_labels, cv::Mat(), cv::Mat(), var_type, cv::Mat(), rtree_params);
//		// End Random trees

//		// SVM
//		CvSVM svm;
//		CvTermCriteria criteria;
//		criteria.max_iter = 1000;//1000;	// 1000
//		criteria.epsilon  = FLT_EPSILON; // FLT_EPSILON
//		criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
//		CvSVMParams svm_params(CvSVM::C_SVC, CvSVM::POLY, 2., 1.0, 1., 1.0, 0.0, 0., 0, criteria);		// RBF, 0.0, 0.1, 0.0, 1.0, 0.4, 0.
//		//CvSVMParams svm_params(CvSVM::C_SVC, CvSVM::RBF, 0., 0.2, 1., 1.0, 0.0, 0., 0, criteria);
//		svm.train(training_data, training_labels, cv::Mat(), cv::Mat(), svm_params);

		// Neural Network
		cv::Mat input;
		training_data.convertTo(input, CV_32F);
		cv::Mat output=cv::Mat::zeros(training_data.rows, number_classes, CV_32FC1);
		cv::Mat labels;
		training_labels.convertTo(labels, CV_32F);
		for(int i=0; i<training_data.rows; ++i)
			output.at<float>(i,(int)labels.at<float>(i,0)) = 1.f;		//change.at<float>(i,0);

		cv::Mat layers = cv::Mat(3,1,CV_32SC1);
		layers.row(0) = cv::Scalar(training_data.cols);
		layers.row(1) = cv::Scalar(400);	//400
		layers.row(2) = cv::Scalar(number_classes);

		CvANN_MLP mlp;
		CvANN_MLP_TrainParams params;
		CvTermCriteria criteria;

		criteria.max_iter = 400;
		criteria.epsilon  = 0.0001f;
		criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;

		params.train_method    = CvANN_MLP_TrainParams::BACKPROP;
		params.bp_dw_scale     = 0.1f;
		params.bp_moment_scale = 0.1f;
		params.term_crit       = criteria;

		mlp.create(layers,CvANN_MLP::SIGMOID_SYM,0.4,1.2);
		int iterations = mlp.train(input, output, cv::Mat(), cv::Mat(), params);
		std::cout << "Neural network training completed after " << iterations << " iterations." << std::endl;		screen_output << "Neural network training completed after " << iterations << " iterations." << std::endl;


		// === apply ml classifier to predict test set ===
		int t = 0, f = 0;
		std::vector<int> true_predictions_rank_fold(true_predictions_rank.size(), 0);
		for(int i = 0; i < test_data.rows ; i++)
		{
			cv::Mat response(1, number_classes, CV_32FC1);
			cv::Mat sample = test_data.row(i);

			int correct_prediction_at_rank = 0;
			mlp.predict(sample, response);
			std::multimap<float, int, std::greater<float> > prediction_order;
			for (int j = 0; j < number_classes; j++)
				prediction_order.insert(std::pair<float, int>(response.at<float>(0, j), j));
			for (std::multimap<float, int, std::greater<float> >::iterator it=prediction_order.begin(); it!=prediction_order.end(); ++it, ++correct_prediction_at_rank)
				if (it->second == test_labels.at<float>(i, 0))
					break;
			true_predictions_rank_fold[correct_prediction_at_rank]++;

			int predicted_class = prediction_order.begin()->second;	// Neural Network
//			int predicted_class = svm.predict(sample);	// SVM
//			int predicted_class = rtree.predict(sample);	// Random Tree
//			int predicted_class = bayesmod.predict(sample);	// Normal Bayes Classifier

			if (predicted_class == test_labels.at<float>(i, 0))
				t++;
			else
				f++;

			std::cout << "value: " << test_labels.at<float>(i, 0) << " (" << texture_classes[test_labels.at<float>(i, 0)] << ")\tpredicted: " << predicted_class << " (" << texture_classes[predicted_class] << ")\tcorrect prediction at rank: " << correct_prediction_at_rank << std::endl;
			screen_output << "value: " << test_labels.at<float>(i, 0) << " (" << texture_classes[test_labels.at<float>(i, 0)] << ")\tpredicted: " << predicted_class << " (" << texture_classes[predicted_class] << ")\tcorrect prediction at rank: " << correct_prediction_at_rank << std::endl;
		}

		std::cout << "Ranking distribution of correct predictions:" << std::endl;	screen_output << "Ranking distribution of correct predictions:" << std::endl;
		for (size_t i=0; i<true_predictions_rank.size(); ++i)
		{
			true_predictions_rank[i] += true_predictions_rank_fold[i];
			std::cout << true_predictions_rank_fold[i] << "\t";	screen_output << true_predictions_rank_fold[i] << "\t";
		}
		std::cout << std::endl;		screen_output << std::endl;

		true_predictions.push_back(t);
		false_predictions.push_back(f);
		double sum = t + f;
		double percentage = t*(100.0 / sum);
		std::cout << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << percentage << "%" << std::endl;
		screen_output << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << percentage << "%" << std::endl;

		// End Neural Network
	}

	std::cout << "=== Total result over " << folds << "-fold cross validation ===" << std::endl;
	screen_output << "=== Total result over " << folds << "-fold cross validation ===" << std::endl;

	std::cout << "Ranking distribution of correct predictions:" << std::endl << "numbers:\t";	screen_output << "Ranking distribution of correct predictions:" << std::endl << "numbers:\t";
	double total_number_samples = 0;
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
		total_number_samples += true_predictions_rank[i];
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
	{
		std::cout << true_predictions_rank[i] << "\t";	screen_output << true_predictions_rank[i] << "\t";
	}
	std::cout << std::endl << "percentages:\t";		screen_output << std::endl << "percentages:\t";
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
	{
		std::cout << 100*(double)true_predictions_rank[i]/total_number_samples << "\t";	screen_output << 100*(double)true_predictions_rank[i]/total_number_samples << "\t";
	}
	std::cout << std::endl << "accumulated %:\t";		screen_output << std::endl << "accumulated %:\t";
	double sum = 0.;
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
	{
		sum += (double)true_predictions_rank[i];
		std::cout << 100*sum/total_number_samples << "\t";	screen_output << 100*sum/total_number_samples << "\t";
	}
	std::cout << std::endl;		screen_output << std::endl;

	int t=0, f=0;
	for (unsigned int i=0; i<true_predictions.size(); ++i)
	{
		t += true_predictions[i];
		f += false_predictions[i];
	}
	std::cout << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << t*100.0/(double)(t+f) << "%" << std::endl;
	screen_output << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << t*100.0/(double)(t+f) << "%" << std::endl;

	// write screen outputs to file
	std::string logfilename = "texture_categorization/screen_output_classification.txt";
	std::ofstream file(logfilename.c_str(), std::ios::out);
	if (file.is_open() == true)
		file << screen_output.str();
	else
		std::cout << "Error: could not write screen output to file " << logfilename << "." << std::endl;
	file.close();
}


void train_ml::cross_validation_with_generated_attributes(int folds, const std::vector<cv::Mat>& computed_attribute_matrices, const cv::Mat& class_label_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy,
		const cv::Mat& generated_attributes_matrix, const cv::Mat& generated_attributes_class_label_matrix, const create_train_data::DataHierarchyType generated_attributes_data_sample_hierarchy)
{
	const int number_classes = 57;
	std::vector<int> true_predictions, false_predictions, true_predictions_rank(number_classes, 0);
	create_train_data data_object;
	std::vector<std::string> texture_classes = data_object.get_texture_classes();
	std::stringstream screen_output;

	std::set<int> considered_classes;

	srand(0);	// random seed --> keep reproducible
	for (size_t fold=0; fold<(size_t)folds; ++fold)
	{
		if (considered_classes.size() > 0 && considered_classes.find(fold) == considered_classes.end())
			continue;

		std::cout << "=== fold " << fold+1 << " ===" << std::endl;		screen_output << "=== fold " << fold+1 << " ===" << std::endl;
		const cv::Mat& computed_attribute_matrix = computed_attribute_matrices[fold];

		// === distribute data into training and test set ===
		std::vector<int> train_indices, train_indices_generated_attributes, test_indices;
		// select the class left out on attribute training for testing, training data comes from verbal description
		for (unsigned int class_index=0; class_index<data_sample_hierarchy.size(); ++class_index)
		{
			if (considered_classes.size() > 0 && considered_classes.find(class_index) == considered_classes.end())
				continue;

			if (class_index == fold)
			{
				// use computed attributes from this class as test case
				int object_number = data_sample_hierarchy[class_index].size();
				for (int object_index=0; object_index<object_number; ++object_index)
					for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
						{
							test_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
							//std::cout << data_sample_hierarchy[class_index][object_index][s] << "\t";
							screen_output << data_sample_hierarchy[class_index][object_index][s] << "\t";
						}
	// todo: try out using the full verbal set for training, in addition
//				// take the generated attributes (e.g. from verbal description) of this class for training the classifier
//				object_number = generated_attributes_data_sample_hierarchy[class_index].size();
//				for (int object_index=0; object_index<object_number; ++object_index)
//					for (unsigned int s=0; s<generated_attributes_data_sample_hierarchy[class_index][object_index].size(); ++s)
//						train_indices_generated_attributes.push_back(generated_attributes_data_sample_hierarchy[class_index][object_index][s]);
			}
			else
			{
				// use computed attributes for remaining classes as training data
				int object_number = data_sample_hierarchy[class_index].size();
				for (int object_index=0; object_index<object_number; ++object_index)
					for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
						train_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
			}
			// take the generated attributes (e.g. from verbal description) as additional training input to the classifier
			int object_number = generated_attributes_data_sample_hierarchy[class_index].size();
			for (int object_index=0; object_index<object_number; ++object_index)
				for (unsigned int s=0; s<generated_attributes_data_sample_hierarchy[class_index][object_index].size(); ++s)
					train_indices_generated_attributes.push_back(generated_attributes_data_sample_hierarchy[class_index][object_index][s]);
		}

		// create training and test data matrices
		cv::Mat training_data(train_indices.size()+train_indices_generated_attributes.size(), computed_attribute_matrix.cols, computed_attribute_matrix.type());
		cv::Mat training_labels(train_indices.size()+train_indices_generated_attributes.size(), 1, class_label_matrix.type());
		cv::Mat test_data(test_indices.size(), computed_attribute_matrix.cols, computed_attribute_matrix.type());
		cv::Mat test_labels(test_indices.size(), 1, class_label_matrix.type());
		std::cout << "training class " << (fold+1)%number_classes << " (" << texture_classes[(fold+1)%number_classes] << "):" << std::endl;
		for (unsigned int r=0; r<train_indices.size(); ++r)
		{
			for (int c=0; c<computed_attribute_matrix.cols; ++c)
				training_data.at<float>(r,c) = computed_attribute_matrix.at<float>(train_indices[r],c);
			training_labels.at<float>(r) = class_label_matrix.at<float>(train_indices[r]);

			if (class_label_matrix.at<float>(train_indices[r]) == (fold+1)%number_classes)	// attribute prediction for next class
			{
				for (int c=0; c<computed_attribute_matrix.cols; ++c)
					std::cout << computed_attribute_matrix.at<float>(train_indices[r],c) << "\t";
				std::cout << std::endl;
			}
		}
		for (unsigned int r=0; r<train_indices_generated_attributes.size(); ++r)
		{
			for (int c=0; c<generated_attributes_matrix.cols; ++c)
				training_data.at<float>(train_indices.size()+r,c) = generated_attributes_matrix.at<float>(train_indices_generated_attributes[r],c);
			training_labels.at<float>(train_indices.size()+r) = generated_attributes_class_label_matrix.at<float>(train_indices_generated_attributes[r]);
		}
		std::cout << "test class " << fold << " (" << texture_classes[fold] << "):" << std::endl;
		for (unsigned int r=0; r<test_indices.size(); ++r)
		{
			for (int c=0; c<computed_attribute_matrix.cols; ++c)
				test_data.at<float>(r,c) = computed_attribute_matrix.at<float>(test_indices[r],c);
			test_labels.at<float>(r) = class_label_matrix.at<float>(test_indices[r]);

			if (class_label_matrix.at<float>(test_indices[r]) == fold)
			{
				for (int c=0; c<computed_attribute_matrix.cols; ++c)
					std::cout << computed_attribute_matrix.at<float>(test_indices[r],c) << "\t";
				std::cout << std::endl;
			}
		}

		AttributeLearning al;
		create_train_data::DataHierarchyType temp_hierarchy(fold+1, std::vector< std::vector<int> >(1));
		for (int r=0; r<training_labels.rows; ++r)
			if (training_labels.at<float>(r) == fold)
				temp_hierarchy[fold][0].push_back(r);
		al.displayAttributes(training_data, temp_hierarchy, fold, false);
		temp_hierarchy.clear();
		temp_hierarchy.resize(fold+1, std::vector< std::vector<int> >(1));
		for (int r=0; r<test_labels.rows; ++r)
			if (test_labels.at<float>(r) == fold)
				temp_hierarchy[fold][0].push_back(r);
		al.displayAttributes(test_data, temp_hierarchy, fold, true, true);

		// === train ml classifier ===

//		// K-Nearest-Neighbor
//		CvKNearest knn;
//		knn.train(training_data, training_labels, cv::Mat(), false, 32, false);

//	Decision Tree

//    CvDTreeParams params = CvDTreeParams(25, // max depth
//                                             5, // min sample count
//                                             0, // regression accuracy: N/A here
//                                             false, // compute surrogate split, no missing data
//                                             15, // max number of categories (use sub-optimal algorithm for larger numbers)
//                                             15, // the number of cross-validation folds
//                                             false, // use 1SE rule => smaller tree
//                                             false, // throw away the pruned tree branches
//                                             NULL // the array of priors
//                                            );
//    CvDTree dtree;
//    dtree.train(training_data, CV_ROW_SAMPLE, training_label, cv::Mat(), cv::Mat(),cv::Mat(),cv::Mat(), params);
//
//    CvDTreeNode *node;
//
//    cv::Mat result(train_data.rows,1,CV_32FC1);
//    	for(int i=0;i<train_data.rows;i++)
//    	{
//    		cv::Mat inputvec(1,train_data.cols, CV_32FC1);
//    		for(int j=0;j<train_data.cols;j++)
//    		{
//    			inputvec.at<float>(0,j)=train_data.at<float>(i,j);
//    		}
//    		node = dtree.predict(inputvec);
//    		result.at<float>(i,0)= (*node).class_idx;
//    	}

//	End Dicision Tree

//	BayesClassifier

//	CvNormalBayesClassifier bayesmod;
//	bayesmod.train(training_data, training_labels, cv::Mat(), cv::Mat());
////	 cv::Mat result(train_data.rows,1,CV_32FC1);
////	bayesmod.predict(train_data, &result);

//	End Bayes Classifier

//		// Random trees
//		float* priors = 0;
//		//CvRTParams rtree_params(78, 6, 0.0, false,28, priors, false,23,28,1.0, 0 );
//		CvRTParams rtree_params(100, 0.0005*training_data.rows, 0.f, false, number_classes, priors, false, training_data.cols, 100, 0.01/*FLT_EPSILON*/, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS);
//		cv::Mat var_type = cv::Mat::ones(training_data.cols+1, 1, CV_8U) * CV_VAR_NUMERICAL;
//		var_type.at<int>(training_data.cols) = CV_VAR_CATEGORICAL;
//		CvRTrees rtree;
//		rtree.train(training_data, CV_ROW_SAMPLE, training_labels, cv::Mat(), cv::Mat(), var_type, cv::Mat(), rtree_params);
//		// End Random trees

//		// SVM
//		CvSVM svm;
//		CvTermCriteria criteria;
//		criteria.max_iter = 1000;//1000;	// 1000
//		criteria.epsilon  = FLT_EPSILON; // FLT_EPSILON
//		criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
//		CvSVMParams svm_params(CvSVM::C_SVC, CvSVM::RBF, 0., 1.0, 0., 1.0, 0.0, 0., 0, criteria);		// RBF, 0.0, 0.1, 0.0, 1.0, 0.4, 0.
//		svm.train(training_data, training_labels, cv::Mat(), cv::Mat(), svm_params);

		// Neural Network
		cv::Mat input;
		training_data.convertTo(input, CV_32F);
		cv::Mat output=cv::Mat::zeros(training_data.rows, number_classes, CV_32FC1);
		cv::Mat labels;
		training_labels.convertTo(labels, CV_32F);
		for(int i=0; i<training_data.rows; ++i)
			output.at<float>(i,(int)labels.at<float>(i,0)) = 1.f;		//change.at<float>(i,0);

		cv::Mat layers = cv::Mat(3,1,CV_32SC1);
		layers.row(0) = cv::Scalar(training_data.cols);
		layers.row(1) = cv::Scalar(400);	//400
		layers.row(2) = cv::Scalar(number_classes);

		CvANN_MLP mlp;
		CvANN_MLP_TrainParams params;
		CvTermCriteria criteria;

		criteria.max_iter = 400;
		criteria.epsilon  = 0.0001f;
		criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;

		params.train_method    = CvANN_MLP_TrainParams::BACKPROP;
		params.bp_dw_scale     = 0.1f;
		params.bp_moment_scale = 0.1f;
		params.term_crit       = criteria;

		mlp.create(layers,CvANN_MLP::SIGMOID_SYM,0.4,1.2);
		int iterations = mlp.train(input, output, cv::Mat(), cv::Mat(), params);
		std::cout << "Neural network training completed after " << iterations << " iterations." << std::endl;		screen_output << "Neural network training completed after " << iterations << " iterations." << std::endl;


		// === apply ml classifier to predict test set ===
		int t = 0, f = 0;
		std::vector<int> true_predictions_rank_fold(true_predictions_rank.size(), 0);
		for(int i = 0; i < test_data.rows ; i++)
		{
			cv::Mat response(1, number_classes, CV_32FC1);
			cv::Mat sample = test_data.row(i);

			int correct_prediction_at_rank = 0;
			mlp.predict(sample, response);
			std::multimap<float, int, std::greater<float> > prediction_order;
			for (int j = 0; j < number_classes; j++)
				prediction_order.insert(std::pair<float, int>(response.at<float>(0, j), j));
			for (std::multimap<float, int, std::greater<float> >::iterator it=prediction_order.begin(); it!=prediction_order.end(); ++it, ++correct_prediction_at_rank)
				if (it->second == test_labels.at<float>(i, 0))
					break;
			true_predictions_rank_fold[correct_prediction_at_rank]++;

			int predicted_class = prediction_order.begin()->second;	// Neural Network
//			int predicted_class = svm.predict(sample);	// SVM
//			int predicted_class = std::max(0, std::min(number_classes-1, (int)rtree.predict(sample)));	// Random Tree
//			int predicted_class = bayesmod.predict(sample);	// Normal Bayes Classifier
//			int predicted_class = knn.find_nearest(sample, 1);

			if (predicted_class == test_labels.at<float>(i, 0))
				t++;
			else
				f++;
			std::cout << "value: " << test_labels.at<float>(i, 0) << " (" << texture_classes[test_labels.at<float>(i, 0)] << ")\tpredicted: " << predicted_class << " (" << texture_classes[predicted_class] << ")\tcorrect prediction at rank: " << correct_prediction_at_rank << "\tclassifier raw value: " << prediction_order.begin()->first << std::endl;
			screen_output << "value: " << test_labels.at<float>(i, 0) << " (" << texture_classes[test_labels.at<float>(i, 0)] << ")\tpredicted: " << predicted_class << " (" << texture_classes[predicted_class] << ")\tcorrect prediction at rank: " << correct_prediction_at_rank << "\tclassifier raw value: " << prediction_order.begin()->first << std::endl;
		}
		std::cout << "Ranking distribution of correct predictions:" << std::endl;	screen_output << "Ranking distribution of correct predictions:" << std::endl;
		for (size_t i=0; i<true_predictions_rank.size(); ++i)
		{
			true_predictions_rank[i] += true_predictions_rank_fold[i];
			std::cout << true_predictions_rank_fold[i] << "\t";	screen_output << true_predictions_rank_fold[i] << "\t";
		}
		std::cout << std::endl;		screen_output << std::endl;

		true_predictions.push_back(t);
		false_predictions.push_back(f);
		double sum = t + f;
		double percentage = t*(100.0 / sum);
		std::cout << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << percentage << "%" << std::endl;
		screen_output << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << percentage << "%" << std::endl;
	}

	std::cout << "=== Total result over " << folds << "-fold cross validation ===" << std::endl;
	screen_output << "=== Total result over " << folds << "-fold cross validation ===" << std::endl;

	std::cout << "Ranking distribution of correct predictions:" << std::endl << "numbers:\t";	screen_output << "Ranking distribution of correct predictions:" << std::endl << "numbers:\t";
	double total_number_samples = 0;
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
		total_number_samples += true_predictions_rank[i];
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
	{
		std::cout << true_predictions_rank[i] << "\t";	screen_output << true_predictions_rank[i] << "\t";
	}
	std::cout << std::endl << "percentages:\t";		screen_output << std::endl << "percentages:\t";
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
	{
		std::cout << 100*(double)true_predictions_rank[i]/total_number_samples << "\t";	screen_output << 100*(double)true_predictions_rank[i]/total_number_samples << "\t";
	}
	std::cout << std::endl << "accumulated %:\t";		screen_output << std::endl << "accumulated %:\t";
	double sum = 0.;
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
	{
		sum += (double)true_predictions_rank[i];
		std::cout << 100*sum/total_number_samples << "\t";	screen_output << 100*sum/total_number_samples << "\t";
	}
	std::cout << std::endl;		screen_output << std::endl;

	int t=0, f=0;
	for (unsigned int i=0; i<true_predictions.size(); ++i)
	{
		t += true_predictions[i];
		f += false_predictions[i];
	}
	std::cout << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << t*100.0/(double)(t+f) << "%" << std::endl;
	screen_output << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << t*100.0/(double)(t+f) << "%" << std::endl;

	// write screen outputs to file
	std::string logfilename = "texture_categorization/screen_output_classification.txt";
	std::ofstream file(logfilename.c_str(), std::ios::out);
	if (file.is_open() == true)
		file << screen_output.str();
	else
		std::cout << "Error: could not write screen output to file " << logfilename << "." << std::endl;
	file.close();
}


void train_ml::train(const cv::Mat& training_data, const cv::Mat& training_labels)
{
	int number_classes = 0;
	label_class_mapping_.clear();
	class_label_mapping_.clear();
	for (int r=0; r<training_labels.rows; ++r)
	{
		if (label_class_mapping_.find(training_labels.at<float>(r)) == label_class_mapping_.end())
		{
			label_class_mapping_[training_labels.at<float>(r)] = number_classes;
			class_label_mapping_[number_classes] = training_labels.at<float>(r);
			std::cout << "mapping original label: " << training_labels.at<float>(r) << " --> " << number_classes << std::endl;
			number_classes++;
		}
	}

	// Neural Network
	cv::Mat input;
	training_data.convertTo(input, CV_32F);
	cv::Mat output=cv::Mat::zeros(training_data.rows, number_classes, CV_32FC1);
	cv::Mat labels;
	training_labels.convertTo(labels, CV_32F);
	for(int i=0; i<labels.rows; ++i)
		output.at<float>(i,label_class_mapping_[(int)labels.at<float>(i,0)]) = 1.f;		//change.at<float>(i,0);

	cv::Mat layers = cv::Mat(3,1,CV_32SC1);
	layers.row(0) = cv::Scalar(training_data.cols);
	layers.row(1) = cv::Scalar(400);	//400
	layers.row(2) = cv::Scalar(number_classes);

	CvANN_MLP_TrainParams params;
	CvTermCriteria criteria;

	criteria.max_iter = 400;
	criteria.epsilon  = 0.0001f;
	criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;

	params.train_method    = CvANN_MLP_TrainParams::BACKPROP;
	params.bp_dw_scale     = 0.1f;
	params.bp_moment_scale = 0.1f;
	params.term_crit       = criteria;

	mlp_.create(layers,CvANN_MLP::SIGMOID_SYM,0.4,1.2);
	int iterations = mlp_.train(input, output, cv::Mat(), cv::Mat(), params);
	std::cout << "Neural network training completed after " << iterations << " iterations." << std::endl;//		screen_output << "Neural network training completed after " << iterations << " iterations." << std::endl;
}


void train_ml::predict(const cv::Mat& test_data, const cv::Mat& test_labels, cv::Mat& predicted_labels)
{
	const int number_classes = (int)class_label_mapping_.size();
	predicted_labels.create(test_labels.rows, test_labels.cols, CV_32FC1);

	// === apply ml classifier to predict test set ===
	create_train_data data_object;
	std::vector<std::string> texture_classes = data_object.get_texture_classes();
	std::stringstream screen_output;
	int t = 0, f = 0;
	std::vector<int> true_predictions_rank(number_classes, 0);
	for(int i = 0; i < test_data.rows ; i++)
	{
		cv::Mat response(1, number_classes, CV_32FC1);
		cv::Mat sample = test_data.row(i);

		int correct_prediction_at_rank = 0;
		mlp_.predict(sample, response);
		std::multimap<float, int, std::greater<float> > prediction_order;
		for (int j = 0; j < number_classes; j++)
			prediction_order.insert(std::pair<float, int>(response.at<float>(0, j), class_label_mapping_[j]));
		for (std::multimap<float, int, std::greater<float> >::iterator it=prediction_order.begin(); it!=prediction_order.end(); ++it, ++correct_prediction_at_rank)
			if (it->second == test_labels.at<float>(i, 0))
				break;
		true_predictions_rank[correct_prediction_at_rank]++;

		int predicted_class = prediction_order.begin()->second;	// Neural Network
//		int predicted_class = svm.predict(sample);	// SVM
//		int predicted_class = rtree.predict(sample);	// Random Tree
//		int predicted_class = bayesmod.predict(sample);	// Normal Bayes Classifier

		predicted_labels.at<float>(i, 0) = predicted_class;
		if (predicted_class == test_labels.at<float>(i, 0))
			t++;
		else
			f++;

		std::cout << "value: " << test_labels.at<float>(i, 0) << " (" << texture_classes[test_labels.at<float>(i, 0)] << ")\tpredicted: " << predicted_class << " (" << texture_classes[predicted_class] << ")\tcorrect prediction at rank: " << correct_prediction_at_rank << std::endl;
		screen_output << "value: " << test_labels.at<float>(i, 0) << " (" << texture_classes[test_labels.at<float>(i, 0)] << ")\tpredicted: " << predicted_class << " (" << texture_classes[predicted_class] << ")\tcorrect prediction at rank: " << correct_prediction_at_rank << std::endl;
	}

	std::cout << "\nRanking distribution of correct predictions:\nnumbers:" << std::endl;	screen_output << "\nRanking distribution of correct predictions:\nnumbers:" << std::endl;
	double total_number_samples = 0;
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
		total_number_samples += true_predictions_rank[i];
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
	{
		std::cout << true_predictions_rank[i] << "\t";	screen_output << true_predictions_rank[i] << "\t";
	}
	std::cout << std::endl << "percentages:\n";		screen_output << std::endl << "percentages:\n";
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
	{
		std::cout << 100*(double)true_predictions_rank[i]/total_number_samples << "\t";	screen_output << 100*(double)true_predictions_rank[i]/total_number_samples << "\t";
	}
	std::cout << std::endl << "accumulated %:\n";		screen_output << std::endl << "accumulated %:\n";
	double sum = 0.;
	for (size_t i=0; i<true_predictions_rank.size(); ++i)
	{
		sum += (double)true_predictions_rank[i];
		std::cout << 100*sum/total_number_samples << "\t";	screen_output << 100*sum/total_number_samples << "\t";
	}
	std::cout << "\n" << std::endl;		screen_output << "\n" << std::endl;

	sum = t + f;
	double percentage = t*(100.0 / sum);
	std::cout << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << percentage << "%" << std::endl;
	screen_output << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << percentage << "%" << std::endl;

	// write screen outputs to file
	std::string logfilename = "texture_categorization/screen_output_classification.txt";
	std::ofstream file(logfilename.c_str(), std::ios::out);
	if (file.is_open() == true)
		file << screen_output.str();
	else
		std::cout << "Error: could not write screen output to file " << logfilename << "." << std::endl;
	file.close();
}


void train_ml::save_mlp(std::string path)
{
	std::string filename = path + "mlp.yml";
	std::cout << "Saving MLP to file " << filename << std::endl;
	mlp_.save(filename.c_str(), "mlp");

	// save mappings
	filename = path + "mlp.txt";
	std::ofstream file(filename.c_str(), std::ios::out);
	if (file.is_open() == true)
	{
		file << label_class_mapping_.size() << std::endl;
		for (std::map<float, float>::iterator it=label_class_mapping_.begin(); it!=label_class_mapping_.end(); ++it)
			file << it->first << "\t" << it->second << std::endl;
		file << class_label_mapping_.size() << std::endl;
		for (std::map<float, float>::iterator it=class_label_mapping_.begin(); it!=class_label_mapping_.end(); ++it)
			file << it->first << "\t" << it->second << std::endl;
	}
	else
		std::cout << "Error: could not write to file " << filename << ".\n";
	file.close();
}


void train_ml::load_mlp(std::string path)
{
	std::string filename = path + "mlp.yml";
	mlp_.load(filename.c_str(), "mlp");

	// load mappings
	label_class_mapping_.clear();
	class_label_mapping_.clear();
	filename = path + "mlp.txt";
	std::ifstream file(filename.c_str(), std::ios::in);
	if (file.is_open() == true)
	{
		size_t length = 0;
		file >> length;
		for (size_t i=0; i<length; ++i)
		{
			float first, second;
			file >> first >> second;
			label_class_mapping_[first] = second;
		}
		file >> length;
		for (size_t i=0; i<length; ++i)
		{
			float first, second;
			file >> first >> second;
			class_label_mapping_[first] = second;
		}
	}
	else
		std::cout << "Error: could not read from file " << filename << ".\n";
	file.close();
}


void train_ml::save_computed_attribute_matrices(std::string path, const std::vector<cv::Mat>& computed_attribute_matrices)
{
	// save data
	std::string data = "ipa_database_computed_attributes_cv_data.yml";
	std::string path_data = path + data;
	std::cout << "Saving data to file " << path_data << " ... ";
	cv::FileStorage fs(path_data, cv::FileStorage::WRITE);
	if (fs.isOpened() == true)
	{
		fs << "computed_attribute_matrices" << computed_attribute_matrices;
	}
	else
		std::cout << "Error: could not open file '" << path_data << "' for writing."<< std::endl;
	fs.release();
	std::cout << "done." << std::endl;
}


void train_ml::load_computed_attribute_matrices(std::string path, std::vector<cv::Mat>& computed_attribute_matrices)
{
	// load data
	std::string data = "ipa_database_computed_attributes_cv_data.yml";
	std::string path_data = path + data;
	std::cout << "Loading data from file " << path_data << " ... " << std::flush;
	cv::FileStorage fs(path_data, cv::FileStorage::READ);
	if (fs.isOpened() == true)
	{
		fs["computed_attribute_matrices"] >> computed_attribute_matrices;
	}
	else
		std::cout << "Error: could not open file '" << path_data << "' for reading."<< std::endl;
	fs.release();

	std::cout << "done." << std::endl;
}


// ===============================================================================
// code grave yard




//void train_ml::newClassTest(const cv::Mat& input, const cv::Mat& output,const cv::Mat& orig)
//{
//	std::vector<int> true_predictions, false_predictions;
//	create_train_data data_object;
//	std::vector<std::string> texture_classes = data_object.get_texture_classes();
////	std::stringstream screen_output;
////
////	bool use_preselected_set_distribution = ((int)preselected_train_indices.size()==folds && (int)feature_matrix_test_data.size()==folds && (int)label_matrix_test_data.size()==folds);
////	if (use_preselected_set_distribution == true)
////	{
////		std::cout << "Using the provided pre-selected sets for training and testing." << std::endl;		screen_output << "Using the provided pre-selected sets for training and testing." << std::endl;
////	}
////	else
////	{
////		std::cout << "Computing the individual training and testing sets for each fold." << std::endl;		screen_output << "Computing the individual training and testing sets for each fold." << std::endl;
////	}
////
////	srand(0);	// random seed --> keep reproducible
////	for (int fold=0; fold<folds; ++fold)
////	{
////		std::cout << "=== fold " << fold+1 << " ===" << std::endl;		screen_output << "=== fold " << fold+1 << " ===" << std::endl;
////
////		// === distribute data into training and test set ===
////		std::vector<int> train_indices, test_indices;
////		if (use_preselected_set_distribution==true)
////		{
////			// just take the provided training indices for this fold
////			if (feature_matrix_test_data[fold].empty()==true || feature_matrix_test_data[fold].cols!=feature_matrix.cols || label_matrix_test_data[fold].empty()==true || label_matrix_test_data[fold].cols!=label_matrix.cols || feature_matrix_test_data[fold].rows!=label_matrix_test_data[fold].rows)
////				std::cout << "Error: provided pre-computed test data and label matrices are not suitable." << std::endl;
////			train_indices = preselected_train_indices[fold];
////			test_indices.resize(feature_matrix_test_data[fold].rows);
////		}
////		else
////		{
////			// select one object per class for testing
////			for (unsigned int class_index=0; class_index<data_sample_hierarchy.size(); ++class_index)
////			{
////				int object_number = data_sample_hierarchy[class_index].size();
////				int test_object = (int)(object_number * (double)rand()/((double)RAND_MAX+1.0));
////	//			std::cout << "object_number=" << object_number << "   test_object=" << test_object << std::endl;
////				for (int object_index=0; object_index<object_number; ++object_index)
////				{
////					if (object_index == test_object)
////						for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
////							test_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
////					else
////						for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
////							train_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
////				}
////			}
////		}
////		assert(test_indices.size() + train_indices.size() == (unsigned int)feature_matrix.rows);
////
////		// create training and test data matrices
////		cv::Mat training_data(train_indices.size(), feature_matrix.cols, feature_matrix.type());
////		cv::Mat training_labels(train_indices.size(), 1, label_matrix.type());
//		cv::Mat test_data(17, 16, CV_32F);
//
//		cv::Mat test_labels(17, 57, CV_32F);
//		for(int i=0;i<17;i++)
//		{
//			test_labels.at<float>(i,53)=1.f;
//		}
//		int place=0;
//		for(int i=0;i<1281;i++)
//		{
//			if(output.at<float>(i,0)==53)
//			{
//				for(int j=0;j<16;j++)
//				{
//					test_data.at<float>(place,j)=orig.at<float>(i,j);
//				}
//				place++;
//			}
//		}
////		for (unsigned int r=0; r<train_indices.size(); ++r)
////		{
////			for (int c=0; c<feature_matrix.cols; ++c)
////				training_data.at<float>(r,c) = feature_matrix.at<float>(train_indices[r],c);
////			training_labels.at<float>(r) = label_matrix.at<float>(train_indices[r]);
////		}
////		if (use_preselected_set_distribution==true)
////		{
////			test_data = feature_matrix_test_data[fold];
////			test_labels = label_matrix_test_data[fold];
////		}
////		else
////		{
////			for (unsigned int r=0; r<test_indices.size(); ++r)
////			{
////				for (int c=0; c<feature_matrix.cols; ++c)
////					test_data.at<float>(r,c) = feature_matrix.at<float>(test_indices[r],c);
////				test_labels.at<float>(r) = label_matrix.at<float>(test_indices[r]);
////			}
////		}
//
//		// === train ml classifier ===
//
////	std::cout<<"Value:"<<test_data_label.at<float>(3,0)<<std::endl;
////	K-Nearest-Neighbor
//
////	cv::Mat test;
////	CvKNearest knn(training_data, training_label, cv::Mat(), false, 32);
////	knn.train(training_data,training_label,test,false,32,false );
////
////	cv::Mat result;
////	cv::Mat neighbor_results;
////	cv::Mat dist;
////	knn.find_nearest(train_data, 1, &result,0,&neighbor_results, &dist);
//
//// 	End K-Nearest-Neighbor
//
////	Randomtree
//
////	float v1 = 0.0;
////	const float *v2 = 0;
////	float v3 = 1.0;
////	CvRTParams treetest(78, 6, v1, false,28, v2, false,23,28,v3, 0 );
////	CvRTrees tree;
////	tree.train(training_data,1, training_label, cv::Mat(), cv::Mat(),cv::Mat(),cv::Mat(), treetest);
////	cv::Mat result(train_data.rows,1,CV_32FC1);
////
////	for(int i=0;i<train_data.rows;i++)
////	{
////		cv::Mat inputvec(1,train_data.cols, CV_32FC1);
////		for(int j=0;j<train_data.cols;j++)
////		{
////			inputvec.at<float>(0,j)=train_data.at<float>(i,j);
////		}
////		result.at<float>(i,0)=tree.predict(inputvec);
////	}
//
////	End Randomtree
//
////	Decision Tree
//
////    CvDTreeParams params = CvDTreeParams(25, // max depth
////                                             5, // min sample count
////                                             0, // regression accuracy: N/A here
////                                             false, // compute surrogate split, no missing data
////                                             15, // max number of categories (use sub-optimal algorithm for larger numbers)
////                                             15, // the number of cross-validation folds
////                                             false, // use 1SE rule => smaller tree
////                                             false, // throw away the pruned tree branches
////                                             NULL // the array of priors
////                                            );
////    CvDTree dtree;
////    dtree.train(training_data, CV_ROW_SAMPLE, training_label, cv::Mat(), cv::Mat(),cv::Mat(),cv::Mat(), params);
////
////    CvDTreeNode *node;
////
////    cv::Mat result(train_data.rows,1,CV_32FC1);
////    	for(int i=0;i<train_data.rows;i++)
////    	{
////    		cv::Mat inputvec(1,train_data.cols, CV_32FC1);
////    		for(int j=0;j<train_data.cols;j++)
////    		{
////    			inputvec.at<float>(0,j)=train_data.at<float>(i,j);
////    		}
////    		node = dtree.predict(inputvec);
////    		result.at<float>(i,0)= (*node).class_idx;
////    	}
//
////	End Dicision Tree
//
////	BayesClassifier
//
////	CvNormalBayesClassifier bayesmod;
////	bayesmod.train(training_data, training_label, cv::Mat(), cv::Mat());
////	 cv::Mat result(train_data.rows,1,CV_32FC1);
////	bayesmod.predict(train_data, &result);
//
////	End Bayes Classifier
//
//		// Neural Network
////		cv::Mat input;
////		training_data.convertTo(input, CV_32F);
////		cv::Mat output=cv::Mat::zeros(training_data.rows, 57, CV_32FC1);
////		cv::Mat labels;
////		training_labels.convertTo(labels, CV_32F);
//
//
//		cv::Mat outputtest=cv::Mat::zeros(1281,57,CV_32F);
//
//		for(int i=0; i<output.rows; ++i)
//			outputtest.at<float>(i,(int)output.at<float>(i,0)) = 1.f;		//change.at<float>(i,0);
//
//		cv::Mat layers = cv::Mat(3,1,CV_32SC1);
//		layers.row(0) = cv::Scalar(input.cols);
//		layers.row(1) = cv::Scalar(400);	//400
//		layers.row(2) = cv::Scalar(57);
//
//		CvANN_MLP mlp;
//		CvANN_MLP_TrainParams params;
//		CvTermCriteria criteria;
//
//		criteria.max_iter = 400;
//		criteria.epsilon  = 0.0001f;
//		criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
//
//		params.train_method    = CvANN_MLP_TrainParams::BACKPROP;
//		params.bp_dw_scale     = 0.1f;
//		params.bp_moment_scale = 0.1f;
//		params.term_crit       = criteria;
//
//		mlp.create(layers,CvANN_MLP::SIGMOID_SYM,0.4,1.2);
//		int iterations = mlp.train(input, outputtest, cv::Mat(), cv::Mat(), params);
//		std::cout << "Neural network training completed after " << iterations << " iterations." << std::endl;	//	screen_output << "Neural network training completed after " << iterations << " iterations." << std::endl;
//
//		// === apply ml classifier to predict test set ===
//		int t = 0, f = 0;
////		int t2 = 0, f2 = 0;
//		std::vector<int> labelres;
//
//		for(int i = 0; i < test_data.rows ; i++)
//		{
//			cv::Mat response(1, 57, CV_32FC1);
//			cv::Mat sample = test_data.row(i);
//
//			mlp.predict(sample, response);
//			//float lastmax;
//			//float best[8];
//			float bestpos[8];
//			float max = -1000000000000.0f;
////			float max2 = -1000000000000.0f;
//			int cls = -1;
////			int cls2 = -1;
//			for (int j = 0; j < 57; j++)
//			{
//				float value = response.at<float>(0, j);
//				if (value > max)
//				{
////					max2 = max;
////					cls2 = cls;
//					max = value;
//					cls = j;
//				}
//			}
//
//			//test find best 5
//			for(int z=0; z<8;z++)
//			{
//				cv::Point min_loc, max_loc;
//				double min1, max1;
//				cv::minMaxLoc(response, &min1, &max1, &min_loc, &max_loc);
//				//best[z]=max1;
//				bestpos[z]=max_loc.x;
//				response.at<float>(max_loc)=-100000;
//			}
//
//			if (cls == test_labels.at<float>(i, 0))
//				t++;
//			else
//				f++;
////			if (cls2 == test_labels.at<float>(i, 0))
////				t2++;
////			else
////				f2++;
//			std::cout<<"Predicted";
//			for(int z=0;z<8;z++)
//				std::cout<<texture_classes[bestpos[z]]<<" ";
//
//			std::cout<<std::endl;
//
//			std::cout << "value: " << test_labels.at<float>(i, 0) << " (" << texture_classes[test_labels.at<float>(i, 0)] << ")\tpredicted: " << cls << " (" << texture_classes[cls] << ")" << std::endl;
////			screen_output << "value: " << test_labels.at<float>(i, 0) << " (" << texture_classes[test_labels.at<float>(i, 0)] << ")\tpredicted: " << cls << " (" << texture_classes[cls] << ")" << std::endl;
//		}
//		true_predictions.push_back(t);
//		false_predictions.push_back(f);
//		double sum = t + f;
//		double percentage = t*(100.0 / sum);
//		std::cout << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << percentage << "%" << std::endl;
////		screen_output << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << percentage << "%" << std::endl;
////		std::cout << "true2:" << t2 << " False2:" << f2 << std::endl;
//
////		 End Neural Network
////	}
//
////	std::cout << "=== Total result over " << folds << "-fold cross validation ===" << std::endl;
////	screen_output << "=== Total result over " << folds << "-fold cross validation ===" << std::endl;
//	t=0; f=0;
//	for (unsigned int i=0; i<true_predictions.size(); ++i)
//	{
//		t += true_predictions[i];
//		f += false_predictions[i];
//	}
//	std::cout << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << t*100.0/(double)(t+f) << "%" << std::endl;
////	screen_output << "true: " << t << "\tfalse: " << f << "\tcorrectly classified: " << t*100.0/(double)(t+f) << "%" << std::endl;
//
//	// write screen outputs to file
////	std::ofstream file("screen_output_classification.txt", std::ios::out);
////	if (file.is_open() == true)
////		file << screen_output.str();
////	else
////		std::cout << "Error: could not write screen output to file.";
////	file.close();
//}


//void train_ml::run_ml(double val, std::string *path_)
//{
//
//	std::string train_data = *path_ + "train_data.yml";
////	cv::FileStorage fs("/home/rmb-dh/Test_dataset/training_data.yml", cv::FileStorage::READ);
//	cv::FileStorage fs(train_data, cv::FileStorage::READ);
//	cv::Mat training_data;
//	fs["train_data"] >> training_data;
////	std::cout<<training_data<<"Matrix"<<std::endl;
//
//	std::string train_label = *path_ + "train_data_label.yml";
////	cv::FileStorage fsl("/home/rmb-dh/Test_dataset/train_data_respons.yml", cv::FileStorage::READ);
//	cv::FileStorage fsl(train_label, cv::FileStorage::READ);
//	cv::Mat training_labels;
//	fsl["train_label"] >> training_labels;
////	std::cout<<training_label<<"Matrix"<<std::endl;
//
//	cv::Mat test_data;
//	std::string test_data_ = *path_ + "test_data.yml";
////	cv::FileStorage fsd("/home/rmb-dh/Test_dataset/test_data.yml", cv::FileStorage::READ);
//	cv::FileStorage fsd(test_data_, cv::FileStorage::READ);
//	fsd["test_data"] >> test_data;
//
////	std::vector<std::string> test_data_label;
//	cv::Mat test_labels;
//	std::string test_label = *path_ + "test_data_label.yml";
////	cv::FileStorage fstl("/home/rmb-dh/Test_dataset/test_data_label.yml", cv::FileStorage::READ);
//	cv::FileStorage fstl(test_label, cv::FileStorage::READ);
//	fstl["test_label"] >> test_labels;
//
//
//	//	std::cout<<"Value:"<<test_data_label.at<float>(3,0)<<std::endl;
////	K-Nearest-Neighbor
//
////	cv::Mat test;
////	CvKNearest knn(training_data, training_label, cv::Mat(), false, 32);
////	knn.train(training_data,training_label,test,false,32,false );
////
////	cv::Mat result;
////	cv::Mat neighbor_results;
////	cv::Mat dist;
////	knn.find_nearest(train_data, 1, &result,0,&neighbor_results, &dist);
//
//// 	End K-Nearest-Neighbor
//
////	Randomtree
//
////	float v1 = 0.0;
////	const float *v2 = 0;
////	float v3 = 1.0;
////	CvRTParams treetest(78, 6, v1, false,28, v2, false,23,28,v3, 0 );
////	CvRTrees tree;
////	tree.train(training_data,1, training_label, cv::Mat(), cv::Mat(),cv::Mat(),cv::Mat(), treetest);
////	cv::Mat result(train_data.rows,1,CV_32FC1);
////
////	for(int i=0;i<train_data.rows;i++)
////	{
////		cv::Mat inputvec(1,train_data.cols, CV_32FC1);
////		for(int j=0;j<train_data.cols;j++)
////		{
////			inputvec.at<float>(0,j)=train_data.at<float>(i,j);
////		}
////		result.at<float>(i,0)=tree.predict(inputvec);
////	}
//
////	End Randomtree
//
////	Decision Tree
//
////    CvDTreeParams params = CvDTreeParams(25, // max depth
////                                             5, // min sample count
////                                             0, // regression accuracy: N/A here
////                                             false, // compute surrogate split, no missing data
////                                             15, // max number of categories (use sub-optimal algorithm for larger numbers)
////                                             15, // the number of cross-validation folds
////                                             false, // use 1SE rule => smaller tree
////                                             false, // throw away the pruned tree branches
////                                             NULL // the array of priors
////                                            );
////    CvDTree dtree;
////    dtree.train(training_data, CV_ROW_SAMPLE, training_label, cv::Mat(), cv::Mat(),cv::Mat(),cv::Mat(), params);
////
////    CvDTreeNode *node;
////
////    cv::Mat result(train_data.rows,1,CV_32FC1);
////    	for(int i=0;i<train_data.rows;i++)
////    	{
////    		cv::Mat inputvec(1,train_data.cols, CV_32FC1);
////    		for(int j=0;j<train_data.cols;j++)
////    		{
////    			inputvec.at<float>(0,j)=train_data.at<float>(i,j);
////    		}
////    		node = dtree.predict(inputvec);
////    		result.at<float>(i,0)= (*node).class_idx;
////    	}
//
////	End Dicision Tree
//
////	BayesClassifier
//
////	CvNormalBayesClassifier bayesmod;
////	bayesmod.train(training_data, training_label, cv::Mat(), cv::Mat());
////	 cv::Mat result(train_data.rows,1,CV_32FC1);
////	bayesmod.predict(train_data, &result);
//
////	End Bayes Classifier
//
////	Neural Network
//
////		cv::Mat input;
////		training_data.convertTo(input, CV_32F);
////		cv::Mat output=cv::Mat::zeros(training_data.rows,57, CV_32FC1);
//
//		cv::Mat input;
//		training_data.convertTo(input, CV_32F);
//		cv::Mat output=cv::Mat::zeros(training_data.rows,57, CV_32FC1);
//
//		cv::Mat change;
//		training_labels.convertTo(change, CV_32F);
//
//
//		for(float i=0;i<training_data.rows;i++)
//		{
//			output.at<float>(i,(int)change.at<float>(i,0))=1;//change.at<float>(i,0);
//		}
//
//
//		 cv::Mat layers = cv::Mat(3,1,CV_32SC1);
////		    int sz = train_data.cols ;
//
//		    layers.row(0) = cv::Scalar(16);
//		    layers.row(1) = cv::Scalar(400);
//		    layers.row(2) = cv::Scalar(57);
//
//		    CvANN_MLP mlp;
//		    CvANN_MLP_TrainParams params;
//		    CvTermCriteria criteria;
//
//		    criteria.max_iter = 1000;
//		    criteria.epsilon  = 0.00001f;
//		    criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
//
//		    params.train_method    = CvANN_MLP_TrainParams::BACKPROP;
//		    params.bp_dw_scale     = 0.1f;
//		    params.bp_moment_scale = 0.1f;
//		    params.term_crit       = criteria;
//
//		    mlp.create(layers,CvANN_MLP::SIGMOID_SYM,0.3,1.2);
//		    mlp.train(input,output,cv::Mat(),cv::Mat(),params);
//
//		    int t = 0, f = 0;
//		    int t2 = 0, f2 = 0;
//
//		    std::cout<<"training completed"<<std::endl;
//		    std::vector<int> labelres;
//		    for(int i = 0; i < test_data.rows ; i++)
//		    {
//		        cv::Mat response(1,57,CV_32FC1);
//		        cv::Mat sample = test_data.row(i);
//
//		        mlp.predict(sample,response);
//
//		        float max = -1000000000000.0f;
//		        //float max2 = -1000000000000.0f;
//		        int cls = -1;
//		        int cls2 = -1;
//
//		        for(int j = 0 ; j < 57 ; j++)
//		        {
//		            float value = response.at<float>(0,j);
//
//		            if(value > max)
//		            {
//		            	//max2=max;
//		            	cls2=cls;
//		                max = value;
//		                cls = j;
//		            }
//		        }
//
//		        if(cls == test_labels.at<float>(i,0))
//		            t++;
//		        else
//		            f++;
//
//		        if(cls2 == test_labels.at<float>(i,0))
//		       		            t2++;
//		       		        else
//		       		            f2++;
//
//
////		        std::cout<<"test"<<std::endl;
//		        std::cout<<"Value:"<<test_labels.at<float>(i,0)<<" Predicted:"<<cls<<std::endl;
//		    }
//
//
//
//		    std::cout<<"true:"<<t<<" False:"<<f<<std::endl;
//		    std::cout<<"true2:"<<t2<<" False2:"<<f2<<std::endl;
//		        double sum = t+f;
//		        double percentage =  (100/sum) ;
//		        std::cout<<"Correct classified: "<<percentage*t<<"%"<<std::endl;
//
////	End Neural Network
//
////	std::cout<<result<<std::endl;
////	int right=0;
////	int wrong=0;
////    for(int i =0;i<result.rows;i++)
////    {
//////    std::cout<<" Results Predictionnum "<<i<<":  "<<prediction_results.at<float>(i)<<"Results prediction"<<std::endl;
////    	if(i==result.at<float>(i))
////    	{
////    		right++;
////    	}else{
////    		wrong++;
////    	}
////    }
////    std::cout<<"True:"<<right<<"  "<<"False:" << wrong<<std::endl;
////    double sum = right+wrong;
////    double percentage =  (100/sum) ;
////    std::cout<<"Correct classified: "<<percentage*right<<"%"<<std::endl;
//
//
////    std::cout<<"Correct "<<std::endl;
//}
