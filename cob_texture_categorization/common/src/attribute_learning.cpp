#include "cob_texture_categorization/attribute_learning.h"

#include <fstream>

#include "ml.h"
#include "highgui.h"


void AttributeLearning::loadTextureDatabaseBaseFeatures(std::string filename, cv::Mat& feature_matrix, cv::Mat& attribute_matrix, create_train_data::DataHierarchyType& data_sample_hierarchy)
{
	// load feature vectors and corresponding labels computed on database and class-object-sample hierarchy
	const int label_number = 17;
	const int feature_number = 9688;
	const int total_sample_number = 1281;
	feature_matrix.create(total_sample_number, feature_number, CV_32FC1);
	attribute_matrix.create(total_sample_number, label_number, CV_32FC1);
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
					for (int l=0; l<label_number; ++l)
						file >> attribute_matrix.at<float>(sample_index, l);
					for (int f=0; f<feature_number; ++f)
						file >> feature_matrix.at<float>(sample_index, f);
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



void AttributeLearning::crossValidation(int folds, const cv::Mat& feature_matrix, const cv::Mat& attribute_matrix, const create_train_data::DataHierarchyType& data_sample_hierarchy)
{
	std::vector<double> sumAbsErrors, numberSamples;
	create_train_data data_object;
	std::vector<std::string> texture_classes = data_object.get_texture_classes();

	srand(0);	// random seed --> keep reproducible
	for (int fold=0; fold<folds; ++fold)
	{
		std::cout << "=== fold " << fold << " ===" << std::endl;

		// === distribute data into training and test set ===
		// select one object per class for testing
		std::vector<int> train_indices, test_indices;
		for (unsigned int class_index=0; class_index<data_sample_hierarchy.size(); ++class_index)
		{
			int object_number = data_sample_hierarchy[class_index].size();
			int test_object = (int)(object_number * (double)rand()/((double)RAND_MAX+1.0));
//			std::cout << "object_number=" << object_number << "   test_object=" << test_object << std::endl;
			for (int object_index=0; object_index<object_number; ++object_index)
			{
				if (object_index == test_object)
					for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
						{test_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
							std::cout << data_sample_hierarchy[class_index][object_index][s] << "\t";}
				else
					for (unsigned int s=0; s<data_sample_hierarchy[class_index][object_index].size(); ++s)
						train_indices.push_back(data_sample_hierarchy[class_index][object_index][s]);
			}
		}
		assert((int)(test_indices.size() + train_indices.size()) == feature_matrix.rows);

		// create training and test data matrices
		cv::Mat training_data(train_indices.size(), feature_matrix.cols, feature_matrix.type());
		cv::Mat training_labels(train_indices.size(), 1, attribute_matrix.type());
		cv::Mat test_data(test_indices.size(), feature_matrix.cols, feature_matrix.type());
		cv::Mat test_labels(test_indices.size(), 1, attribute_matrix.type());
		for (unsigned int r=0; r<train_indices.size(); ++r)
		{
			for (int c=0; c<feature_matrix.cols; ++c)
				training_data.at<float>(r,c) = feature_matrix.at<float>(train_indices[r],c);
			training_labels.at<float>(r) = attribute_matrix.at<float>(train_indices[r], 6);
		}
		for (unsigned int r=0; r<test_indices.size(); ++r)
		{
			for (int c=0; c<feature_matrix.cols; ++c)
				test_data.at<float>(r,c) = feature_matrix.at<float>(test_indices[r],c);
			test_labels.at<float>(r) = attribute_matrix.at<float>(test_indices[r], 6);
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

//	Randomtree

//	float v1 = 0.0;
//	const float *v2 = 0;
//	float v3 = 1.0;
//	CvRTParams treetest(78, 6, v1, false,28, v2, false,23,28,v3, 0 );
//	CvRTrees tree;
//	tree.train(training_data,1, training_label, cv::Mat(), cv::Mat(),cv::Mat(),cv::Mat(), treetest);
//	cv::Mat result(train_data.rows,1,CV_32FC1);
//
//	for(int i=0;i<train_data.rows;i++)
//	{
//		cv::Mat inputvec(1,train_data.cols, CV_32FC1);
//		for(int j=0;j<train_data.cols;j++)
//		{
//			inputvec.at<float>(0,j)=train_data.at<float>(i,j);
//		}
//		result.at<float>(i,0)=tree.predict(inputvec);
//	}

//	End Randomtree

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
//	bayesmod.train(training_data, training_label, cv::Mat(), cv::Mat());
//	 cv::Mat result(train_data.rows,1,CV_32FC1);
//	bayesmod.predict(train_data, &result);

//	End Bayes Classifier

//	Neural Network

		cv::Mat input;
		training_data.convertTo(input, CV_32F);
		cv::Mat output=cv::Mat::zeros(training_data.rows, 1, CV_32FC1);

		cv::Mat labels;
		training_labels.convertTo(labels, CV_32F);

		for(int i=0; i<training_data.rows; ++i)
			output.at<float>(i,0) = labels.at<float>(i,0);

		cv::Mat layers = cv::Mat(3,1,CV_32SC1);

		layers.row(0) = cv::Scalar(training_data.cols);
		layers.row(1) = cv::Scalar(10);
		layers.row(2) = cv::Scalar(1);

		CvANN_MLP mlp;
		CvANN_MLP_TrainParams params;
		CvTermCriteria criteria;

		criteria.max_iter = 100;
		criteria.epsilon  = 0.0001f;
		criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;

		params.train_method    = CvANN_MLP_TrainParams::BACKPROP;
		params.bp_dw_scale     = 0.1f;
		params.bp_moment_scale = 0.1f;
		params.term_crit       = criteria;

		mlp.create(layers,CvANN_MLP::SIGMOID_SYM,0.4,1.0);
		int iterations = mlp.train(input, output, cv::Mat(), cv::Mat(), params);
		std::cout << "Neural network training completed after " << iterations << " iterations." << std::endl;

		// === apply ml classifier to predict test set ===
		double sumAbsError = 0.;
		int numberTestSamples = 0;
		for(int i = 0; i < test_data.rows ; ++i)
		{
			cv::Mat response(1, 1, CV_32FC1);
			cv::Mat sample = test_data.row(i);

			mlp.predict(sample, response);

			float absdiff = fabs(std::max(0.f, response.at<float>(0,0)) - test_labels.at<float>(i, 0));
			sumAbsError += absdiff;
			++numberTestSamples;

			std::cout << "value: " << test_labels.at<float>(i, 0) <<
					"\t predicted: " << response.at<float>(0,0) << "\t abs difference: " << absdiff << std::endl;
		}

		sumAbsErrors.push_back(sumAbsError);
		numberSamples.push_back((double)numberTestSamples);
		std::cout << "mean abs error: " << sumAbsError/(double)numberTestSamples << std::endl;
	}
//	End Neural Network

	std::cout << "=== Total result over " << folds << "-fold cross validation ===" << std::endl;
	double s=0., n=0.;
	for (unsigned int i=0; i<sumAbsErrors.size(); ++i)
	{
		s += sumAbsErrors[i];
		n += numberSamples[i];
	}
	std::cout << "mean abs error: " << s/n << std::endl;
}

/*
void train_ml::run_ml(double val, std::string *path_)
{

	std::string train_data = *path_ + "train_data.yml";
//	cv::FileStorage fs("/home/rmb-dh/Test_dataset/training_data.yml", cv::FileStorage::READ);
	cv::FileStorage fs(train_data, cv::FileStorage::READ);
	cv::Mat training_data;
	fs["train_data"] >> training_data;
//	std::cout<<training_data<<"Matrix"<<std::endl;

	std::string train_label = *path_ + "train_data_label.yml";
//	cv::FileStorage fsl("/home/rmb-dh/Test_dataset/train_data_respons.yml", cv::FileStorage::READ);
	cv::FileStorage fsl(train_label, cv::FileStorage::READ);
	cv::Mat training_labels;
	fsl["train_label"] >> training_labels;
//	std::cout<<training_label<<"Matrix"<<std::endl;

	cv::Mat test_data;
	std::string test_data_ = *path_ + "test_data.yml";
//	cv::FileStorage fsd("/home/rmb-dh/Test_dataset/test_data.yml", cv::FileStorage::READ);
	cv::FileStorage fsd(test_data_, cv::FileStorage::READ);
	fsd["test_data"] >> test_data;

//	std::vector<std::string> test_data_label;
	cv::Mat test_labels;
	std::string test_label = *path_ + "test_data_label.yml";
//	cv::FileStorage fstl("/home/rmb-dh/Test_dataset/test_data_label.yml", cv::FileStorage::READ);
	cv::FileStorage fstl(test_label, cv::FileStorage::READ);
	fstl["test_label"] >> test_labels;


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

//	Randomtree

//	float v1 = 0.0;
//	const float *v2 = 0;
//	float v3 = 1.0;
//	CvRTParams treetest(78, 6, v1, false,28, v2, false,23,28,v3, 0 );
//	CvRTrees tree;
//	tree.train(training_data,1, training_label, cv::Mat(), cv::Mat(),cv::Mat(),cv::Mat(), treetest);
//	cv::Mat result(train_data.rows,1,CV_32FC1);
//
//	for(int i=0;i<train_data.rows;i++)
//	{
//		cv::Mat inputvec(1,train_data.cols, CV_32FC1);
//		for(int j=0;j<train_data.cols;j++)
//		{
//			inputvec.at<float>(0,j)=train_data.at<float>(i,j);
//		}
//		result.at<float>(i,0)=tree.predict(inputvec);
//	}

//	End Randomtree

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
//	bayesmod.train(training_data, training_label, cv::Mat(), cv::Mat());
//	 cv::Mat result(train_data.rows,1,CV_32FC1);
//	bayesmod.predict(train_data, &result);

//	End Bayes Classifier

//	Neural Network

//		cv::Mat input;
//		training_data.convertTo(input, CV_32F);
//		cv::Mat output=cv::Mat::zeros(training_data.rows,57, CV_32FC1);

		cv::Mat input;
		training_data.convertTo(input, CV_32F);
		cv::Mat output=cv::Mat::zeros(training_data.rows,57, CV_32FC1);

		cv::Mat change;
		training_labels.convertTo(change, CV_32F);


		for(float i=0;i<training_data.rows;i++)
		{
			output.at<float>(i,(int)change.at<float>(i,0))=1;//change.at<float>(i,0);
		}


		 cv::Mat layers = cv::Mat(3,1,CV_32SC1);
//		    int sz = train_data.cols ;

		    layers.row(0) = cv::Scalar(16);
		    layers.row(1) = cv::Scalar(400);
		    layers.row(2) = cv::Scalar(57);

		    CvANN_MLP mlp;
		    CvANN_MLP_TrainParams params;
		    CvTermCriteria criteria;

		    criteria.max_iter = 1000;
		    criteria.epsilon  = 0.00001f;
		    criteria.type     = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;

		    params.train_method    = CvANN_MLP_TrainParams::BACKPROP;
		    params.bp_dw_scale     = 0.1f;
		    params.bp_moment_scale = 0.1f;
		    params.term_crit       = criteria;

		    mlp.create(layers,CvANN_MLP::SIGMOID_SYM,0.3,1.2);
		    int i = mlp.train(input,output,cv::Mat(),cv::Mat(),params);

		    int t = 0, f = 0;
		    int t2 = 0, f2 = 0;

		    std::cout<<"training completed"<<std::endl;
		    std::vector<int> labelres;
		    for(int i = 0; i < test_data.rows ; i++)
		    {
		        cv::Mat response(1,57,CV_32FC1);
		        cv::Mat sample = test_data.row(i);

		        mlp.predict(sample,response);

		        float max = -1000000000000.0f;
		        float max2 = -1000000000000.0f;
		        int cls = -1;
		        int cls2 = -1;

		        for(int j = 0 ; j < 57 ; j++)
		        {
		            float value = response.at<float>(0,j);

		            if(value > max)
		            {
		            	max2=max;
		            	cls2=cls;
		                max = value;
		                cls = j;
		            }
		        }

		        if(cls == test_labels.at<float>(i,0))
		            t++;
		        else
		            f++;

		        if(cls2 == test_labels.at<float>(i,0))
		       		            t2++;
		       		        else
		       		            f2++;


//		        std::cout<<"test"<<std::endl;
		        std::cout<<"Value:"<<test_labels.at<float>(i,0)<<" Predicted:"<<cls<<std::endl;
		    }



		    std::cout<<"true:"<<t<<" False:"<<f<<std::endl;
		    std::cout<<"true2:"<<t2<<" False2:"<<f2<<std::endl;
		        double sum = t+f;
		        double percentage =  (100/sum) ;
		        std::cout<<"Correct classified: "<<percentage*t<<"%"<<std::endl;

//	End Neural Network

//	std::cout<<result<<std::endl;
//	int right=0;
//	int wrong=0;
//    for(int i =0;i<result.rows;i++)
//    {
////    std::cout<<" Results Predictionnum "<<i<<":  "<<prediction_results.at<float>(i)<<"Results prediction"<<std::endl;
//    	if(i==result.at<float>(i))
//    	{
//    		right++;
//    	}else{
//    		wrong++;
//    	}
//    }
//    std::cout<<"True:"<<right<<"  "<<"False:" << wrong<<std::endl;
//    double sum = right+wrong;
//    double percentage =  (100/sum) ;
//    std::cout<<"Correct classified: "<<percentage*right<<"%"<<std::endl;


//    std::cout<<"Correct "<<std::endl;
}

*/
