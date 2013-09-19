#include <object_categorization/object_categorization.h>
#include <boost/filesystem.hpp>
#include <fstream>

ObjectCategorization::ObjectCategorization()
{
}

ObjectCategorization::ObjectCategorization(ros::NodeHandle nh)
: node_handle_(nh),
  object_classifier_(ros::package::getPath("cob_object_categorization") + "/common/files/classifier/EMClusterer5.txt", ros::package::getPath("cob_object_categorization") + "/common/files/classifier/")
{
	// Parameters
	global_feature_params_.minNumber3DPixels = 50;
	global_feature_params_.numberLinesX.push_back(7);
//	global_feature_params_.numberLinesX.push_back(2);
	global_feature_params_.numberLinesY.push_back(7);
//	global_feature_params_.numberLinesY.push_back(2);
	global_feature_params_.polynomOrder.push_back(2);
//	global_feature_params_.polynomOrder.push_back(2);
	global_feature_params_.pointDataExcess = 0;	//int(3.01*(globalFeatureParams.polynomOrder+1));	// excess decreases the accuracy
	global_feature_params_.cellCount[0] = 5;
	global_feature_params_.cellCount[1] = 5;
	global_feature_params_.cellSize[0] = 0.5;
	global_feature_params_.cellSize[1] = 0.5;
	global_feature_params_.vocabularySize = 5;
	//global_feature_params_.additionalArtificialTiltedViewAngle.push_back(0.);
	//global_feature_params_.additionalArtificialTiltedViewAngle.push_back(45.);
	global_feature_params_.thinningFactor = 1.0;
	global_feature_params_.useFeature["bow"] = false;
	global_feature_params_.useFeature["sap"] = true;
	global_feature_params_.useFeature["sap2"] = false;
	global_feature_params_.useFeature["pointdistribution"] = false;
	global_feature_params_.useFeature["normalstatistics"] = false;
	global_feature_params_.useFeature["vfh"] = false;
	global_feature_params_.useFeature["grsd"] = false;
	global_feature_params_.useFeature["gfpfh"] = false;
	global_feature_params_.useFullPCAPoseNormalization = false;
	global_feature_params_.useRollPoseNormalization = false;

	projection_matrix_ = (cv::Mat_<double>(3, 4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	pointcloud_width_ = 640;
	pointcloud_height_ = 480;

	node_handle_.param("/object_categorization/object_categorization/mode_of_operation", mode_of_operation_, 1);
	std::cout<< "mode_of_operation: " << mode_of_operation_ << "\n";

	std::string object_name = "shoe_black";

	// initialize special modes
	if (mode_of_operation_ == 2)
	{
		object_classifier_.HermesLoadCameraCalibration(object_name, projection_matrix_);
		object_classifier_.HermesDetectInit((ClusterMode)CLUSTER_EM, (ClassifierType)CLASSIFIER_RTC, global_feature_params_);
	}
	else if (mode_of_operation_ == 3)
	{
		object_classifier_.HermesLoadCameraCalibration(object_name, projection_matrix_);
		object_classifier_.HermesBuildDetectionModelFromRecordedData(object_name, projection_matrix_, (ClusterMode)CLUSTER_EM, (ClassifierType)CLASSIFIER_RTC, global_feature_params_);
		std::cout << "training done.";
		return;
	}

	// subscribers
	it_ = new image_transport::ImageTransport(node_handle_);
	color_image_sub_.subscribe(*it_, "input_color_image", 1);
	//input_pointcloud_sub_ = node_handle_.subscribe("input_pointcloud_segments", 10, &ObjectCategorization::inputCallback, this);
	input_pointcloud_sub_.subscribe(node_handle_, "input_pointcloud_segments", 1);
	input_pointcloud_camera_info_sub_ = node_handle_.subscribe("input_pointcloud_camera_info", 1, &ObjectCategorization::calibrationCallback, this);

	// input synchronization
	sync_input_ = new message_filters::Synchronizer< message_filters::sync_policies::ApproximateTime<cob_perception_msgs::PointCloud2Array, sensor_msgs::Image> >(60);
	sync_input_->connectInput(input_pointcloud_sub_, color_image_sub_);
	sync_input_->registerCallback(boost::bind(&ObjectCategorization::inputCallback, this, _1, _2));
}

ObjectCategorization::~ObjectCategorization()
{

	if (it_ != 0) delete it_;
	if (sync_input_ != 0) delete sync_input_;
}

/// callback for the incoming pointcloud data stream
void ObjectCategorization::inputCallback(const cob_perception_msgs::PointCloud2Array::ConstPtr& input_pointcloud_segments_msg, const sensor_msgs::Image::ConstPtr& input_image_msg)
{
	std::cout << "Categorizing data..." << std::endl;

	// convert color image to cv::Mat
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat display_color;
	cv::Mat display_segmentation(pointcloud_height_, pointcloud_width_, CV_8UC3);
	display_segmentation.setTo(cv::Scalar(255,255,255,255));
	if (convertColorImageMessageToMat(input_image_msg, color_image_ptr, display_color) == false)
		return;

	for (int segmentIndex=0; segmentIndex<(int)input_pointcloud_segments_msg->segments.size(); segmentIndex++)
	{
		typedef pcl::PointXYZRGB PointType;
		pcl::PointCloud<PointType>::Ptr input_pointcloud(new pcl::PointCloud<PointType>);
		pcl::fromROSMsg(input_pointcloud_segments_msg->segments[segmentIndex], *input_pointcloud);

		// convert to shared image
		int umin=1e8, vmin=1e8;
		IplImage* color_image = cvCreateImage(cvSize(pointcloud_width_, pointcloud_height_), IPL_DEPTH_8U, 3);
		cvSetZero(color_image);
		IplImage* coordinate_image = cvCreateImage(cvSize(pointcloud_width_, pointcloud_height_), IPL_DEPTH_32F, 3);
		cvSetZero(coordinate_image);
		pcl::PointXYZ avgPoint(0., 0., 0.);
		unsigned int number_valid_points = 0;
		for (unsigned int i=0; i<input_pointcloud->size(); i++)
		{
			if ((*input_pointcloud)[i].x==0 && (*input_pointcloud)[i].y==0 && (*input_pointcloud)[i].z==0)
				continue;
			++number_valid_points;
			avgPoint.x += (*input_pointcloud)[i].x;
			avgPoint.y += (*input_pointcloud)[i].y;
			avgPoint.z += (*input_pointcloud)[i].z;
			cv::Mat X = (cv::Mat_<double>(4, 1) << (*input_pointcloud)[i].x, (*input_pointcloud)[i].y, (*input_pointcloud)[i].z, 1.0);
			cv::Mat x = projection_matrix_ * X;
			int v = x.at<double>(1)/x.at<double>(2), u = x.at<double>(0)/x.at<double>(2);
			cvSet2D(color_image, v, u, CV_RGB((*input_pointcloud)[i].r, (*input_pointcloud)[i].g, (*input_pointcloud)[i].b));
			cvSet2D(coordinate_image, v, u, cvScalar((*input_pointcloud)[i].x, (*input_pointcloud)[i].y, (*input_pointcloud)[i].z));
			display_segmentation.at< cv::Point3_<uchar> >(v,u) = cv::Point3_<uchar>((*input_pointcloud)[i].b, (*input_pointcloud)[i].g, (*input_pointcloud)[i].r);

			if (u<umin) umin=u;
			if (v<vmin) vmin=v;
		}
		avgPoint.x /= (double)number_valid_points;
		avgPoint.y /= (double)number_valid_points;
		avgPoint.z /= (double)number_valid_points;

		SharedImage si;
		si.setCoord(coordinate_image);
		si.setShared(color_image);
		if (mode_of_operation_ == 1)
		{
			// normal mode of operation
			std::map<double, std::string> resultsOrdered;
			std::map<std::string, double> results;
			object_classifier_.CategorizeObject(&si, results, resultsOrdered, (ClusterMode)CLUSTER_EM, (ClassifierType)CLASSIFIER_RTC, global_feature_params_);
			std::map<double, std::string>::iterator it = resultsOrdered.end();
			it--;
			std::stringstream label;
			label << it->second;
			label << " (" << setprecision(3) << 100*it->first << "%)";
			cv::putText(display_color, label.str().c_str(), cvPoint(umin, max(0,vmin-20)), cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(0, 255, 0));
			cv::putText(display_segmentation, label.str().c_str(), cvPoint(umin, max(0,vmin-20)), cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(0, 255, 0));
		}
		else if (mode_of_operation_ == 2)
		{
			// Hermes mode
			double pan=0, tilt=0, roll=0;
			Eigen::Matrix4f finalTransform;
			object_classifier_.HermesCategorizeObject(input_pointcloud, avgPoint, &si, (ClusterMode)CLUSTER_EM, (ClassifierType)CLASSIFIER_RTC, global_feature_params_, pan, tilt, roll, finalTransform);
		}
		si.Release();

	}
	cv::imshow("categorized objects", display_color);
	cv::imshow("segmented image", display_segmentation);
	cv::waitKey(40);
}

/// Converts a color image message to cv::Mat format.
unsigned long ObjectCategorization::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
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

void ObjectCategorization::calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg)
{
	pointcloud_height_ = calibration_msg->height;
	pointcloud_width_ = calibration_msg->width;
	cv::Mat temp(3,4,CV_64FC1);
	for (int i=0; i<12; i++)
		temp.at<double>(i/4,i%4) = calibration_msg->P.at(i);
//		std::cout << "projection_matrix: [";
//		for (int v=0; v<3; v++)
//			for (int u=0; u<4; u++)
//				std::cout << temp.at<double>(v,u) << " ";
//		std::cout << "]" << std::endl;
	projection_matrix_ = temp;

	input_pointcloud_camera_info_sub_.shutdown();
}

namespace fs = boost::filesystem;
void ObjectCategorization::Training()
{
	// parameters
	ObjectClassifier::LocalFeatureParams localFeatureParams;
	localFeatureParams.useFeature = "surf";		// "surf" //"rsd" //"fpfh"

	ObjectClassifier::GlobalFeatureParams globalFeatureParams;
	globalFeatureParams.minNumber3DPixels = 50;
	globalFeatureParams.numberLinesX.push_back(7);
//	globalFeatureParams.numberLinesX.push_back(2);
	globalFeatureParams.numberLinesY.push_back(7);
//	globalFeatureParams.numberLinesY.push_back(2);
	globalFeatureParams.polynomOrder.push_back(2);
//	globalFeatureParams.polynomOrder.push_back(4);
	globalFeatureParams.pointDataExcess = 0;	//int(3.01*(globalFeatureParams.polynomOrder+1));	// excess decreases the accuracy
	globalFeatureParams.cellCount[0] = 5;
	globalFeatureParams.cellCount[1] = 5;
	globalFeatureParams.cellSize[0] = 0.5;
	globalFeatureParams.cellSize[1] = 0.5;
	globalFeatureParams.vocabularySize = 5;
//	globalFeatureParams.additionalArtificialTiltedViewAngle.push_back(0.);
//	globalFeatureParams.additionalArtificialTiltedViewAngle.push_back(45.);
	double factorSamplesTrainData = 1.;//2./3.;		// for each object, this ratio of samples should go to the training set, the rest is for testing (i.e. there are training and test objects but only factorSamplesTrainData of the samples of each training object are used for training - and only 1.0-factorSamplesTrainData of the samples from test objects are used for testing, e.g. use this to put samples from different tilt angles into the list - first tilt=0, then tilt=beta and finally tilt=beta/2 -> test set only with beta/2)
	globalFeatureParams.thinningFactor = 1.0;
	globalFeatureParams.useFeature["bow"] = false;
	globalFeatureParams.useFeature["sap"] = true;
	globalFeatureParams.useFeature["sap2"] = false;
	globalFeatureParams.useFeature["pointdistribution"] = false;
	globalFeatureParams.useFeature["normalstatistics"] = false;
	globalFeatureParams.useFeature["vfh"] = false;
	globalFeatureParams.useFeature["grsd"] = false;
	globalFeatureParams.useFeature["gfpfh"] = false;
	globalFeatureParams.useFullPCAPoseNormalization = false;
	globalFeatureParams.useRollPoseNormalization = true;

	bool useSloppyMasks = true;

	int viewsPerObject = -1;

	const ClusterMode clusterMode = CLUSTER_EM;
	const ClassifierType classifierType = CLASSIFIER_RTC;
	const int crossValidationFold = 10;
	const float factorNegativeSet = 6.f;	// very old: 5.f
	const float percentTest = 0.f;
	const float percentValidation = 0.1f;

	std::string comments = "Surf64 on RGBI image data.\n\n";

	bool experimentalSeries = false;
	if (experimentalSeries)
	{
		//for (globalFeatureParams.useFullPCAPoseNormalization = true; globalFeatureParams.useFullPCAPoseNormalization == true; globalFeatureParams.useFullPCAPoseNormalization = false)
		//{
			std::string baseFolder = "common/files/IPA2Data/";//"IPA2Data/";
			std::string experimentSubFolder = "exp 17 - vfh/"; //"exp 11 - double sap/";//"exp 1 - PCA3CF/"; //"exp 2 - PCA3CF no pose normalization/"; //"Exp15 - PCA3CF - roll pose normalization/";//"exp 21 - PCA3CF - roll pose normalization/"; //"exp 24 - tilt/";	//"exp 20 - thin point clouds/"; //"Exp15 - PCA3CF - roll pose normalization/";//"exp 4 - PCA3CF less views/";//"exp 6 - single descriptors/";//"exp 5 - PCA3CF no pose normalization - less views/";//"exp 6 - single descriptors/";//"exp 2 - PCA3CF no pose normalization/";//"Exp13 - PCA3CF-nonorm/";//"Exp7 - PCA3CF/";//"exp 1 - PCA3CF/";
			/*if (globalFeatureParams.useFullPCAPoseNormalization == true)
				experimentSubFolder = "exp 4 - PCA3CF less views/";
			else
				experimentSubFolder = "exp 5 - PCA3CF no pose normalization - less views/";*/
			std::string classifierFolder = "Classifier/";
			std::string statisticsFolder = "Statistics/";
			fs::path basePath(baseFolder);
			if (!fs::exists(basePath))
			{
				std::cout << "Error: main: The provided base folder '" << baseFolder << "' does not exist." << std::endl;
				return;
			}
			fs::path experimentSubPath(baseFolder + experimentSubFolder);
			if (!fs::exists(experimentSubPath))
			{
				bool res = fs::create_directory(experimentSubPath);
				if (res == false)
				{
					std::cout << "Error: main: The provided sub folder '" << experimentSubPath << "' could not be created." << std::endl;
					return;
				}
			}

			// descriptor parameters
			//for (globalFeatureParams.polynomOrder[0] = 2; globalFeatureParams.polynomOrder[0] <= 2; globalFeatureParams.polynomOrder[0] += 2)
			//{
				//for (globalFeatureParams.numberLinesX[0] = 7; globalFeatureParams.numberLinesX[0] <= 8; globalFeatureParams.numberLinesX[0]++)
				//{
				//	globalFeatureParams.numberLinesY[0] = globalFeatureParams.numberLinesX[0];
			//for (globalFeatureParams.polynomOrder[1] = 4; globalFeatureParams.polynomOrder[1] <= 4; globalFeatureParams.polynomOrder[1] += 2)
			//{
				//for (globalFeatureParams.numberLinesX[1] = 2; globalFeatureParams.numberLinesX[1] <= 7; globalFeatureParams.numberLinesX[1]++)
				//{
				//	globalFeatureParams.numberLinesY[1] = globalFeatureParams.numberLinesX[1];


			// thinning
			std::vector<double> thinningFactors;
			thinningFactors.push_back(0.04);
			//thinningFactors.push_back(0.0625);
			thinningFactors.push_back(0.1);
			thinningFactors.push_back(0.25);
			//thinningFactors.push_back(0.5);

			for (int thf=0; thf<(int)thinningFactors.size(); thf++)
			{
				globalFeatureParams.thinningFactor = thinningFactors[thf];

			//for (globalFeatureParams.vocabularySize = 150; globalFeatureParams.vocabularySize<=250; globalFeatureParams.vocabularySize+=50)
			//{

			// less views
			//std::vector<int> viewsPerObjectVec;
			//viewsPerObjectVec.push_back(4);
			//viewsPerObjectVec.push_back(6);
			//viewsPerObjectVec.push_back(8);
			//viewsPerObjectVec.push_back(12);
			//viewsPerObjectVec.push_back(16);
			//viewsPerObjectVec.push_back(18);
			//viewsPerObjectVec.push_back(24);
			//for (int viewsPerObjectIndex = 0; viewsPerObjectIndex < (int)viewsPerObjectVec.size(); viewsPerObjectIndex++)
			//{
			//	viewsPerObject = viewsPerObjectVec[viewsPerObjectIndex];

			// tilt angles
			//for (double tiltAngle=20.; tiltAngle<=40.0; tiltAngle+=10.)
			//{
			//	globalFeatureParams.additionalArtificialTiltedViewAngle[0] = 2*tiltAngle;
			//	globalFeatureParams.additionalArtificialTiltedViewAngle[1] = tiltAngle;

					std::cout << "\n\n\nProcessing PCA3CF" << globalFeatureParams.numberLinesX[0] << "-" << globalFeatureParams.numberLinesY[0] << "-" << globalFeatureParams.polynomOrder[0] << "+"
															<< (globalFeatureParams.numberLinesX.size()>1 ? globalFeatureParams.numberLinesX[1] : 0) << "-" << (globalFeatureParams.numberLinesY.size()>1 ? globalFeatureParams.numberLinesY[1] : 0) << "-" << (globalFeatureParams.polynomOrder.size()>1 ? globalFeatureParams.polynomOrder[1] : 0)
															<< " Pose normalization=" << globalFeatureParams.useFullPCAPoseNormalization << " viewsPerObject=" << viewsPerObject << "\n" << std::endl;

					std::stringstream parameters;
					parameters << "Parameters:\nminNumber3DPixels = " << globalFeatureParams.minNumber3DPixels <<
						"\nnumberLinesX = {" << globalFeatureParams.numberLinesX[0] << ", " << (globalFeatureParams.numberLinesX.size()>1 ? globalFeatureParams.numberLinesX[1] : 0) << "}" <<
						"\nnumberLinesY = {" << globalFeatureParams.numberLinesY[0] << ", " << (globalFeatureParams.numberLinesY.size()>1 ? globalFeatureParams.numberLinesY[1] : 0) << "}" <<
						"\npolynomOrder = {" << globalFeatureParams.polynomOrder[0] << ", " << (globalFeatureParams.polynomOrder.size()>1 ? globalFeatureParams.polynomOrder[1] : 0) << "}" <<
						"\npointDataExcess = " << globalFeatureParams.pointDataExcess <<
						"\ncellCount = (" << globalFeatureParams.cellCount[0] << ", " << globalFeatureParams.cellCount[1] << ")" <<
						"\ncellSize = (" << globalFeatureParams.cellSize[0] << ", " << globalFeatureParams.cellSize[1] << ")" <<
						"\nvocabularySize = " << globalFeatureParams.vocabularySize <<
						"\nadditionalArtificialTiltedViewAngle = " << globalFeatureParams.additionalArtificialTiltedViewAngle.size() << "times,  {" << (globalFeatureParams.additionalArtificialTiltedViewAngle.size()>0 ? globalFeatureParams.additionalArtificialTiltedViewAngle[0] : 0) << ", " << (globalFeatureParams.additionalArtificialTiltedViewAngle.size()>1 ? globalFeatureParams.additionalArtificialTiltedViewAngle[1] : 0) << "}" <<
						"\nfactorSamplesTrainData = " << factorSamplesTrainData <<
						"\nthinningFactor = " << globalFeatureParams.thinningFactor <<
						"\nuseFeature['bow'] = " << globalFeatureParams.useFeature["bow"] <<
						"\nuseFeature['sap'] = " << globalFeatureParams.useFeature["sap"] <<
						"\nuseFeature['sap2'] = " << globalFeatureParams.useFeature["sap2"] <<
						"\nuseFeature['pointdistribution'] = " << globalFeatureParams.useFeature["pointdistribution"] <<
						"\nuseFeature['normalstatistics'] = " << globalFeatureParams.useFeature["normalstatistics"] <<
						"\nuseFeature['vfh'] = " << globalFeatureParams.useFeature["vfh"] <<
						"\nuseFeature['grsd'] = " << globalFeatureParams.useFeature["grsd"] <<
						"\nuseFeature['gfpfh'] = " << globalFeatureParams.useFeature["gfpfh"] <<
						"\nuseFullPCAPoseNormalization = " << globalFeatureParams.useFullPCAPoseNormalization <<
						"\nuseRollPoseNormalization = " << globalFeatureParams.useRollPoseNormalization <<
						"\nviewsPerObject (-1 = all) = " << viewsPerObject <<
						"\nclusterMode = " << clusterMode <<
						"\nclassifierType = " << classifierType <<
						"\ncrossValidationFold = " << crossValidationFold <<
						"\nfactorNegativeSet = " << factorNegativeSet <<
						"\npercentTest = " << percentTest <<
						"\npercentValidation = " << percentValidation <<
						"\nuseSloppyMasks = " << useSloppyMasks << std::endl;

					// create paths
					std::stringstream ss;
					ss << "vfh"/*"PCA3CF" << globalFeatureParams.numberLinesX[0] << "-" << globalFeatureParams.numberLinesY[0] << "-" << globalFeatureParams.polynomOrder[0] << "+"
									<< globalFeatureParams.numberLinesX[1] << "-" << globalFeatureParams.numberLinesY[1] << "-" << globalFeatureParams.polynomOrder[1];*/  << " - " << globalFeatureParams.thinningFactor << "points" ; //<< "-tilt-0-" << globalFeatureParams.additionalArtificialTiltedViewAngle[1] << "-" << globalFeatureParams.additionalArtificialTiltedViewAngle[0]; //<< "-view" << viewsPerObject;
		//			ss << /*"EM" << globalFeatureParams.vocabularySize /*<<*/ "PCA3CF" << globalFeatureParams.numberLinesX[0] << "-" << globalFeatureParams.numberLinesY[0] << "-" << globalFeatureParams.polynomOrder[0]; // << "-view" << viewsPerObject;	//<< " - rollnorm - tilt 0-" << tiltAngle << "-" << 2*tiltAngle; //(globalFeatureParams.useFullPCAPoseNormalization == false ? " - nonorm - " : " - ") << globalFeatureParams.thinningFactor << "points";    // << "-view" << viewsPerObject;
					//if (globalFeatureParams.useFullPCAPoseNormalization == true) ss << "normalization - ";
					//else ss << "no normalization - ";
					//if (useSloppyMasks == true) ss << "oldmask";
					//else ss << "goodmask";
					std::string instanceFolder = ss.str() + "/";
					ss.str("");
					ss.clear();
					ss << "vfh";//"PCA3CF" << globalFeatureParams.numberLinesX[0] << "-" << globalFeatureParams.numberLinesY[0] << "-" << globalFeatureParams.polynomOrder[0] << "+"
								//	<< globalFeatureParams.numberLinesX[1] << "-" << globalFeatureParams.numberLinesY[1] << "-" << globalFeatureParams.polynomOrder[1];
		//			ss << /*"EM" << globalFeatureParams.vocabularySize /*<<*/ "PCA3CF" << globalFeatureParams.numberLinesX[0] << "-" << globalFeatureParams.numberLinesY[0] << "-" << globalFeatureParams.polynomOrder[0];
					fs::path instancePath(baseFolder + experimentSubFolder + instanceFolder);
					if (!fs::exists(instancePath))
					{
						bool res = fs::create_directory(instancePath);
						if (res == false)
						{
							std::cout << "Error: main: The provided sub folder '" << instancePath << "' could not be created." << std::endl;
							return;
						}
					}
					fs::path classifierPath(baseFolder + experimentSubFolder + instanceFolder + classifierFolder);
					if (!fs::exists(classifierPath))
					{
						bool res = fs::create_directory(classifierPath);
						if (res == false)
						{
							std::cout << "Error: main: The provided sub folder '" << classifierPath << "' could not be created." << std::endl;
							return;
						}
					}
					fs::path statisticsPath(baseFolder + experimentSubFolder + instanceFolder + classifierFolder + statisticsFolder);
					if (!fs::exists(statisticsPath))
					{
						bool res = fs::create_directory(statisticsPath);
						if (res == false)
						{
							std::cout << "Error: main: The provided sub folder '" << statisticsPath << "' could not be created." << std::endl;
							return;
						}
					}

					// load database or compute features
					std::string localFeatureFileName = baseFolder + experimentSubFolder + "IPA2_Surf64Dev2_loc.txt";//"IPA2_Surf64Dev2_loc.txt";
					std::string globalFeatureFileName = baseFolder + experimentSubFolder + instanceFolder + /*"IPA2_Surf64Dev2_"*/"IPA2_Surf64Dev2_"  + ss.str() + "_glob.txt";
					//std::string globalFeatureFileName = baseFolder + experimentSubFolder + "IPA2_Surf64Dev2_"  + ss.str() + "_glob.txt";
					std::string screenOutputLogFileName = baseFolder + experimentSubFolder + instanceFolder + /*"IPA2_Surf64Dev2_"*/"IPA2_Surf64Dev2_"  + ss.str() + "_screen_log.txt";
					//std::string screenOutputLogFileName = baseFolder + experimentSubFolder + /*"IPA2_Surf64Dev2_"*/"IPA2_Surf64Dev2_"  + ss.str() + "_screen_log_trash.txt";
					std::string timingLogFileName = baseFolder + experimentSubFolder + instanceFolder + /*"IPA2_Surf64Dev2_"*/"IPA2_Surf64Dev2_"  + ss.str() + "_timing.txt";
					std::string covarianceMatrixFileName = baseFolder + experimentSubFolder + "IPA2_Surf64Dev2_loc_covar";//"IPA2_Surf64Dev2_loc_covar";
					std::string localFeatureClustererPath = baseFolder + experimentSubFolder + instanceFolder + "Classifier";
					std::string databasePath;
					if (useSloppyMasks == true)
						databasePath = "F:/ObjectDataNew/TrainingData/";
					else
					{
						databasePath = "F:/ObjectDataNew_goodmasks/TrainingData/";
						localFeatureFileName = baseFolder + experimentSubFolder + "IPA2_Surf64Dev2_loc_goodmask.txt";
					}
					std::ofstream screenLogFile(screenOutputLogFileName.c_str(), std::ios::app);
					if (!screenLogFile.is_open())
					{
						std::cout << "Error: main: The log file " << screenOutputLogFileName << " could not be opened." << std::endl;
						return;
					}
					screenLogFile << "\n\n\n\nCorrected multi-class computation, exp. factor 2, with same random seed for all experiments:\n\n" << comments << parameters.str() << std::endl;
					std::cout << comments << parameters.str() << std::endl;

					// reset random seed to have same randomized conditions for all experiments
					srand(1);

					//object_classifier_.LoadCINDatabase("common/files/object_lists/annotation_cin.txt", "../../Datenbanken/IPA/", 1, clusterMode, globalFeatureParams, localFeatureFileName, globalFeatureFileName, covarianceMatrixFileName,
					//	localFeatureClustererPath, timingLogFileName, MASK_LOAD);
					object_classifier_.LoadCIN2Database("common/files/object_lists/annotation_cin2.txt", databasePath, 2, clusterMode, globalFeatureParams, localFeatureParams, localFeatureFileName, globalFeatureFileName, covarianceMatrixFileName,
										localFeatureClustererPath, timingLogFileName, screenLogFile, MASK_LOAD);
					//object_classifier_.LoadCIN2Database("common/files/object_lists/annotation_cin2.txt", databasePath, 2, clusterMode, globalFeatureParams, localFeatureFileName, globalFeatureFileName, covarianceMatrixFileName,
					//	localFeatureClustererPath, timingLogFileName, screenLogFile, MASK_LOAD);


					// cross-validate
					std::string nameTag = /*"IPA2_Surf64Dev2_"*/"IPA2_Surf64Dev2_" + ss.str();
					//object_classifier_.CrossValidationGlobalSampleRange(statisticsPath.string(), nameTag, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile, factorSamplesTrainData);
					object_classifier_.CrossValidationGlobal(statisticsPath.string(), nameTag, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile);
					object_classifier_.GetDataPointer()->SaveGlobalClassifiers(classifierPath.string(), classifierType);
					//object_classifier_.GetDataPointer()->LoadGlobalClassifiers(classifierPath.string(), classifierType);
					//object_classifier_.CrossValidationGlobalMultiClassSampleRange(statisticsPath.string(), nameTag, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile, factorSamplesTrainData);
					object_classifier_.CrossValidationGlobalMultiClass(statisticsPath.string(), nameTag, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile);

					screenLogFile.close();
				}
			//}
		//}

		return;
	}
	else
	{
		std::stringstream parameters;
		parameters << "Parameters:\nminNumber3DPixels = " << globalFeatureParams.minNumber3DPixels <<
			"\nnumberLinesX = {" << globalFeatureParams.numberLinesX[0] << ", " << (globalFeatureParams.numberLinesX.size()>1 ? globalFeatureParams.numberLinesX[1] : 0) << "}" <<
			"\nnumberLinesY = {" << globalFeatureParams.numberLinesY[0] << ", " << (globalFeatureParams.numberLinesY.size()>1 ? globalFeatureParams.numberLinesY[1] : 0) << "}" <<
			"\npolynomOrder = {" << globalFeatureParams.polynomOrder[0] << ", " << (globalFeatureParams.polynomOrder.size()>1 ? globalFeatureParams.polynomOrder[1] : 0) << "}" <<
			"\npointDataExcess = " << globalFeatureParams.pointDataExcess <<
			"\ncellCount = (" << globalFeatureParams.cellCount[0] << ", " << globalFeatureParams.cellCount[1] << ")" <<
			"\ncellSize = (" << globalFeatureParams.cellSize[0] << ", " << globalFeatureParams.cellSize[1] << ")" <<
			"\nvocabularySize = " << globalFeatureParams.vocabularySize <<
			"\nadditionalArtificialTiltedViewAngle = " << globalFeatureParams.additionalArtificialTiltedViewAngle.size() << "times,  {" << (globalFeatureParams.additionalArtificialTiltedViewAngle.size()>0 ? globalFeatureParams.additionalArtificialTiltedViewAngle[0] : 0) << ", " << (globalFeatureParams.additionalArtificialTiltedViewAngle.size()>1 ? globalFeatureParams.additionalArtificialTiltedViewAngle[1] : 0) << "}" <<
			"\nfactorSamplesTrainData = " << factorSamplesTrainData <<
			"\nthinningFactor = " << globalFeatureParams.thinningFactor <<
			"\nuseFeature['bow'] = " << globalFeatureParams.useFeature["bow"] <<
			"\nuseFeature['sap'] = " << globalFeatureParams.useFeature["sap"] <<
			"\nuseFeature['sap2'] = " << globalFeatureParams.useFeature["sap2"] <<
			"\nuseFeature['pointdistribution'] = " << globalFeatureParams.useFeature["pointdistribution"] <<
			"\nuseFeature['normalstatistics'] = " << globalFeatureParams.useFeature["normalstatistics"] <<
			"\nuseFeature['vfh'] = " << globalFeatureParams.useFeature["vfh"] <<
			"\nuseFeature['grsd'] = " << globalFeatureParams.useFeature["grsd"] <<
			"\nuseFeature['gfpfh'] = " << globalFeatureParams.useFeature["gfpfh"] <<
			"\nuseFullPCAPoseNormalization = " << globalFeatureParams.useFullPCAPoseNormalization <<
			"\nuseRollPoseNormalization = " << globalFeatureParams.useRollPoseNormalization <<
			"\nviewsPerObject (-1 = all) = " << viewsPerObject <<
			"\nclusterMode = " << clusterMode <<
			"\nclassifierType = " << classifierType <<
			"\ncrossValidationFold = " << crossValidationFold <<
			"\nfactorNegativeSet = " << factorNegativeSet <<
			"\npercentTest = " << percentTest <<
			"\npercentValidation = " << percentValidation <<
			"\nuseSloppyMasks = " << useSloppyMasks << std::endl;

		std::cout << parameters.str() << std::endl;

		std::string baseFolder = ros::package::getPath("cob_object_categorization") + "/common/files/";
		std::string databasePath;
		std::string localFeatureFileName = baseFolder + "WashingtonData/Wa_Surf64Dev2_loc.txt"; //"IPA3Data/IPA3_Surf64Dev2_loc.txt"; //"IPA2Data/IPA2_Surf64Dev2_loc.txt"; //"IPA2Data/IPA2_FPFH_loc.txt"; //"IPA2Data/IPA2_RSD_loc.txt"; //"WashingtonData/Wa_Surf64Dev2_loc.txt";//"IPA2Data/IPA2_Surf64Dev2_loc.txt";	//"IPAData/IPA_Surf64Dev2_loc.txt";
		if (useSloppyMasks == true)
			databasePath = "/media/RMB1/Washington3dObjectsDataset/segmented"; //"G:/Washington3dObjectsDataset/segmented"; //"C:/Users/rmb/Documents/Diplomarbeit/Software/object_categorization/common/files/capture/"; //"F:/ObjectDataNew/TrainingData/";
		else
		{
			databasePath = "F:/ObjectDataNew_goodmasks/TrainingData/";
			localFeatureFileName = baseFolder + "IPA2Data/IPA2_Surf64Dev2_loc_goodmask.txt";
		}
		//std::string screenOutputLogFileName = baseFolder + "IPA3Data/IPA3_Surf64Dev2_PCA3CF7-7-2_screen_log.txt";
		std::string screenOutputLogFileName = baseFolder + "WashingtonData/Wa_Surf64Dev2_PCA3CF7-7-2_screen_log.txt";
		//std::string screenOutputLogFileName = baseFolder + "IPA2Data/IPA2_Surf64Dev2_PCA3CF7-7-2_screen_log.txt";
		//std::string screenOutputLogFileName = baseFolder + "IPA2Data/IPA2_RSD_GRSD_screen_log.txt";
		//std::string screenOutputLogFileName = baseFolder + "IPA2Data/IPA2_FPFH_GFPFH_screen_log.txt";
		//std::string screenOutputLogFileName = baseFolder + "IPAData/IPA_Surf64Dev2_PCA3CF7-7-2_screen_log.txt";
		std::ofstream screenLogFile(screenOutputLogFileName.c_str(), std::ios::app);
		if (!screenLogFile.is_open())
		{
			std::cout << "Error: main: The log file " << screenOutputLogFileName << " could not be opened." << std::endl;
			return;
		}
		// screenLogFile << "thin point cloud: 10% of original points used.\n";
		screenLogFile << comments << parameters.str() << std::endl;

		srand(1);

		//std::string annotationFileName = baseFolder + "object_lists/annotation_ipa3.txt";
		//std::string globalFeatureFileName = baseFolder + "IPA3Data/IPA3_Surf64Dev2_PCA3CF7-7-2_glob.txt";
		//std::string covarianceMatrixFileName = baseFolder + "IPA3Data/IPA3_Surf64Dev2_loc_covar";
		//std::string localFeatureClustererPath = baseFolder + "IPA3Data/Classifier";
		//std::string timingLogFileName = baseFolder + "IPA3Data/IPA3_Surf64Dev2_PCA3CF7-7-2_timing.txt";
		//object_classifier_.LoadWashingtonDatabase(annotationFileName, databasePath, 2, clusterMode, globalFeatureParams, localFeatureFileName, globalFeatureFileName,
		//					covarianceMatrixFileName, localFeatureClustererPath, timingLogFileName, screenLogFile, MASK_LOAD, true);

		std::string annotationFileName = baseFolder + "object_lists/annotation_washington_bestperformers10.txt";
		std::string globalFeatureFileName = baseFolder + "WashingtonData/Wa_Surf64Dev2_PCA3CF7-7-2_glob.txt";
		std::string covarianceMatrixFileName = baseFolder + "WashingtonData/Wa_Surf64Dev2_loc_covar";
		std::string localFeatureClustererPath = baseFolder + "WashingtonData/Classifier";
		std::string timingLogFileName = baseFolder + "WashingtonData/Wa_Surf64Dev2_PCA3CF7-7-2_timing.txt";
		object_classifier_.LoadWashingtonDatabase(annotationFileName, databasePath, 2, clusterMode, globalFeatureParams, localFeatureFileName, globalFeatureFileName,
							covarianceMatrixFileName, localFeatureClustererPath, timingLogFileName, screenLogFile, MASK_LOAD);

		//std::string annotationFileName = baseFolder + "object_lists/annotation_cin2.txt";
		//std::string globalFeatureFileName = baseFolder + "IPA2Data/IPA2_Surf64Dev2_PCA3CF7-7-2_glob.txt";
		//std::string covarianceMatrixFileName = baseFolder + "IPA2Data/IPA2_Surf64Dev2_loc_covar";
		//std::string localFeatureClustererPath = baseFolder + "IPA2Data/Classifier";
		//std::string timingLogFileName = baseFolder + "IPA2Data/IPA2_Surf64Dev2_PCA3CF7-7-2_timing.txt";
		//object_classifier_.LoadCIN2Database(annotationFileName, databasePath, 1, clusterMode, globalFeatureParams, localFeatureParams, localFeatureFileName, globalFeatureFileName,
		//					covarianceMatrixFileName, localFeatureClustererPath, timingLogFileName, screenLogFile, MASK_LOAD);
		////object_classifier_.LoadCIN2Database("common/files/object_lists/annotation_cin2.txt", databasePath, 2, clusterMode, globalFeatureParams, localFeatureParams, localFeatureFileName, "IPA2Data/IPA2_RSD_GRSD_glob.txt",
		////					"IPA2Data/IPA2_RSD_loc_covar", "IPA2Data/Classifier", "IPA2Data/IPA2_RSD_GRSD_timing.txt", screenLogFile, MASK_LOAD);
		////object_classifier_.LoadCIN2Database("common/files/object_lists/annotation_cin2.txt", databasePath, 2, clusterMode, globalFeatureParams, localFeatureParams, localFeatureFileName, "IPA2Data/IPA2_FPFH_GFPFH_glob.txt",
		////	"IPA2Data/IPA2_FPFH_loc_covar", "IPA2Data/Classifier", "IPA2Data/IPA2_FPFH_GFPFH_timing.txt", screenLogFile, MASK_LOAD);

		//std::string annotationFileName = baseFolder + "object_lists/annotation_cin.txt";
		//std::string globalFeatureFileName = baseFolder + "IPAData/IPA_Surf64Dev2_EM250_glob.txt"; //"IPAData/IPA_Surf64Dev2_PCA3CF7-7-2_glob.txt";
		//std::string covarianceMatrixFileName = baseFolder + "IPAData/IPA_Surf64Dev2_loc_covar";
		//std::string localFeatureClustererPath = baseFolder + "IPAData/Classifier";
		//std::string timingLogFileName = baseFolder + "IPAData/IPA_Surf64Dev2_EM250_timing.txt"; //"IPAData/IPA_Surf64Dev2_PCA3CF7-7-2_timing.txt";
		//object_classifier_.LoadCINDatabase(annotationFileName, "../../Datenbanken/IPA/", 2, clusterMode, globalFeatureParams, localFeatureFileName, globalFeatureFileName,
		//	covarianceMatrixFileName, localFeatureClustererPath, timingLogFileName, MASK_LOAD);

		//// descriptor repeatability
		//std::cout << "\n\noffset\tintra\ttrans\textra\n";
		//for (int i=0; i<19; i++)
		//{
		//	double intraDist = 0.0, transDist = 0.0, extraDist = 0.0;
		//	object_classifier_.GetDataPointer()->analyzeGlobalFeatureRepeatability(i, intraDist, transDist, extraDist);
		//	std::cout << i << "\t" << intraDist << "\t" << transDist << "\t" << extraDist << "\n";
		//	screenLogFile << i << "\t" << intraDist << "\t" << transDist << "\t" << extraDist << "\n";
		//}
		//return 0;

		srand(1);

		std::string statisticsPath = baseFolder + "WashingtonData/Classifier/Statistics/"; //"IPA3Data/Classifier/Statistics/"; //"IPA2Data/Classifier/Statistics/";	//"WashingtonData/Classifier/Statistics/";	//"IPA2Data/Classifier/Statistics/"         //"IPAData/Classifier/Statistics/"
		std::string configurationPrefix = "IPA3_Surf64Dev2_PCA3CF7-7-2"; //"IPA2_Surf64Dev2_PCA3CF7-7-2"; //"IPA_Surf64Dev2_EM250";	//"IPA2_Surf64Dev2_PCA3CF7-7-2";	// "IPA2_FPFH_GFPFH" //"IPA2_RSD_GRSD"	//"IPA_Surf64Dev2_vfh"
		//object_classifier_.CrossValidationGlobalSampleRange(statisticsPath, configurationPrefix, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile, factorSamplesTrainData);
		object_classifier_.CrossValidationGlobal(statisticsPath, configurationPrefix, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile);
		std::string localClassifierSavePath = localFeatureClustererPath + "/";
		object_classifier_.GetDataPointer()->SaveGlobalClassifiers(localClassifierSavePath, classifierType);
		//object_classifier_.GetDataPointer()->LoadGlobalClassifiers(localClassifierSavePath, CLASSIFIER_RTC);
		//object_classifier_.CrossValidationGlobalMultiClassSampleRange(statisticsPath, configurationPrefix, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile, factorSamplesTrainData);
		object_classifier_.CrossValidationGlobalMultiClass(statisticsPath, configurationPrefix, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile);

		screenLogFile.close();

		return;
	}
}


int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "object_categorization");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of CameraDriver
	ObjectCategorization objectCategorization(nh);

	// Training
//	ObjectCategorization objectCategorization;
//	objectCategorization.Training();

	ros::spin();

	return (0);
}
