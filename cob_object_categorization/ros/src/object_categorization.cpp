#include <object_categorization/object_categorization.h>

ObjectCategorization::ObjectCategorization(ros::NodeHandle nh)
: node_handle_(nh),
  object_classifier_(ros::package::getPath("cob_object_categorization") + "/common/files/classifier/EMClusterer5.txt", ros::package::getPath("cob_object_categorization") + "/common/files/classifier/")
{
	projection_matrix_ = (cv::Mat_<double>(3, 4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
	pointcloud_width_ = 640;
	pointcloud_height_ = 480;

	// subscribers
	it_ = new image_transport::ImageTransport(node_handle_);
	color_image_sub_.subscribe(*it_, "input_color_image", 1);
	//input_pointcloud_sub_ = node_handle_.subscribe("input_pointcloud_segments", 10, &ObjectCategorization::inputCallback, this);
	input_pointcloud_sub_.subscribe(node_handle_, "input_pointcloud_segments", 1);
	input_pointcloud_camera_info_sub_ = node_handle_.subscribe("input_pointcloud_camera_info", 1, &ObjectCategorization::calibrationCallback, this);

	// input synchronization
	sync_input_ = new message_filters::Synchronizer< message_filters::sync_policies::ApproximateTime<cob_object_categorization::PointCloud2Array, sensor_msgs::Image> >(30);
	sync_input_->connectInput(input_pointcloud_sub_, color_image_sub_);
	sync_input_->registerCallback(boost::bind(&ObjectCategorization::inputCallback, this, _1, _2));
}

ObjectCategorization::~ObjectCategorization()
{

	if (it_ != 0) delete it_;
	if (sync_input_ != 0) delete sync_input_;
}

/// callback for the incoming pointcloud data stream
void ObjectCategorization::inputCallback(const cob_object_categorization::PointCloud2Array::ConstPtr& input_pointcloud_segments_msg, const sensor_msgs::Image::ConstPtr& input_image_msg)
{
	std::cout << "Categorizing data..." << std::endl;

	// convert color image to cv::Mat
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat display_color;
	cv::Mat display_segmentation = cv::Mat::zeros(pointcloud_height_, pointcloud_width_, CV_8UC3);
	if (convertColorImageMessageToMat(input_image_msg, color_image_ptr, display_color) == false)
		return;

	for (int segmentIndex=0; segmentIndex<(int)input_pointcloud_segments_msg->segments.size(); segmentIndex++)
	{
		typedef pcl::PointXYZRGB PointType;
		pcl::PointCloud<PointType> input_pointcloud;
		pcl::fromROSMsg(input_pointcloud_segments_msg->segments[segmentIndex], input_pointcloud);

		// convert to shared image
		int umin=1e8, vmin=1e8;
		IplImage* color_image = cvCreateImage(cvSize(pointcloud_width_, pointcloud_height_), IPL_DEPTH_8U, 3);
		cvSetZero(color_image);
		IplImage* coordinate_image = cvCreateImage(cvSize(pointcloud_width_, pointcloud_height_), IPL_DEPTH_32F, 3);
		cvSetZero(coordinate_image);
		for (unsigned int i=0; i<input_pointcloud.size(); i++)
		{
			cv::Mat X = (cv::Mat_<double>(4, 1) << input_pointcloud[i].x, input_pointcloud[i].y, input_pointcloud[i].z, 1.0);
			cv::Mat x = projection_matrix_ * X;
			int v = x.at<double>(1)/x.at<double>(2), u = x.at<double>(0)/x.at<double>(2);
			cvSet2D(color_image, v, u, CV_RGB(input_pointcloud[i].r, input_pointcloud[i].g, input_pointcloud[i].b));
			cvSet2D(coordinate_image, v, u, cvScalar(input_pointcloud[i].x, input_pointcloud[i].y, input_pointcloud[i].z));
			display_segmentation.at< cv::Point3_<uchar> >(v,u) = cv::Point3_<uchar>(input_pointcloud[i].b, input_pointcloud[i].g, input_pointcloud[i].r);

			if (u<umin) umin=u;
			if (v<vmin) vmin=v;
		}

		// Parameters
		ObjectClassifier::GlobalFeatureParams globalFeatureParams;
		globalFeatureParams.minNumber3DPixels = 50;
		globalFeatureParams.numberLinesX.push_back(7);
	//	globalFeatureParams.numberLinesX.push_back(2);
		globalFeatureParams.numberLinesY.push_back(7);
	//	globalFeatureParams.numberLinesY.push_back(2);
		globalFeatureParams.polynomOrder.push_back(2);
	//	globalFeatureParams.polynomOrder.push_back(2);
		globalFeatureParams.pointDataExcess = 0;	//int(3.01*(globalFeatureParams.polynomOrder+1));	// excess decreases the accuracy
		globalFeatureParams.cellCount[0] = 5;
		globalFeatureParams.cellCount[1] = 5;
		globalFeatureParams.cellSize[0] = 0.5;
		globalFeatureParams.cellSize[1] = 0.5;
		globalFeatureParams.vocabularySize = 5;
		//globalFeatureParams.additionalArtificialTiltedViewAngle.push_back(0.);
		//globalFeatureParams.additionalArtificialTiltedViewAngle.push_back(45.);
		double factorSamplesTrainData = 1.;		// for each object, this ratio of samples should go to the training set, the rest is for testing
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

		SharedImage si;
		si.setCoord(coordinate_image);
		si.setShared(color_image);
		std::map<double, std::string> resultsOrdered;
		object_classifier_.CategorizeObject(&si, resultsOrdered, (ClusterMode)CLUSTER_EM, (ClassifierType)CLASSIFIER_RTC, globalFeatureParams);
		si.Release();

		std::map<double, std::string>::iterator it = resultsOrdered.end();
		it--;
		std::stringstream label;
		label << it->second;
		label << " (" << setprecision(3) << 100*it->first << "%)";
		cv::putText(display_color, label.str().c_str(), cvPoint(umin, max(0,vmin-20)), cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(0, 255, 0));
		cv::putText(display_segmentation, label.str().c_str(), cvPoint(umin, max(0,vmin-20)), cv::FONT_HERSHEY_SIMPLEX, 1.0, CV_RGB(0, 255, 0));
	}
	cv::imshow("categorized objects", display_color);
	cv::imshow("segmented image", display_segmentation);
	cv::waitKey(10);
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
}


int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "object_categorization");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of CameraDriver
	ObjectCategorization objectCategorization(nh);

	ros::spin();

	return (0);
}
