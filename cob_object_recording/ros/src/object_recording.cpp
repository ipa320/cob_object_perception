#include <cob_object_recording/object_recording.h>
#include <boost/filesystem.hpp>
#include <fstream>

ObjectRecording::ObjectRecording()
{
	it_ = 0;
	sync_input_ = 0;
}

ObjectRecording::ObjectRecording(ros::NodeHandle nh)
: node_handle_(nh)
{
//	projection_matrix_ = (cv::Mat_<double>(3, 4) << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
//	pointcloud_width_ = 640;
//	pointcloud_height_ = 480;

	// subscribers
	input_marker_detection_sub_.subscribe(node_handle_, "input_marker_detections", 1);
	it_ = new image_transport::ImageTransport(node_handle_);
	color_image_sub_.subscribe(*it_, "input_color_image", 1);
	//input_pointcloud_sub_ = node_handle_.subscribe("input_pointcloud_segments", 10, &ObjectRecording::inputCallback, this);
	input_pointcloud_sub_.subscribe(node_handle_, "input_pointcloud", 1);
//	input_pointcloud_camera_info_sub_ = node_handle_.subscribe("input_pointcloud_camera_info", 1, &ObjectRecording::calibrationCallback, this);

	// input synchronization
	sync_input_ = new message_filters::Synchronizer< message_filters::sync_policies::ApproximateTime<cob_object_detection_msgs::DetectionArray, sensor_msgs::PointCloud2, sensor_msgs::Image> >(10);
	sync_input_->connectInput(input_marker_detection_sub_, input_pointcloud_sub_, color_image_sub_);
	sync_input_->registerCallback(boost::bind(&ObjectRecording::inputCallback, this, _1, _2, _3));
}

ObjectRecording::~ObjectRecording()
{

	if (it_ != 0) delete it_;
	if (sync_input_ != 0) delete sync_input_;
}

/// callback for the incoming pointcloud data stream
void ObjectRecording::inputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg, const sensor_msgs::PointCloud2::ConstPtr& input_pointcloud_msg, const sensor_msgs::Image::ConstPtr& input_image_msg)
{
	std::cout << "Recording data..." << std::endl;

	// convert color image to cv::Mat
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	if (convertColorImageMessageToMat(input_image_msg, color_image_ptr, color_image) == false)
		return;

	// todo: select ROI for sharpness computation

	// compute sharpness measure
	cv::Mat dx, dy;
	cv::Mat gray_image;
	cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);
	cv::Sobel(gray_image, dx, CV_32FC1, 1, 0, 3);
	cv::Sobel(gray_image, dy, CV_32FC1, 0, 1, 3);
	double score = (cv::sum(cv::abs(dx)) + cv::sum(cv::abs(dy))).val[0] / ((double)color_image.cols*color_image.rows);
	std::cout << "sharpness score=" << score << std::endl;

	typedef pcl::PointXYZRGB PointType;
	pcl::PointCloud<PointType> input_pointcloud;
	pcl::fromROSMsg(*input_pointcloud_msg, input_pointcloud);

//	cv::Mat display_segmentation = cv::Mat::zeros(input_pointcloud_msg->height, input_pointcloud_msg->width, CV_8UC3);
	cv::imshow("color image", color_image);
	//cv::imshow("segmented image", display_segmentation);
	cv::waitKey(10);
}

/// Converts a color image message to cv::Mat format.
unsigned long ObjectRecording::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
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

//void ObjectRecording::calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg)
//{
//	pointcloud_height_ = calibration_msg->height;
//	pointcloud_width_ = calibration_msg->width;
//	cv::Mat temp(3,4,CV_64FC1);
//	for (int i=0; i<12; i++)
//		temp.at<double>(i/4,i%4) = calibration_msg->P.at(i);
////		std::cout << "projection_matrix: [";
////		for (int v=0; v<3; v++)
////			for (int u=0; u<4; u++)
////				std::cout << temp.at<double>(v,u) << " ";
////		std::cout << "]" << std::endl;
//	projection_matrix_ = temp;
//}


int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "object_recording");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of ObjectRecording
	ObjectRecording objectRecording(nh);

	ros::spin();

	return (0);
}
