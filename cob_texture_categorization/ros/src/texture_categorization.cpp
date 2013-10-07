#include <cob_texture_categorization/texture_categorization.h>



TextCategorizationNode::TextCategorizationNode(ros::NodeHandle nh) :
node_handle_(nh)
{
//	camera_matrix_received_ = false;

	// subscribers

	it_ = 0;
	sync_input_ = 0;

	it_ = new image_transport::ImageTransport(node_handle_);
	colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
	pointcloud_sub_.subscribe(node_handle_, "pointcloud_in", 1);

	sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >(30);
	sync_input_->connectInput(colorimage_sub_, pointcloud_sub_);
	sync_input_->registerCallback(boost::bind(&TextCategorizationNode::inputCallback, this, _1, _2));

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
}


/// callback for the incoming  data stream
void TextCategorizationNode::inputCallback(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
	ROS_INFO("Input Callback");

	// convert color image to cv::Mat
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

	// do something useful with code located in common/src
	cv::Mat gray_image, dx;
	cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);
	cv::Sobel(gray_image, dx, -1, 1, 0, 3);

	cv::imshow("image", color_image);
	cv::imshow("gray image", gray_image);
	cv::imshow("dx image", dx);
	cv::waitKey(10);

//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
//	pcl::fromROSMsg(*pointcloud_msg, *cloud);
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

	ros::spin();

	return (0);
}
