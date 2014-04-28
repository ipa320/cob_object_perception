#include <cob_texture_categorization/texture_categorization.h>

#include "create_lbp.h"
#include "splitandmerge.h"
#include "texture_features.h"
#include "compute_textures.h"
#include "depth_image.h"
#include "segment_trans.h"
#include "perspective_transformation.h"

#include <iostream>
#include <fstream>
#include <dirent.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "run_meanshift_test.h"

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






#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;





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
//	TextCategorizationNode::inputCallbackNoCam();










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
//   	coordinatesystem = node_handle_.advertise<visualization_msgs::MarkerArray>("markertest", 1 );
////    cloudpub = node_handle_.advertise<PointCloud> ("points2", 1);
//    pub_cloud = node_handle_.advertise<sensor_msgs::PointCloud2> ("cloud", 1);

}

void TextCategorizationNode::inputCallbackNoCam()
{
	ROS_INFO("Input Callback No Cam");
	compute_textures test = compute_textures();
	test.compute_textures_all();
	std::cout<<"test ";
//	cv::waitKey(10);
}

/// callback for the incoming  data stream
void TextCategorizationNode::inputCallback(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
{
	ROS_INFO("Input Callback");
	// convert color image to cv::Mat
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);





	cv::Mat dst;


/// convert depth data to cv::Mat
	cv::Mat depth(480, 640, CV_32F);
	depth_image dimage = depth_image();
	dimage.get_depth_image(pointcloud_msg, &depth);
//	cv::imshow("3D",depth);
//	cv::moveWindow("3D", 800,600);



///	Filter to smooth depthimage

	/// Medianfilter
//		dimage.medianfilter(&depth);

	///	Morphological closeing
		dimage.close_operation(&depth);

//		cv::imshow("Smoothened 3D", depth);


// Imagetransformation


//		cv::Mat rotated(480, 640, CV_8UC3);
//
//		segment_trans seg_test = segment_trans();
//		seg_test.transformation(&color_image, &rotated, &depth);
//		cv::waitKey(1000);


		cv::Rect rbb= cv::Rect(70, 90, 300, 300);
		cv::Mat test=color_image(rbb);
		cv::Mat test2=depth(rbb);
		cv::imshow("test", color_image);

		p_transformation transform = p_transformation();
		transform.run_pca(&color_image, &depth, pointcloud_msg, &marker);



//			    coordinatesystem.publish(marker);
////			    cloudpub.publish(msg);
//			    pub_cloud.publish(pointcloud_msg);


//	Run meanshift with rgbxy and rgbxyd
//	run_meanshift_test test = run_meanshift_test();
//	test.run_test(color_image, depth);
//		cv::imshow("Rotated", rotated);
//		cv::imshow("RGB", color_image);
//		cv::moveWindow("RGB", 10,600);





//	cv::Mat test1 = cv::imread("/home/rmb-dh/obst.jpg"); //TEST
//	cv::Mat test2 = cv::imread("/home/rmb-dh/strasse.jpg"); //TEST
//	cv::Mat test3 = cv::imread("/home/rmb-dh/strasse2.jpg"); //TEST
//	cv::Mat test4 = cv::imread("/home/rmb-dh/stadt.jpg"); //TEST
//	cv::Mat test5 = cv::imread("/home/rmb-dh/test4.jpg"); //TEST
//	cv::Mat test6 = cv::imread("/home/rmb-dh/test5.jpg"); //TEST
//	resize(test4, test4, cv::Size(), 0.2, 0.2, cv::INTER_CUBIC);

	// do something useful with code located in common/src
//	cv::Mat gray_image, dx;
//	cv::cvtColor(color_image, gray_image, CV_BGR2GRAY);
//	cv::Sobel(gray_image, dx, -1, 1, 0, 3);
//	double lbp_hist[10];






	//LBP only
//	create_lbp lbp = create_lbp();
//	lbp.create_lbp_class(test4, 1, 8, false, lbp_hist);
//	for(int i=0;i<10;i++)
//	{
//		std::cout << lbp_hist[i]<<"--" << i << "- ";
//	}std::cout <<std::endl;


	//Split and Merge with LBP

//	for(int i=0; i<newimg.rows-2;i++)
//	{for(int j=0;j<newimg.cols;j++)
//	{for(int rgb=0;rgb<3;rgb++)
//	{
//		newimg.at<cv::Vec3b>(i,j)[rgb]=255;
//	}
//	}
//	}
//
//	splitandmerge test = splitandmerge();
//	cv::Mat pic1 = test.categorize(test1);
//	cv::Mat pic2 = test.categorize(test2);
//	cv::Mat pic3 = test.categorize(test3);
//	cv::Mat pic4 = test.categorize(test4);
//	cv::Mat pic5 = test.categorize(test5);
//	cv::Mat pic6 = test.categorize(test6);
//	cv::imshow("sam1", pic1);
//	cv::moveWindow("sam1", 0,0);
//	cv::imshow("sam2", pic2);
//	cv::moveWindow("sam2", 1380,0);
//	cv::imshow("sam3", pic3);
//	cv::moveWindow("sam3", 740,0);
//	cv::imshow("sam4", pic4);
//	cv::moveWindow("sam4", 0,480);
//	cv::imshow("sam5", pic5);
//	cv::moveWindow("sam5", 1380,480);
//	cv::imshow("sam6", pic6);
//	cv::moveWindow("sam6", 740,480);
//	cv::imshow("sam6o", test6);
//	cv::imshow("orig1", test1);
//	cv::imshow("orig2", test2);
//	cv::imshow("orig3", test3);
//	cv::imshow("orig4", test4);
//	cv::imshow("orig5", test5);


//	cv::Mat picstream = test.categorize(color_image);
//	cv::imshow("image", color_image);
//	cv::imshow("image2", picstream);

//	texture_features edge = texture_features();
//	edge.primitive_size(test1);



//	compute_textures test = compute_textures();
//	test.compute_textures_all();


//	cv::imshow("gray image", gray_image);
//	cv::imshow("dx image", dx);
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
