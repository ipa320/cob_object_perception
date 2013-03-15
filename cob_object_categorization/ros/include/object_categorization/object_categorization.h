/*
 * object_categorization.h
 *
 *  Created on: 16.10.2012
 *      Author: rbormann
 */

#ifndef OBJECT_CATEGORIZATION_H_
#define OBJECT_CATEGORIZATION_H_

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cob_perception_msgs/PointCloud2Array.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// boost
#include <boost/bind.hpp>

// PCL
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

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <object_categorization/ObjectClassifier.h>


class ObjectCategorization
{
public:

	ObjectCategorization();
	ObjectCategorization(ros::NodeHandle nh);

	~ObjectCategorization();

	void Training();

protected:
	/// callback for the incoming pointcloud data stream
	void inputCallback(const cob_perception_msgs::PointCloud2Array::ConstPtr& input_pointcloud_segments_msg, const sensor_msgs::Image::ConstPtr& input_image_msg);

	/// Converts a color image message to cv::Mat format.
	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	void calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg);

//	ros::Subscriber input_pointcloud_sub_;	///< incoming point cloud topic
	message_filters::Subscriber<cob_perception_msgs::PointCloud2Array> input_pointcloud_sub_;	///< incoming point cloud topic
	ros::Subscriber input_pointcloud_camera_info_sub_;	///< camera calibration of incoming data
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter color_image_sub_; ///< color camera image topic
	message_filters::Synchronizer< message_filters::sync_policies::ApproximateTime<cob_perception_msgs::PointCloud2Array, sensor_msgs::Image> >* sync_input_;

	ros::NodeHandle node_handle_;			///< ROS node handle

	unsigned int pointcloud_width_;			///< width of the received point cloud
	unsigned int pointcloud_height_;			///< height of the received point cloud
	cv::Mat projection_matrix_;	///< projection matrix of the calibrated camera that transforms points from 3D to image plane in homogeneous coordinates: [u,v,w]=P*[X,Y,Z,1]

	ObjectClassifier object_classifier_;
};

#endif /* OBJECT_CATEGORIZATION_H_ */
