/*
 * object_recording.h
 *
 *  Created on: 02.07.2013
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef TEXTURE_CATEGORIZATION_H
#define TEXTURE_CATEGORIZATION_H

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>
//#include <tf/tf.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
//#include <sensor_msgs/CameraInfo.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// boost
#include <boost/bind.hpp>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// pcl
//#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <vector>

#include <visualization_msgs/MarkerArray.h>


#include "std_msgs/String.h"


//#include </home/rmb-dh/git/care-o-bot/cob_object_perception/cob_surface_classification/msg_gen/cpp/include/cob_surface_classification/SegmentedPointCloud2.h>
#include "cob_surface_classification/Int32Array.h"
#include "cob_surface_classification/SegmentedPointCloud2.h"

#include "cob_texture_categorization/train_ml.h"
#include "cob_texture_categorization/attribute_learning.h"


class TextCategorizationNode
{
public:

	TextCategorizationNode(ros::NodeHandle nh);
	~TextCategorizationNode();
	void init();


//	Marker to visualize Nomals of plane
	ros::Publisher coordinatesystem;
	visualization_msgs::MarkerArray marker;
	ros::Publisher cloudpub;
//	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	visualization_msgs::Marker cloud;
	ros::Publisher pub_cloud;

	void segmentationCallback(const std_msgs::String::ConstPtr& msg);
	void segmented_pointcloud_callback(const cob_surface_classification::SegmentedPointCloud2& msg2);
protected:



	ros::Subscriber input_color_camera_info_sub_;	///< camera calibration of incoming color image data

	ros::Subscriber segmented_pointcloud_;

	ros::NodeHandle node_handle_;			///< ROS node handle
	bool camera_matrix_received_;
	///< projection matrix of the calibrated camera that transforms points from 3D to image plane in homogeneous coordinates:
	///[u,v,w]=P*[X,Y,Z,1]
	cv::Mat color_camera_matrix_;

	///< detection of coordinate system the object is placed on
//	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> input_marker_detection_sub_;
//	boost::shared_ptr<FiducialModelPi> arm_fidu;

	// messages
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter colorimage_sub_; ///< Color camera image topic
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >* sync_input_;



	/// Callback for the incoming pointcloud data stream.
	void inputCallback(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
	/// Converts a color image message to cv::Mat format.
	bool convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);
	/// Projects a 3D point into the coordinates of the color camera image.
	unsigned long ProjectXYZ(double x, double y, double z, int& u, int& v);
	/// Callback function for receiving the camera calibration.
	void calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg);


//	void inputCallbackNoCam();

	void attributeLearningGeneratedDatabaseTestHandcrafted();

	void attributeLearningDatabaseTestFarhadi();
	void attributeLearningDatabaseTestHandcrafted();
	void attributeLearningDatabaseTestCimpoi();
//	void attributeLearningDatabaseTestAutomatedClass();

	void crossValidationVerbalClassDescription();

	// defines a set of Neural Network configurations for cross-validation
	void setNNConfigurations(CrossValidationParams& cvp, const std::string& experiment_key);
	// defines a set of SVM configurations for cross-validation
	void setSVMConfigurations(CrossValidationParams& cvp, const std::string& experiment_key);

	std::set<int> considered_classes_;	// considered classes may limit the amount of classes used from a database for training and recognition

	IfvFeatures ifv_;
	AttributeLearning al_;
	train_ml ml_;
};


#endif // TEXTURE_CATEGORIZATION_H
