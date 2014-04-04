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
//#include <ros/package.h>
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



class TextCategorizationNode
{
public:

	TextCategorizationNode(ros::NodeHandle nh);
	~TextCategorizationNode();
	void init();



protected:

	ros::Subscriber input_color_camera_info_sub_;	///< camera calibration of incoming color image data

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


	void inputCallbackNoCam();

};


#endif // TEXTURE_CATEGORIZATION_H
