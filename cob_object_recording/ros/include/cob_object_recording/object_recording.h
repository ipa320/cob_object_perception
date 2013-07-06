/*
 * object_recording.h
 *
 *  Created on: 02.07.2013
 *      Author: rbormann
 */

#ifndef OBJECT_RECORDING_H_
#define OBJECT_RECORDING_H_

// ROS includes
#include <ros/ros.h>
//#include <ros/package.h>
#include <tf/tf.h>

// ROS message includes
#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <cob_object_detection_msgs/DetectionArray.h>
#include <cob_object_detection_msgs/StartObjectRecording.h>
#include <cob_object_detection_msgs/StopObjectRecording.h>
#include <cob_object_detection_msgs/SaveRecordedObject.h>

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


class ObjectRecording
{
public:

//	ObjectRecording();
	ObjectRecording(ros::NodeHandle nh);

	~ObjectRecording();

protected:
	/// callback for the incoming pointcloud data stream
	void inputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg, const sensor_msgs::PointCloud2::ConstPtr& input_pointcloud_msg, const sensor_msgs::Image::ConstPtr& input_image_msg);

	/// Converts a color image message to cv::Mat format.
	bool convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	tf::Transform computeMarkerPose(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);
//	unsigned long ProjectXYZ(double x, double y, double z, int& u, int& v);

//	void calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg);

	bool startRecording(cob_object_detection_msgs::StartObjectRecording::Request &req, cob_object_detection_msgs::StartObjectRecording::Response &res);
	bool stopRecording(cob_object_detection_msgs::StopObjectRecording::Request &req, cob_object_detection_msgs::StopObjectRecording::Response &res);
	bool saveRecordedObject(cob_object_detection_msgs::SaveRecordedObject::Request &req, cob_object_detection_msgs::SaveRecordedObject::Response &res);

	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> input_marker_detection_sub_;	///< detection of coordinate system the object is placed on
//	ros::Subscriber input_pointcloud_sub_;	///< incoming point cloud topic
	message_filters::Subscriber<sensor_msgs::PointCloud2> input_pointcloud_sub_;	///< incoming point cloud topic
//	ros::Subscriber input_color_camera_info_sub_;	///< camera calibration of incoming color image data
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter color_image_sub_; ///< color camera image topic
	message_filters::Synchronizer< message_filters::sync_policies::ApproximateTime<cob_object_detection_msgs::DetectionArray, sensor_msgs::PointCloud2, sensor_msgs::Image> >* sync_input_;
	message_filters::Connection registered_callback_;

	ros::ServiceServer service_server_start_recording_; ///< Service server which accepts requests for starting recording
	ros::ServiceServer service_server_stop_recording_; ///< Service server which accepts requests for stopping recording
	ros::ServiceServer service_server_save_recorded_object_; ///< Service server which accepts requests for saving recorded data to disk

	ros::NodeHandle node_handle_;			///< ROS node handle

//	unsigned int pointcloud_width_;			///< width of the received point cloud
//	unsigned int pointcloud_height_;			///< height of the received point cloud
	cv::Mat color_camera_matrix_;	///< projection matrix of the calibrated camera that transforms points from 3D to image plane in homogeneous coordinates: [u,v,w]=P*[X,Y,Z,1]

	double sharpness_threshold_;	///< threshold for the image sharpness, images with lower sharpness are not utilized for data recording

	int pan_divisions_;		///< the number of images that need to be recorded along the pan direction around the object at every tilt level, pan=[0°...360°]
	int tilt_divisions_;	///< the number of images that need to be recorded along the tilt direction around the object at every pan level, tilt=[0°...90°], i.e. only the upper hemisphere
};

#endif /* OBJECT_RECORDING_H_ */
