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
#include <dynamic_reconfigure/server.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
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

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// config
#include <cob_object_recording/ObjectRecordingConfig.h>

// boost
#include <boost/bind.hpp>

// SFML
#include <SFML/Audio.hpp>

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


#include <vector>

class ObjectRecording
{
public:

//	ObjectRecording();
	ObjectRecording(ros::NodeHandle nh);

	~ObjectRecording();

	/// data structure for storing recorded images and accompanying information
	struct RecordingData
	{
		cv::Mat image;										///< the color image
		pcl::PointCloud<pcl::PointXYZRGB> pointcloud;		///< the xyzrdb point cloud
		tf::Transform pose_desired;							///< the target recording perspective (pose of the camera relative to the object center defined by the marker coordinate system)
		tf::Transform pose_recorded;						///< the actually recorded perspective (which might deviate to some extend from the desired w.r.t to the allowed deviation)
		double distance_to_desired_pose;					///< a combined distance measure for the translational and rotational distance of the recorded pose to the desired pose, mainly for internal use on deciding for updating a pose with new data
		double sharpness_score;								///< a measure of image sharpness, mainly used for comparing a new recording against the existing
		bool perspective_recorded;							///< this flag is only true, when data for this perspective has been recorded at least once

		RecordingData()
		{
			perspective_recorded = false;
			distance_to_desired_pose = 1e10;
			sharpness_score = 0.;
		};
	};

protected:

	/// Implementation of the service for starting recording.
	bool startRecording(cob_object_detection_msgs::StartObjectRecording::Request &req, cob_object_detection_msgs::StartObjectRecording::Response &res);

	/// Implementation of the service for stopping recording.
	bool stopRecording(cob_object_detection_msgs::StopObjectRecording::Request &req, cob_object_detection_msgs::StopObjectRecording::Response &res);

	/// Implementation of the service for storing the recorded object on disc.
	bool saveRecordedObject(cob_object_detection_msgs::SaveRecordedObject::Request &req, cob_object_detection_msgs::SaveRecordedObject::Response &res);

	/// Callback for the incoming pointcloud data stream.
	void inputCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg, const sensor_msgs::PointCloud2::ConstPtr& input_pointcloud_msg, const sensor_msgs::Image::ConstPtr& input_image_msg);

	/// Converts a color image message to cv::Mat format.
	bool convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	/// Computes an average pose from multiple detected markers.
	tf::Transform computeMarkerPose(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg);

	/// Publishes all target poses for recording and their status: captured or not captured already.
	void publishRecordingPoseMarkers(const cob_object_detection_msgs::DetectionArray::ConstPtr& input_marker_detections_msg, tf::Transform fiducial_pose);

	/// Helper function for computing a target camera perspective w.r.t. to the object coordinate system.
	/// @param pan The azimuth angle of the camera relative to the object center.
	/// @param tilt The height angle of the camera relative to the object center.
	/// @param preferred_recording_distance	The target distance of the camera from the object center while capturing data.
	/// @param perspective_pose	The computed target pose.
	void computePerspective(const double& pan, const double& tilt, const double& preferred_recording_distance, tf::Transform& perspective_pose);

	/// Segmentation function for removing all parts from the color image and the point cloud that do not belong to the object.
	/// @param color_image A color image.
	/// @param pointcloud A point cloud.
	/// @param pose_OfromC The transform from the object coordinate system to the camera, i.e. that transform which converts from camera coordinates to object coordinates.
	/// @param xyzr_learning_coordinates The bounding box of the recording area, all data inside this box is considered part of the object. Attention: val[0]=half length, val[1]=half width, val[2]=full height, val[3]=offset to minimal height 0 (to exclude outliers of the ground plane)
	/// @param uv_learning_boundaries Returned 2D bounding box of the recorded area in the color image, val[0]=minU, val[1]=maxU, val[2]=minV, val[3]=maxV
	/// @return 0 if everything went well.
	unsigned long ImageAndRangeSegmentation(cv::Mat& color_image, pcl::PointCloud<pcl::PointXYZRGB>& pointcloud, const tf::Transform& pose_OfromC, cv::Scalar& xyzr_learning_coordinates, cv::Scalar& uv_learning_boundaries);

	/// Projects a 3D point into the coordinates of the color camera image.
	unsigned long ProjectXYZ(double x, double y, double z, int& u, int& v);

	/// Callback function for receiving the camera calibration.
	void calibrationCallback(const sensor_msgs::CameraInfo::ConstPtr& calibration_msg);

	/// Dynamic reconfigure callback.
	void dynamicReconfigureCallback(cob_object_recording::ObjectRecordingConfig &config, uint32_t level);

	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> input_marker_detection_sub_;	///< detection of coordinate system the object is placed on
//	ros::Subscriber input_pointcloud_sub_;	///< incoming point cloud topic
	message_filters::Subscriber<sensor_msgs::PointCloud2> input_pointcloud_sub_;	///< incoming point cloud topic
	ros::Subscriber input_color_camera_info_sub_;	///< camera calibration of incoming color image data
	boost::shared_ptr<image_transport::ImageTransport> it_sub_;
	image_transport::SubscriberFilter color_image_sub_; ///< color camera image topic
	message_filters::Synchronizer< message_filters::sync_policies::ApproximateTime<cob_object_detection_msgs::DetectionArray, sensor_msgs::PointCloud2, sensor_msgs::Image> >* sync_input_;
	message_filters::Connection registered_callback_;
	boost::shared_ptr<image_transport::ImageTransport> it_pub_;
	image_transport::Publisher display_image_pub_; ///< publishes 2D image data to display currently visible image with some hints for useful camera movements

	ros::ServiceServer service_server_start_recording_; ///< Service server which accepts requests for starting recording
	ros::ServiceServer service_server_stop_recording_; ///< Service server which accepts requests for stopping recording
	ros::ServiceServer service_server_save_recorded_object_; ///< Service server which accepts requests for saving recorded data to disk

	dynamic_reconfigure::Server<cob_object_recording::ObjectRecordingConfig> dynamic_reconfigure_server_;

	ros::Publisher recording_pose_marker_array_publisher_;
	unsigned int prev_marker_array_size_; ///< Size of previously published marker array
	visualization_msgs::MarkerArray marker_array_msg_;

	ros::NodeHandle node_handle_;			///< ROS node handle

	// sound feedback
	std::vector<sf::Int16> sound_feedback_samples_proximity_;
	sf::SoundBuffer sound_feedback_buffer_proximity_;
	sf::Sound sound_feedback_sound_proximity_;
	std::vector<sf::Int16> sound_feedback_samples_hit_;
	sf::SoundBuffer sound_feedback_buffer_hit_;
	sf::Sound sound_feedback_sound_hit_;

//	unsigned int pointcloud_width_;			///< width of the received point cloud
//	unsigned int pointcloud_height_;			///< height of the received point cloud
	bool camera_matrix_received_;
	cv::Mat color_camera_matrix_;	///< projection matrix of the calibrated camera that transforms points from 3D to image plane in homogeneous coordinates: [u,v,w]=P*[X,Y,Z,1]

	double sharpness_threshold_;	///< threshold for the image sharpness, images with lower sharpness are not utilized for data recording
	int pan_divisions_;		///< the number of images that need to be recorded along the pan direction around the object at every tilt level, pan=[0°...360°]
	int tilt_divisions_;	///< the number of images that need to be recorded along the tilt direction around the object at every pan level, tilt=[0°...-90°], i.e. only the upper hemisphere
	double preferred_recording_distance_;	///< desired camera distance to object while recording in [m]
	double distance_threshold_translation_;		///< only record an image if the camera is closer to the target perspective than this length (in [m])
	double distance_threshold_orientation_;		///< only record an image if the camera orientation is closer to the target perspective than this angle (in radiant)

	std::string current_object_label_;		///< label of the recorded object
	std::vector<RecordingData> recording_data_;		///< container for the desired perspectives and the recorded data

	std::string data_storage_path_;		///< folder for data storage
	cv::Scalar xyzr_recording_bounding_box_;	///< (maximum) bounding box for the recorded object, i.e. the bounding box may be specified too big. (val[0]=half length, val[1]=half width, val[2]=full height, val[3]=offset to minimal height 0 (to exclude outliers of the ground plane))
};

#endif /* OBJECT_RECORDING_H_ */
