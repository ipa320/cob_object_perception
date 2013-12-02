/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_object_perception
 * ROS package name: cob_fiducials
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
 * Supervised by: Jan Fischer, email:jan.fischer@ipa.fhg.de
 *
 * Date of creation: March 2013
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary rforms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
//#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_srvs/Empty.h>

// external includes
//#include <cob_fiducials/fiducialsConfig.h>
//#include <dynamic_reconfigure/server.h>
#include <cob_object_detection_msgs/DetectObjects.h>
#include <cob_vision_utils/GlobalDefines.h>
#include <cob_vision_utils/VisionUtils.h>
#include <cob_fiducials/FiducialDefines.h>
#include <cob_fiducials/pi/FiducialModelPi.h>
#include <cob_fiducials/aruco/FiducialModelAruco.h>

#include <boost/thread/mutex.hpp>
#include <boost/timer.hpp>

//#include "opencv/highgui.h"

using namespace message_filters;

namespace ipa_Fiducials
{

typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> ColorImageSyncPolicy;


/// @class CobFiducialsNode
/// This node gathers images from a 'color camera'
/// to recognize fiducials
class CobFiducialsNode //: public nodelet::Nodelet
{
    enum t_Mode
    {
        MODE_TOPIC = 0,
        MODE_SERVICE,
        MODE_TOPIC_AND_SERVICE
    };


private:
    ros::NodeHandle node_handle_;

    boost::shared_ptr<image_transport::ImageTransport> image_transport_0_;
    boost::shared_ptr<image_transport::ImageTransport> image_transport_1_;

    // Subscriptions
    image_transport::SubscriberFilter color_camera_image_sub_;	///< color camera image topic
    message_filters::Subscriber<sensor_msgs::CameraInfo> color_camera_info_sub_;	///< camera information service

    boost::shared_ptr<message_filters::Synchronizer<ColorImageSyncPolicy > > color_image_sub_sync_; ///< Synchronizer
    tf::TransformListener transform_listener_; ///< tf transforms

    int sub_counter_; /// Number of subscribers to topic
    unsigned int endless_counter_; ///< A counter to show that node is still receiving images
    bool synchronizer_received_; ///< Set to true, when synchronizer fires

    // Service definitions
    ros::ServiceServer detect_fiducials_service_; ///< Service server to request fidcuial detection
    ros::ServiceServer stop_tf_service_; 
    // Publisher definitions
    ros::Publisher detect_fiducials_pub_;
    ros::Publisher fiducials_marker_array_publisher_;
    image_transport::Publisher img2D_pub_; ///< Publishes 2D image data to show detection results

    cv::Mat color_mat_8U3_;
    cv::Mat camera_matrix_;
    bool camera_matrix_initialized_;

//    ros::Time received_timestamp_;
//    std::string received_frame_id_;
    cob_object_detection_msgs::DetectionArray detection_array_;


    //fiducials::fiducialsConfig launch_reconfigure_config_;
    //dynamic_reconfigure::Server<cob_fiducials::fiducialsConfig> dynamic_reconfigure_server_;

    bool compute_sharpness_measure_;	///< computes a measure for image sharpness
    bool log_or_calibrate_sharpness_measurements_;		///< if true, the sharpness measurements are logged and saved to disc for calibration of the curve or directly calibrated within the program
    double sharpness_calibration_parameter_m_;		///< the m and n parameters of the linear calibration function sharpness_score = m * pixel_count + n -> m,n can be determined with the 'log_or_calibrate_sharpness_measurements_' option
    double sharpness_calibration_parameter_n_;
    bool publish_tf_;
    tf::TransformBroadcaster tf_broadcaster_; ///< Broadcast transforms of detected fiducials
    bool publish_2d_image_;
    bool publish_marker_array_; ///< Publish coordinate systems of detected fiducials as marker for rviz
    unsigned int prev_marker_array_size_; ///< Size of previously published marker array
    visualization_msgs::MarkerArray marker_array_msg_;

    int debug_verbosity_; ///< verbosity level on output (1=all outputs, 2=only warnings,infos and errors)

    static std::string color_image_encoding_; ///< Color encoding of incoming messages
    CobFiducialsNode::t_Mode ros_node_mode_;	///< Specifys if node is started as topic or service
    std::string model_directory_; ///< Working directory, from which models are loaded and saved
    std::string model_filename_;

    boost::mutex mutexQ_;
    boost::condition_variable condQ_;

    t_FiducialType fiducial_type_;
    boost::shared_ptr<ipa_Fiducials::AbstractFiducialModel> tag_detector_;
    
    boost::mutex tf_lock_;
    ros::Timer tf_pub_timer_;
    tf::StampedTransform marker_tf_;

public:
    /// Constructor.
    CobFiducialsNode(ros::NodeHandle& nh)
        : sub_counter_(0),
          endless_counter_(0)
    {
        camera_matrix_initialized_ = false;
        /// Void
        node_handle_ = nh;
        onInit();
    }

    /// Destructor.
    ~CobFiducialsNode()
    {
        fiducials_marker_array_publisher_.shutdown();
    }

    void onInit()
    {
        /// Create a handle for this node, initialize node
        //	  node_handle_ = getMTNodeHandle();
        image_transport_0_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(node_handle_));
        image_transport_1_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(node_handle_));

        /// Initialize camera node
        if (!init()) return;
    }

    /// Initialize sensor fusion node.
    /// Setup publisher of point cloud and corresponding color data,
    /// setup camera toolboxes and colored point cloud toolbox
    /// @return <code>true</code> on success, <code>false</code> otherwise
    bool init()
    {
        if (loadParameters() == false) return false;

        ros::SubscriberStatusCallback imgConnect    = boost::bind(&CobFiducialsNode::connectCallback, this);
        ros::SubscriberStatusCallback imgDisconnect = boost::bind(&CobFiducialsNode::disconnectCallback, this);

        // Synchronize inputs of incoming image data.
        // Topic subscriptions happen on demand in the connection callback.
        ROS_INFO("[fiducials] Setting up image data subscribers");
        if (ros_node_mode_ == MODE_TOPIC || ros_node_mode_ == MODE_TOPIC_AND_SERVICE)
        {
            detect_fiducials_pub_ = node_handle_.advertise<cob_object_detection_msgs::DetectionArray>("detect_fiducials", 1, imgConnect, imgDisconnect);
        }
        if (ros_node_mode_ == MODE_SERVICE || ros_node_mode_ == MODE_TOPIC_AND_SERVICE)
        {
            detect_fiducials_service_ = node_handle_.advertiseService("get_fiducials", &CobFiducialsNode::detectFiducialsServiceCallback, this);
        }

        // Publisher for visualization/debugging
        fiducials_marker_array_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>( "fiducial_marker_array", 0 );
        img2D_pub_= image_transport_1_->advertise("image", 1);

        synchronizer_received_ = false;
        prev_marker_array_size_ = 0;

        //dynamic_reconfigure::Server<cob_fiducials::fiducialsConfig>::CallbackType f;
        //f = boost::bind(&CobFiducialsNode::dynamicReconfigureCallback, this, _1, _2);
        //dynamic_reconfigure_server_.clearCallback();
        //dynamic_reconfigure_server_.updateConfig(launch_reconfigure_config_);
        //dynamic_reconfigure_server_.setCallback(f);

        ROS_INFO("[fiducials] Setting up tag library");
	switch(fiducial_type_)
	{
	case ipa_Fiducials::TYPE_PI:
	        tag_detector_ = boost::shared_ptr<FiducialModelPi>(new FiducialModelPi());
		break;
	case ipa_Fiducials::TYPE_ARUCO:
	        tag_detector_ = boost::shared_ptr<FiducialModelAruco>(new FiducialModelAruco());
		break;
	default:
        	ROS_ERROR("[fiducials] Unknown fiducial type");
		return false;
	}

	if(publish_tf_) {
		tf_pub_timer_.stop();
		tf_pub_timer_ = node_handle_.createTimer(ros::Duration(0.1), &CobFiducialsNode::callback_pub_tf, this);
		stop_tf_service_ = node_handle_.advertiseService("stop_tf", &CobFiducialsNode::stopTfServiceCallback, this);
	}

        ROS_INFO("[fiducials] Initializing [OK]");
        ROS_INFO("[fiducials] Up and running");
        return true;
    }

	void callback_pub_tf(const ros::TimerEvent& event)
	{
		if(marker_tf_.frame_id_.size()>0)
		{
			tf_lock_.lock();
			marker_tf_.stamp_ = ros::Time::now();
			tf_broadcaster_.sendTransform(marker_tf_);
			tf_lock_.unlock();
		}
	}

    //void dynamicReconfigureCallback(cob_fiducials::fiducialsConfig &config, uint32_t level)
    //{
    //	<localVariable> = config.<dynamic_reconfigure_variable>;
    //}


    /// Subscribe to camera topics if not already done.
    void connectCallback()
    {
        ROS_INFO("[fiducials] Subscribing to camera topics");

        color_camera_image_sub_.subscribe(*image_transport_0_, "image_color", 1);
        color_camera_info_sub_.subscribe(node_handle_, "camera_info", 1);

        color_image_sub_sync_ = boost::shared_ptr<message_filters::Synchronizer<ColorImageSyncPolicy> >(new message_filters::Synchronizer<ColorImageSyncPolicy>(ColorImageSyncPolicy(3)));
        color_image_sub_sync_->connectInput(color_camera_image_sub_, color_camera_info_sub_);
        color_image_sub_sync_->registerCallback(boost::bind(&CobFiducialsNode::colorImageCallback, this, _1, _2));

        sub_counter_++;
        ROS_INFO("[fiducials] %i subscribers on camera topics [OK]", sub_counter_);
    }

    /// Unsubscribe from camera topics if possible.
    void disconnectCallback()
    {
    	if (sub_counter_ > 0)
            sub_counter_--;

    	if (sub_counter_ == 0)
        {
            ROS_INFO("[fiducials] Unsubscribing from camera topics");

            color_camera_image_sub_.unsubscribe();
            color_camera_info_sub_.unsubscribe();
        }
    	ROS_INFO("[fiducials] %i subscribers on camera topics [OK]", sub_counter_);
    }

    /// Callback is executed, when shared mode is selected
    /// Left and right is expressed when facing the back of the camera in horizontal orientation.
    void colorImageCallback(const sensor_msgs::ImageConstPtr& color_camera_data,
                            const sensor_msgs::CameraInfoConstPtr& color_camera_info)
    {
        {
            boost::mutex::scoped_lock lock( mutexQ_ );

            ROS_DEBUG("[fiducials] color image callback");

            if (camera_matrix_initialized_ == false)
            {
                camera_matrix_ = cv::Mat::zeros(3,3,CV_64FC1);
                camera_matrix_.at<double>(0,0) = color_camera_info->K[0];
                camera_matrix_.at<double>(0,2) = color_camera_info->K[2];
                camera_matrix_.at<double>(1,1) = color_camera_info->K[4];
                camera_matrix_.at<double>(1,2) = color_camera_info->K[5];
                camera_matrix_.at<double>(2,2) = 1;

                ROS_INFO("[fiducials] Initializing fiducial detector with camera matrix");
		
				if (tag_detector_->Init(camera_matrix_, model_directory_ + model_filename_, log_or_calibrate_sharpness_measurements_) & ipa_Utils::RET_FAILED)
				{
					ROS_ERROR("[fiducials] Initializing fiducial detector with camera matrix [FAILED]");
					return;
				}
                camera_matrix_initialized_ = true;
            }

            // Receive
            cv_bridge::CvImageConstPtr cv_ptr;
            try
            {
              cv_ptr = cv_bridge::toCvShare(color_camera_data, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e)
            {
              ROS_ERROR("cv_bridge exception: %s", e.what());
              return;
            }

//            received_timestamp_ = color_camera_data->header.stamp;
//            received_frame_id_ = color_camera_data->header.frame_id;
            cv::Mat tmp = cv_ptr->image;
            color_mat_8U3_ = tmp.clone();

//            cob_object_detection_msgs::DetectionArray detection_array;
            detection_array_.detections.clear();
			detection_array_.header = color_camera_data->header;
			//hack for simulation and bag file:
			//detection_array_.header.stamp = ros::Time::now();
			//detection_array_.header.frame_id = "/head_cam3d_link";

			detectFiducials(detection_array_, color_mat_8U3_);
            if (ros_node_mode_ == MODE_TOPIC || ros_node_mode_ == MODE_TOPIC_AND_SERVICE)
            {
                // Publish
                detect_fiducials_pub_.publish(detection_array_);
            }

//            synchronizer_received_ = true;

            // Notify waiting thread
        }
        condQ_.notify_one();
    }

    bool stopTfServiceCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
    {
        ROS_DEBUG("[fiducials] Service Callback");

	tf_lock_.lock();
	marker_tf_.frame_id_ = "";
	tf_lock_.unlock();

	return true;
    }


    bool detectFiducialsServiceCallback(cob_object_detection_msgs::DetectObjects::Request &req,
                                        cob_object_detection_msgs::DetectObjects::Response &res)
    {
        ROS_DEBUG("[fiducials] Service Callback");

        // Connect to image topics
        bool result = false;
//        synchronizer_received_ = false;
        connectCallback();

        // Wait for data
        {
            boost::mutex::scoped_lock lock( mutexQ_);
            boost::system_time const timeout=boost::get_system_time()+ boost::posix_time::milliseconds(5000);

            ROS_INFO("[fiducials] Waiting for image data");
            if (condQ_.timed_wait(lock, timeout))		// wait until condition is set by callback function
                ROS_INFO("[fiducials] Waiting for image data [OK]");
            else
            {
                ROS_WARN("[fiducials] Could not receive image data from ApproximateTime synchronizer");
                return false;
            }

            // Wait for data (at least 5 seconds)
            //int nSecPassed = 0;
            //float nSecIncrement = 0.5;
            //while (!synchronizer_received_ && nSecPassed < 10)
            //{
            //	ros::Duration(nSecIncrement).sleep();
            //	nSecPassed += nSecIncrement;
            //	ROS_INFO("[fiducials] Waiting");
            //}

            //if (!synchronizer_received_)
            //{
            //	ROS_WARN("[fiducials] Could not receive image data");
            //	return false;
            //}
            
            
            if (req.object_name.data == "ALL" || req.object_name.data == "")
            {
				// Publish all detections
				res.object_list = detection_array_;
			}
			else
			{
				// Publish specific detections
				cob_object_detection_msgs::DetectionArray filtered_detection_array;
				filtered_detection_array.header = detection_array_.header;
				
				for (unsigned int i=0; i<detection_array_.detections.size(); i++)
				{
					if (detection_array_.detections[i].label == req.object_name.data)
					{
						
						filtered_detection_array.detections.push_back(detection_array_.detections[i]);
					}
				}
                
				res.object_list = filtered_detection_array;
			}
			
            
            result = !(res.object_list.detections.empty()); //detectFiducials(res.object_list, color_mat_8U3_);
        }
        disconnectCallback();

        return result;
    }

    bool detectFiducials(cob_object_detection_msgs::DetectionArray& detection_array, cv::Mat& color_image)
    {
        int id_start_idx = 2351;
        unsigned int marker_array_size = 0;
        unsigned int pose_array_size = 0;

        // Detect fiducials and assign results
        std::vector<ipa_Fiducials::t_pose> tags_vec;
        std::vector<std::vector<double> >vec_vec7d;

	tf_lock_.lock();
	marker_tf_.frame_id_ = "";
	tf_lock_.unlock();

	unsigned long ret_val = ipa_Utils::RET_OK;
	ret_val = tag_detector_->GetPose(color_image, tags_vec);

	if (ret_val & ipa_Utils::RET_OK)
        {
            pose_array_size = tags_vec.size();

            // TODO: Average results
            for (unsigned int i=0; i<pose_array_size; i++)
            {
                cob_object_detection_msgs::Detection fiducial_instance;

				std::stringstream ss;
				ss << "tag_" << tags_vec[i].id;
				fiducial_instance.header = detection_array.header;
                fiducial_instance.label = ss.str();
                fiducial_instance.detector = tag_detector_->GetType();
                fiducial_instance.score = 0;
                fiducial_instance.bounding_box_lwh.x = 0;
                fiducial_instance.bounding_box_lwh.y = 0;
                fiducial_instance.bounding_box_lwh.z = 0;

                // TODO: Set Mask
                cv::Mat frame(3,4, CV_64FC1);
                for (int k=0; k<3; k++)
                    for (int l=0; l<3; l++)
                        frame.at<double>(k,l) = tags_vec[i].rot.at<double>(k,l);
                frame.at<double>(0,3) = tags_vec[i].trans.at<double>(0,0);
                frame.at<double>(1,3) = tags_vec[i].trans.at<double>(1,0);
                frame.at<double>(2,3) = tags_vec[i].trans.at<double>(2,0);
                std::vector<double> vec7d = FrameToVec7(frame);
                vec_vec7d.push_back(vec7d);

                // Results are given in CfromO
                fiducial_instance.pose.pose.position.x =  vec7d[0];
                fiducial_instance.pose.pose.position.y =  vec7d[1];
                fiducial_instance.pose.pose.position.z =  vec7d[2];
                fiducial_instance.pose.pose.orientation.w =  vec7d[3];
                fiducial_instance.pose.pose.orientation.x =  vec7d[4];
                fiducial_instance.pose.pose.orientation.y =  vec7d[5];
                fiducial_instance.pose.pose.orientation.z =  vec7d[6];

                fiducial_instance.pose.header = detection_array.header;
//                fiducial_instance.pose.header.stamp = received_timestamp_;
//                fiducial_instance.pose.header.frame_id = received_frame_id_;

        		// Analyze the image sharpness at the area inside the detected marker
                if (compute_sharpness_measure_ == true)
                {
                	double sharpness_measure;
                	tag_detector_->GetSharpnessMeasure(color_image, tags_vec[i], tag_detector_->GetGeneralFiducialParameters(tags_vec[i].id), sharpness_measure, sharpness_calibration_parameter_m_, sharpness_calibration_parameter_n_);
                	fiducial_instance.score = sharpness_measure;
                }

                detection_array.detections.push_back(fiducial_instance);
                if (debug_verbosity_ == 1)
                    ROS_INFO("[fiducials] Detected Tag '%s' at x,y,z,rw,rx,ry,rz ( %f, %f, %f, %f, %f, %f, %f ) ",
                             fiducial_instance.label.c_str(), vec7d[0], vec7d[1], vec7d[2],
                             vec7d[3], vec7d[4], vec7d[5], vec7d[6]);
            }
        }
        else
        {
            pose_array_size = 0;
        }

        // Publish 2d image
        if (publish_2d_image_)
        {
            for (unsigned int i=0; i<pose_array_size; i++)
                RenderPose(color_image, tags_vec[i].rot, tags_vec[i].trans);
            cv_bridge::CvImage cv_ptr;
            cv_ptr.image = color_image;
            cv_ptr.encoding = CobFiducialsNode::color_image_encoding_;
            img2D_pub_.publish(cv_ptr.toImageMsg());
        }

        // Publish tf
        if (publish_tf_)
        {
            for (unsigned int i=0; i<pose_array_size; i++)
            {
                // Broadcast transform of fiducial
                tf::Transform transform;
                //std::stringstream tf_name;
                //tf_name << detection_array.detections[i].label;
                transform.setOrigin(tf::Vector3(vec_vec7d[i][0], vec_vec7d[i][1], vec_vec7d[i][2]));
                transform.setRotation(tf::Quaternion(vec_vec7d[i][4], vec_vec7d[i][5], vec_vec7d[i][6], vec_vec7d[i][3]));
		tf_lock_.lock();
		marker_tf_ = tf::StampedTransform(transform, ros::Time::now(), detection_array.header.frame_id, "marker");	//TODO: make parameter
		tf_lock_.unlock();
            }
        }

        // Publish marker array
        if (publish_marker_array_)
        {
            // 3 arrows for each coordinate system of each detected fiducial
            marker_array_size = 3*pose_array_size;
            if (marker_array_size >= prev_marker_array_size_)
            {
                marker_array_msg_.markers.resize(marker_array_size);
            }

            // publish a coordinate system from arrow markers for each object
            for (unsigned int i=0; i<pose_array_size; i++)
            {
                for (unsigned int j=0; j<3; j++)
                {
                    unsigned int idx = 3*i+j;
//                    marker_array_msg_.markers[idx].header.frame_id = received_frame_id_;// "/" + frame_id;//"tf_name.str()";
//                    marker_array_msg_.markers[idx].header.stamp = received_timestamp_;
                    marker_array_msg_.markers[idx].header = detection_array.header;
                    marker_array_msg_.markers[idx].ns = "fiducials";
                    marker_array_msg_.markers[idx].id =  id_start_idx + idx;
                    marker_array_msg_.markers[idx].type = visualization_msgs::Marker::ARROW;
                    marker_array_msg_.markers[idx].action = visualization_msgs::Marker::ADD;
                    marker_array_msg_.markers[idx].color.a = 0.85;
                    marker_array_msg_.markers[idx].color.r = 0;
                    marker_array_msg_.markers[idx].color.g = 0;
                    marker_array_msg_.markers[idx].color.b = 0;

                    marker_array_msg_.markers[idx].points.resize(2);
                    marker_array_msg_.markers[idx].points[0].x = 0.0;
                    marker_array_msg_.markers[idx].points[0].y = 0.0;
                    marker_array_msg_.markers[idx].points[0].z = 0.0;
                    marker_array_msg_.markers[idx].points[1].x = 0.0;
                    marker_array_msg_.markers[idx].points[1].y = 0.0;
                    marker_array_msg_.markers[idx].points[1].z = 0.0;

                    if (j==0)
                    {
                        marker_array_msg_.markers[idx].points[1].x = 0.2;
                        marker_array_msg_.markers[idx].color.r = 255;
                    }
                    else if (j==1)
                    {
                        marker_array_msg_.markers[idx].points[1].y = 0.2;
                        marker_array_msg_.markers[idx].color.g = 255;
                    }
                    else if (j==2)
                    {
                        marker_array_msg_.markers[idx].points[1].z = 0.2;
                        marker_array_msg_.markers[idx].color.b = 255;
                    }

                    marker_array_msg_.markers[idx].pose.position.x = vec_vec7d[i][0];
                    marker_array_msg_.markers[idx].pose.position.y = vec_vec7d[i][1];
                    marker_array_msg_.markers[idx].pose.position.z = vec_vec7d[i][2];
                    marker_array_msg_.markers[idx].pose.orientation.x = vec_vec7d[i][4];
                    marker_array_msg_.markers[idx].pose.orientation.y = vec_vec7d[i][5];
                    marker_array_msg_.markers[idx].pose.orientation.z = vec_vec7d[i][6];
                    marker_array_msg_.markers[idx].pose.orientation.w = vec_vec7d[i][3];

                    ros::Duration one_hour = ros::Duration(1); // 1 second
                    marker_array_msg_.markers[idx].lifetime = one_hour;
                    marker_array_msg_.markers[idx].scale.x = 0.01; // shaft diameter
                    marker_array_msg_.markers[idx].scale.y = 0.015; // head diameter
                    marker_array_msg_.markers[idx].scale.z = 0; // head length 0=default
                }

                if (prev_marker_array_size_ > marker_array_size)
                {
                    for (unsigned int i = marker_array_size; i < prev_marker_array_size_; ++i)
                    {
                        marker_array_msg_.markers[i].action = visualization_msgs::Marker::DELETE;
                    }
                }
                prev_marker_array_size_ = marker_array_size;

                fiducials_marker_array_publisher_.publish(marker_array_msg_);
            }
        } // End: publish markers

        if (tags_vec.empty())
            return false;
        return true;
    }

    unsigned long RenderPose(cv::Mat& image, cv::Mat& rot_3x3_CfromO, cv::Mat& trans_3x1_CfromO)
    {
        cv::Mat object_center(3, 1, CV_64FC1);
        double* p_object_center = object_center.ptr<double>(0);
        p_object_center[0] = 0;
        p_object_center[1] = 0;
        p_object_center[2] = 0;

        cv::Mat rot_inv = rot_3x3_CfromO.inv();

        // Compute coordinate axis for visualization
        cv::Mat pt_axis(4, 3, CV_64FC1);
        double* p_pt_axis = pt_axis.ptr<double>(0);
        p_pt_axis[0] = 0 + p_object_center[0];
        p_pt_axis[1] = 0 + p_object_center[1];
        p_pt_axis[2] = 0 + p_object_center[2];
        p_pt_axis = pt_axis.ptr<double>(1);
        p_pt_axis[0] = 0.1 + p_object_center[0];
        p_pt_axis[1] = 0 + p_object_center[1];
        p_pt_axis[2] = 0 + p_object_center[2];
        p_pt_axis = pt_axis.ptr<double>(2);
        p_pt_axis[0] = 0 + p_object_center[0];
        p_pt_axis[1] = 0.1 + p_object_center[1];
        p_pt_axis[2] = 0 + p_object_center[2];
        p_pt_axis = pt_axis.ptr<double>(3);
        p_pt_axis[0] = 0 + p_object_center[0];
        p_pt_axis[1] = 0 + p_object_center[1];
        p_pt_axis[2] = 0.1 + p_object_center[2];

        // Transform data points
        std::vector<cv::Point> vec_2d(4, cv::Point());
        for (int i=0; i<4; i++)
        {
            cv::Mat vec_3d = pt_axis.row(i).clone();
            vec_3d = vec_3d.t();
            vec_3d = rot_3x3_CfromO*vec_3d;
            vec_3d += trans_3x1_CfromO;
            double* p_vec_3d = vec_3d.ptr<double>(0);

            ReprojectXYZ(p_vec_3d[0], p_vec_3d[1], p_vec_3d[2],
                         vec_2d[i].x , vec_2d[i].y);
        }

        // Render results
        int line_width = 1;
        cv::line(image, vec_2d[0], vec_2d[1], cv::Scalar(0, 0, 255), line_width);
        cv::line(image, vec_2d[0], vec_2d[2], cv::Scalar(0, 255, 0), line_width);
        cv::line(image, vec_2d[0], vec_2d[3], cv::Scalar(255, 0, 0), line_width);

        return ipa_Utils::RET_OK;
    }

    unsigned long ReprojectXYZ(double x, double y, double z, int& u, int& v)
    {
        cv::Mat XYZ(3, 1, CV_64FC1);
        cv::Mat UVW(3, 1, CV_64FC1);

        double* d_ptr = 0;
        double du = 0;
        double dv = 0;
        double dw = 0;

        x *= 1000;
        y *= 1000;
        z *= 1000;

        d_ptr = XYZ.ptr<double>(0);
        d_ptr[0] = x;
        d_ptr[1] = y;
        d_ptr[2] = z;

        UVW = camera_matrix_ * XYZ;

        d_ptr = UVW.ptr<double>(0);
        du = d_ptr[0];
        dv = d_ptr[1];
        dw = d_ptr[2];

        u = cvRound(du/dw);
        v = cvRound(dv/dw);

        return ipa_Utils::RET_OK;
    }

// Function copied from cob_vision_ipa_utils/MathUtils.h to avoid dependency
    inline float SIGN(float x)
    {
        return (x >= 0.0f) ? +1.0f : -1.0f;
    }
    std::vector<double> FrameToVec7(const cv::Mat frame)
    {
        // [0]-[2]: translation xyz
        // [3]-[6]: quaternion wxyz
        std::vector<double> pose(7, 0.0);

        double r11 = frame.at<double>(0,0);
        double r12 = frame.at<double>(0,1);
        double r13 = frame.at<double>(0,2);
        double r21 = frame.at<double>(1,0);
        double r22 = frame.at<double>(1,1);
        double r23 = frame.at<double>(1,2);
        double r31 = frame.at<double>(2,0);
        double r32 = frame.at<double>(2,1);
        double r33 = frame.at<double>(2,2);

        double qw = ( r11 + r22 + r33 + 1.0) / 4.0;
        double qx = ( r11 - r22 - r33 + 1.0) / 4.0;
        double qy = (-r11 + r22 - r33 + 1.0) / 4.0;
        double qz = (-r11 - r22 + r33 + 1.0) / 4.0;
        if(qw < 0.0f) qw = 0.0;
        if(qx < 0.0f) qx = 0.0;
        if(qy < 0.0f) qy = 0.0;
        if(qz < 0.0f) qz = 0.0;
        qw = std::sqrt(qw);
        qx = std::sqrt(qx);
        qy = std::sqrt(qy);
        qz = std::sqrt(qz);
        if(qw >= qx && qw >= qy && qw >= qz)
        {
            qw *= +1.0;
            qx *= SIGN(r32 - r23);
            qy *= SIGN(r13 - r31);
            qz *= SIGN(r21 - r12);
        }
        else if(qx >= qw && qx >= qy && qx >= qz)
        {
            qw *= SIGN(r32 - r23);
            qx *= +1.0;
            qy *= SIGN(r21 + r12);
            qz *= SIGN(r13 + r31);
        }
        else if(qy >= qw && qy >= qx && qy >= qz)
        {
            qw *= SIGN(r13 - r31);
            qx *= SIGN(r21 + r12);
            qy *= +1.0;
            qz *= SIGN(r32 + r23);
        }
        else if(qz >= qw && qz >= qx && qz >= qy)
        {
            qw *= SIGN(r21 - r12);
            qx *= SIGN(r31 + r13);
            qy *= SIGN(r32 + r23);
            qz *= +1.0;
        }
        else
        {
            printf("coding error\n");
        }
        double r = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
        qw /= r;
        qx /= r;
        qy /= r;
        qz /= r;

        pose[3] = qw;
        pose[4] = qx;
        pose[5] = qy;
        pose[6] = qz;

        // Translation
        pose[0] = frame.at<double>(0,3);
        pose[1] = frame.at<double>(1,3);
        pose[2] = frame.at<double>(2,3);
        return pose;
    }

    unsigned long loadParameters()
    {
       std::string tmp_string;
        /// Parameters are set within the launch file
        if (node_handle_.getParam("fiducial_type", tmp_string) == false)
        {
            ROS_ERROR("[fiducials] fiducial type not specified");
            return false;
        }
        if (tmp_string == "TYPE_ARUCO")
        {
            fiducial_type_ = ipa_Fiducials::TYPE_ARUCO;
        }
        else if (tmp_string == "TYPE_PI")
        {
            fiducial_type_ = ipa_Fiducials::TYPE_PI;
        }
        else
        {
            std::string str = "[fiducials] TYPE '" + tmp_string + "' unknown, try 'TYPE_ARUCO' or 'TYPE_PI'";
            ROS_ERROR("%s", str.c_str());
            return false;
        }

        ROS_INFO("Fiducial type: %s", tmp_string.c_str());

        /// Parameters are set within the launch file
        if (node_handle_.getParam("ros_node_mode", tmp_string) == false)
        {
            ROS_ERROR("[fiducials] Mode for fiducial node not specified");
            return false;
        }
        if (tmp_string == "MODE_SERVICE")
        {
            ros_node_mode_ = CobFiducialsNode::MODE_SERVICE;
        }
        else if (tmp_string == "MODE_TOPIC")
        {
            ros_node_mode_ = CobFiducialsNode::MODE_TOPIC;
        }
        else if (tmp_string == "MODE_TOPIC_AND_SERVICE")
        {
            ros_node_mode_ = CobFiducialsNode::MODE_TOPIC_AND_SERVICE;
        }
        else
        {
            std::string str = "[fiducials] Mode '" + tmp_string + "' unknown, try 'MODE_SERVICE' or 'MODE_TOPIC'";
            ROS_ERROR("%s", str.c_str());
            return false;
        }

        ROS_INFO("ROS node mode: %s", tmp_string.c_str());

        // Parameters are set within the launch file
        if (node_handle_.getParam("model_directory", model_directory_) == false)
        {
            ROS_ERROR("[fiducials] 'model_directory=<dir1>/ydir2>/' not specified in launch file");
            return false;
        }
        ROS_INFO("[fiducials] model_directory: %s", model_directory_.c_str());
        if (node_handle_.getParam("model_filename", model_filename_) == false)
        {
            ROS_ERROR("[fiducials] 'model_filename=<filename>.xml' not specified in yaml file");
            return false;
        }
        ROS_INFO("[fiducials] model_filename: %s", model_filename_.c_str());
        if (node_handle_.getParam("compute_sharpness_measure", compute_sharpness_measure_) == false)
        {
            ROS_ERROR("[fiducials] 'compute_sharpness_measure=[true/false]' not specified in yaml file");
            return false;
        }
        if (compute_sharpness_measure_)
            ROS_INFO("[fiducials] compute_sharpness_measure: true");
        else
            ROS_INFO("[fiducials] compute_sharpness_measure: false");
        if (node_handle_.getParam("sharpness_calibration_parameter_m", sharpness_calibration_parameter_m_) == false)
		{
        	if (compute_sharpness_measure_ == true)
        	{
				ROS_ERROR("[fiducials] 'sharpness_calibration_parameter_m=<double>' not specified in yaml file");
				return false;
        	}
        	else
        		sharpness_calibration_parameter_m_ = 0.;
		}
		ROS_INFO("[fiducials] sharpness_calibration_parameter_m: %f", sharpness_calibration_parameter_m_);
		if (node_handle_.getParam("sharpness_calibration_parameter_n", sharpness_calibration_parameter_n_) == false)
		{
			if (compute_sharpness_measure_ == true)
			{
				ROS_ERROR("[fiducials] 'sharpness_calibration_parameter_n=<double>' not specified in yaml file");
				return false;
			}
			else
				sharpness_calibration_parameter_n_ = 0.;
		}
		ROS_INFO("[fiducials] sharpness_calibration_parameter_n: %f", sharpness_calibration_parameter_n_);
        if (node_handle_.getParam("log_or_calibrate_sharpness_measurements", log_or_calibrate_sharpness_measurements_) == false)
        {
        	log_or_calibrate_sharpness_measurements_ = false;
        }
		if (log_or_calibrate_sharpness_measurements_)
			ROS_INFO("[fiducials] log_or_calibrate_sharpness_measurements: true");
		else
			ROS_INFO("[fiducials] log_or_calibrate_sharpness_measurements: false");
        if (node_handle_.getParam("publish_marker_array", publish_marker_array_) == false)
        {
            ROS_ERROR("[fiducials] 'publish_marker_array=[true/false]' not specified in yaml file");
            return false;
        }
        if (publish_marker_array_)
            ROS_INFO("[fiducials] publish_marker_array: true");
        else
            ROS_INFO("[fiducials] publish_marker_array: false");
        if (node_handle_.getParam("publish_tf", publish_tf_) == false)
        {
            ROS_ERROR("[fiducials] 'publish_tf=[true/false]' not specified in yaml file");
            return false;
        }
        if (publish_tf_)
            ROS_INFO("[fiducials] publish_tf: true");
        else
            ROS_INFO("[fiducials] publish_tf: false");
        if (node_handle_.getParam("publish_2d_image", publish_2d_image_) == false)
        {
            ROS_ERROR("[fiducials] 'publish_2d_image=[true/false]' not specified in yaml file");
            return false;
        }
        if (publish_2d_image_)
            ROS_INFO("[fiducials] publish_2d_image: true");
        else
            ROS_INFO("[fiducials] publish_2d_image: false");
        if (node_handle_.getParam("debug_verbosity", debug_verbosity_) == false)
        {
            ROS_ERROR("[fiducials] 'debug_verbosity=[1,2]' not specified in yaml file");
            return false;
        }
        ROS_INFO("[fiducials] debug_verbosity: %i", debug_verbosity_);
        //if (node_handle_.getParam("StereoPreFilterCap", StereoPreFilterCap_) == false)
        //{
        //	ROS_ERROR("[sensor_fusion] StereoPreFilterCap for sensor fusion node not specified");
        //	return false;
        //}
        //launch_reconfigure_config_.preFilterCap = StereoPreFilterCap_;

        return ipa_Utils::RET_OK;
    }
};

std::string CobFiducialsNode::color_image_encoding_ = "bgr8";
}; // END namepsace
//1: The namespace in which the Triangle plugin will live. Typically, we use the name of the package
// that contains the library that Triangle is a part of. In this case, that's pluginlib_tutorials
// which is the name of the package we created in step one of this tutorial.
//2: The name we wish to give to the plugin.... we'll call ours regular_triangle.
//3: The fully-qualified type of the plugin class, in this case, polygon_plugins::Triangle.
//4: The fully-qualified type of the base class, in this case, polygon_base::RegularPolygon
// plugin/nodelet namespace, plugin/nodelet name, qualified class name, qualified nodelete class name
//PLUGINLIB_DECLARE_CLASS(cob_sensor_fusion, cob_sensor_fusion_nodelet, ipa_SensorFusion::CobSensorFusionNode, nodelet::Nodelet);

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
    /// initialize ROS, specify name of node
    ros::init(argc, argv, "sensor_fusion");

    /// Create a handle for this node, initialize node
    ros::NodeHandle nh;

    /// Create camera node class instance
    ipa_Fiducials::CobFiducialsNode fiducials_node(nh);

    ros::MultiThreadedSpinner spinner(2); // Use 4 threads
    spinner.spin();
    //	ros::spin();

    return 0;
}
