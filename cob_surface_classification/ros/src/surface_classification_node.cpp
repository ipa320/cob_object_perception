/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2013 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: cob_object_perception
 * \note
 * ROS package name: cob_surface_classification
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 22.04.2013
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

/*switches for execution of processing steps*/

#define DATA_SOURCE					2			// 0=from camera, 1=from camera but only publishing on demand, 2=from file
#define DATA_NUMBER_FILES			100			// number of input files if loaded from file
#define RECORD_MODE					false		// save color image and cloud for usage in EVALUATION_OFFLINE_MODE
#define COMPUTATION_MODE			true		// computations without record
#define EVALUATION_OFFLINE_MODE		false		// evaluation of stored pointcloud and image
#define EVALUATION_ONLINE_MODE		true		// computations plus evaluation of current computations plus record of evaluation

//steps in computation/evaluation_online mode:

#define NORMAL_COMP					true	// compute the normals with cross product + edges
#define ALTERNATIVE_NORMAL_COMP		false	// compute the normals without edges (classic procedure)
#define SEG 						false 	// segmentation + refinement
#define SEG_WITHOUT_EDGES 			false 	// segmentation without considering edge image (wie Steffen)
#define SEG_REFINE					false 	// segmentation refinement according to curvatures (outdated)
#define CLASSIFY 					false	// classification
#define SIMPLE_OBJECT_CLASSIFICATION false	// simple object classification and localization (for symmetric simple objects made of one cluster)


#define EDGE_VIS					false	// visualization of edges
#define NORMAL_VIS 					false 	// visualisation of normals
#define SEG_VIS 					false 	// visualisation of segmentation
#define SEG_WITHOUT_EDGES_VIS 		false 	// visualisation of segmentation without edge image
#define CLASS_VIS 					false 	// visualisation of classification

#define PUBLISH_SEGMENTATION		false	//publish segmented point cloud on topic

#include <random>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cob_surface_classification/SegmentedPointCloud2.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// boost
#include <boost/bind.hpp>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d_omp.h>


//internal includes
#include <cob_surface_classification/edge_detection.h>
//#include <cob_surface_classification/surface_classification.h>
//#include <cob_surface_classification/organized_normal_estimation.h>
#include <cob_surface_classification/refine_segmentation.h>

//package includes
#include <cob_3d_segmentation/depth_segmentation.h>
#include <cob_3d_segmentation/cluster_classifier.h>
#include <cob_3d_mapping_common/point_types.h>
#include <cob_3d_features/organized_normal_estimation_omp.h>
#include <cob_3d_features/organized_normal_estimation_edge_omp.h>


//records
#include "cob_surface_classification/scene_recording.h"
//evaluation
#include "cob_surface_classification/evaluation.h"

#include "cob_surface_classification/simple_object_classification.h"
#include "cob_object_detection_msgs/DetectionArray.h"

int global_imagecount;

class SurfaceClassificationNode
{
public:
	typedef cob_3d_segmentation::PredefinedSegmentationTypes ST;

	struct NormalEstimationConfig
	{
		int cross_pixel_radius;
		int cross_pixel_steps;
		int cross_circle_steps;
		typedef pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal>::NormalEstimationMethod IntegralNormalEstimationMethod;
		IntegralNormalEstimationMethod integral_normal_estimation_method;
		float integral_normal_smoothing_size;
		int vanillapcl_kneighbors;

		NormalEstimationConfig()
		{
			cross_pixel_radius = 4;
			cross_pixel_steps = 2;
			cross_circle_steps = 2;
			integral_normal_estimation_method = IntegralNormalEstimationMethod::COVARIANCE_MATRIX;
			integral_normal_smoothing_size = 10.f;
			vanillapcl_kneighbors = 64;
		}

		NormalEstimationConfig(const int cross_pixel_radius_, const int cross_pixel_steps_, const int cross_circle_steps_, const IntegralNormalEstimationMethod integral_normal_estimation_method_, const float integral_normal_smoothing_size_, const int vanillapcl_kneighbors_)
		{
			cross_pixel_radius = cross_pixel_radius_;
			cross_pixel_steps = cross_pixel_steps_;
			cross_circle_steps = cross_circle_steps_;
			integral_normal_estimation_method = integral_normal_estimation_method_;
			integral_normal_smoothing_size = integral_normal_smoothing_size_;
			vanillapcl_kneighbors = vanillapcl_kneighbors_;
		}
	};

	struct ExperimentConfig
	{
		EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig edge_detection_config;
		NormalEstimationConfig normal_estimation_config;
		double simulated_sensor_noise_sigma;

		ExperimentConfig()
		{
			edge_detection_config = EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig();
			normal_estimation_config = NormalEstimationConfig();
			simulated_sensor_noise_sigma = 0.;
		}

		ExperimentConfig(const EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig& edge_detection_config_, const NormalEstimationConfig& normal_estimation_config_, const double simulated_sensor_noise_sigma_)
		{
			edge_detection_config = edge_detection_config_;
			normal_estimation_config = normal_estimation_config_;
			simulated_sensor_noise_sigma = simulated_sensor_noise_sigma_;
		}
	};

	SurfaceClassificationNode(ros::NodeHandle nh)
	: node_handle_(nh)
	{
		runtime_total_ = 0.;
		runtime_depth_image_ = 0.;
		runtime_sobel_ = 0.;	// image derivatives
		runtime_edge_ = 0.;
		runtime_visibility_ = 0.;
		runtime_normal_original_ = 0.;
		runtime_normal_edge_ = 0.;
		number_processed_images_ = 0;

		rec_.setImageRecordCounter(DATA_NUMBER_FILES+1);

		it_ = 0;
		sync_input_ = 0;

		segmented_pointcloud_pub_ = node_handle_.advertise<cob_surface_classification::SegmentedPointCloud2>("segmented_pointcloud", 1);

		if (DATA_SOURCE == 0)
		{
			// from camera with broadcast
			it_ = new image_transport::ImageTransport(node_handle_);
			colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
			pointcloud_sub_.subscribe(node_handle_, "pointcloud_in", 1);

			sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >(30);
			sync_input_->connectInput(colorimage_sub_, pointcloud_sub_);
			sync_input_->registerCallback(boost::bind(&SurfaceClassificationNode::inputCallback, this, _1, _2));
		}
		else if (DATA_SOURCE == 1)
		{
			// from camera but publishing only on demand
			sub_counter_ = 0;
			it_ = new image_transport::ImageTransport(node_handle_);
			sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >(30);
			object_detection_pub_ = node_handle_.advertise<cob_object_detection_msgs::DetectionArray>("object_detections", 1, boost::bind(&SurfaceClassificationNode::connectCallback, this), boost::bind(&SurfaceClassificationNode::disconnectCallback, this));
		}
		else if (DATA_SOURCE == 2)
		{
			// from stored files
			const int image_number = DATA_NUMBER_FILES;
			std::vector<cv::Mat> image_vector(image_number);
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointcloud_vector(image_number);
			for (int i=1; i<=image_number; ++i)
			{
				std::stringstream ss_image, ss_cloud;
				ss_image << (i<1000 ? "0" : "") << (i<100 ? "0" : "") << (i<10 ? "0" : "") << i << "color";
				rec_.loadImage(image_vector[i-1], ss_image.str());
				ss_cloud << (i<1000 ? "0" : "") << (i<100 ? "0" : "") << (i<10 ? "0" : "") << i << "cloud";
				pointcloud_vector[i-1] = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
				rec_.loadCloud(pointcloud_vector[i-1], ss_cloud.str());
			}

			// do computations

			// edge evaluation
			std::vector<double> noise_sigmas;
			noise_sigmas.push_back(0.);	noise_sigmas.push_back(0.0005);	noise_sigmas.push_back(0.001); noise_sigmas.push_back(0.002);	noise_sigmas.push_back(0.005);
//			std::vector<EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig::NoiseReductionMode> noise_mode;
//			noise_mode.push_back(EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig::NONE);	noise_mode.push_back(EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig::GAUSSIAN);	noise_mode.push_back(EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig::BILATERAL);
//			std::vector<bool> use_adaptive_scan_line;
//			use_adaptive_scan_line.push_back(true);	use_adaptive_scan_line.push_back(false);
//			std::vector<int> scan_line_width_at_2m;
//			scan_line_width_at_2m.push_back(10); scan_line_width_at_2m.push_back(15); scan_line_width_at_2m.push_back(20);
//			std::vector<double> min_detectable_edge_angle;
//			min_detectable_edge_angle.push_back(35); min_detectable_edge_angle.push_back(45); min_detectable_edge_angle.push_back(60);
//			for (size_t i_noise_sigma=0; i_noise_sigma<noise_sigmas.size(); ++i_noise_sigma)
//			{
//				for (size_t i_noise_mode=0; i_noise_mode<noise_mode.size(); ++i_noise_mode)
//				{
//					for (int i_noise_kernel_size=3; (i_noise_kernel_size<=7 && i_noise_mode>0) || (i_noise_mode==0 && i_noise_kernel_size==3); i_noise_kernel_size+=2)
//					{
//						for (size_t i_adaptive_scan_line=0; i_adaptive_scan_line<use_adaptive_scan_line.size(); ++i_adaptive_scan_line)
//						{
//							for (size_t i_scan_line_width_at_2m=0; i_scan_line_width_at_2m<scan_line_width_at_2m.size(); ++i_scan_line_width_at_2m)
//							{
//								for (size_t i_min_detectable_edge_angle=0; i_min_detectable_edge_angle<min_detectable_edge_angle.size(); ++i_min_detectable_edge_angle)
//								{
//									EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig edge_detection_config(noise_mode[i_noise_mode], i_noise_kernel_size, use_adaptive_scan_line[i_adaptive_scan_line], min_detectable_edge_angle[i_min_detectable_edge_angle], scan_line_width_at_2m[i_scan_line_width_at_2m]);
//									ExperimentConfig exp_config(edge_detection_config, noise_sigmas[i_noise_sigma]);
//									std::cout << "---------------------------------------------------------------"
//											<< "\nsimulated_sensor_noise_sigma:\t" << exp_config.simulated_sensor_noise_sigma
//											<< "\nedge_detection_config.noise_reduction_mode:\t" << exp_config.edge_detection_config.noise_reduction_mode
//											<< "\nedge_detection_config.noise_reduction_kernel_size:\t" << exp_config.edge_detection_config.noise_reduction_kernel_size
//											<< "\nedge_detection_config.use_adaptive_scan_line:\t" << exp_config.edge_detection_config.use_adaptive_scan_line
//											<< "\nedge_detection_config.scan_line_width_at_2m:\t" << exp_config.edge_detection_config.scan_line_width_at_2m
//											<< "\nedge_detection_config.min_detectable_edge_angle:\t" << exp_config.edge_detection_config.min_detectable_edge_angle
//											<< std::endl;
//									computationsEvaluation(image_vector, pointcloud_vector, exp_config);
//								}
//							}
//						}
//					}
//				}
//			}

//			// normal evaluation with normals directly from edge image
//			std::vector<int> scan_line_width_at_2m;
//			scan_line_width_at_2m.push_back(10); scan_line_width_at_2m.push_back(15); scan_line_width_at_2m.push_back(20);
//			std::vector<double> min_detectable_edge_angle;
//			min_detectable_edge_angle.push_back(35); min_detectable_edge_angle.push_back(45); min_detectable_edge_angle.push_back(60);
//			for (size_t i_noise_sigma=0; i_noise_sigma<noise_sigmas.size(); ++i_noise_sigma)
//			{
//				for (size_t i_scan_line_width_at_2m=0; i_scan_line_width_at_2m<scan_line_width_at_2m.size(); ++i_scan_line_width_at_2m)
//				{
//					for (size_t i_min_detectable_edge_angle=0; i_min_detectable_edge_angle<min_detectable_edge_angle.size(); ++i_min_detectable_edge_angle)
//					{
//						EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig edge_detection_config(EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig::GAUSSIAN, 3, true, min_detectable_edge_angle[i_min_detectable_edge_angle], scan_line_width_at_2m[i_scan_line_width_at_2m]);
//						if (noise_sigmas[i_noise_sigma] == 0.)
//							edge_detection_config.noise_reduction_mode = EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig::NONE;
//						NormalEstimationConfig normal_estimation_config(0, 0, 0, NormalEstimationConfig::IntegralNormalEstimationMethod::COVARIANCE_MATRIX, 0, 0);
//						ExperimentConfig exp_config(edge_detection_config, normal_estimation_config, noise_sigmas[i_noise_sigma]);
//						std::cout << "---------------------------------------------------------------"
//								<< "\nsimulated_sensor_noise_sigma:\t" << exp_config.simulated_sensor_noise_sigma
//								<< "\nedge_detection_config.noise_reduction_mode:\t" << exp_config.edge_detection_config.noise_reduction_mode
//								<< "\nedge_detection_config.noise_reduction_kernel_size:\t" << exp_config.edge_detection_config.noise_reduction_kernel_size
//								<< "\nedge_detection_config.use_adaptive_scan_line:\t" << exp_config.edge_detection_config.use_adaptive_scan_line
//								<< "\nedge_detection_config.scan_line_width_at_2m:\t" << exp_config.edge_detection_config.scan_line_width_at_2m
//								<< "\nedge_detection_config.min_detectable_edge_angle:\t" << exp_config.edge_detection_config.min_detectable_edge_angle
//								<< std::endl;
//						computationsEvaluation(image_vector, pointcloud_vector, exp_config);
//					}
//				}
//			}

			// normal evaluation with cross-product
			for (size_t i_noise_sigma=0; i_noise_sigma<noise_sigmas.size(); ++i_noise_sigma)
			{
				for (int i_pixel_radius=2; i_pixel_radius<=8; i_pixel_radius+=2)
				{
					for (int i_pixel_step=1; i_pixel_step<=2; ++i_pixel_step)
					{
						for (int i_circle_step=1; i_circle_step<=2; ++i_circle_step)
						{
							EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig edge_detection_config;
							if (noise_sigmas[i_noise_sigma] == 0.)
								edge_detection_config.noise_reduction_mode = EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig::NONE;
							NormalEstimationConfig normal_estimation_config(i_pixel_radius, i_pixel_step, i_circle_step, NormalEstimationConfig::IntegralNormalEstimationMethod::COVARIANCE_MATRIX, 0, 0);
							ExperimentConfig exp_config(edge_detection_config, normal_estimation_config, noise_sigmas[i_noise_sigma]);
							std::cout << "---------------------------------------------------------------"
									<< "\nsimulated_sensor_noise_sigma:\t" << exp_config.simulated_sensor_noise_sigma
									<< "\nedge_detection_config.noise_reduction_mode:\t" << exp_config.edge_detection_config.noise_reduction_mode
									<< "\nedge_detection_config.noise_reduction_kernel_size:\t" << exp_config.edge_detection_config.noise_reduction_kernel_size
									<< "\nedge_detection_config.use_adaptive_scan_line:\t" << exp_config.edge_detection_config.use_adaptive_scan_line
									<< "\nedge_detection_config.scan_line_width_at_2m:\t" << exp_config.edge_detection_config.scan_line_width_at_2m
									<< "\nedge_detection_config.min_detectable_edge_angle:\t" << exp_config.edge_detection_config.min_detectable_edge_angle
									<< "\nnormal_estimation_config.cross_pixel_radius:\t" << exp_config.normal_estimation_config.cross_pixel_radius
									<< "\nnormal_estimation_config.cross_pixel_steps:\t" << exp_config.normal_estimation_config.cross_pixel_steps
									<< "\nnormal_estimation_config.cross_circle_steps:\t" << exp_config.normal_estimation_config.cross_circle_steps
									<< std::endl;
							computationsEvaluation(image_vector, pointcloud_vector, exp_config);
						}
					}
				}
			}

//			// normal evaluation with integral image
//			std::vector<NormalEstimationConfig::IntegralNormalEstimationMethod> integral_method;
//			integral_method.push_back(NormalEstimationConfig::IntegralNormalEstimationMethod::COVARIANCE_MATRIX); integral_method.push_back(NormalEstimationConfig::IntegralNormalEstimationMethod::AVERAGE_3D_GRADIENT); integral_method.push_back(NormalEstimationConfig::IntegralNormalEstimationMethod::AVERAGE_DEPTH_CHANGE);
//			std::vector<float> smoothing;
//			smoothing.push_back(2.5f); smoothing.push_back(5.f); smoothing.push_back(7.5f); smoothing.push_back(10.f); smoothing.push_back(12.5f); smoothing.push_back(15.f); smoothing.push_back(17.5f); smoothing.push_back(20.f);
//			for (size_t i_noise_sigma=0; i_noise_sigma<noise_sigmas.size(); ++i_noise_sigma)
//			{
//				for (size_t i_method=0; i_method<integral_method.size(); ++i_method)
//				{
//					for (size_t i_smoothing=0; i_smoothing<smoothing.size(); ++i_smoothing)
//					{
//						EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig edge_detection_config;
//						if (noise_sigmas[i_noise_sigma] == 0.)
//							edge_detection_config.noise_reduction_mode = EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig::NONE;
//						NormalEstimationConfig normal_estimation_config(0, 0, 0, integral_method[i_method], smoothing[i_smoothing], 0);
//						ExperimentConfig exp_config(edge_detection_config, normal_estimation_config, noise_sigmas[i_noise_sigma]);
//						std::cout << "---------------------------------------------------------------"
//								<< "\nsimulated_sensor_noise_sigma:\t" << exp_config.simulated_sensor_noise_sigma
//								<< "\nedge_detection_config.noise_reduction_mode:\t" << exp_config.edge_detection_config.noise_reduction_mode
//								<< "\nedge_detection_config.noise_reduction_kernel_size:\t" << exp_config.edge_detection_config.noise_reduction_kernel_size
//								<< "\nedge_detection_config.use_adaptive_scan_line:\t" << exp_config.edge_detection_config.use_adaptive_scan_line
//								<< "\nedge_detection_config.scan_line_width_at_2m:\t" << exp_config.edge_detection_config.scan_line_width_at_2m
//								<< "\nedge_detection_config.min_detectable_edge_angle:\t" << exp_config.edge_detection_config.min_detectable_edge_angle
//								<< "\nnormal_estimation_config.integral_normal_estimation_method:\t" << exp_config.normal_estimation_config.integral_normal_estimation_method
//								<< "\nnormal_estimation_config.integral_normal_smoothing_size:\t" << exp_config.normal_estimation_config.integral_normal_smoothing_size
//								<< std::endl;
//						computationsEvaluation(image_vector, pointcloud_vector, exp_config);
//					}
//				}
//			}

//			// vanilla normal evaluation with kNN (PCL)
//			std::vector<int> kneighbors;
//			kneighbors.push_back(8); kneighbors.push_back(16); kneighbors.push_back(32); kneighbors.push_back(64); kneighbors.push_back(128); kneighbors.push_back(256); kneighbors.push_back(512);
//			for (size_t i_noise_sigma=0; i_noise_sigma<noise_sigmas.size(); ++i_noise_sigma)
//			{
//				for (size_t i_kneighbors=0; i_kneighbors<kneighbors.size(); ++i_kneighbors)
//				{
//					EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig edge_detection_config;
//					if (noise_sigmas[i_noise_sigma] == 0.)
//						edge_detection_config.noise_reduction_mode = EdgeDetection<pcl::PointXYZRGB>::EdgeDetectionConfig::NONE;
//					NormalEstimationConfig normal_estimation_config(0, 0, 0, NormalEstimationConfig::IntegralNormalEstimationMethod::COVARIANCE_MATRIX, 0, kneighbors[i_kneighbors]);
//					ExperimentConfig exp_config(edge_detection_config, normal_estimation_config, noise_sigmas[i_noise_sigma]);
//					std::cout << "---------------------------------------------------------------"
//							<< "\nsimulated_sensor_noise_sigma:\t" << exp_config.simulated_sensor_noise_sigma
//							<< "\nnormal_estimation_config.vanillapcl_kneighbors:\t" << exp_config.normal_estimation_config.vanillapcl_kneighbors
//							<< std::endl;
//					computationsEvaluation(image_vector, pointcloud_vector, exp_config);
//				}
//			}

			exit(0);
		}
	}

	~SurfaceClassificationNode()
	{
		if (it_ != 0)
			delete it_;
		if (sync_input_ != 0)
			delete sync_input_;
	}


	// Converts a color image message to cv::Mat format.
	void convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
	{
		try
		{
			image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
		}
		image = image_ptr->image;
	}

	void inputCallback(const sensor_msgs::Image::ConstPtr& color_image_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
	{
		// convert color image to cv::Mat
		cv_bridge::CvImageConstPtr color_image_ptr;
		cv::Mat color_image;
		convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*pointcloud_msg, *cloud);
		if(cloud->height == 1 && cloud->points.size() == 307200)
		{
			cloud->height = 480;
			cloud->width = 640;
			cloud->is_dense = true;
		}

		computations(color_image, cloud);
	}

	void computationsEvaluation(const std::vector<cv::Mat>& image_vector, const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& pointcloud_vector, const ExperimentConfig& config)
	{
		int index = 0;
		int counter = 0;
		edge_detection_statistics_.clear();
		ne_statistics_direct_edge_.clear();
		ne_statistics_cross_edge_.clear();
		ne_statistics_cross_.clear();
		ne_statistics_integral_edge_.clear();
		ne_statistics_integral_.clear();
		ne_statistics_vanilla_.clear();

		Timer tim;
		tim.start();
		const int image_number = image_vector.size();
		while (counter<image_number)			// 600 runs if runtime is to be measured
		{
			computations(image_vector[index], pointcloud_vector[index], config);
			index = (index+1)%image_number;
			++counter;
			//std:cout << "=============================================================> Finished iteration " << counter << std::endl;
		}
		std::stringstream ss;

		std::cout << "Total runtime: " << tim.getElapsedTimeInSec() << "s \t Runtime per cycle: " << tim.getElapsedTimeInMilliSec()/(double)counter << "ms\n" << std::endl;

		std::cout << "Results on edge estimation:\n\trecall=" << edge_detection_statistics_.recall << "\tprecision=" << edge_detection_statistics_.precision << "\n\n";
		ss << "simulated_sensor_noise_sigma:\t" << config.simulated_sensor_noise_sigma
				<< "\tedge_detection_config.noise_reduction_mode:\t" << config.edge_detection_config.noise_reduction_mode
				<< "\tedge_detection_config.noise_reduction_kernel_size:\t" << config.edge_detection_config.noise_reduction_kernel_size
				<< "\tedge_detection_config.use_adaptive_scan_line:\t" << config.edge_detection_config.use_adaptive_scan_line
				<< "\tedge_detection_config.scan_line_width_at_2m:\t" << config.edge_detection_config.scan_line_width_at_2m
				<< "\tedge_detection_config.min_detectable_edge_angle:\t" << config.edge_detection_config.min_detectable_edge_angle
				<< "\tedge.recall:\t" << edge_detection_statistics_.recall
				<< "\tedge.precision:\t" << edge_detection_statistics_.precision;
//				<< std::endl;
//		std::cout << "Direct normals from edge computation:"
//				<< "\nCoverage of estimated normals on gt_normals: " << ne_statistics_direct_edge_.coverage_gt_normals
//				<< "\nPercentage of good normals: " << ne_statistics_direct_edge_.percentage_good_normals
//				<< "\nAverage normal estimation error: " << ne_statistics_direct_edge_.average_angular_error
//				<< "\nAverage normal estimation error [deg]: " << ne_statistics_direct_edge_.average_angular_error_deg << "\n" << std::endl;
//		ss << "\tne_statistics_cross_edge_.coverage_gt_normals:\t" << ne_statistics_direct_edge_.coverage_gt_normals
//				<< "\tne_statistics_cross_edge_.percentage_good_normals:\t" << ne_statistics_direct_edge_.percentage_good_normals
//				<< "\tne_statistics_cross_edge_.average_angular_error:\t" << ne_statistics_direct_edge_.average_angular_error
//				<< "\tne_statistics_cross_edge_.average_angular_error_deg:\t" << ne_statistics_direct_edge_.average_angular_error_deg
//				<< std::endl;


		if (NORMAL_COMP)
		{
			std::cout << "Cross-product-based normals with edges:"
					<< "\nCoverage of estimated normals on gt_normals: " << ne_statistics_cross_edge_.coverage_gt_normals
					<< "\nAverage normal estimation error: " << ne_statistics_cross_edge_.average_angular_error
					<< "\nAverage normal estimation error [deg]: " << ne_statistics_cross_edge_.average_angular_error_deg
					<< "\nPercentage of good normals: " << ne_statistics_cross_edge_.percentage_good_normals << "\n" << std::endl;
			ss //<< "simulated_sensor_noise_sigma:\t" << config.simulated_sensor_noise_sigma
					<< "\tnormal_estimation_config.cross_pixel_radius:\t" << config.normal_estimation_config.cross_pixel_radius
					<< "\tnormal_estimation_config.cross_pixel_steps:\t" << config.normal_estimation_config.cross_pixel_steps
					<< "\tnormal_estimation_config.cross_circle_steps:\t" << config.normal_estimation_config.cross_circle_steps
					<< "\tne_statistics_cross_edge_.coverage_gt_normals:\t" << ne_statistics_cross_edge_.coverage_gt_normals
					<< "\tne_statistics_cross_edge_.percentage_good_normals:\t" << ne_statistics_cross_edge_.percentage_good_normals
					<< "\tne_statistics_cross_edge_.average_angular_error:\t" << ne_statistics_cross_edge_.average_angular_error
					<< "\tne_statistics_cross_edge_.average_angular_error_deg:\t" << ne_statistics_cross_edge_.average_angular_error_deg
					<< std::endl;
		}
		if (ALTERNATIVE_NORMAL_COMP)
		{
//			std::cout << "Cross-product-based normals:"
//					<< "\nCoverage of estimated normals on gt_normals: " << ne_statistics_cross_.coverage_gt_normals
//					<< "\nAverage normal estimation error: " << ne_statistics_cross_.average_angular_error
//					<< "\nAverage normal estimation error [deg]: " << ne_statistics_cross_.average_angular_error_deg
//					<< "\nPercentage of good normals: " << ne_statistics_cross_.percentage_good_normals << "\n" << std::endl;
//			ss << "\tnormal_estimation_config.cross_pixel_radius:\t" << config.normal_estimation_config.cross_pixel_radius
//					<< "\tnormal_estimation_config.cross_pixel_steps:\t" << config.normal_estimation_config.cross_pixel_steps
//					<< "\tnormal_estimation_config.cross_circle_steps:\t" << config.normal_estimation_config.cross_circle_steps
//					<< "\tne_statistics_cross_.coverage_gt_normals:\t" << ne_statistics_cross_.coverage_gt_normals
//					<< "\tne_statistics_cross_.percentage_good_normals:\t" << ne_statistics_cross_.percentage_good_normals
//					<< "\tne_statistics_cross_.average_angular_error:\t" << ne_statistics_cross_.average_angular_error
//					<< "\tne_statistics_cross_.average_angular_error_deg:\t" << ne_statistics_cross_.average_angular_error_deg
//					<< std::endl;

//			std::cout << "Integral image-based normals with edges:"
//					<< "\nCoverage of estimated normals on gt_normals: " << ne_statistics_integral_edge_.coverage_gt_normals
//					<< "\nAverage normal estimation error: " << ne_statistics_integral_edge_.average_angular_error
//					<< "\nAverage normal estimation error [deg]: " << ne_statistics_integral_edge_.average_angular_error_deg
//					<< "\nPercentage of good normals: " << ne_statistics_integral_edge_.percentage_good_normals << "\n" << std::endl;
//			ss << "\tnormal_estimation_config.integral_normal_estimation_method:\t" << config.normal_estimation_config.integral_normal_estimation_method
//					<< "\tnormal_estimation_config.integral_normal_smoothing_size:\t" << config.normal_estimation_config.integral_normal_smoothing_size
//					<< "\tne_statistics_integral_edge_.coverage_gt_normals:\t" << ne_statistics_integral_edge_.coverage_gt_normals
//					<< "\tne_statistics_integral_edge_.percentage_good_normals:\t" << ne_statistics_integral_edge_.percentage_good_normals
//					<< "\tne_statistics_integral_edge_.average_angular_error:\t" << ne_statistics_integral_edge_.average_angular_error
//					<< "\tne_statistics_integral_edge_.average_angular_error_deg:\t" << ne_statistics_integral_edge_.average_angular_error_deg
//					<< std::endl;

//			std::cout << "Integral image-based normals:"
//					<< "\nCoverage of estimated normals on gt_normals: " << ne_statistics_integral_.coverage_gt_normals
//					<< "\nAverage normal estimation error: " << ne_statistics_integral_.average_angular_error
//					<< "\nAverage normal estimation error [deg]: " << ne_statistics_integral_.average_angular_error_deg
//					<< "\nPercentage of good normals: " << ne_statistics_integral_.percentage_good_normals << "\n" << std::endl;
//			ss << "\tnormal_estimation_config.integral_normal_estimation_method:\t" << config.normal_estimation_config.integral_normal_estimation_method
//					<< "\tnormal_estimation_config.integral_normal_smoothing_size:\t" << config.normal_estimation_config.integral_normal_smoothing_size
//					<< "\tne_statistics_integral_.coverage_gt_normals:\t" << ne_statistics_integral_.coverage_gt_normals
//					<< "\tne_statistics_integral_.percentage_good_normals:\t" << ne_statistics_integral_.percentage_good_normals
//					<< "\tne_statistics_integral_.average_angular_error:\t" << ne_statistics_integral_.average_angular_error
//					<< "\tne_statistics_integral_.average_angular_error_deg:\t" << ne_statistics_integral_.average_angular_error_deg
//					<< std::endl;

//			std::cout << "Vanilla normal estimation:"
//					<< "\nCoverage of estimated normals on gt_normals: " << ne_statistics_vanilla_.coverage_gt_normals
//					<< "\nPercentage of good normals: " << ne_statistics_vanilla_.percentage_good_normals
//					<< "\nAverage normal estimation error: " << ne_statistics_vanilla_.average_angular_error
//					<< "\nAverage normal estimation error [deg]: " << ne_statistics_vanilla_.average_angular_error_deg << "\n" << std::endl;
//			ss << "\tnormal_estimation_config.vanillapcl_kneighbors:\t" << config.normal_estimation_config.vanillapcl_kneighbors
//					<< "\tne_statistics_vanilla_.coverage_gt_normals:\t" << ne_statistics_vanilla_.coverage_gt_normals
//					<< "\tne_statistics_vanilla_.percentage_good_normals:\t" << ne_statistics_vanilla_.percentage_good_normals
//					<< "\tne_statistics_vanilla_.average_angular_error:\t" << ne_statistics_vanilla_.average_angular_error
//					<< "\tne_statistics_vanilla_.average_angular_error_deg:\t" << ne_statistics_vanilla_.average_angular_error_deg
//					<< std::endl;
		}

		// write results to file
		std::ofstream file("surface_classification_results.txt", std::ios::out | std::ios::app);
		if (file.is_open())
		{
			file << ss.str();
		}
		file.close();
	}

	void computations(const cv::Mat& color_image, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud, const ExperimentConfig& config = ExperimentConfig())
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

		// add noise
		if (config.simulated_sensor_noise_sigma > 0.)
		{
			cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud(*point_cloud, *cloud);
			std::default_random_engine generator;
			std::normal_distribution<double> distribution(1.0, config.simulated_sensor_noise_sigma);		// Kinect: ca. 0.002,  realistic maximum: 0.005
			for (int i=0; i<cloud->size(); ++i)
			{
				if (pcl_isnan(cloud->points[i].z) == false)
				{
					double random_factor = distribution(generator);
					cloud->points[i].x *= random_factor;
					cloud->points[i].y *= random_factor;
					cloud->points[i].z *= random_factor;
				}
			}
		}
		else
			cloud = point_cloud;

		/*
		//Load Pointcloud for Evaluation
		bool loadpointcloud = false;
		sensor_msgs::PointCloud2 cloud_blob;

		std::vector<int> image_indices;
		image_indices.push_back(16);	//!
		image_indices.push_back(17);
		image_indices.push_back(19);
		image_indices.push_back(21);
		image_indices.push_back(27);
		image_indices.push_back(29);
		image_indices.push_back(33);
		image_indices.push_back(34);	//!
		image_indices.push_back(35);
		image_indices.push_back(36);
		image_indices.push_back(37);
		image_indices.push_back(38);
		image_indices.push_back(39);
		image_indices.push_back(40);
		image_indices.push_back(43);
		image_indices.push_back(44);
		image_indices.push_back(45);
		image_indices.push_back(46);
		image_indices.push_back(47);
		image_indices.push_back(48);
		image_indices.push_back(49);
		image_indices.push_back(50);
		image_indices.push_back(51);	//!

		std::ostringstream outStream;
		outStream << image_indices[global_imagecount];
		std::string num;
		num  = outStream.str();

//		std::ostringstream outStream2;
//		if(global_imagecount==1)
//		{
//			outStream2 << global_imagecount;
//		}else{
//			outStream2 << global_imagecount-1;
//		}
//		std::string num2;
//		num2  = outStream2.str();
//		std::string num = "34";
//		std::string num2 = "34";


//		std::string pcd = "/home/rmb-dh/evaluation/Final2/ev"+ num + ".pcd";
//		std::string jpg = "/home/rmb-dh/evaluation/Final2/ev"+ num +".jpg";
////		std::string pcd = "/home/rmb-dh/evaluation/test.pcd";
////		std::string jpg = "/home/rmb-dh/evaluation/test.jpg";
//		std::string segmented = "/home/rmb-dh/evaluation/Segmented/Segmented"+ num2 +".jpg";
		std::string pcd = "/home/rbormann/git/care-o-bot/cob_object_perception/cob_texture_categorization/common/files/scenes_database/Table"+ num + ".pcd";
		std::string jpg = "/home/rbormann/git/care-o-bot/cob_object_perception/cob_texture_categorization/common/files/scenes_database/Table"+ num +".jpg";
		std::cout<< pcd <<" Used PCD File   "<<jpg<<" Used JPG File"<<std::endl;

		--global_imagecount;
		if(global_imagecount>=(int)image_indices.size())
			global_imagecount=0;
		if(global_imagecount<0)
			global_imagecount=(int)image_indices.size()-1;

//		cv::imshow("imagebefore", cv::imread(segmented,1));


		if(loadpointcloud)
		{
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (pcd, *cloud) == -1) //* load the file
			{
			PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
			}
			io::loadPCDFile (pcd, cloud_blob);
			color_image = cv::imread(jpg, 1);
			std::cout << "Loaded "
					<< cloud->width * cloud->height
					<< " data points from test_pcd.pcd with the following fields: "
					<< std::endl;
		}
*/


		int key = 0;
//		cv::imshow("image", color_image);
////		if(!EVALUATION_ONLINE_MODE)
////			cv::waitKey(10);
////		if(EVALUATION_ONLINE_MODE)
//			key = cv::waitKey(10);

		//record scene
		//----------------------------------------
		if(RECORD_MODE)
		{
			//cv::Mat im_flipped;
			//cv::flip(color_image, im_flipped,-1);
			//cv::imshow("image", im_flipped);
			cv::imshow("image", color_image);
			int key = cv::waitKey(50);
			if(key == 'r') //record if "r" is pressed while "image"-window is activated
			{
				rec_.saveImage(color_image,"color");
				rec_.saveCloud(point_cloud,"cloud");
				std::cout << "Saved images with index: " << rec_.getImageRecordCounter()-1 << std::endl;
			}
			else if (key == 'm')
			{
				rec_.setImageRecordCounter(rec_.getImageRecordCounter()+1);
				std::cout << "Changed image storage index to: " << rec_.getImageRecordCounter() << std::endl;
			}
			else if (key == 'n')
			{
				rec_.setImageRecordCounter(rec_.getImageRecordCounter()-1);
				std::cout << "Changed image storage index to: " << rec_.getImageRecordCounter() << std::endl;
			}
			else if (key=='q')
				exit(0);
			return;
		}
		//----------------------------------------

		Timer tim;

		//std::cout << key <<endl;
		//record if "e" is pressed while "image"-window is activated
		if(COMPUTATION_MODE || (EVALUATION_ONLINE_MODE && key == 1048677))
		{
			pcl::PointCloud<pcl::Normal>::Ptr normalsEdgeDirect = 0;//(new pcl::PointCloud<pcl::Normal>);
			cv::Mat edge;
			edge_detection_.computeDepthEdges(cloud, edge, config.edge_detection_config, normalsEdgeDirect);
			//edge_detection_.sobelLaplace(color_image,depth_image);

			// visualization on color image
			if (EDGE_VIS)
			{
				cv::Mat color_image_edge = color_image.clone();
//				cv::line(color_image, cv::Point(320-edge_detection_.getScanLineWidth(),240), cv::Point(320+edge_detection_.getScanLineWidth(),240),CV_RGB(255,0,0), 2);
//				cv::line(color_image, cv::Point(320,240-edge_detection_.getScanLineWidth()), cv::Point(320,240+edge_detection_.getScanLineWidth()),CV_RGB(255,0,0), 2);
				const cv::Vec3b green = cv::Vec3b(0, 255, 0);
				const cv::Vec3b blue = cv::Vec3b(255, 0, 0);
				for (int v=0; v<color_image_edge.rows; ++v)
					for (int u=0; u<color_image_edge.cols; ++u)
					{
						if (edge.at<uchar>(v,u) == 254)
							color_image_edge.at<cv::Vec3b>(v,u) = blue;
						if (edge.at<uchar>(v,u) == 255)
							color_image_edge.at<cv::Vec3b>(v,u) = green;
					}
				cv::imshow("color_image", color_image);
				cv::imshow("color with edge", color_image_edge);
				int quit = cv::waitKey();
				if (quit=='q')
					exit(0);
			}

			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			pcl::PointCloud<PointLabel>::Ptr labels(new pcl::PointCloud<PointLabel>);
			ST::Graph::Ptr graph(new ST::Graph);
			if (NORMAL_COMP) // || key=='n')
			{
				//tim.start();
				one_.setInputCloud(cloud);
				one_.setPixelSearchRadius(config.normal_estimation_config.cross_pixel_radius,config.normal_estimation_config.cross_pixel_steps,config.normal_estimation_config.cross_circle_steps);	// 4,2,2	//call before calling computeMaskManually()!!!
				//one_.computeMaskManually_increasing(cloud->width);
				one_.computeMaskManually(cloud->width);
				one_.computePointAngleLookupTable(8);
				one_.setEdgeImage(edge);
				one_.setOutputLabels(labels);
				//one_.setSameDirectionThres(0.94);
				one_.setSkipDistantPointThreshold(8);	//don't consider points in neighborhood with depth distance larger than 8
				one_.compute(*normals);
				//std::cout << tim.getElapsedTimeInMilliSec() << "ms\t for cross-product normal computation with edges" << std::endl;
//				runtime_normal_edge_ += tim.getElapsedTimeInMilliSec();
//				++number_processed_images_;
				//std::cout << "runtime_normal_original: " << runtime_normal_original_/(double)number_processed_images_ <<
				//			"\n\t\t\t\truntime_normal_edge: " << runtime_normal_edge_/(double)number_processed_images_ << std::endl;
			}

//			if (key=='n')
//			{
				pcl::PointCloud<pcl::Normal>::Ptr normalsCrossProduct(new pcl::PointCloud<pcl::Normal>);
				pcl::PointCloud<pcl::Normal>::Ptr normalsIntegralImage(new pcl::PointCloud<pcl::Normal>);
				pcl::PointCloud<pcl::Normal>::Ptr normalsIntegralImageEdge(new pcl::PointCloud<pcl::Normal>);
				pcl::PointCloud<pcl::Normal>::Ptr normalsVanilla(new pcl::PointCloud<pcl::Normal>);
				pcl::PointCloud<PointLabel>::Ptr labelsWithoutEdges(new pcl::PointCloud<PointLabel>);
				ST::Graph::Ptr graphWithoutEdges(new ST::Graph);
				if (ALTERNATIVE_NORMAL_COMP)
				{
/*					//tim.start();
					oneWithoutEdges_.setInputCloud(cloud);
					oneWithoutEdges_.setPixelSearchRadius(config.normal_estimation_config.cross_pixel_radius,config.normal_estimation_config.cross_pixel_steps,config.normal_estimation_config.cross_circle_steps);	//4,2,2 //(8,1,1)   (8,2,2)
					oneWithoutEdges_.computeMaskManually(cloud->width);
					oneWithoutEdges_.setOutputLabels(labelsWithoutEdges);
					oneWithoutEdges_.setSkipDistantPointThreshold(8);	//PUnkte mit einem Abstand in der Tiefe von 8 werden nicht mehr zur Nachbarschaft gezählt
					oneWithoutEdges_.compute(*normalsCrossProduct);
					//std::cout << tim.getElapsedTimeInMilliSec() << "ms\t for cross-product normal computation\n";
					//runtime_normal_original_ += tim.getElapsedTimeInMilliSec();
					//return;

					// alternative 1: integral image based normal estimation
					// edge based option:
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_edge(new pcl::PointCloud<pcl::PointXYZRGB>);
					pcl::copyPointCloud(*cloud, *cloud_edge);
					for (int v=0; v<edge.rows; ++v)
						for (int u=0; u<edge.cols; ++u)
							if (edge.at<uchar>(v,u) != 0)
								cloud_edge->points[v*edge.cols+u].z = std::numeric_limits<float>::quiet_NaN();

					//tim.start();
					pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne2;
					ne2.setNormalEstimationMethod(config.normal_estimation_config.integral_normal_estimation_method);
					ne2.setMaxDepthChangeFactor(0.02f);
					ne2.setNormalSmoothingSize(config.normal_estimation_config.integral_normal_smoothing_size);
					ne2.setDepthDependentSmoothing(true);
					ne2.setInputCloud(cloud_edge);
					ne2.compute(*normalsIntegralImageEdge);
					//std::cout << tim.getElapsedTimeInMilliSec() << "ms\t for integral image normal estimation with edges" << std::endl;

					//tim.start();
					ne2.setInputCloud(cloud);
					ne2.compute(*normalsIntegralImage);
					//std::cout << tim.getElapsedTimeInMilliSec() << "ms\t for integral image normal estimation" << std::endl;

*/
					// alternative 2: vanilla PCL normal estimation
					//tim.start();
					pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne3;
					ne3.setInputCloud(cloud);
					ne3.setNumberOfThreads(0);
					ne3.setKSearch((int)config.normal_estimation_config.vanillapcl_kneighbors); //256
					//ne3.setRadiusSearch(0.01);
					ne3.compute(*normalsVanilla);
					//std::cout << tim.getElapsedTimeInMilliSec() << "ms\t for vanilla normal estimation" << std::endl;
				}
//			}

			if(NORMAL_VIS || key=='n')
			{
				// visualize normals
				pcl::visualization::PCLVisualizer viewerNormals("Cloud and Normals");
				viewerNormals.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbNormals(cloud);

				viewerNormals.addPointCloud<pcl::PointXYZRGB> (cloud, rgbNormals, "cloud");
				//viewerNormals.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normals,2,0.005,"normals");
				//viewerNormals.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normalsWithoutEdges,2,0.005,"normalsWithoutEdges");
				viewerNormals.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normalsEdgeDirect,2,0.005,"normalsEdgeDirect");
				viewerNormals.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

				while (!viewerNormals.wasStopped ())
				{
					viewerNormals.spinOnce();
				}
				viewerNormals.removePointCloud("cloud");
			}

			//return;

			if(SEG || key=='n')	// || EVALUATION_ONLINE_MODE
			{
				tim.start();
				seg_.setInputCloud(cloud);
				seg_.setNormalCloudIn(normals);
				seg_.setLabelCloudInOut(labels);
				seg_.setClusterGraphOut(graph);
				seg_.performInitialSegmentation();
				seg_.refineSegmentation();
				double runtime_segmentation = tim.getElapsedTimeInMilliSec();
				std::cout << "runtime segmentation: " << runtime_segmentation << std::endl;
			}
			if(SEG_WITHOUT_EDGES)
			{
				segWithoutEdges_.setInputCloud(cloud);
				segWithoutEdges_.setNormalCloudIn(normalsCrossProduct);
				segWithoutEdges_.setLabelCloudInOut(labelsWithoutEdges);
				segWithoutEdges_.setClusterGraphOut(graphWithoutEdges);
				segWithoutEdges_.performInitialSegmentation();
			}

			if(SEG_VIS || key=='n')
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented(new pcl::PointCloud<pcl::PointXYZRGB>);
				*segmented = *cloud;
				graph->clusters()->mapClusterColor(segmented);

				// visualize segmentation
				pcl::visualization::PCLVisualizer viewer("segmentation");
				viewer.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(segmented);
				viewer.addPointCloud<pcl::PointXYZRGB> (segmented,rgb,"seg");
				//viewer.setCameraPosition()
				while (!viewer.wasStopped ())
				{
					viewer.spinOnce();
				}
				viewer.removePointCloud("seg");
			}
			if(SEG_WITHOUT_EDGES_VIS)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedWithoutEdges(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::copyPointCloud<pcl::PointXYZRGB,pcl::PointXYZRGB>(*cloud, *segmentedWithoutEdges);
				graphWithoutEdges->clusters()->mapClusterColor(segmentedWithoutEdges);

				pcl::visualization::PCLVisualizer viewerWithoutEdges("segmentationWithoutEdges");

				viewerWithoutEdges.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbWithoutEdges(segmentedWithoutEdges);
				viewerWithoutEdges.addPointCloud<pcl::PointXYZRGB> (segmentedWithoutEdges,rgbWithoutEdges,"segWithoutEdges");
				while (!viewerWithoutEdges.wasStopped ())
				{
					viewerWithoutEdges.spinOnce();
				}
			}

			if(SEG_REFINE)
			{
				//merge segments with similar curvature characteristics
				segRefined_.setInputCloud(cloud);
				segRefined_.setClusterGraphInOut(graph);
				segRefined_.setLabelCloudInOut(labels);
				segRefined_.setNormalCloudIn(normals);
				//segRefined_.setCurvThres()
				segRefined_.refineUsingCurvature();
				//segRefined_.printCurvature(color_image);

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentedRef(new pcl::PointCloud<pcl::PointXYZRGB>);
				*segmentedRef = *cloud;
				graph->clusters()->mapClusterColor(segmentedRef);

				// visualize refined segmentation
				pcl::visualization::PCLVisualizer viewerRef("segmentationRef");
				viewerRef.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbRef(segmentedRef);
				viewerRef.addPointCloud<pcl::PointXYZRGB> (segmentedRef,rgbRef,"segRef");

				while (!viewerRef.wasStopped ())
				{
					viewerRef.spinOnce();
				}
				viewerRef.removePointCloud("segRef");
			}

			if (PUBLISH_SEGMENTATION)
			{
				cob_surface_classification::SegmentedPointCloud2 msg;
				pcl::toROSMsg(*cloud, msg.pointcloud);
//				if(!loadpointcloud)
//					msg.pointcloud = *pointcloud_msg;
//				else
//					msg.pointcloud = cloud_blob;
				for (ST::Graph::ClusterPtr c = graph->clusters()->begin(); c != graph->clusters()->end(); ++c)
				{
					cob_surface_classification::Int32Array point_indices;
					point_indices.array.resize(c->size());
					int i=0;
					for (ST::Graph::ClusterType::iterator it = c->begin(); it != c->end(); ++it, ++i)
						point_indices.array[i] = *it;
					msg.clusters.push_back(point_indices);
				}
				segmented_pointcloud_pub_.publish(msg);
			}

			//return;

			if(CLASSIFY)  //|| EVALUATION_ONLINE_MODE
			{
				//classification
				tim.start();
				cc_.setClusterHandler(graph->clusters());
				cc_.setNormalCloudInOut(normals);
				cc_.setLabelCloudIn(labels);
				cc_.setPointCloudIn(cloud);
				//cc_.setMaskSizeSmooth(14);
				cc_.classify();
				std::cout << "runtime classification: " << tim.getElapsedTimeInMilliSec() << std::endl;
			}
			if (SIMPLE_OBJECT_CLASSIFICATION)
			{
				cob_object_detection_msgs::DetectionArray msg;
				msg.header = pcl_conversions::fromPCL(cloud->header);
				simple_object_classification_.classifyObjects(cloud, graph, msg);
				object_detection_pub_.publish(msg);
			}
			if(CLASS_VIS)
			{

				pcl::PointCloud<pcl::PointXYZRGB>::Ptr classified(new pcl::PointCloud<pcl::PointXYZRGB>);
				*classified = *cloud;
				graph->clusters()->mapTypeColor(classified);
				graph->clusters()->mapClusterBorders(classified);

				// visualize classification
				pcl::visualization::PCLVisualizer viewerClass("classification");
				viewerClass.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbClass(classified);
				viewerClass.addPointCloud<pcl::PointXYZRGB> (classified,rgbClass,"class");

				while (!viewerClass.wasStopped ())
				{
					viewerClass.spinOnce();
				}
				viewerClass.removePointCloud("class");
			}

			if(EVALUATION_ONLINE_MODE)
			{
				// edge statistics
				eval_.evaluateEdgeRecognition(point_cloud, color_image, edge, &edge_detection_statistics_);
				//eval_.evaluateNormalEstimation(point_cloud, normalsEdgeDirect, &ne_statistics_direct_edge_);

				// normal estimation statistics
				if (NORMAL_COMP)
				{
					//std::cout << "Cross-product-based normals with edges:\n";
					eval_.evaluateNormalEstimation(point_cloud, normals, &ne_statistics_cross_edge_);
				}
				if (ALTERNATIVE_NORMAL_COMP)
				{
					//std::cout << "Cross-product-based normals:\n";
//					eval_.evaluateNormalEstimation(point_cloud, normalsCrossProduct, &ne_statistics_cross_);
					//std::cout << "Integral image-based normals with edges:\n";
//					eval_.evaluateNormalEstimation(point_cloud, normalsIntegralImageEdge, &ne_statistics_integral_edge_);
					//std::cout << "Integral image-based normals:\n";
//					eval_.evaluateNormalEstimation(point_cloud, normalsIntegralImage, &ne_statistics_integral_);
					//std::cout << "Vanilla normal estimation:\n";
//					eval_.evaluateNormalEstimation(point_cloud, normalsVanilla, &ne_statistics_vanilla_);
				}

				// surface classification statistics
				if (CLASSIFY)
				{
					eval_.setClusterHandler(graph->clusters());
					eval_.evaluateSurfaceTypeRecognition(point_cloud, color_image);
				}
			}


		}
//		if(EVALUATION_OFFLINE_MODE)
//		{
//			TODO
//			std::string gt_filename = ...; //path to ground truth cloud
//			eval_.compareClassification(gt_filename);
//		}


	}


private:
	ros::NodeHandle node_handle_;

	// messages
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter colorimage_sub_; ///< Color camera image topic
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >* sync_input_;
	ros::Publisher segmented_pointcloud_pub_;	// publisher for the segmented point cloud

	ros::Publisher object_detection_pub_;	// publisher for object detections
	SimpleObjectClassification simple_object_classification_;
	int sub_counter_; /// Number of subscribers to topic

	//records

	SceneRecording rec_;
	cob_3d_features::OrganizedNormalEstimationEdgeOMP<pcl::PointXYZRGB, pcl::Normal, PointLabel> one_;
	cob_3d_features::OrganizedNormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal, PointLabel> oneWithoutEdges_;

	EdgeDetection<pcl::PointXYZRGB> edge_detection_;
	cob_3d_segmentation::DepthSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> seg_;
	cob_3d_segmentation::RefineSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> segRefined_;

	cob_3d_segmentation::DepthSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> segWithoutEdges_;

	cob_3d_segmentation::ClusterClassifier<ST::CH, ST::Point, ST::Normal, ST::Label> cc_;

	//evaluation
	Evaluation eval_;
	EdgeDetectionStatistics edge_detection_statistics_;
	NormalEstimationStatistics ne_statistics_direct_edge_;
	NormalEstimationStatistics ne_statistics_cross_edge_;
	NormalEstimationStatistics ne_statistics_cross_;
	NormalEstimationStatistics ne_statistics_integral_edge_;
	NormalEstimationStatistics ne_statistics_integral_;
	NormalEstimationStatistics ne_statistics_vanilla_;

	double runtime_total_;
	double runtime_depth_image_;
	double runtime_sobel_;	// image derivatives
	double runtime_edge_;
	double runtime_visibility_;
	double runtime_normal_original_;
	double runtime_normal_edge_;
	int number_processed_images_;


	/// Subscribe to camera topics if not already done.
	void connectCallback()
	{
		ROS_INFO("[surface_classification] Subscribing to camera topics");

		if (sub_counter_ == 0)
		{
			colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
			pointcloud_sub_.subscribe(node_handle_, "pointcloud_in", 1);

			sync_input_->connectInput(colorimage_sub_, pointcloud_sub_);
			sync_input_->registerCallback(boost::bind(&SurfaceClassificationNode::inputCallback, this, _1, _2));
		}
		sub_counter_++;
		ROS_INFO("[surface_classification] %i subscribers on camera topics [OK]", sub_counter_);
	}

	/// Unsubscribe from camera topics if possible.
	void disconnectCallback()
	{
		if (sub_counter_ > 0)
			sub_counter_--;

		if (sub_counter_ == 0)
		{
			ROS_INFO("[surface_classification] Unsubscribing from camera topics");

			colorimage_sub_.unsubscribe();
			pointcloud_sub_.unsubscribe();
		}
		ROS_INFO("[surface_classification] %i subscribers on camera topics [OK]", sub_counter_);
	}
};

int main (int argc, char** argv)
{
	global_imagecount=0;
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "cob_surface_classification");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of CameraDriver
	SurfaceClassificationNode surfaceClassification(nh);

	ros::spin();

	return (0);
}
