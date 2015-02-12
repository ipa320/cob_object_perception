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

#define DATA_SOURCE					1			// 0=from camera, 1=from file
#define RECORD_MODE					false		//save color image and cloud for usage in EVALUATION_OFFLINE_MODE
#define COMPUTATION_MODE			true		//computations without record
#define EVALUATION_OFFLINE_MODE		false		//evaluation of stored pointcloud and image
#define EVALUATION_ONLINE_MODE		false		//computations plus evaluation of current computations plus record of evaluation

//steps in computation/evaluation_online mode:

#define SEG 						false 	//segmentation + refinement
#define SEG_WITHOUT_EDGES 			false 	//segmentation without considering edge image (wie Steffen)
#define SEG_REFINE					false 	//segmentation refinement according to curvatures (outdated)
#define CLASSIFY 					false	//classification


#define NORMAL_VIS 					false 	//visualisation of normals
#define SEG_VIS 					false 	//visualisation of segmentation
#define SEG_WITHOUT_EDGES_VIS 		false 	//visualisation of segmentation without edge image
#define CLASS_VIS 					false 	//visualisation of classification

#define PUBLISH_SEGMENTATION		false //true	//publish segmented point cloud on topic


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

int global_imagecount;

class SurfaceClassificationNode
{
public:
	typedef cob_3d_segmentation::PredefinedSegmentationTypes ST;

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

		depth_factor_ = 0.01f;

		it_ = 0;
		sync_input_ = 0;

		segmented_pointcloud_pub_ = node_handle_.advertise<cob_surface_classification::SegmentedPointCloud2>("segmented_pointcloud", 1);

		if (DATA_SOURCE == 0)
		{
			it_ = new image_transport::ImageTransport(node_handle_);
			colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
			pointcloud_sub_.subscribe(node_handle_, "pointcloud_in", 1);

			sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >(30);
			sync_input_->connectInput(colorimage_sub_, pointcloud_sub_);
			sync_input_->registerCallback(boost::bind(&SurfaceClassificationNode::inputCallback, this, _1, _2));
		}
		else if (DATA_SOURCE == 1)
		{
			const int image_number = 6;
			std::vector<cv::Mat> image_vector(image_number);
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointcloud_vector(image_number);
			for (int i=1; i<=image_number; ++i)
			{
				std::stringstream ss_image, ss_cloud;
				ss_image << i << "color";
				rec_.loadImage(image_vector[i-1], ss_image.str());
				ss_cloud << i << "cloud";
				pointcloud_vector[i-1] = boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >(new pcl::PointCloud<pcl::PointXYZRGB>);
				rec_.loadCloud(pointcloud_vector[i-1], ss_cloud.str());
			}

			int index = 0;
			int counter = 1;
			while (counter<=600)
			{
				computations(image_vector[index], pointcloud_vector[index]);
				index = (index+1)%image_number;
				std:cout << "=============================================================> Finished iteration " << counter++ << std::endl;
			}
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
		//ROS_INFO("Input Callback");

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
		}

		computations(color_image, cloud);
	}

	void computations(cv::Mat& color_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
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
				rec_.saveCloud(cloud,"cloud");
			}

		}

		//----------------------------------------

		Timer tim;

		//std::cout << key <<endl;
		//record if "e" is pressed while "image"-window is activated
		if(COMPUTATION_MODE || (EVALUATION_ONLINE_MODE && key == 1048677))
		{
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			pcl::PointCloud<pcl::Normal>::Ptr normalsWithoutEdges(new pcl::PointCloud<pcl::Normal>);
			pcl::PointCloud<PointLabel>::Ptr labels(new pcl::PointCloud<PointLabel>);
			pcl::PointCloud<PointLabel>::Ptr labelsWithoutEdges(new pcl::PointCloud<PointLabel>);
			ST::Graph::Ptr graph(new ST::Graph);
			ST::Graph::Ptr graphWithoutEdges(new ST::Graph);

/*
			if (key=='n')
			{
				tim.start();
				oneWithoutEdges_.setInputCloud(cloud);
				oneWithoutEdges_.setPixelSearchRadius(8,2,2);	//(8,1,1)   (8,2,2)
				oneWithoutEdges_.setOutputLabels(labelsWithoutEdges);
				oneWithoutEdges_.setSkipDistantPointThreshold(8);	//PUnkte mit einem Abstand in der Tiefe von 8 werden nicht mehr zur Nachbarschaft gezählt
				oneWithoutEdges_.compute(*normalsWithoutEdges);
				//std::cout << "Normal computation without edges: " << tim.getElapsedTimeInMilliSec() << "\n";
				runtime_normal_original_ += tim.getElapsedTimeInMilliSec();
				//return;
			}
//*/
			cv::Mat edge;
			edge_detection_.computeDepthEdges(cloud, edge, depth_factor_);
			//edge_detection_.sobelLaplace(color_image,depth_image);

			// visualization on color image
//			cv::line(color_image, cv::Point(320-edge_detection_.getScanLineWidth(),240), cv::Point(320+edge_detection_.getScanLineWidth(),240),CV_RGB(255,0,0), 2);
//			cv::line(color_image, cv::Point(320,240-edge_detection_.getScanLineWidth()), cv::Point(320,240+edge_detection_.getScanLineWidth()),CV_RGB(255,0,0), 2);
//			const cv::Vec3b green = cv::Vec3b(0, 255, 0);
//			const cv::Vec3b blue = cv::Vec3b(255, 0, 0);
//			for (int v=0; v<color_image.rows; ++v)
//				for (int u=0; u<color_image.cols; ++u)
//				{
//					if (edge.at<uchar>(v,u) == 254)
//						color_image.at<cv::Vec3b>(v,u) = blue;
//					if (edge.at<uchar>(v,u) == 255)
//						color_image.at<cv::Vec3b>(v,u) = green;
//				}
//			cv::imshow("color with edge", color_image);
//			int quit = cv::waitKey(1000);
//			if (quit=='q')
//				exit(0);

//			cv::imshow("edge", edge);
//			int key2 = cv::waitKey(10);
//			if (key2 == 'i')
//				depth_factor_ -= 0.005;
//			else if (key2 == 'o')
//				depth_factor_ += 0.005;
//			std::cout << "depth_factor_ = " << depth_factor_ << std::endl;

			//Timer timer;
			//timer.start();
			//for(int i=0; i<10; i++)
			//{

//			if (key=='n')
//			{
				tim.start();
				one_.setInputCloud(cloud);
				one_.setPixelSearchRadius(4,2,2);	//call before calling computeMaskManually()!!!
				//one_.computeMaskManually_increasing(cloud->width);
				one_.computeMaskManually(cloud->width);
				one_.computePointAngleLookupTable(16);
				one_.setEdgeImage(edge);
				one_.setOutputLabels(labels);
				//one_.setSameDirectionThres(0.94);
				one_.setSkipDistantPointThreshold(8);	//don't consider points in neighbourhood with depth distance larger than 8
				one_.compute(*normals);
				//std::cout << "Normal computation obeying edges: " << tim.getElapsedTimeInMilliSec() << "\n";
				runtime_normal_edge_ += tim.getElapsedTimeInMilliSec();
				++number_processed_images_;
				std::cout << "runtime_normal_original: " << runtime_normal_original_/(double)number_processed_images_ <<
							"\n\t\t\t\truntime_normal_edge: " << runtime_normal_edge_/(double)number_processed_images_ << std::endl;
//			}
			//}timer.stop();
			//std::cout << timer.getElapsedTimeInMilliSec() << " ms for normalEstimation on the whole image, averaged over 10 iterations\n";



			if(NORMAL_VIS || key=='n')
			{
				// visualize normals
				pcl::visualization::PCLVisualizer viewerNormals("Cloud and Normals");
				viewerNormals.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbNormals(cloud);

				viewerNormals.addPointCloud<pcl::PointXYZRGB> (cloud, rgbNormals, "cloud");
				viewerNormals.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normals,2,0.005,"normals");
				//viewerNormals.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normalsWithoutEdges,2,0.005,"normalsWithoutEdges");
				viewerNormals.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

				while (!viewerNormals.wasStopped ())
				{
					viewerNormals.spinOnce();
				}
				viewerNormals.removePointCloud("cloud");
			}

			//return;

			if(SEG || EVALUATION_ONLINE_MODE || key=='n')
			{
				tim.start();
				seg_.setInputCloud(cloud);
				seg_.setNormalCloudIn(normals);
				seg_.setLabelCloudInOut(labels);
				seg_.setClusterGraphOut(graph);
				seg_.performInitialSegmentation();
				seg_.refineSegmentation();
				double runtime_segmentation = tim.getElapsedTimeInMilliSec();
				std::cout << "runtime_segmentation: " << runtime_segmentation << std::endl;
			}
			if(SEG_WITHOUT_EDGES)
			{
				segWithoutEdges_.setInputCloud(cloud);
				segWithoutEdges_.setNormalCloudIn(normalsWithoutEdges);
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

			return;

			if(CLASSIFY|| EVALUATION_ONLINE_MODE)
			{
				//classification

				cc_.setClusterHandler(graph->clusters());
				cc_.setNormalCloudInOut(normals);
				cc_.setLabelCloudIn(labels);
				cc_.setPointCloudIn(cloud);
				//cc_.setMaskSizeSmooth(14);
				cc_.classify();
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
				eval_.setClusterHandler(graph->clusters());
				eval_.compareClassification(cloud,color_image);
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

	//records

	SceneRecording rec_;
	cob_3d_features::OrganizedNormalEstimationEdgeOMP<pcl::PointXYZRGB, pcl::Normal, PointLabel> one_;
	cob_3d_features::OrganizedNormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal, PointLabel> oneWithoutEdges_;

	EdgeDetection<pcl::PointXYZRGB> edge_detection_;
	cob_3d_segmentation::DepthSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> seg_;
	cob_3d_segmentation::RefineSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> segRefined_;

	cob_3d_segmentation::DepthSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> segWithoutEdges_;

	cob_3d_segmentation::ClusterClassifier<ST::CH, ST::Point, ST::Normal, ST::Label> cc_;

	float depth_factor_;

	//evaluation
	Evaluation eval_;

	double runtime_total_;
	double runtime_depth_image_;
	double runtime_sobel_;	// image derivatives
	double runtime_edge_;
	double runtime_visibility_;
	double runtime_normal_original_;
	double runtime_normal_edge_;
	int number_processed_images_;


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
