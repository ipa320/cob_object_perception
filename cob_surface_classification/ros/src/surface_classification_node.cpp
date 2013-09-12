/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2013 \n+
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
 * \date Date of creation: 07.08.2012
 *
 * \brief
 * functions for display of people detections
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

#define RECORD_MODE					false		//save color image and cloud for usage in EVALUATION_OFFLINE_MODE
#define COMPUTATION_MODE			false		//computations without record
#define EVALUATION_OFFLINE_MODE		false		//evaluation of stored pointcloud and image
#define EVALUATION_ONLINE_MODE		true		//computations plus evaluation of current computations plus record of evaluation

//steps in computation/evaluation_online mode:

#define SEG 						true 	//segmentation + refinement
#define SEG_WITHOUT_EDGES 			false 	//segmentation without considering edge image (wie Steffen)
#define SEG_REFINE					false 	//segmentation refinement according to curvatures (outdated)
#define CLASSIFY 					true	//classification


#define NORMAL_VIS 					false 	//visualisation of normals
#define SEG_VIS 					false 	//visualisation of segmentation
#define SEG_WITHOUT_EDGES_VIS 		false 	//visualisation of segmentation without edge image
#define CLASS_VIS 					false 	//visualisation of classification




// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

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
#include <cob_surface_classification/organized_normal_estimation.h>
#include <cob_surface_classification/refine_segmentation.h>

//package includes
#include <cob_3d_segmentation/depth_segmentation.h>
#include <cob_3d_segmentation/cluster_classifier.h>
#include <cob_3d_mapping_common/point_types.h>


//records
#include "scene_recording.h"
//evaluation
#include "Evaluation.h"



class SurfaceClassificationNode
{
public:
	typedef cob_3d_segmentation::PredefinedSegmentationTypes ST;

	SurfaceClassificationNode(ros::NodeHandle nh)
	: node_handle_(nh)
	{
		it_ = 0;
		sync_input_ = 0;

		it_ = new image_transport::ImageTransport(node_handle_);
		colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
		pointcloud_sub_.subscribe(node_handle_, "pointcloud_in", 1);

		sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >(30);
		sync_input_->connectInput(colorimage_sub_, pointcloud_sub_);
		sync_input_->registerCallback(boost::bind(&SurfaceClassificationNode::inputCallback, this, _1, _2));
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

		ROS_INFO("Input Callback");

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

		//compute depth_image: greyvalue represents depth z
		cv::Mat depth_image = cv::Mat::zeros(cloud->height, cloud->width, CV_32FC1);
		int i=0;
		for (unsigned int v=0; v<cloud->height; v++)
		{
			for (unsigned int u=0; u<cloud->width; u++, i++)
			{
				//matrix indices: row y, column x!
				pcl::PointXYZRGB& point = cloud->at(u,v);
//				if(point.z == point.z)	//test nan
					depth_image.at< float >(v,u) = point.z;

			}
		}

		//visualization
//		cv::Mat depth_im_scaled;
//		cv::normalize(depth_image, depth_im_scaled,0,1,cv::NORM_MINMAX);
//		cv::imshow("depth_image", depth_im_scaled);
//		cv::waitKey(10);

		//draw crossline
		//int lineLength = 20;
		//cv::line(color_image,cv::Point2f(2*depth_image.cols/3 -lineLength/2, 2*depth_image.rows/3),cv::Point2f(2*depth_image.cols/3 +lineLength/2, 2*depth_image.rows/3),CV_RGB(0,1,0),1);
		//cv::line(color_image,cv::Point2f(2*depth_image.cols/3 , 2*depth_image.rows/3 +lineLength/2),cv::Point2f(2*depth_image.cols/3 , 2*depth_image.rows/3 -lineLength/2),CV_RGB(0,1,0),1);



		//record scene
		//----------------------------------------
		if(RECORD_MODE)
		{
			cv::Mat im_flipped;
			cv::flip(color_image, im_flipped,-1);
			cv::imshow("image", im_flipped);
			int key = cv::waitKey(50);
			//record if "r" is pressed while "image"-window is activated
			if(key == 1048690)
			{
				rec_.saveImage(im_flipped,"color");
				rec_.saveCloud(cloud,"cloud");
			}

		}

		//----------------------------------------

		int key = 0;
		cv::imshow("image", color_image);
		if(!EVALUATION_ONLINE_MODE) cv::waitKey(50);
		if(EVALUATION_ONLINE_MODE){ key = cv::waitKey(50);}
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


//		oneWithoutEdges_.setInputCloud(cloud);
//		oneWithoutEdges_.setPixelSearchRadius(8,1,1);
//		oneWithoutEdges_.setOutputLabels(labelsWithoutEdges);
//		oneWithoutEdges_.setSkipDistantPointThreshold(8);	//PUnkte mit einem Abstand in der Tiefe von 8 werden nicht mehr zur Nachbarschaft gezählt
//		oneWithoutEdges_.compute(*normalsWithoutEdges);

			cv::Mat edgeImage = cv::Mat::ones(depth_image.rows,depth_image.cols,CV_32FC1);
			edge_detection_.computeDepthEdges( depth_image, cloud, edgeImage);

			//edge_detection_.sobelLaplace(color_image,depth_image);

			//cv::imshow("edge_image", edgeImage);
			//cv::waitKey(10);

			//Timer timer;
			//timer.start();
			//for(int i=0; i<10; i++)
			//{

			one_.setInputCloud(cloud);
			one_.setPixelSearchRadius(8,1,1);	//call before calling computeMaskManually()!!!
			one_.computeMaskManually_increasing(cloud->width);
			one_.setEdgeImage(edgeImage);
			one_.setOutputLabels(labels);
			one_.setSameDirectionThres(0.94);
			one_.setSkipDistantPointThreshold(8);	//don't consider points in neighbourhood with depth distance larger than 8
			one_.compute(*normals);

			//}timer.stop();
			//std::cout << timer.getElapsedTimeInMilliSec() << " ms for normalEstimation on the whole image, averaged over 10 iterations\n";



			if(NORMAL_VIS)
			{
				// visualize normals
				pcl::visualization::PCLVisualizer viewerNormals("Cloud and Normals");
				viewerNormals.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbNormals(cloud);

				viewerNormals.addPointCloud<pcl::PointXYZRGB> (cloud, rgbNormals, "cloud");
				viewerNormals.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normals,2,0.005,"normals");
				viewerNormals.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

				while (!viewerNormals.wasStopped ())
				{
					viewerNormals.spinOnce();
				}
				viewerNormals.removePointCloud("cloud");
			}

			if(SEG || EVALUATION_ONLINE_MODE)
			{
				seg_.setInputCloud(cloud);
				seg_.setNormalCloudIn(normals);
				seg_.setLabelCloudInOut(labels);
				seg_.setClusterGraphOut(graph);
				seg_.performInitialSegmentation();
				seg_.refineSegmentation();
			}
			if(SEG_WITHOUT_EDGES)
			{
				segWithoutEdges_.setInputCloud(cloud);
				segWithoutEdges_.setNormalCloudIn(normalsWithoutEdges);
				segWithoutEdges_.setLabelCloudInOut(labelsWithoutEdges);
				segWithoutEdges_.setClusterGraphOut(graphWithoutEdges);
				segWithoutEdges_.performInitialSegmentation();
			}

			if(SEG_VIS)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented(new pcl::PointCloud<pcl::PointXYZRGB>);
				*segmented = *cloud;
				graph->clusters()->mapClusterColor(segmented);

				// visualize segmentation
				pcl::visualization::PCLVisualizer viewer("segmentation");
				viewer.setBackgroundColor (0.0, 0.0, 0);
				pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(segmented);
				viewer.addPointCloud<pcl::PointXYZRGB> (segmented,rgb,"seg");
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



			if(CLASSIFY|| EVALUATION_ONLINE_MODE)
			{
				//classification

				cc_.setClusterHandler(graph->clusters());
				cc_.setNormalCloudInOut(normals);
				cc_.setLabelCloudIn(labels);
				cc_.setPointCloudIn(cloud);
				cc_.setMaskSizeSmooth(14);
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

	}//inputCallback()


private:
	ros::NodeHandle node_handle_;

	// messages
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter colorimage_sub_; ///< Color camera image topic
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >* sync_input_;

	//records
	Scene_recording rec_;

	cob_features::OrganizedNormalEstimation<pcl::PointXYZRGB, pcl::Normal, PointLabel> one_;
	cob_features::OrganizedNormalEstimation<pcl::PointXYZRGB, pcl::Normal, PointLabel> oneWithoutEdges_;

	EdgeDetection<pcl::PointXYZRGB> edge_detection_;
	cob_3d_segmentation::DepthSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> seg_;
	cob_3d_segmentation::RefineSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> segRefined_;

	cob_3d_segmentation::DepthSegmentation<ST::Graph, ST::Point, ST::Normal, ST::Label> segWithoutEdges_;

	cob_3d_segmentation::ClusterClassifier<ST::CH, ST::Point, ST::Normal, ST::Label> cc_;

	//evaluation
	Evaluation eval_;

};

int main (int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "cob_surface_classification");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create and initialize an instance of CameraDriver
	SurfaceClassificationNode surfaceClassification(nh);

	ros::spin();

	return (0);
}
