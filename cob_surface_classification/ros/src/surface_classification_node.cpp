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



#include <cob_surface_classification/edge_detection.h>
//#include <cob_surface_classification/surface_classification.h>
#include <cob_surface_classification/organized_normal_estimation.h>

class SurfaceClassificationNode
{
public:
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
		Timer timer;
		timer.start();
		ROS_INFO("Input Callback");

		// convert color image to cv::Mat
		cv_bridge::CvImageConstPtr color_image_ptr;
		cv::Mat color_image;
		convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

		//visualization
		cv::imshow("image", color_image);
		cv::waitKey(10);

		// get color image from point cloud
		/*pcl::PointCloud<pcl::PointXYZRGB> point_cloud_src;
		pcl::fromROSMsg(*pointcloud_msg, point_cloud_src);
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pc_ptr(&point_cloud_src);*/

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
		//Inhalt des Pointers übergeben
		pcl::fromROSMsg(*pointcloud_msg, *cloud);




		//		cv::Mat color_image = cv::Mat::zeros(point_cloud_src.height, point_cloud_src.width, CV_8UC3);
		//		for (unsigned int v=0; v<point_cloud_src.height; v++)
		//		{
		//			for (unsigned int u=0; u<point_cloud_src.width; u++)
		//			{
		//				pcl::PointXYZRGB point = point_cloud_src(u,v);
		//				if (isnan_(point.z) == false)
		//					color_image.at<cv::Point3_<unsigned char> >(v,u) = cv::Point3_<unsigned char>(point.b, point.g, point.r);
		//			}
		//		}

		//compute depth_image: greyvalue corresponds to depth z
		cv::Mat depth_image = cv::Mat::zeros(cloud->height, cloud->width, CV_32FC1);
		for (unsigned int v=0; v<cloud->height; v++)
		{
			for (unsigned int u=0; u<cloud->width; u++)
			{
				//bei Aufruf aus der Matrix ist der Index (Zeile,Spalte), also (y-Wert,x-Wert)!!!
				pcl::PointXYZRGB point = cloud->at(u,v);
				if(std::isnan(point.z) == false)
					depth_image.at< float >(v,u) = point.z;
				//std::cout << "di: " << depth_image.at< float >(v,u )<<"\n";
				//std::cout << "point: " << point.z <<"\n";
			}
		}

		/*cv::imshow("depth_image", depth_image);
		cv::waitKey(10);*/

		cv::Mat edgeImage = cv::Mat::ones(depth_image.rows,depth_image.cols,CV_32FC1);


		edge_detection_.computeDepthEdges( depth_image, cloud, edgeImage);
		//cv::imshow("edge_image", edgeImage);
		//cv::waitKey(10);

		one_.setInputCloud(cloud);
		one_.setPixelSearchRadius(8,1,1);	//call before calling computeMaskManually()!!!
		one_.computeMaskManually_increasing(cloud->width);
		one_.setEdgeImage(edgeImage);
		one_.setSkipDistantPointThreshold(8);	//PUnkte mit einem Abstand in der Tiefe von 8 werden nicht mehr zur Nachbarschaft gezählt

		//int index = cloud->width * (cloud->height/2-1) + cloud->width/2; //index des Punktes in der Mitte vom Fenster: Fensterbreite * halbe Anzahl Zeilen + halbe Zeile
		/*boost::shared_ptr <std::vector<int> > ind;
		ind.reset (new std::vector<int>);
		ind->resize(std::floor(cloud->points.size() /5));
		for(size_t i=0; i<ind->size();i++)
		{
			(*ind)[i] = i*5;
			std::cout << "ind " << (*ind)[i] <<endl;
		}
		//std::cout <<endl;
		//ind->push_back(index);
		one_.setIndices(ind);*/
		//std::cout << "width " << cloud->width<<endl;
		one_.compute(*normals);


/*


		// visualize normals
		pcl::visualization::PCLVisualizer viewer("Cloud and Normals");
		viewer.setBackgroundColor (0.0, 0.0, 0);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);


		viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "cloud");
		viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(cloud, normals,1,0.005,"normals");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
		//viewer.addCoordinateSystem (1.0);
		//viewer.initCameraParameters ();

		while (!viewer.wasStopped ())
			{
				viewer.spinOnce();

			}
			viewer.removePointCloud("cloud");*/



		timer.stop();
		std::cout << timer.getElapsedTimeInMilliSec() << " ms for InputCallback\n";

	}

private:
	ros::NodeHandle node_handle_;

	// messages
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter colorimage_sub_; ///< Color camera image topic
	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> >* sync_input_;

	//SurfaceClassification surface_classification_;
	cob_features::OrganizedNormalEstimation<pcl::PointXYZRGB, pcl::Normal> one_;

	EdgeDetection<pcl::PointXYZRGB> edge_detection_;
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
