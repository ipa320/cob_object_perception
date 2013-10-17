/*
 * PCLVisualization.h
 *
 *  Created on: 03.07.2013
 *      Author: matthias
 */

#ifndef PCLVISUALIZATION_H_
#define PCLVISUALIZATION_H_

// PCL specific includes
#include <ros/ros.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/io/pcd_io.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

#include <sstream>
#include <cstring>

class PCLVisualization{
public:
	PCLVisualization();
	virtual ~PCLVisualization();

	void addRealtimeInformation(cv::Mat,cv::Mat,cv::Mat,double);
	void setCameraToPOV(double,double,double,double,double,double,double,double,double);
	void addVecToCloud(cv::Mat vec3d,int rr,int gg ,int bb);
	void addCube(cv::Mat vec3d,double size);
	void viewerOneOff (pcl::visualization::PCLVisualizer& viewer);
	void showCloud();
	void spin();

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> PCLviewer;
	unsigned int cubecounter;
	unsigned int realtimeid;

	double camera_x;
	double camera_y;
	double camera_z;

	double  camera_alpha;
	double  camera_beta;
	double  camera_gamma;

	void Init();
};


#endif /* PCLVISUALIZATION_H_ */
