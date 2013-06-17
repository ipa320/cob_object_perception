/*
 * edge_detection.h
 *
 *  Created on: May 17, 2013
 *      Author: rmb-ce
 */

#ifndef EDGE_DETECTION_H_
#define EDGE_DETECTION_H_

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/imgproc/imgproc.hpp>

// PCL
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/pcl_base.h>

// timer
#include <iostream>
#include "timer.h"


template <typename PointInT>
class EdgeDetection
{
public:

	typedef pcl::PointCloud<PointInT> PointCloudIn;
	typedef typename PointCloudIn::Ptr PointCloudInPtr;

	EdgeDetection ():
		edgeThreshold_(0.5),
		stepThreshold_(0.05),
		lineLength_(20),
		windowX_(600),
		windowY_(600)
	{};

	inline void setEdgeThreshold(float th)
	{
		edgeThreshold_ = th;
	}
	inline void setStepThreshold(float th)
	{
		stepThreshold_ = th;
	}
	inline void setLineLength(int l)
	{
		lineLength_ = l;
	}
	inline void setWindowSize(int x, int y)
	{
		windowX_ = x;
		windowY_ = y;
	}

	void computeDepthEdges(cv::Mat depth_image, PointCloudInPtr pointcloud, cv::Mat& edgeImage);


private:

	void approximateLine(cv::Mat& depth_image, PointCloudInPtr pointcloud, cv::Point2f dotLeft, cv::Point2f dotRight, cv::Mat& abc,cv::Mat& n, cv::Mat& coordinates, bool& step);
	void approximateLine(cv::Mat& depth_image, PointCloudInPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& abc);
	void scalarProduct(cv::Mat& abc1,cv::Mat& abc2,float& scalarProduct, int& concaveConvex, bool& step);
	void approximateLineFullAndHalfDist (cv::Mat& depth_image, PointCloudInPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& abc);

	void thinEdges(cv::Mat& edgePicture, int xy);
	void drawLines(cv::Mat& plotXY, cv::Mat& coordinates, cv::Mat& abc);
	void drawLineAlongN(cv::Mat& plotZW, cv::Mat& coordinates, cv::Mat& n);

	float edgeThreshold_;	//scalarproduct > edgeThreshold is set to 1 and thus not detected as edge. the larger the threshold, the more lines are detected as edges.
	float stepThreshold_;	//minimum distance which is detected as a step in depth coordinates
	int lineLength_;	//depth coordinates along two lines with length lineLength/2 are considered
	int windowX_;	//size of visualization windows in x-direction
	int windowY_;
};


#include "cob_surface_classification/impl/edge_detection.hpp"


#endif /* EDGE_DETECTION_H_ */
