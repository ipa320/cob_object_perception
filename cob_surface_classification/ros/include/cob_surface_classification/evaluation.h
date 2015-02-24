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

/*
 * Evaluation.h
 *
 *  Created on: Jul 10, 2013
 *      Author: rmb-ce
 */

#ifndef EVALUATION_H_
#define EVALUATION_H_

//package includes
#include "cob_3d_segmentation/cluster_handler.h"
#include <cob_3d_segmentation/cluster_classifier.h>
#include <cob_3d_segmentation/depth_segmentation.h>
#include <cob_3d_mapping_common/label_defines.h>
#include "cob_surface_classification/scene_recording.h"

//pcl
#include <pcl/point_types.h>

// opencv
#include <opencv2/core/core.hpp>

#include <boost/tuple/tuple.hpp>


#define I_CONCAVE 8
#define I_CONVEX 9
#define NUMBER_SURFACE_CLASS_TYPES 10

struct EdgeDetectionStatistics
{
	double recall;
	double precision;
	double number_images;

	EdgeDetectionStatistics()
	{
		clear();
	}

	void clear()
	{
		recall = 0.;
		precision = 0.;
		number_images = 0.;
	}

	void addStatistics(double recall_val, double precision_val)
	{
		recall = (recall*number_images + recall_val)/(number_images+1.);
		precision = (precision*number_images + precision_val)/(number_images+1.);
		number_images += 1.;
	}
};

struct NormalEstimationStatistics
{
	double coverage_gt_normals;		// how many normals are computed w.r.t. the number of ground truth normals
	double average_angular_error;	// average normal estimation error
	double percentage_good_normals;	// ratio of sufficiently accurate normals
	double number_images;

	NormalEstimationStatistics()
	{
		clear();
	}

	void clear()
	{
		coverage_gt_normals = 0.;
		average_angular_error = 0.;
		percentage_good_normals = 0.;
		number_images = 0.;
	}

	void addStatistics(double coverage_gt_normals_val, double average_angular_error_val, double percentage_good_normals_val)
	{
		coverage_gt_normals = (coverage_gt_normals*number_images + coverage_gt_normals_val)/(number_images+1.);
		average_angular_error = (average_angular_error*number_images + average_angular_error_val)/(number_images+1.);
		percentage_good_normals = (percentage_good_normals*number_images + percentage_good_normals_val)/(number_images+1.);
		number_images += 1.;
	}
};

class Evaluation
{
public:
	Evaluation();
	virtual ~Evaluation();

	void evaluateSurfaceTypeRecognition(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, const cv::Mat& gt_color_image);
	void evaluateEdgeRecognition(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, const cv::Mat& gt_color_image, const cv::Mat& edge_estimate, EdgeDetectionStatistics* edge_detection_statistics = 0);
	void evaluateNormalEstimation(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals, NormalEstimationStatistics* ne_statistics = 0);

	typedef cob_3d_segmentation::PredefinedSegmentationTypes ST;
    inline void setClusterHandler(ST::CH::Ptr cHdl) { clusterHandler = cHdl; }

	int compareClassification(std::string gt_filename);
	int compareClassification(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr gt, cv::Mat gt_color_image);


private:

	std::string search_directory;
	SceneRecording rec;

	ST::CH::Ptr clusterHandler;

    std::vector<int> color_table_;
    std::vector<cv::Vec3b> color_table_sim_;			// surface type enumerated color coding as used in the gazebo simulation

	struct Statistics
	{
		int countCorrect;
		int countCorrectEdge;
		int countCorrectPlane;
		int countCorrectConc;
		int countCorrectConv;
		int countCompared;
		int countNoColorAssigned;
		int countEdge;
		int countPlane;
		int countConc;
		int countConv;

		Statistics()
		{
			countCorrect = 0;
			countCorrectEdge = 0;
			countCorrectPlane = 0;
			countCorrectConc = 0;
			countCorrectConv = 0;
			countCompared = 0;
			countNoColorAssigned = 0;
			countEdge = 0;
			countPlane = 0;
			countConc = 0;
			countConv = 0;
		}
	} ;

	struct percentages
	{
		float plane;
		float edge;
		float conc;
		float conv;
		float overall;
	};

	double divide(double a, double b);
	void compareImagesUsingColor(cv::Mat imOrigin, cv::Mat imComp, Evaluation::Statistics& c);
    void clusterTypesToColorImage(cv::Mat& test_image, unsigned int height,unsigned int width);

    int getSurfaceTypeFromColor(const cv::Vec3b& color);

    // recover surface type encoding from a color encoded real or simulated image
	void generateGroundTruthImage(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, const cv::Mat& gt_color_image, cv::Mat& gt_color_image_normalized);

	// generates a surface type image with color coding similar to the ground truth image
	void generateSurfaceTypeEstimateImage(cv::Mat& color_image, const int height, const int width);

	bool checkDepthEdge(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, int u, int v);

	void computeGroundTruthNormals(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, pcl::PointCloud<pcl::Normal>::Ptr gt_normals);

	void computeNormalEstimationError(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& gt_point_cloud, const pcl::PointCloud<pcl::Normal>::Ptr& gt_normals,
			const pcl::PointCloud<pcl::Normal>::Ptr& normals, const int padding, int& number_gt_normals, int& number_normals, int& number_good_normals, double& normal_error);

	// compute the numbers for recall and precision
	// @param search_radius is half the side length of the search neighborhood for fitting estimates and gt pixels (e.g. an edge pixel is correctly labeled in the estimate if there is an edge pixel within the (2*search_radius+1) neighborhood in the gt_image)
	void computePerformanceMeasures(const cv::Mat& gt_image, const cv::Mat& estimate, const int search_radius, std::vector<double>& recall, std::vector<double>& precision);

};

#endif /* EVALUATION_H_ */
