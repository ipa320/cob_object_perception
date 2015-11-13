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

class Evaluation {
public:
	Evaluation();
	virtual ~Evaluation();

	typedef cob_3d_segmentation::PredefinedSegmentationTypes ST;
    inline void setClusterHandler(ST::CH::Ptr cHdl) { clusterHandler = cHdl; }

	int compareClassification(std::string gt_filename);
	int compareClassification(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr gt, cv::Mat gt_color_image);


private:
	std::string search_directory;
	Scene_recording rec;

	ST::CH::Ptr clusterHandler;

    std::vector<int> color_tab;

	struct count
	{
		int countCorrect ;
		int countCorrectEdge;
		int countCorrectPlane ;
		int countCorrectConc ;
		int countCorrectConv ;
		int countCompared ;
		int countNoColorAssigned ;
		int countEdge ;
		int countPlane ;
		int countConc ;
		int countConv ;
	} ;

	struct percentages
	{
		float plane;
		float edge;
		float conc;
		float conv;
		float overall;
	};

	float divide(float a, float b);
	void compareImagesUsingColor(cv::Mat imOrigin, cv::Mat imComp, Evaluation::count& c);
    void clusterTypesToColorImage(cv::Mat& test_image, unsigned int height,unsigned int width);

};

#endif /* EVALUATION_H_ */
