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

//pcl
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>


#include <boost/tuple/tuple.hpp>


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


	ST::CH::Ptr clusterHandler;

    std::vector<int> color_tab;




};

#endif /* EVALUATION_H_ */
