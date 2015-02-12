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
 * scene_recording.h
 *
 *  Created on: Jul 10, 2013
 *      Author: rmb-ce
 */

#ifndef SCENE_RECORDING_H_
#define SCENE_RECORDING_H_

#include <string>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>


class SceneRecording {
public:
	SceneRecording();
	virtual ~SceneRecording();
	void saveImage(const cv::Mat& color_image, std::string name);
	void loadImage(cv::Mat& color_image, std::string name);
	void saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointcloud, std::string name);
	void loadCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, std::string name);
	void saveText(std::string txt, std::string name);

	inline void setPath(std::string p) { data_storage_path = p; }


private:
	std::string data_storage_path;
	int nr_records;
};

#endif /* SCENE_RECORDING_H_ */
