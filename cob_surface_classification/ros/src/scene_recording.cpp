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
 * scene_recording.cpp
 *
 *  Created on: Jul 10, 2013
 *      Author: rmb-ce
 */

#include "cob_surface_classification/scene_recording.h"


Scene_recording::Scene_recording() {
	nr_records = 1;
	data_storage_path = std::string(getenv("HOME"));
}

Scene_recording::~Scene_recording() {
}

//save files to "data_storage_path/records/"
//Cloud, image and textfile for each record are labeled with the same number.
//(Number iterated only with new cloud recorded.)
//-------------------------------------------------------------------------------


void Scene_recording::saveImage(cv::Mat color_image, std::string name)
{

	//specify path
	std::stringstream nr;
	nr << nr_records;
	std::string image_filename = data_storage_path + "/records/"  + nr.str() + name + ".png";

	// save image
	cv::imwrite(image_filename, color_image);
}

void Scene_recording::saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointcloud, std::string name)
{
	//specify path
	std::stringstream nr;
	nr << nr_records;
	std::string pcd_filename = data_storage_path + "/records/"  + nr.str() + name + ".pcd";

	//save pointcloud
	pcl::io::savePCDFile(pcd_filename, *pointcloud, false);

	nr_records++;
}

void Scene_recording::saveText(std::string txt, std::string name)
{
	std::ofstream file;
	const char* s;
	std::stringstream nr;
	nr << nr_records;
	std::string filename = data_storage_path + "/records/" +  nr.str() + name + ".txt";
	s = filename.c_str();

	file.open (s);
	file << txt;
	file << "\n";
	file.close();
}
