/*
 * scene_recording.cpp
 *
 *  Created on: Jul 10, 2013
 *      Author: rmb-ce
 */

#include "scene_recording.h"


Scene_recording::Scene_recording() {
	nr_records = 1;
	data_storage_path = std::string(getenv("HOME"));
}

Scene_recording::~Scene_recording() {
}

void Scene_recording::saveImage(cv::Mat color_image, pcl::PointCloud<pcl::PointXYZRGB> pointcloud)
{

	//specify path
	std::stringstream nr;//create a stringstream
	nr << nr_records;//add number to the stream
	std::string image_filename = data_storage_path + "/records/im" + nr.str() + ".png";

	// save image
	cv::imwrite(image_filename, color_image);

	//save pointcloud
	std::string pcd_filename = data_storage_path + "/records/cloud" + nr.str() + ".pcd";
	pcl::io::savePCDFile(pcd_filename, pointcloud, false);


	std::cout << "path: " << image_filename << "\n";
	nr_records++;
}
