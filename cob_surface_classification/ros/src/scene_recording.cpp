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

void Scene_recording::saveImage(cv::Mat color_image, std::string name)
{

	//specify path
	std::stringstream nr;//create a stringstream
	nr << nr_records;//add number to the stream
	std::string image_filename = data_storage_path + "/records/" + name + nr.str() + ".png";

	// save image
	cv::imwrite(image_filename, color_image);


	//std::cout << "path: " << image_filename << "\n";
	//nr_records++;
}

void Scene_recording::saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointcloud, std::string name)
{
	//specify path
		std::stringstream nr;//create a stringstream
		nr << nr_records;//add number to the stream
		std::string pcd_filename = data_storage_path + "/records/" + name + nr.str() + ".pcd";

		//save pointcloud
		pcl::io::savePCDFile(pcd_filename, *pointcloud, false);

		nr_records++;
}

void Scene_recording::saveText(std::string txt, std::string name)
{
  std::ofstream file;
  const char* s;
  std::stringstream nr;//create a stringstream
  nr << nr_records;//add number to the stream
  std::string filename = data_storage_path + "/records/" + name + nr.str() + ".txt";
  s = filename.c_str();

  file.open (s);
  file << txt;
  file << "\n";
  file.close();
}
