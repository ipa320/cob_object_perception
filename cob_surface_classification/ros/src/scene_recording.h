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


class Scene_recording {
public:
	Scene_recording();
	virtual ~Scene_recording();
	void saveImage(cv::Mat color_image, std::string name);
	void saveCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointcloud, std::string name);
	void saveText(std::string txt, std::string name);


private:
	std::string data_storage_path;
	int nr_records;
};

#endif /* SCENE_RECORDING_H_ */
