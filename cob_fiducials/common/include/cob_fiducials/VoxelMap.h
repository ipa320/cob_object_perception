/*
 * VoxelMap.h
 *
 *  Created on: 19.06.2013
 *      Author: matthias
 */

#ifndef VOXELMAP_H_
#define VOXELMAP_H_

#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <SerializeIO.h>
#include <PCLVisualization.h>

const double PI = 3.14159265358979323843;

//-----------------------------------------------Structs-----------------------------------------------------------------------
//position + position error
struct point4d{
	double x;
	double y;
	double z;
	double e;
	bool detected;

};
struct pointallocation{
	unsigned int fiducialID;
	unsigned int pointNumber;
};
struct fiducialmarker{
	unsigned int id;
	cv::Point3d trans;
	cv::Point3d rot;
	std::vector<point4d> points;
};
struct voxel{
	unsigned int id;
	std::vector<pointallocation> pointalloc;
	std::vector<unsigned int> premiumvoxels;
	unsigned int nextpremiumvoxel;
	bool issupervoxel;
	cv::Point3i pos;
	cv::Scalar size;
};
//scala in mm
struct voxelmap{
	bool initialized;
	std::vector<voxel> voxels;
	int size;
	int voxelsize;
};

struct fiducialerror{
	unsigned int fiducialid;
	double error;
};

//----------------------------------------------------------------------------------------------------------------------------------

class VoxelMap {
private:
	void init();

	bool readFiducialMarkers(const char*,std::vector<fiducialmarker>*);
	bool readPointsOfView(const char*,fiducialmarker*);
	bool assignPOVToVoxel(fiducialmarker*);

	std::vector<cv::Mat> getCameraPOS(int,double,double,double,double,double,double);

	point4d transformToPoint4d(std::vector<double>);
	cv::Mat eulerToMatrixFiducialSystem(double,double,double);
	cv::Mat eulerToMatrixOriginSystem(double,double,double);

	cv::Point3d getPointInOriginSystem(fiducialmarker*,unsigned int);
	fiducialmarker getFiducialById(unsigned int);
	PCLVisualization *cloudhandler;

	void findSuperVoxels();
	void drawMarkerImage(fiducialmarker*,PCLVisualization*);
	bool newMap(int,int);
	std::vector<fiducialmarker> fiducialmarkers;

	//TEST
	std::ofstream output;
	bool supervoxel;
	unsigned int framenumber;


public:
	VoxelMap();
	virtual ~VoxelMap();
	unsigned int voxelID(double,double,double);
	void setDetectedMarkerArray(std::vector<std::vector<double> >);
	void spin();
	voxelmap globalmap;
};


#endif /* VOXELMAP_H_ */
