/*
 * FiducialMarkerEvaluationToolkit.h
 *
 *	This Toolkit provides Methods to evaluate fiducial marker systems
 *
 *  Created on: 28.05.2013
 *      Author: matthias
 */

#ifndef FIDUCIALMARKEREVALUATIONTOOLKIT_H_
#define FIDUCIALMARKEREVALUATIONTOOLKIT_H_

#include <ros/ros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <math.h>
#include <time.h>
#include <SerializeIO.h>

class FiducialMarkerEvaluationToolkit {

private:

	//detection Rates
	unsigned int tp; // true positive
	unsigned int fn; // false negative
	unsigned int fp; // false positive

	cv::Mat camera_intrinsics;
	unsigned int imagenumber;
	unsigned int lastimage;
	std::ifstream datafile;
	std::ofstream outputfile;
	std::ofstream outputtheta;
	std::ofstream output_eucdist;
	std::ofstream output_detectionTime;
	std::vector<std::vector<double> >  detectedmarkers_vec;
	std::vector<double> gtdata; //Patience Euler Angles
	double actdepth;
	timeval start;
	long dif;
	long dt_frame;
	std::vector<unsigned int> falsenegatives;
	bool debug;

	double frobeniusNorm(cv::Mat,cv::Mat);
	double anglefromFrobenius(double fnorm);
	double computeAngleFromVec(std::vector<double>);
	double computeEuclideanDistFromVec(std::vector<double> vec);
	void writeToFile();
	cv::Mat readImage();
	std::vector<double> readFile();
	void writeFile(bool,std::vector<double>);
	void init();


	//additional Stuff
	std::vector<double> matrixToQuat(cv::Mat frame);
	cv::Mat eulerToMatrix(double, double, double);
	unsigned long RenderPose(cv::Mat& image, cv::Mat& rot_3x3_CfromO, cv::Mat& trans_3x1_CfromO);
	unsigned long ReprojectXYZ(double x, double y, double z, int& u, int& v);

	//Serialize
	SerializeIO *fileparser;

	//Video
	cv::VideoCapture cap;


public:

	FiducialMarkerEvaluationToolkit();
	virtual ~FiducialMarkerEvaluationToolkit();

	//Get - Methods
	cv::Mat getNextMarker();
	cv::Mat getIntrinsicMatrix();

	//Set -Methods
	void setIntrinsicMatrix(cv::Mat);
	void setPushMarkerPosition(std::vector<double>);
	void setPushMarkerPosition(cv::Mat);
	void setStartAnalysis();
};

#endif /* FIDUCIALMARKEREVALUATIONTOOLKIT_H_ */
