/*
 * FiducialMarkerEvaluationToolkit.cpp
 *
 *  Created on: 28.05.2013
 *      Author: matthias
 */

#include "cob_fiducials/FiducialMarkerEvaluationToolkit.h"

std::vector<int> falsepos;
static const char WINDOW[] = "Image window";
const double PI = 3.14159265358979323843;

//-------------------Constructor - Destructor------------------------------------------------------
FiducialMarkerEvaluationToolkit::FiducialMarkerEvaluationToolkit() {
	init();
}

FiducialMarkerEvaluationToolkit::~FiducialMarkerEvaluationToolkit() {
	// TODO Auto-generated destructor stub
}

//--------------------------init()------------------------------------------------------------------
void FiducialMarkerEvaluationToolkit::init(){

	//Video
//	cap.open("/home/matthias/Diplomarbeit/Videos/WP_20130826_001.mp4");
//	if(!cap.isOpened())  {
//		std::cout << "Failed to Load Video" << std::endl;
//	}
//	cap.set(CV_CAP_PROP_POS_FRAMES,300);
//	cap.get(CV_CAP_PROP_FRAME_COUNT);
	bool video = false;
	//Video

	//File Parser Serialize IO
	fileparser = new SerializeIO("//media/DATEN/Extended_PiTag10/MAP/tag1_newerror.log",'o');
	fileparser->openArray("TESTTEST");

	cv::namedWindow(WINDOW);

    //Variable init
    imagenumber = 0;//280 für stark verdrehte 1.39
    actdepth = 0;
    lastimage = -1;
    tp = 0;
    fp = 0;
    fn = 0;
    dif = 0;
    debug = false;

    //Read Number of images in current file
    datafile.open("/media/DATEN/Extended_PiTag10/MAP/Tag1/a_pitag.log");

    //If no specific lastimage is given (-1).. read the number of images from the info file
    if(lastimage == -1){
    	char tmp[100];
    	datafile.getline(tmp,100);
    	datafile.getline(tmp,100);
    	while(!datafile.eof()){
    		datafile.getline(tmp,100);
    		lastimage++;
    	}
    	lastimage = lastimage - 2;
    	datafile.close();
    }

    //Video
    if(video){
    	lastimage = cap.get(CV_CAP_PROP_FRAME_COUNT);
    }
    dt_frame = 0;
    //Video

	// File Init
	//Output
	outputfile.open ("/media/DATEN/Extended_PiTag10/Results/blured15/trash/pitag_output.log", std::ios::out | std::ios::app);
	outputtheta.open ("/media/DATEN/Extended_PiTag10/Results/blured15/trash/aa_theta.log", std::ios::out | std::ios::app);
	output_eucdist.open ("/media/DATEN/Extended_PiTag10/Results/blured15/trash/aa_eucdist.log", std::ios::out | std::ios::app);

	//Input
	char s[100];
	datafile.open("/media/DATEN/Extended_PiTag10/MAP/Tag1/a_pitag.log");
	datafile.getline(s,100);
	datafile.getline(s,100);
	//Start from Image number X
	for(unsigned int i = 0; i < imagenumber ;i++){
		datafile.getline(s,100);
	}

	//Camera Parameters
    //AVT GE1050
//	cv::Mat intrinsics = cv::Mat::zeros(3,3,CV_64FC1);
//	intrinsics.at<double>(0,0) = 1269.29;
//	intrinsics.at<double>(0,2) = 512;
//	intrinsics.at<double>(1,1) = 1269.29;
//	intrinsics.at<double>(1,2) = 512;
//	intrinsics.at<double>(2,2) = 1;

//  Kinect
	cv::Mat intrinsics = cv::Mat::zeros(3,3,CV_64FC1);
	intrinsics.at<double>(0,0) = 1050;
	intrinsics.at<double>(0,2) = 640;
	intrinsics.at<double>(1,1) = 1050;
	intrinsics.at<double>(1,2) = 512;
	intrinsics.at<double>(2,2) = 1;

//  Lumia 925
//	cv::Mat intrinsics = cv::Mat::zeros(3,3,CV_64FC1);
//	intrinsics.at<double>(0,0) = 1.154002e+03;
//	intrinsics.at<double>(0,2) = 6.306171e+02;
//	intrinsics.at<double>(1,1) = 1.150629e+03;
//	intrinsics.at<double>(1,2) = 3.643014e+02;
//	intrinsics.at<double>(2,2) = 1;

	setIntrinsicMatrix(intrinsics);
	//Camera Parameters
}

//--------------------------readIMage()-------------------------------------------------------------
cv::Mat FiducialMarkerEvaluationToolkit::readImage(){
	std::stringstream ss;
	if(imagenumber < 10){
		ss << 0 << 0 << 0 << imagenumber;
	}
	if(imagenumber >= 10 && imagenumber < 100){
		ss << 0 << 0 << imagenumber;
	}
	if(imagenumber >= 100 && imagenumber < 1000){
		ss << 0 << imagenumber;
	}
	if(imagenumber >= 1000 && imagenumber < 10000){
		ss << imagenumber;
	}

	//Video
//	cv::Mat frame;
//	cv::Mat dframe;
//	cap >> dframe;
	//distortion coefficience of lumia 925
//	cv::Mat dist = cv::Mat::ones(1,5,CV_64FC1);
//	dist.at<double>(0) = -1.493578e-03;
//	dist.at<double>(1) = -3.417376e-02;
//	dist.at<double>(2) = -4.060949e-03;
//	dist.at<double>(3) = 5.141476e-03;
//	dist.at<double>(4) = 0;
//	cv::undistort(dframe,frame,getIntrinsicMatrix(),dist);
	//Video

	cv::Mat img = cv::imread("/media/DATEN/Extended_PiTag10/MAP/Tag1/pitag" + ss.str() + ".png", CV_LOAD_IMAGE_COLOR);
	//cv::GaussianBlur(img,img,cv::Size(15,15),0,0);

	cv::imshow(WINDOW,img);
	cv::waitKey(10);
	gettimeofday(&start,NULL);
	//return frame;
	return img;
}

//--------------------------readFile()--------------------------------------------------------------
std::vector<double> FiducialMarkerEvaluationToolkit::readFile(){
	char line[1024];
	for(unsigned int i=0;i<1024;i++)
		line[i] = '\n';

	datafile.getline(line,1024);

	//Read Data and write to variables
	int *separators = new int[3];
	int ind = 0;
	for(int i=0;line[i] != ';';i++){
		if(line[i] == '_'){
			separators[ind] = i;
			ind++;
		}
	}

	int var_count = 0;
	int var_amount = 4;
	double *var  = new double(var_amount);
	bool init_reading = false;
	std::vector<char> vtmp;
	for(int i=0;line[i] != ';';i++){
		if(init_reading){
			if(i == separators[var_count]){

				char *tmp = new char(vtmp.size()+1);
				for(size_t j = 0; j < vtmp.size(); j++){
					tmp[j] = vtmp.at(j);
				}

				tmp[vtmp.size()] = '\n';
				var[var_count] = (double)atof(tmp);
				vtmp.clear();
				var_count++;
			} else {
				vtmp.push_back(line[i]);
			}
		}

//		char number_1[1];
//		char number_2[2];
//		char number_3[3];
//		char number_4[4];
		if(line[i] == ':') {
//			if(i == 1){
//				number_1[0] = line[i-1];
//				//imagenumber = std::atoi(number_1);
//			}
//			if(i == 2){
//				number_2[0] = line[i-2];
//				number_2[1] = line[i-1];
//				//imagenumber = std::atoi(number_2);
//			}
//			if(i == 3){
//				number_3[0] = line[i-3];
//				number_3[1] = line[i-2];
//				number_3[2] = line[i-1];
//				//imagenumber = std::atoi(number_3);
//			}
//			if(i == 4){
//				number_4[0] = line[i-4];
//				number_4[1] = line[i-3];
//				number_4[2] = line[i-2];
//				number_4[3] = line[i-1];
//				//imagenumber = std::atoi(number_4);
//			}
			init_reading = true;
		}

	}

	char *htmp = new char(vtmp.size()+1);
	for(size_t j = 0; j < vtmp.size(); j++){
		htmp[j] = vtmp.at(j);
	}
	htmp[vtmp.size()] = '\n';
	//var[3] = (double)atof(htmp);

	std::vector<double> ret;
	ret.push_back((double)0.0);//x
	ret.push_back((double)0.0);//y
	ret.push_back(-(double)atof(htmp)/100);//z
	ret.push_back(var[0]);//rot1
	ret.push_back(var[1]);//rot2
	ret.push_back(var[2]);//rot3


	return ret;
}

//--------------------------writeFile()--------------------------------------------------------------
void FiducialMarkerEvaluationToolkit::writeFile(bool validmatching,std::vector<double> markerpos){


	if(markerpos.size() > 6){
		std::cout << "ERROR:FiducialMarkerEvaluationToolkit::writeFile -> Markerpos is to Big. Should be 6 " << std::endl;
	}

	std::stringstream ss;
	std::stringstream ss_gt;
	std::stringstream ss_theta;
	std::stringstream ss_eucdist;

  	ss_gt << "[" << imagenumber << "]";
  	ss_gt << "[" << "gt"<< "]:";
  	ss_gt << "[" << gtdata[0] << "]";
  	ss_gt << "[" << gtdata[1] << "]";
  	ss_gt << "[" << gtdata[2] << "]";//depth
  	ss_gt << "[" << gtdata[3] << "]";//alpha
  	ss_gt << "[" << gtdata[4] << "]";//beta
  	ss_gt << "[" << gtdata[5] << "]";//gamma
	ss_gt << "\n";
  	outputfile <<  ss_gt.str();

  	if(gtdata[2] > actdepth + 0.1){
  		actdepth = gtdata[2];
  		ss_theta << "\n";
  		ss_eucdist << "\n";
  		outputtheta << ss_theta.str();
  		output_eucdist << ss_eucdist.str();
  	}

	//Fileparser
  	//Write ground truth
	std::vector<double> parse_out;
	for(unsigned int i = 0; i < gtdata.size(); i++)
		parse_out.push_back(gtdata[i]);
	//Filerparser



  	if(validmatching){

  		double distance = sqrt(markerpos[0]*markerpos[0]+markerpos[1]*markerpos[1]+markerpos[2]*markerpos[2]);

  		cv::Mat gt_rot_matrix = eulerToMatrix(gtdata[3],gtdata[4],gtdata[5]);
  		cv::Mat gt_trans(3,1,CV_64FC1);
  		gt_trans.at<double>(0) = 0;
  		gt_trans.at<double>(1) = 0;
  		gt_trans.at<double>(2) = gtdata[2];

  		cv::Mat cp_rot_matrix(3,3,CV_64FC1);
  		cv::Mat cp_rot_vec(3,1,CV_64FC1);
  		cp_rot_vec.at<double>(0) = markerpos[3];
  		cp_rot_vec.at<double>(1) = markerpos[4];
  		cp_rot_vec.at<double>(2) = markerpos[5];
  		cv::Rodrigues(cp_rot_vec,cp_rot_matrix);
  		cv::Mat cp_rot_trans(3,1,CV_64FC1);
  		cp_rot_trans.at<double>(0) = 0;
  		cp_rot_trans.at<double>(1) = 0;
  		cp_rot_trans.at<double>(2) = distance;

  		cv::Mat gt(3,1,CV_64FC1);
  		cv::Mat er(3,1,CV_64FC1);

  		gt = gt_rot_matrix.t()*gt_trans;
  		er = cp_rot_matrix.t()*cp_rot_trans;

  		double pos_error = sqrt((gt.at<double>(0)-er.at<double>(0))*(gt.at<double>(0)-er.at<double>(0)) +
  								 (gt.at<double>(1)-er.at<double>(1))*(gt.at<double>(1)-er.at<double>(1)) +
  								 (gt.at<double>(2)-er.at<double>(2))*(gt.at<double>(2)-er.at<double>(2)));


  		parse_out.push_back(pos_error);
  		//Fileparser
  		//parse_out.push_back(computeEuclideanDistFromVec(markerpos));
  		parse_out.push_back(computeAngleFromVec(markerpos));
  		//Filerparser

  		ss_eucdist << computeEuclideanDistFromVec(markerpos) << " ";
  		output_eucdist << ss_eucdist.str();

  		ss_theta << computeAngleFromVec(markerpos) << " ";
  		outputtheta << ss_theta.str();

  		ss << "[" << imagenumber  << "]";
  		ss << "[" << "error"      << "]:";
  		ss << "[" << markerpos[0] << "]";//trans
  		ss << "[" << markerpos[1] << "]";//trans
  		ss << "[" << markerpos[2] << "]";//trans
  		ss << "[" << markerpos[3] << "]";//rot - rodriguez
  		ss << "[" << markerpos[4] << "]";//rot - rodriguez
  		ss << "[" << markerpos[5] << "]";//rot - rodriguez
  		ss << "\n";
  		outputfile <<  ss.str();

  		ROS_INFO("[fiducials/validationmode] Computed Marker Position (Rodrigues)    x,y,z,r1,r2,r3(%f,%f,%f,%f,%f,%f)",
  				markerpos[0],markerpos[1],markerpos[2],markerpos[3],markerpos[4],markerpos[5]);

  	} else {

  		//Filerparser
  		parse_out.push_back(-666);
  		parse_out.push_back(-666);
  		//Filerparser

  		ss_eucdist << "NaN" << " ";
  		output_eucdist << ss_eucdist.str();

  		ss_theta << "NaN" << " ";
  		outputtheta << ss_theta.str();

  		ss << "[" << imagenumber << "]";
  		ss << "[" << "error" << "]:";
  		ss << "[" << "X" << "]";
  		ss << "[" << "X" << "]";
  		ss << "[" << "X" << "]";
  		ss << "[" << "X" << "]";
  		ss << "[" << "X" << "]";
  		ss << "[" << "X" << "]";
  		ss << "\n";
  		outputfile <<  ss.str();

  		ROS_INFO("[fiducials/validationmode] No marker detected!!!!");
  	}


  	fileparser->writeVector(parse_out);

  	gtdata.empty();
}

//--------------------------frobeniusNorm-----------------------------------------------------------
//Used to calculate the rotation error
double FiducialMarkerEvaluationToolkit::frobeniusNorm(cv::Mat m1,cv::Mat m2){

	double res = 0;
	cv::Mat sum = m1 - m2;

    for (int k=0; k<3; k++)
        for (int l=0; l<3; l++)
        	res += sum.at<double>(k,l)*sum.at<double>(k,l);

    res = sqrt(res);

	return res;
}

//--------------------------frobeniusNorm-----------------------------------------------------------
//Used to calculate the rotation error
double FiducialMarkerEvaluationToolkit::anglefromFrobenius(double fnorm){

	double angle = acos( (1.0/4.0)*(4.0 - (fnorm*fnorm)) );

	return angle;
}

//--------------------------computeAngleFromVec-----------------------------------------------------------
//For better accessibillity... angle in degrees
double FiducialMarkerEvaluationToolkit::computeAngleFromVec(std::vector<double> vec){

	if(vec.size() != 6){
		std::cout << "ERROR:FiducialMarkerEvaluationToolkit::computeAngleFromVec -> vec not in Size. Should be 6 " << std::endl;
		return -1.0;
	}

	cv::Mat gt_rot_matrix = eulerToMatrix(gtdata[3],gtdata[4],gtdata[5]);

	cv::Mat cp_rot_matrix(3,3,CV_64FC1);
	cv::Mat cp_rot_vec(1,3,CV_64FC1);

	cp_rot_vec.at<double>(0) = vec[3];
	cp_rot_vec.at<double>(1) = vec[4];
	cp_rot_vec.at<double>(2) = vec[5];

	cv::Rodrigues(cp_rot_vec,cp_rot_matrix);

	double fnorm = frobeniusNorm(gt_rot_matrix,cp_rot_matrix);
	double min_angle_rad  = anglefromFrobenius(fnorm);
	double min_angle_deg  = min_angle_rad*180/PI;

	return min_angle_deg;
}

//--------------------------computeEuclidieanDist-----------------------------------------------------------
//Translation Error
double FiducialMarkerEvaluationToolkit::computeEuclideanDistFromVec(std::vector<double> vec){

	if(vec.size() != 6){
		std::cout << "ERROR:FiducialMarkerEvaluationToolkit::computeEuclideanDistFromVec -> vec not in Size. Should be 6 " << std::endl;
		return -1.0;
	}

	double dx,dy,dz;

	dx = vec[0]-gtdata[0];
	dy = vec[1]-gtdata[1];
	dz = vec[2]-gtdata[2];
	double euclideandist = sqrt(dx*dx+dy*dy+dz*dz);

	return euclideandist;
}

//--------------------------setIntrinsicMatrix------------------------------------------------------
void FiducialMarkerEvaluationToolkit::setIntrinsicMatrix(cv::Mat intrinsics){
	camera_intrinsics = intrinsics;
}

//--------------------------setPushMarkerPosition----------------------------------------------------
//marker pos must be converted to std::vector<double> 0-2 -> trans(x,y,z), 3-5 -> rot_rodriguez
void FiducialMarkerEvaluationToolkit::setPushMarkerPosition(std::vector<double> marker){
	detectedmarkers_vec.push_back(marker);
}
void FiducialMarkerEvaluationToolkit::setPushMarkerPosition(cv::Mat marker){

	std::vector<double> tmp;
	cv::Mat rot_3x3(3,3,CV_64FC1);


    for (int k=0; k<3; k++)
        for (int l=0; l<3; l++)
        	rot_3x3.at<double>(k,l) = marker.at<double>(k,l);

	cv::Mat rot_1x3(1,3,CV_64FC1);
	cv::Rodrigues(rot_3x3,rot_1x3);

	tmp.push_back(marker.at<double>(0,3));
	tmp.push_back(marker.at<double>(1,3));
	tmp.push_back(marker.at<double>(2,3));
	tmp.push_back(rot_1x3.at<double>(0));
	tmp.push_back(rot_1x3.at<double>(1));
	tmp.push_back(rot_1x3.at<double>(2));

	detectedmarkers_vec.push_back(tmp);
}

//--------------------------setStartAnalysis------------------------------------------------------

void FiducialMarkerEvaluationToolkit::setStartAnalysis() {

	timeval end;
	gettimeofday(&end,NULL);
	dt_frame = end.tv_usec-start.tv_usec;
	dif += dt_frame;


	int bestmatch = -1;

	for(unsigned int i = 0; i < detectedmarkers_vec.size();i++){

		if(bestmatch == -1){
			bestmatch = i;
			continue;
		}

		//Compare translation with bestmatch (bm)
		double dx = detectedmarkers_vec[i][0]-gtdata[0];
		double dy = detectedmarkers_vec[i][1]-gtdata[1];
		double dz = detectedmarkers_vec[i][2]-gtdata[2];
		double euclidian_norm = sqrt(dx*dx+dy*dy+dz*dz);

		double bm_dx = detectedmarkers_vec[bestmatch][0]-gtdata[0];
		double bm_dy = detectedmarkers_vec[bestmatch][1]-gtdata[1];
		double bm_dz = detectedmarkers_vec[bestmatch][2]-gtdata[2];
		double bm_euclidian_norm = sqrt(bm_dx*bm_dx+bm_dy*bm_dy+bm_dz*bm_dz);

		if(euclidian_norm < bm_euclidian_norm){
			//We've got a new best match!
			bestmatch = i;
		}
	}

	//Write only bestmatch to File
	bool validmatch = false;
	std::vector<double> computedmarker;

	//Marker detected?
	if(bestmatch >= 0){

		//If there is more than one marker detected -> Increase False Positives
		for(unsigned int i = 0;i < detectedmarkers_vec.size()-1;i++ ){
			fp++;
			falsepos.push_back(imagenumber);
		}

		double bm_dx = detectedmarkers_vec[bestmatch][0]-gtdata[0];
		double bm_dy = detectedmarkers_vec[bestmatch][1]-gtdata[1];
		double bm_dz = detectedmarkers_vec[bestmatch][2]-gtdata[2];
		double bm_euclidian_norm = sqrt(bm_dx*bm_dx+bm_dy*bm_dy+bm_dz*bm_dz);

		//Translation Treshold - When is a marker a valid match
		if(bm_euclidian_norm < 0.10){

			tp++;
			validmatch = true;
			for(unsigned int i = 0; i < 6; i++){
				computedmarker.push_back(detectedmarkers_vec[bestmatch][i]);
			}
		} else {
			fp++;
			falsepos.push_back(imagenumber);
		}

	} else {
		fn++;
		falsenegatives.push_back(imagenumber);
	}


	writeFile(validmatch,computedmarker);

	imagenumber++;
}


//--------------------------getIntrinsicMatrix------------------------------------------------------
cv::Mat FiducialMarkerEvaluationToolkit::getIntrinsicMatrix(){
	return camera_intrinsics;
}

//--------------------------getNextMarker-----------------------------------------------------------
cv::Mat FiducialMarkerEvaluationToolkit::getNextMarker(){

	if(imagenumber >= (unsigned int)lastimage){ // Do stuff before programm exits.
		//Fileparser
		fileparser->closeArray();
		fileparser->close();

		//Fileparser
		outputfile.close();
		outputtheta.close();
		output_eucdist.close();
		double time_per_marker = dif/(lastimage+1);
		std::cout << "ImageNumber: " << imagenumber-1 << std::endl;
		std::cout << "True Positive: " << tp << std::endl;
		std::cout << "False Positive: " << fp << std::endl;
		std::cout << "False Negative: " << fn << std::endl;
		std::cout << "Used Time in seconds : " << time_per_marker/1000 << std::endl;

//		for(unsigned int i = 0; i < falsepos.size();i++){
//			std::cout << "False Positive Images: " << falsepos[i] << std::endl;
//		}

//		for(unsigned int i = 0; i < falsenegatives.size();i++){
//			std::cout << "False Negative Images: " << falsenegatives[i] << std::endl;
//		}

		if(debug){
			cv::waitKey(100);
			std::cin.get();
		}

		cv::waitKey(1000);
		exit(0);
	}

	//Video
	std::cout << "ImageNumber: " << imagenumber-1 << std::endl;
	std::cout << "True Positive: " << tp << std::endl;
	std::cout << "False Positive: " << fp << std::endl;
	std::cout << "False Negative: " << fn << std::endl;
	//Video

	detectedmarkers_vec.clear();

	gtdata = readFile();
	ROS_INFO("-------------------------------------------------------------------------------------------");
	ROS_INFO("[fiducials/validationmode] Ground Truth Marker Position x,y,z,r1,r2,r3(%f,%f,%f,%f,%f,%f)",gtdata[0],gtdata[1],gtdata[2],gtdata[3],gtdata[4],gtdata[5]);//EULER ANGLES

	return readImage();
}

//-------------------------------------------------------------------------------------------------
//---------------------------additional Stuff------------------------------------------------------
//-------------------------------------------------------------------------------------------------
inline float SIGN(float x)
{
	return (x >= 0.0f) ? +1.0f : -1.0f;
}

// Matrix -> Quaternion conversion
std::vector<double> FiducialMarkerEvaluationToolkit::matrixToQuat(cv::Mat frame)
{
	// [0]-[2]: translation xyz
	// [3]-[6]: quaternion wxyz
	std::vector<double> pose(4, 0.0);

	double r11 = frame.at<double>(0,0);
	double r12 = frame.at<double>(0,1);
	double r13 = frame.at<double>(0,2);
	double r21 = frame.at<double>(1,0);
	double r22 = frame.at<double>(1,1);
	double r23 = frame.at<double>(1,2);
	double r31 = frame.at<double>(2,0);
	double r32 = frame.at<double>(2,1);
	double r33 = frame.at<double>(2,2);

	double qw = ( r11 + r22 + r33 + 1.0) / 4.0;
	double qx = ( r11 - r22 - r33 + 1.0) / 4.0;
	double qy = (-r11 + r22 - r33 + 1.0) / 4.0;
	double qz = (-r11 - r22 + r33 + 1.0) / 4.0;
	if(qw < 0.0f) qw = 0.0;
	if(qx < 0.0f) qx = 0.0;
	if(qy < 0.0f) qy = 0.0;
	if(qz < 0.0f) qz = 0.0;
	qw = std::sqrt(qw);
	qx = std::sqrt(qx);
	qy = std::sqrt(qy);
	qz = std::sqrt(qz);
	if(qw >= qx && qw >= qy && qw >= qz)
	{
		qw *= +1.0;
		qx *= SIGN(r32 - r23);
		qy *= SIGN(r13 - r31);
		qz *= SIGN(r21 - r12);
	}
	else if(qx >= qw && qx >= qy && qx >= qz)
	{
		qw *= SIGN(r32 - r23);
		qx *= +1.0;
		qy *= SIGN(r21 + r12);
		qz *= SIGN(r13 + r31);
	}
	else if(qy >= qw && qy >= qx && qy >= qz)
	{
		qw *= SIGN(r13 - r31);
		qx *= SIGN(r21 + r12);
		qy *= +1.0;
		qz *= SIGN(r32 + r23);
	}
	else if(qz >= qw && qz >= qx && qz >= qy)
	{
		qw *= SIGN(r21 - r12);
		qx *= SIGN(r31 + r13);
		qy *= SIGN(r32 + r23);
		qz *= +1.0;
	}
	else
	{
		printf("coding error\n");
	}
	double r = std::sqrt(qw*qw + qx*qx + qy*qy + qz*qz);
	qw /= r;
	qx /= r;
	qy /= r;
	qz /= r;

	pose[0] = qw;
	pose[1] = qx;
	pose[2] = qy;
	pose[3] = qz;

	return pose;
}

cv::Mat FiducialMarkerEvaluationToolkit::eulerToMatrix(double rot1, double rot2, double rot3) {

	cv::Mat m(3,3,CV_64FC1);

	cv::Mat X_rot(3,3,CV_64F);
	cv::Mat Y_rot(3,3,CV_64F);
	cv::Mat Z_rot(3,3,CV_64F);

//	double alpha =   rot1 + PI;
//	double beta  =  -rot2 + PI;
//	double gamma =  -rot3 + PI;

	double alpha =  rot1 - PI;
	double beta  =  rot2;
	double gamma =  rot3;

	//Achtung auf Rotationsreihenfolge achten....  sonst stimmt die Darstellung nicht
	X_rot.at<double>(0,0) =  1.0;
	X_rot.at<double>(0,1) =  0.0;
	X_rot.at<double>(0,2) =  0.0;
	X_rot.at<double>(1,0) =  0.0;
	X_rot.at<double>(1,1) =  cos(alpha);
	X_rot.at<double>(1,2) =  -sin(alpha);
	X_rot.at<double>(2,0) =  0.0;
	X_rot.at<double>(2,1) =  sin(alpha);
	X_rot.at<double>(2,2) =  cos(alpha);

	Y_rot.at<double>(0,0) =  cos(beta);
	Y_rot.at<double>(0,1) =  0.0;
	Y_rot.at<double>(0,2) =  sin(beta);
	Y_rot.at<double>(1,0) =  0.0;
	Y_rot.at<double>(1,1) =  1.0;
	Y_rot.at<double>(1,2) =  0.0;
	Y_rot.at<double>(2,0) =  -sin(beta);
	Y_rot.at<double>(2,1) =  0.0;
	Y_rot.at<double>(2,2) =  cos(beta);

	Z_rot.at<double>(0,0) =  cos(gamma);
	Z_rot.at<double>(0,1) =  -sin(gamma);
	Z_rot.at<double>(0,2) =  0.0;
	Z_rot.at<double>(1,0) =  sin(gamma);
	Z_rot.at<double>(1,1) =  cos(gamma);
	Z_rot.at<double>(1,2) =  0.0;
	Z_rot.at<double>(2,0) =  0.0;
	Z_rot.at<double>(2,1) =  0.0;
	Z_rot.at<double>(2,2) =  1.0;

	m = X_rot*Z_rot;

	return m;
}

//-------------Visualization-----------------------
//I think thi stuff is not used... Kick it out if your sure
//unsigned long FiducialMarkerEvaluationToolkit::RenderPose(cv::Mat& image, cv::Mat& rot_3x3_CfromO, cv::Mat& trans_3x1_CfromO)
//{
//	cv::Mat object_center(3, 1, CV_64FC1);
//	double* p_object_center = object_center.ptr<double>(0);
//	p_object_center[0] = 0;
//	p_object_center[1] = 0;
//	p_object_center[2] = 0;
//
//	cv::Mat rot_inv = rot_3x3_CfromO.inv();
//
//	// Compute coordinate axis for visualization
//	cv::Mat pt_axis(4, 3, CV_64FC1);
//	double* p_pt_axis = pt_axis.ptr<double>(0);
//	p_pt_axis[0] = 0 + p_object_center[0];
//	p_pt_axis[1] = 0 + p_object_center[1];
//	p_pt_axis[2] = 0 + p_object_center[2];
//	p_pt_axis = pt_axis.ptr<double>(1);
//	p_pt_axis[0] = 0.1 + p_object_center[0];
//	p_pt_axis[1] = 0 + p_object_center[1];
//	p_pt_axis[2] = 0 + p_object_center[2];
//	p_pt_axis = pt_axis.ptr<double>(2);
//	p_pt_axis[0] = 0 + p_object_center[0];
//	p_pt_axis[1] = 0.1 + p_object_center[1];
//	p_pt_axis[2] = 0 + p_object_center[2];
//	p_pt_axis = pt_axis.ptr<double>(3);
//	p_pt_axis[0] = 0 + p_object_center[0];
//	p_pt_axis[1] = 0 + p_object_center[1];
//	p_pt_axis[2] = 0.1 + p_object_center[2];
//
//	// Transform data points
//	std::vector<cv::Point> vec_2d(4, cv::Point());
//	for (int i=0; i<4; i++)
//	{
//		cv::Mat vec_3d = pt_axis.row(i).clone();
//		vec_3d = vec_3d.t();
//		vec_3d = rot_3x3_CfromO*vec_3d;
//		vec_3d += trans_3x1_CfromO;
//		double* p_vec_3d = vec_3d.ptr<double>(0);
//
//		ReprojectXYZ(p_vec_3d[0], p_vec_3d[1], p_vec_3d[2],
//			vec_2d[i].x , vec_2d[i].y);
//	}
//
//	// Render results
//	int line_width = 1;
//	cv::line(image, vec_2d[0], vec_2d[1], cv::Scalar(0, 0, 255), line_width);
//	cv::line(image, vec_2d[0], vec_2d[2], cv::Scalar(0, 255, 0), line_width);
//	cv::line(image, vec_2d[0], vec_2d[3], cv::Scalar(255, 0, 0), line_width);
//
//	return 1;
//}
//
//unsigned long FiducialMarkerEvaluationToolkit::ReprojectXYZ(double x, double y, double z, int& u, int& v)
//{
//	cv::Mat XYZ(3, 1, CV_64FC1);
//	cv::Mat UVW(3, 1, CV_64FC1);
//
//	double* d_ptr = 0;
//	double du = 0;
//	double dv = 0;
//	double dw = 0;
//
//	x *= 1000;
//	y *= 1000;
//	z *= 1000;
//
//	d_ptr = XYZ.ptr<double>(0);
//	d_ptr[0] = x;
//	d_ptr[1] = y;
//	d_ptr[2] = z;
//
//	UVW = camera_intrinsics * XYZ;
//
//	d_ptr = UVW.ptr<double>(0);
//	du = d_ptr[0];
//	dv = d_ptr[1];
//	dw = d_ptr[2];
//
//	u = cvRound(du/dw);
//	v = cvRound(dv/dw);
//
//	return 1;
//}
