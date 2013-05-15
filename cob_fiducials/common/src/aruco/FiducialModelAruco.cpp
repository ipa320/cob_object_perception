//#include "../../../../cob_object_perception_intern/windows/src/PreCompiledHeaders/StdAfx.h"
#ifdef __LINUX__
	#include "cob_fiducials/FiducialModelAruco.h"
#else
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/aruco/FiducialModelAruco.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/aruco/arucofidmarkers.h"
#endif
#include <opencv/highgui.h>

using namespace ipa_Fiducials;


FiducialModelAruco::FiducialModelAruco()
{
	int pyrDownLevels = 0;
	int speed_level = 0; //0: slow but exact, 1: fast, but inaccurate
	m_marker_size=0.07;

	//Configure other parameters
    if (pyrDownLevels>0)
        MDetector.pyrDown(pyrDownLevels);

	MDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
	MDetector.setDesiredSpeed(0);
}

FiducialModelAruco::~FiducialModelAruco()
{
	
}


unsigned long FiducialModelAruco::GetPose(cv::Mat& image, std::vector<t_pose>& vec_pose)
{
	std::vector<aruco::Marker> markers;
	aruco::CameraParameters camera_parameters;
	camera_parameters.setParams(GetCameraMatrix(), GetDistortionCoeffs(), image.size());

	try
	{
		MDetector.detect(image, markers, camera_parameters, m_marker_size);
	} 
	catch(std::exception &ex)
	{
		std::cout << "Exception: " << ex.what() << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	for (int i=0; i<markers.size(); i++)
	{
		t_pose pose;
		
		// Apply transformation
		cv::Mat trans_3x1_CfromO = markers[i].Tvec.clone();
		cv::Mat rot_3x3_CfromO;
		cv::Rodrigues(markers[i].Rvec, rot_3x3_CfromO);

		cv::Mat rot_3x3_CfromO_D;
		cv::Mat trans_3x1_CfromO_D;
		rot_3x3_CfromO.convertTo(rot_3x3_CfromO_D, CV_64FC1);
		trans_3x1_CfromO.convertTo(trans_3x1_CfromO_D, CV_64FC1);

		ApplyExtrinsics(rot_3x3_CfromO_D, trans_3x1_CfromO_D);

		rot_3x3_CfromO_D.copyTo(pose.rot);
		trans_3x1_CfromO_D.copyTo(pose.trans);

		vec_pose.push_back(pose);
	}

	return ipa_Utils::RET_OK;
}

unsigned long FiducialModelAruco::LoadParameters(std::vector<FiducialArucoParameters> pi_tags)
{
	
	return ipa_Utils::RET_OK;
}

unsigned long FiducialModelAruco::LoadParameters(std::string directory_and_filename)
{
	

	return ipa_Utils::RET_OK;
}


