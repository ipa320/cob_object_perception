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
	m_marker_size=-1;

	//Configure other parameters
	if (pyrDownLevels>0)
		MDetector.pyrDown(pyrDownLevels);

	MDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
	MDetector.setDesiredSpeed(speed_level);
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
		pose.id = markers[i].id;
		
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

unsigned long FiducialModelAruco::LoadParameters(std::vector<FiducialArucoParameters> aruco_tags)
{
	m_tag_parameters = aruco_tags;
	
	return ipa_Utils::RET_OK;
}

unsigned long FiducialModelAruco::LoadParameters(std::string directory_and_filename)
{
	std::vector<FiducialArucoParameters> vec_aruco_parameters;
	std::string tempString = "";
	// Load parameters from file
	boost::shared_ptr<TiXmlDocument> p_configXmlDocument (new TiXmlDocument( directory_and_filename ));

	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file (Check filename and syntax of the file):" << std::endl;
		std::cerr << "\t ... '" << directory_and_filename << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	std::cout << "INFO - FiducialArucoParameters::LoadParameters:" << std::endl;
	std::cout << "\t ... Parsing xml configuration file:" << std::endl;
	std::cout << "\t ... " << directory_and_filename << std::endl;

	m_marker_size = -1;

	if ( p_configXmlDocument )
	{

//************************************************************************************
//	BEGIN FiducialDetector
//************************************************************************************
		// Tag element "ObjectDetector" of Xml Inifile
		TiXmlElement *p_xmlElement_Root = NULL;
		p_xmlElement_Root = p_configXmlDocument->FirstChildElement( "FiducialDetector" );

		if ( p_xmlElement_Root )
		{

//************************************************************************************
//	BEGIN FiducialDetector->Aruco
//************************************************************************************
			// Tag element "ObjectDetectorParameters" of Xml Inifile

			for(TiXmlElement* p_xmlElement_Root_FI = p_xmlElement_Root->FirstChildElement("Aruco"); 
				p_xmlElement_Root_FI != NULL; 
				p_xmlElement_Root_FI = p_xmlElement_Root_FI->NextSiblingElement("Aruco"))
			{
				FiducialArucoParameters aruco_parameters;

//************************************************************************************
//	BEGIN FiducialDetector->Aruco->LineWidthHeight
//************************************************************************************
				// Subtag element "ObjectDetectorParameters" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_FI->FirstChildElement( "LineWidthHeight" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_marker_size) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'LineWidthHeight'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}

					continue;
				}

//************************************************************************************





//************************************************************************************
//	BEGIN FiducialDetector->Aruco->ID
//************************************************************************************
				// Subtag element "ObjectDetectorParameters" of Xml Inifile
				p_xmlElement_Child = p_xmlElement_Root_FI->FirstChildElement( "ID" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &aruco_parameters.id) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'ID'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'ID'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}

//************************************************************************************
//	BEGIN FiducialDetector->Aruco->Offset
//************************************************************************************
				// Subtag element "ObjectDetectorParameters" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_FI->FirstChildElement( "Offset" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "x", &aruco_parameters.offset.x) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'x' of tag 'Offset'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}

					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "y", &aruco_parameters.offset.y) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'y' of tag 'Offset'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Offset'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}

				vec_aruco_parameters.push_back(aruco_parameters);

//************************************************************************************
//	END FiducialDetector->Fiducial
//************************************************************************************
			}
			
			if (vec_aruco_parameters.empty())
			{
				std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
				std::cerr << "\t ... Could't find tag 'Aruco'" << std::endl;
				return ipa_Utils::RET_FAILED;
			}

		}
//************************************************************************************
//	END FiducialDetector
//************************************************************************************
		else
		{
			std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'FiducialDetector'" << std::endl;
			return ipa_Utils::RET_FAILED;
		}
	}

	if (LoadParameters(vec_aruco_parameters) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
		std::cerr << "\t ... Couldn't set tag parameters'" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}


