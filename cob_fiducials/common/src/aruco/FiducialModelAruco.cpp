//#include "../../../../../cob_object_perception_intern/windows/src/PreCompiledHeaders/StdAfx.h"
#ifdef __LINUX__
	#include "cob_fiducials/aruco/FiducialModelAruco.h"
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

	//Configure detector
	m_detector = boost::shared_ptr<aruco::MarkerDetector>(new aruco::MarkerDetector());
	if (pyrDownLevels>0)
		m_detector->pyrDown(pyrDownLevels);

	m_detector->setCornerRefinementMethod(aruco::MarkerDetector::LINES);
	m_detector->setDesiredSpeed(speed_level);

	// There are at most 1024 different marker
	for (int i=0; i<1024; i++)
	{
		FiducialArucoParameters params;
		params.m_isInit = false;
		m_tag_parameters.push_back(params);
	}
}

FiducialModelAruco::~FiducialModelAruco()
{
	
}


unsigned long FiducialModelAruco::GetPose(cv::Mat& image, std::vector<t_pose>& vec_pose)
{
	std::vector<aruco::Marker> markers;
	aruco::CameraParameters camera_parameters;
	camera_parameters.setParams(GetCameraMatrix(), GetDistortionCoeffs(), image.size());

	m_detector = boost::shared_ptr<aruco::MarkerDetector>(new aruco::MarkerDetector());


	try
	{
		m_detector->detect(image, markers, camera_parameters, -1);
	} 
	catch(std::exception &ex)
	{
		std::cout << "Exception: " << ex.what() << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// ------------ Compute pose --------------------------------------

	for (unsigned int i=0; i<markers.size(); i++)
	{
		int nPoints = markers[i].size();	
		int id = markers[i].id;
		cv::Mat pattern_coords(nPoints, 3, CV_32F);
		cv::Mat image_coords(nPoints, 2, CV_32F);
		double halfSize=m_marker_size/2.;
		cv::Point2d offset(0.0, 0.0); 

		// Local parameters
		if (m_tag_parameters[id].m_isInit != 0)
		{
			offset = m_tag_parameters[id].m_offset;
			halfSize = m_tag_parameters[id].m_line_width_height;
		}

		pattern_coords.at<float>(1,0)=-halfSize + offset.x;
		pattern_coords.at<float>(1,1)=halfSize + offset.y;
		pattern_coords.at<float>(1,2)=0;
		pattern_coords.at<float>(2,0)=halfSize + offset.x;
		pattern_coords.at<float>(2,1)=halfSize + offset.y;
		pattern_coords.at<float>(2,2)=0;
		pattern_coords.at<float>(3,0)=halfSize + offset.x;
		pattern_coords.at<float>(3,1)=-halfSize + offset.y;
		pattern_coords.at<float>(3,2)=0;
		pattern_coords.at<float>(0,0)=-halfSize + offset.x;
		pattern_coords.at<float>(0,1)=-halfSize + offset.y;
		pattern_coords.at<float>(0,2)=0;
		
		float* p_image_coords = 0;
		for (int j=0; j<nPoints; j++)
		{
			p_image_coords = image_coords.ptr<float>(j);
			p_image_coords[0] = markers[i][j].x;
			p_image_coords[1] = markers[i][j].y;
		}

		t_pose tag_pose;
		tag_pose.id = id;
		cv::solvePnP(pattern_coords, image_coords, GetCameraMatrix(), GetDistortionCoeffs(), 
			tag_pose.rot, tag_pose.trans);
		
		//rotate the X axis so that Y is perpendicular to the marker plane
		bool setYPerperdicular = true;
		if (setYPerperdicular)
		{
			cv::Mat R(3,3,CV_64F);
			cv::Rodrigues(tag_pose.rot, R);
			//create a rotation matrix for x axis
			cv::Mat RX=cv::Mat::eye(3,3,CV_64F);
			double my_pi = 3.14159265359;
			float angleRad=my_pi/2;
			RX.at<float>(1,1)=cos(angleRad);
			RX.at<float>(1,2)=-sin(angleRad);
			RX.at<float>(2,1)=sin(angleRad);
			RX.at<float>(2,2)=cos(angleRad);
			//now multiply
			R=R*RX;
			//finally, the the rodrigues back
			cv::Rodrigues(R,tag_pose.rot);
		}

		// Apply transformation
		cv::Mat rot_3x3_CfromO;
		cv::Rodrigues(tag_pose.rot, rot_3x3_CfromO);

		//if (!ProjectionValid(rot_3x3_CfromO, tag_pose.trans, GetCameraMatrix(), pattern_coords, image_coords))
		//	continue;

		ApplyExtrinsics(rot_3x3_CfromO, tag_pose.trans);
		rot_3x3_CfromO.copyTo(tag_pose.rot);
		vec_pose.push_back(tag_pose);
	}

	// When using the aruco internal computation
	//for (int i=0; i<markers.size(); i++)
	//{
	//	t_pose pose;
	//	pose.id = markers[i].id;
	//	
	//	// Apply transformation
	//	cv::Mat trans_3x1_CfromO = markers[i].Tvec.clone();
	//	cv::Mat rot_3x3_CfromO;
	//	cv::Rodrigues(markers[i].Rvec, rot_3x3_CfromO);

	//	cv::Mat rot_3x3_CfromO_D;
	//	cv::Mat trans_3x1_CfromO_D;
	//	rot_3x3_CfromO.convertTo(rot_3x3_CfromO_D, CV_64FC1);
	//	trans_3x1_CfromO.convertTo(trans_3x1_CfromO_D, CV_64FC1);

	//	ApplyExtrinsics(rot_3x3_CfromO_D, trans_3x1_CfromO_D);

	//	rot_3x3_CfromO_D.copyTo(pose.rot);
	//	trans_3x1_CfromO_D.copyTo(pose.trans);

	//	vec_pose.push_back(pose);
	//}

	return ipa_Utils::RET_OK;
}

unsigned long FiducialModelAruco::LoadParameters(std::vector<FiducialArucoParameters> aruco_tags)
{
	if (m_marker_size == -1)
	{
		std::cerr << "FiducialModelAruco::LoadParameters:" << std::endl;
		std::cerr << "\t ... [ERROR] Global Xml-tag 'LineWidthHeight' not set" << std::endl;
		std::cerr << "\t ... [ERROR] Aborting" << std::endl;
		return ipa_Utils::RET_FAILED;
	}


	for (unsigned int i=0; i<aruco_tags.size(); i++)
	{
		if (aruco_tags[i].m_id<m_tag_parameters.size())
		{
			m_tag_parameters[aruco_tags[i].m_id] = aruco_tags[i];
		}
		else
		{
			std::cerr << "FiducialModelAruco::LoadParameters:" << std::endl;
			std::cerr << "\t ... [ERROR] Tag ID exceeds vector index" << std::endl;
			std::cerr << "\t ... [ERROR] Aborting" << std::endl;
			return ipa_Utils::RET_FAILED;
		}
	}

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
				double marker_size = -1;
				bool localParameter = false;


//************************************************************************************
//  LOCAL PARAMETERS

//************************************************************************************
//	BEGIN FiducialDetector->Aruco->ID
//************************************************************************************
				// Subtag element "ObjectDetectorParameters" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_FI->FirstChildElement( "ID" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &aruco_parameters.m_id) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'ID'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}

					localParameter = true;
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
					if ( p_xmlElement_Child->QueryValueAttribute( "x", &aruco_parameters.m_offset.x) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'x' of tag 'Offset'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}

					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "y", &aruco_parameters.m_offset.y) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'y' of tag 'Offset'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else if (localParameter)
				{
					std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Offset'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}

//************************************************************************************
//  GLOBAL PARAMETERS

//************************************************************************************
//	BEGIN FiducialDetector->Aruco->LineWidthHeight
//************************************************************************************
				// Subtag element "ObjectDetectorParameters" of Xml Inifile
				
				p_xmlElement_Child = p_xmlElement_Root_FI->FirstChildElement( "LineWidthHeight" );

				
				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &marker_size) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - FiducialArucoParameters::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'LineWidthHeight'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}

//************************************************************************************
				if (localParameter)
				{
					aruco_parameters.m_line_width_height = marker_size;
					vec_aruco_parameters.push_back(aruco_parameters);
				}
				else
				{
					m_marker_size = marker_size;
				}

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


