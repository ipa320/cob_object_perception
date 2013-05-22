#ifndef __IPA_ABSTRACT_FIDUCIAL_MODEL_H__
#define __IPA_ABSTRACT_FIDUCIAL_MODEL_H__

//#include "../../../../../cob_object_perception_intern/windows/src/PreCompiledHeaders/StdAfx.h"
#ifdef __LINUX__
	#include "cob_vision_utils/VisionUtils.h"
	#include "cob_fiducials/FiducialDefines.h"

	#include "tinyxml.h"
#else
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/VisionUtils.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/FiducialDefines.h"

	#include "cob_object_perception_intern/windows/src/extern/TinyXml/tinyxml.h"	
#endif

#include <boost/shared_ptr.hpp>
#include <opencv/cv.h>

namespace ipa_Fiducials
{

/// Define smart pointer type for object model
class AbstractFiducialModel;
typedef boost::shared_ptr<AbstractFiducialModel> AbstractFiducialModelPtr;
/// @class FiducialModelPi
///
/// A concrete class to represent a fiducial
class __DLL_LIBFIDUCIALS__ AbstractFiducialModel
{
public:

	//*******************************************************************************
	// AbstractFiducial interface implementation
	//*******************************************************************************
	virtual ~AbstractFiducialModel(){};

	unsigned long Init(cv::Mat& camera_matrix, std::string directory_and_filename, cv::Mat extrinsic_matrix = cv::Mat())
	{
		if (camera_matrix.empty())
		{
			std::cerr << "ERROR - AbstractFiducialModel::Init" << std::endl;
			std::cerr << "\t [FAILED] Camera matrix not initialized" << std::endl;
			return ipa_Utils::RET_FAILED;
		}
		m_camera_matrix = camera_matrix.clone();

		m_extrinsic_XYfromC = cv::Mat::zeros(4, 4, CV_64FC1);
		if (extrinsic_matrix.empty())
		{
			// Unit matrix
			for (int i=0; i<3; i++)
				m_extrinsic_XYfromC.at<double>(i,i) = 1.0;
		}
		else
		{
			for (int i=0; i<3; i++)
				for (int j=0; j<4; j++)
				{
					m_extrinsic_XYfromC.at<double>(i,j) = extrinsic_matrix.at<double>(i,j);
				}
			m_extrinsic_XYfromC.at<double>(3,3) = 1.0;
		}
		
		return LoadParameters(directory_and_filename);
	};

	unsigned long ApplyExtrinsics(cv::Mat& rot_CfromO, cv::Mat& trans_CfromO)
	{
		cv::Mat frame_CfromO = cv::Mat::zeros(4, 4, CV_64FC1);

		// Copy ORIGINAL rotation and translation to frame
		for (int i=0; i<3; i++)
		{
			frame_CfromO.at<double>(i, 3) = trans_CfromO.at<double>(i,0);
			for (int j=0; j<3; j++)
			{
				frame_CfromO.at<double>(i,j) = rot_CfromO.at<double>(i,j);
			}
		}
		frame_CfromO.at<double>(3,3) = 1.0;

		cv::Mat frame_XYfromO = m_extrinsic_XYfromC * frame_CfromO;

		// Copy MODIFIED rotation and translation to frame
		for (int i=0; i<3; i++)
		{
			trans_CfromO.at<double>(i, 0) = frame_XYfromO.at<double>(i,3);
			for (int j=0; j<3; j++)
			{
				rot_CfromO.at<double>(i,j) = frame_XYfromO.at<double>(i,j);
			}
		}

		return ipa_Utils::RET_OK;
	}

	/// Locates the fiducial within the image and inferes the camera pose from it
	/// @param image scene image
	/// @param vec_pose Vector of poses from all detected tags relative to the camera
	/// @return <code>RET_FAILED</code> if no tag could be detected
	/// <code>RET_OK</code> on success
	virtual unsigned long GetPose(cv::Mat& image, std::vector<t_pose>& vec_pose_CfromO) = 0;

	// Gets the camera matrix
	// @return 3x3 camera matrix (fx 0 cx, 0 fy cy, 0 0 1) 
	cv::Mat GetCameraMatrix()
	{
		return m_camera_matrix;
	};

	// Set the camera matrix
	// @param camera_matrix 3x3 camera matrix (fx 0 cx, 0 fy cy, 0 0 1) 
	unsigned long SetCameraMatrix(cv::Mat camera_matrix)
	{
		m_camera_matrix = camera_matrix.clone();
		return ipa_Utils::RET_OK;
	}

	// Gets the distortion coeffs
	// @return 1x4 distortion coeffs matrix (k1,k2,p1,p2)
	cv::Mat GetDistortionCoeffs()
	{
		if (m_dist_coeffs.empty())
			return cv::Mat::zeros(1, 4, CV_64FC1);
		return m_dist_coeffs;
	};

	// Sets the distortion coeffs
	// @param dist_coeffs 1x4 distortion coeffs matrix (k1,k2,p1,p2)
	unsigned long SetDistortionCoeffs(cv::Mat dist_coeffs)
	{
		m_dist_coeffs = dist_coeffs.clone();
		return ipa_Utils::RET_OK;
	}

	/// Load fiducial-centric coordinates of markers from file
	/// @param directory_and_filename Path to XML filename, where the parameters of all fiducials are stores.
	/// When using ROS, this function is replaced by parsing a launch file
	virtual unsigned long LoadParameters(std::string directory_and_filename) = 0;

	/// Return the name (fiducial type) of the detector
	/// @return fiducial type
	virtual std::string GetType() = 0;
private:

	cv::Mat m_camera_matrix; ///< Intrinsics of camera for PnP estimation
	cv::Mat m_dist_coeffs; ///< Intrinsics of camera for PnP estimation
	cv::Mat m_extrinsic_XYfromC; ///< Extrinsics 4x4 of camera to rotate and translate determined transformation before returning it
};

} // end namespace ipa_Fiducials

#endif // __IPA_ABSTRACT_FIDUCIAL_MODEL_H__
