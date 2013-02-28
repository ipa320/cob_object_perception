#ifndef __IPA_ABSTRACT_FIDUCIAL_MODEL_H__
#define __IPA_ABSTRACT_FIDUCIAL_MODEL_H__

#include "../../../../../cob_object_perception_intern/windows/src/PreCompiledHeaders/StdAfx.h"
#ifdef __LINUX__
	#include "cob_vision_utils/VisionUtils.h"
	#include "cob_fiducials/FiducialDefines.h"
#else
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/VisionUtils.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/FiducialDefines.h"
#endif


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

	unsigned long Init(cv::Mat& camera_matrix, std::string directory_and_filename)
	{
		m_camera_matrix = camera_matrix.clone();
		return LoadParameters(directory_and_filename);
	};

	/// Locates the fiducial within the image and inferes the camera pose from it
	/// @param image scene image
	/// @param vec_pose Vector of poses from all detected tags
	/// @return <code>RET_FAILED</code> if no tag could be detected
	/// <code>RET_OK</code> on success
	virtual unsigned long GetPose(cv::Mat& image, std::vector<t_pose>& vec_pose) = 0;

	cv::Mat GetCameraMatrix()
	{
		return m_camera_matrix;
	};

	unsigned long SetCameraMatrix(cv::Mat camera_matrix)
	{
		m_camera_matrix = camera_matrix.clone();
		return ipa_Utils::RET_OK;
	}

	/// Load fiducial-centric coordinates of markers from file
	/// @param directory_and_filename Path to XML filename, where the parameters of all fiducials are stores.
	/// When using ROS, this function is replaced by parsing a launch file
	virtual unsigned long LoadParameters(std::string directory_and_filename) = 0;

private:

	cv::Mat m_camera_matrix; ///< Intrinsics of camera for PnP estimation
};

} // end namespace ipa_Fiducials

#endif // __IPA_ABSTRACT_FIDUCIAL_MODEL_H__
