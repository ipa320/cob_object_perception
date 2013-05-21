#ifndef __IPA_FIDUCIAL_MODEL_ARUCO_H__
#define __IPA_FIDUCIAL_MODEL_ARUCO_H__

//#include "../../../../../cob_object_perception_intern/windows/src/PreCompiledHeaders/StdAfx.h"
#ifdef __LINUX__
	#include "cob_vision_utils/VisionUtils.h"
	#include "cob_fiducials/FiducialDefines.h"
	#include "cob_fiducials/AbstractFiducialModel.h"
	#include "cob_fiducials/aruco/FiducialArucoParameters.h"
	#include "cob_fiducials/aruco/aruco.h"
#else
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/VisionUtils.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/FiducialDefines.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/AbstractFiducialModel.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/aruco/FiducialArucoParameters.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/aruco/aruco.h"
#endif


namespace ipa_Fiducials
{

/// @class FiducialModelAruco
///
/// A concrete class to represent a fiducial
class __DLL_LIBFIDUCIALS__ FiducialModelAruco : public AbstractFiducialModel
{
public:

	//*******************************************************************************
	// AbstractFiducial interface implementation
	//*******************************************************************************
	FiducialModelAruco();
	~FiducialModelAruco();

	/// Locates the fiducial within the image and inferes the camera pose from it
	/// @param scene image
	/// @return <code>RET_FAILED</code> if no tag could be detected
	/// <code>RET_OK</code> on success
	unsigned long GetPose(cv::Mat& image, std::vector<t_pose>& vec_pose);

	/// Load fiducial-centric coordinates of markers from file
	/// @param directory Directory, where the parameters of all fiducials are stores
	unsigned long LoadParameters(std::string directory_and_filename);

	//*******************************************************************************
	// Class specific functions
	//*******************************************************************************
	unsigned long LoadParameters(std::vector<FiducialArucoParameters> pi_tags);
private:
	boost::shared_ptr<aruco::MarkerDetector> m_detector; ///< instance of aruco detector
	double m_marker_size; ///< Aruco allows only a common marker size for all markers
	std::vector<FiducialArucoParameters> m_tag_parameters; ///< Individual parameters for each tag
};

} // end namespace ipa_Fiducials

#endif // __IPA_FIDUCIAL_MODEL_ARUCO_H__
