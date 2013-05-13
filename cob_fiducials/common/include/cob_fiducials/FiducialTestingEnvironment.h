#ifndef __IPA_FIDUCIAL_TESTING_ENVIRONMENT_H__
#define __IPA_FIDUCIAL_TESTING_ENVIRONMENT_H__

//#include "../../../../../cob_object_perception_intern/windows/src/PreCompiledHeaders/StdAfx.h"
#ifdef __LINUX__
	#include "cob_vision_utils/GlobalDefines.h"
	#include "cob_fiducials/FiducialDefines.h"
	#include "cob_fiducials/FiducialModelPi.h"
	#include "cob_fiducials/aruco/FiducialModelAruco.h"
#else
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/FiducialDefines.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/FiducialModelPi.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/aruco/FiducialModelAruco.h"
#endif

namespace ipa_Fiducials
{

/// @class FiducialTestingEnvironment
///
/// Testing environment for fiducial detection
class __DLL_LIBFIDUCIALS__ FiducialTestingEnvironment
{

public:

	FiducialTestingEnvironment(cv::Mat& camera_matrix);	///< Constructor.
	~FiducialTestingEnvironment();	///< Destructor.

	unsigned long FiducialTestPI();
	unsigned long FiducialTestAruco();
private:
	unsigned long RenderPose(cv::Mat& image, cv::Mat& rot, cv::Mat& trans);
	unsigned long ReprojectXYZ(double x, double y, double z, int& u, int& v);
	boost::shared_ptr<FiducialModelPi> m_pi_tag;
	boost::shared_ptr<FiducialModelAruco> m_aruco_tag;
	cv::Mat m_camera_matrix;
};

} // end namespace ipa_Fiducials

#endif // __IPA_FIDUCIAL_TESTING_ENVIRONMENT_H__
