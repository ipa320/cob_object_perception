#ifndef __IPA_FIDUCIAL_ARUCO_PARAMETERS_H__
#define __IPA_FIDUCIAL_ARUCO_PARAMETERS_H__

#ifdef __LINUX__
	#include "cob_fiducials/AbstractFiducialParameters.h"
#else
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/AbstractFiducialParameters.h"
#endif

namespace ipa_Fiducials
{

/// @class AbstractFiduciaParameters
///
/// Abstract class to represent the parameters of a fiducial
class __DLL_LIBFIDUCIALS__ FiducialArucoParameters : public AbstractFiducialParameters
{
public:
	double line_width_height; ///< Common width and height of fiducial
};

} // end namespace ipa_Fiducials

#endif // __IPA_FIDUCIAL_PI_PARAMETERS_H__
