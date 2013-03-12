#ifndef __IPA_ABSTRACT_FIDUCIAL_PARAMETERS_H__
#define __IPA_ABSTRACT_FIDUCIAL_PARAMETERS_H__

namespace ipa_Fiducials
{

/// @class AbstractFiduciaParameters
///
/// Abstract class to represent the parameters of a fiducial
class __DLL_LIBFIDUCIALS__ AbstractFiducialParameters
{
public:
	int id; // Unique ID of tag
	cv::Point2d offset; // Offset of tag to target coordinate system

};

} // end namespace ipa_Fiducials

#endif // __IPA_ABSTRACT_FIDUCIAL_PARAMETERS_H__
