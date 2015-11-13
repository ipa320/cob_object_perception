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
	int m_id; // Unique ID of tag
	cv::Point2d m_offset; // Offset of tag to target coordinate system

	cv::Rect_<double> m_sharpness_pattern_area_rect3d;	// rectangle describing the area for sharpness computation in 2d coordinates within the marker plane with respect to the marker origin
};

} // end namespace ipa_Fiducials

#endif // __IPA_ABSTRACT_FIDUCIAL_PARAMETERS_H__
