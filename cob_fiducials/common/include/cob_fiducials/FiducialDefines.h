/// @file FiducialDefines.h
/// Defines for fiducial library.
/// @author Jan Fischer
/// @date June 2013.

#ifndef __IPA_FIDUCIAL_DEFINES_H__
#define __IPA_FIDUCIAL_DEFINES_H__

namespace ipa_Fiducials {

#if defined _MSC_VER && _MSC_VER >= 1200
	// disable warnings related to inline functions
	#pragma warning( disable: 4251 4275)
#endif

/// Define, if we need to import or export the libraries
#ifdef __LINUX__
	#define __DLL_LIBFIDUCIALS__ 
	#define APIENTRY
#else
	#ifdef __LIBFIDUCIALS_EXPORT__
		#define __DLL_LIBFIDUCIALS__ __declspec(dllexport)
	#else
		#define __DLL_LIBFIDUCIALS__ __declspec(dllimport)
	#endif
#endif

/// Enum to encode the different feature types
typedef enum 
{
TYPE_UNDEFINED      = 0x00000000,	    ///< Undefined feature type
	TYPE_PI             = 0x00000001,		///< PI fiducial from Bergamasco et al. 'Pi-Tag: a fast image-space marker design basedon projective invariants'
	TYPE_ARUCO          = 0x00000002		///< ArUco fiducial from http://www.uco.es/investiga/grupos/ava/node/26'
}t_FiducialType;

struct t_pose
{
	int id; ///< Unique ID of the marker

	cv::Mat rot; ///< rodrigues rotation vector from tag coordinate system to camera coordinate system
	cv::Mat trans; ///< translation from tag coordinate system to camera coordinate system
};




} // namespace ipa_Fiducials

#endif // __IPA_FIDUCIAL_DEFINES_H__
