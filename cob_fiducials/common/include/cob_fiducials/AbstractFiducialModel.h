#ifndef __IPA_ABSTRACT_FIDUCIAL_MODEL_H__
#define __IPA_ABSTRACT_FIDUCIAL_MODEL_H__


#ifdef __LINUX__
	#include "cob_vision_utils/VisionUtils.h"
	#include "cob_fiducials/FiducialDefines.h"
	#include "cob_fiducials/AbstractFiducialParameters.h"

	#include "tinyxml.h"
#else
	#include "cob_perception_common/cob_vision_utils/common/include/cob_vision_utils/VisionUtils.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/FiducialDefines.h"
	#include "cob_object_perception/cob_fiducials/common/include/cob_fiducials/AbstractFiducialParameters.h"

	#include "cob_object_perception_intern/windows/src/extern/TinyXml/tinyxml.h"	
#endif

#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
// todo: remove after debugging

#include <map>
#include <fstream>

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

	unsigned long Init(cv::Mat& camera_matrix, std::string directory_and_filename, 
		bool log_or_calibrate_sharpness_measurements = false, cv::Mat extrinsic_matrix = cv::Mat());

	unsigned long SetExtrinsics(cv::Mat& camera_matrix, cv::Mat extrinsic_matrix = cv::Mat());

	unsigned long ApplyExtrinsics(cv::Mat& rot_CfromO, cv::Mat& trans_CfromO);

	/// Locates the fiducial within the image and inferes the camera pose from it
	/// @param image scene image
	/// @param vec_pose Vector of poses from all detected tags relative to the camera
	/// @return <code>RET_FAILED</code> if no tag could be detected
	/// <code>RET_OK</code> on success
	virtual unsigned long GetPose(cv::Mat& image, std::vector<t_pose>& vec_pose_CfromO) = 0;

	/// Computes a measure of image sharpness by analyzing the marker region and the inserted Siemens star
	/// @param image Scene image
	/// @param pose_CfromO Pose of the detected tag relative to the camera
	/// @param fiducial_parameters Container for several parameters of the utilized marker. Mainly the offsets are relevant to the function.
	/// @param sharpness_measure Degree of image sharpness (in [0...1] = [blurry...sharp]) computed by the function.
	/// @return <code>RET_FAILED</code> if the tag id is invalid
	/// <code>RET_OK</code> on success
	unsigned long GetSharpnessMeasure(const cv::Mat& image, t_pose pose_CfromO, 
		const AbstractFiducialParameters& fiducial_parameters, double& sharpness_measure, 
		double sharpness_calibration_parameter_m = 9139.749632393357, 
		double sharpness_calibration_parameter_n = -2670187.875850272);

	// Gets the camera matrix
	// @return 3x3 camera matrix (fx 0 cx, 0 fy cy, 0 0 1) 
	cv::Mat GetCameraMatrix();

	// Set the camera matrix
	// @param camera_matrix 3x3 camera matrix (fx 0 cx, 0 fy cy, 0 0 1) 
	unsigned long SetCameraMatrix(cv::Mat camera_matrix);

	// Gets the distortion coeffs
	// @return 1x4 distortion coeffs matrix (k1,k2,p1,p2)
	cv::Mat GetDistortionCoeffs();

	// Sets the distortion coeffs
	// @param dist_coeffs 1x4 distortion coeffs matrix (k1,k2,p1,p2)
	unsigned long SetDistortionCoeffs(cv::Mat dist_coeffs);

	// Gets the general fiducial parameters for a certain marker
	// @return general fiducial parameters
	AbstractFiducialParameters GetGeneralFiducialParameters(int marker_id);

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

protected:
	std::map<int, AbstractFiducialParameters> m_general_fiducial_parameters;	///< map of marker id to some general parameters like offsets

	double m_image_size_factor;		// width of current image divided by reference width of 640px

	struct SharpnessLogData
	{
		int pixel_count;
		double distance_to_camera;
		double sharpness_score;
	};
	bool m_log_or_calibrate_sharpness_measurements;	///< if true, the sharpness measurements are logged and saved to disc for calibration of the curve or directly calibrated within the program
	std::vector<SharpnessLogData> m_log_data;	///< structure for logging measured data
};

} // end namespace ipa_Fiducials

#endif // __IPA_ABSTRACT_FIDUCIAL_MODEL_H__
