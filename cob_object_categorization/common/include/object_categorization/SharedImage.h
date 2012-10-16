#ifdef __USE_SHAREDIMAGE_JBK__
#include "SharedImageJBK.h"
#endif // __USE_SHAREDIMAGE_JBK__
#ifndef __USE_SHAREDIMAGE_JBK__

#ifndef __SHAREDIMAGE_H__
#define __SHAREDIMAGE_H__

#include <cstdio>
#include <string>
#include <cv.h>
#include <highgui.h>
#include <map>

#include "object_categorization/JBKUtils.h"
#include "object_categorization/ThreeDUtils.h"
#include "object_categorization/OpenCVUtils.h"

//#include "Vision/CameraSensors/AbstractRangeImagingSensor.h"
//#include "Vision/CameraSensors/AbstractColorCamera.h"

//#include "boost/progress.hpp"


#ifdef USE_POWERCUBE
#include "Vision/ObjectDetection/Utilities/PowerCube.h" 
#endif

static const CvSize SharedImageDefaultSize = cvSize(528, 432);

/// Shared image representation.
/// This class implements a shared images that contains (r, g, b) and (x, y, z) data.
class SharedImage
{
public:

	SharedImage();	///< Constructor.
	//SharedImage(const CvSize& SharedImageSize);//, std::string directory);
	SharedImage(const SharedImage& SImg); ///< Copy constructor
	/// Overwritten assignment operator
	SharedImage& operator= (const SharedImage& SImg);
	void Release(); ///< Releases images if allocated.
	~SharedImage();	///< Destructor.

	/// Allocates images.
	int AllocateImages(const CvSize& SharedImageSize);//, std::string directory);

	// Determines initialization status.
	//bool isInitialized() { return m_initialized; }

	/// Loads the shared and carthesian coordinate image.
	/// @param Name Prefix for the actual filename (i.e. <Name>_Coord.xml).
	/// @return RET_OK if both images could be loaded, RET_FAILED otherwise.
 	int LoadSharedImage(const std::string& Name);
	int DeleteSharedImage(const std::string& Name);
	int SaveSharedImage(const std::string& Name);
	
	/// Returns the corresponding image with carthesian coordinates from SwissRanger camera.
	/// @return The carthesian coordinate image.
	IplImage* Coord(){return m_CoordImage;}

	/// Returns the shared image.
	/// The shared image incooperates both. Color information and carthesian 3D coordinate information.
	/// @return The shared image.
	IplImage* Shared(){return m_SharedImage;}

	IplImage* Inten(){return m_IntenImage;}

	unsigned long GetData(int i, int j, double& x, double& y, double& z,
		double& R, double& G, double& B);

	/// Display image with carthesian coordinates (i.e. from SwissRanger camera) with openCV.
	/// Image color is determined based on to the depth value. The closer the object resides relative
	/// to the camera, the darker its color value will be.
	/// @param WinName The name of the openCV window.
	/// @param Save Saving image to disk on/off.
	/// @param DrawFeatures Enable/disable graphical representation of features within the picture.
	/// @param DrawAsPoints Enable/disable graphical representation of features as points.
	void DisplayCoord(std::string WinName, bool Save=false);//, FeaturePointList* DrawFeatures=NULL,  bool DrawAsPoints=false);

	/// Display shared image with openCV.
	/// Image color is determined based on to pictures RGB values.
	/// @param WinName The name of the openCV window.
	/// @param Save Saving image to disk on/off.
	/// @param DrawFeatures Enable/disable graphical representation of features within the picture.
	/// @param DrawAsPoints Enable/disable graphical representation of features as points.
	void DisplayShared(std::string WinName, bool Save=false);//, FeaturePointList* DrawFeatures=NULL, bool DrawAsPoints=false);
	void DisplayInten(std::string WinName, bool Save=false);//, FeaturePointList* DrawFeatures=NULL, bool DrawAsPoints=false);


	/// input/output as colored point cloud
	void ExportToPointCloud(ipa_utils::DblMatrix& M);
	void ImportFromPointCloud(ipa_utils::DblMatrix& M);
	

	/// Retrieve images from sensors.
	/// Acquires a SwissRanger image with cartesian coordinates and a color camera image. The shared
	/// image is created in two steps: \n
	/// 1. Generate openCV IplImage m_CoordImage with (u,v) to (x,y,z) mapping information from the 
	/// calibrated SwissRanger camera. \n
	/// 2. Generate openCV IplImage m_SharedImage with same dimensions as m_CoordImage and assign each
	/// (u,v) the corresponding color information from the color camera. \n
	/// @param RangeCam SwissRanger camera instance.
	/// @param ColorCam Color camera instance.
	/// @return
	unsigned long GetImagesFromSensors(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam, const CvSize& SharedImageSize);
	
	unsigned long GetRawImagesFromSensors(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam);


	/// Gets shared image from raw image.
	/*
	void GetFromRawImages(IplImage* ColorImage, IplImage* RangeImage,
			bool RangeSegmentation=false, Point3Dbl Center=Point3Dbl(0,0,0), double Radius=DBL_MAX,
			bool UsePlane=false, Point3Dbl TableNormal=Point3Dbl(0), Point3Dbl TableCenter=Point3Dbl(0));
	*/

	/// Performs range segmentation on the shared image.
	/// Determines for every point of the range image if it resides within the sphere of
	/// radius 'Radius' and center 'Center'. If so, the point stays within m_CoordImage and 
	/// m_SharedImage, if not, the point is 'removed' from the images.
	/// 'ZOff' is used to avoid cut off edges in z-direction.
	/// @param Center The segmentation center.
	/// @param Radius The segmentation radius around the center.
	/// @param ZOff The maximal z-distance a segmented point may reside apart from the 'Center' point.
	void DoRangeSegmentation(ipa_utils::Point3Dbl& Center, double Radius, double ZOff=DBL_MAX);

	/// Cuts a square out of the image.
	/// The square is centered around the segmented image center and is of size 'CutWidth' * 'CutWidth'.
	/// @param CutWidth The width and height of the resulting image.
	void CutImageBorder(int CutWidth, int MaskVal=0);

	//CvSize m_SharedImageSize;
	void Undistort();

private:
	
	//bool m_initialized;
	IplImage* m_CoordImage;
	IplImage* m_SharedImage;
	IplImage* m_IntenImage;
	ipa_utils::Point3Dbl m_Min;
	ipa_utils::Point3Dbl m_Max;
	ipa_utils::Point3Dbl m_MinBounds;
	ipa_utils::Point3Dbl m_MaxBounds;
	bool m_SaveAsPNG;

	static std::string m_CoordExtension;
	static std::string m_CoordExtensionPNG;
	static std::string m_SharedExtension;
	static std::string m_IntenExtension;
	static std::string m_IntenExtensionPNG;
	static unsigned int m_CoordDepth;
	static unsigned int m_SharedDepth;
	static unsigned int m_IntenDepth;
	static unsigned int m_CoordNChannels;
	static unsigned int m_SharedNChannels;
	static unsigned int m_IntenNChannels;
	static std::string m_SaveSharedDisplay;
	static std::string m_SaveCoordDisplay;
	static std::string m_SaveIntenDisplay;

};

#endif // __SHAREDIMAGE_H__
#endif // NOT __USE_SHAREDIMAGE_JBK__
