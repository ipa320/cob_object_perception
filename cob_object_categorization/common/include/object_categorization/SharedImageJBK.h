#ifndef __SHAREDIMAGEJBK_H__
#define __SHAREDIMAGEJBK_H__

#include <cstdio>
#include <string>
#include <opencv/cv.h>
#include <map>

#include "object_categorization/JBKUtils.h"
#include "object_categorization/ThreeDUtils.h"
#include "object_categorization/OpenCVUtils.h"
#include "object_categorization/ICP.h"
#include "object_categorization/Math3d.h"
#include "object_categorization/SharedImageGlobalConstants.h"

//#include "Vision/CameraSensors/AbstractRangeImagingSensor.h"
//#include "Vision/CameraSensors/AbstractColorCamera.h"

#include <boost/progress.hpp>

#define MAX_POINTS_USED 1024

typedef unsigned char BYTE;				// /�nderung RiB/

#ifdef __USE_POWERCUBE__
#include "Vision/ObjectDetection/Utilities/PowerCube.h" 
#endif

class SharedImageParams
{
public:
	SharedImageParams();
	~SharedImageParams();

	CvMat* m_ColCamIntrinsic;		///< Intrinsic color cam parameters
	CvMat* m_ColCamDistortion;		///< Color cam distortion parameters
	CvMat* m_ExtrTranslationVector; ///< Extrinsic translation vector
	CvMat* m_ExtrRotationMatrix;	///< Extrinsic rotation matrix

	IplImage* m_SRMapX;
	IplImage* m_SRMapY;
	
	double m_f;
	double m_k;
	double m_l;
	double m_m;

	bool m_Initialized;
};

class SharedImagePoint
{
public:
	SharedImagePoint(double _x=0, double _y=0, double _z=0, int _i=0, int _j=0, int _k=0)
	{
		x=_x; y=_y; z=_z; i=_i; j=_j; k=_k;
	}
	~SharedImagePoint(){};
	double x;
	double y;
	double z;
	int i;
	int j;
	int k;
};

struct PixelNeighborStruct
{
	int i;
	int j;
	int k;
	int l;
	double dCoor;
};

/// Function to compare two gradient differences
bool SortPixelNeighborStruct(const PixelNeighborStruct& s0, const PixelNeighborStruct& s1);

/// Shared image representation.
/// This class implements a shared images that contains (r, g, b) and (x, y, z) data.
class SharedImage
{
public:

	SharedImage();	///< Constructor.
	SharedImage(const SharedImage& SImg); ///< Copy constructor
	SharedImage& operator= (const SharedImage& SImg); 	///< Overwritten assignment operator
	void Release(); ///< Releases images if allocated.
	~SharedImage();	///< Destructor.
	int AllocateImages();	///< Allocates images.

	int LoadCoordinateImage(const std::string& Name);

	/// Loads the shared and cartesian coordinate image.
	/// @param Name Prefix for the actual filename (i.e. <Name>_Coord.xml).
	/// @return RET_OK if both images could be loaded, RET_FAILED otherwise.
 	int LoadSharedImage(const std::string& Name);
	int DeleteSharedImage(const std::string& Name); ///< deletes old sequence (important if it is longer)
	int SaveCoordinateImage(const std::string& Name);
	int SaveSharedImage(const std::string& Name); ///< Saves sequence
	
	/// Returns the corresponding image with cartesian coordinates from SwissRanger camera.
	/// @return The carthesian coordinate image.
	IplImage* Coord(){return m_CoordImage;}

	void setCoord(IplImage* coordImg) { if (m_CoordImage) delete m_CoordImage; m_CoordImage = coordImg; }

	/// Returns the shared image.
	/// @return The shared (color) image.
	IplImage* Shared(){return m_SharedImage;}

	void setShared(IplImage* sharedImg) { if (m_SharedImage) delete m_SharedImage; m_SharedImage = sharedImg; }

	/// Returns the shared image.
	/// @return The intensity image of the range camera.
	IplImage* Inten(){return m_IntenImage;}

	void setInten(IplImage* intenImg) { if (m_IntenImage) delete m_IntenImage; m_IntenImage = intenImg; }

	/// Gets the full range of data of one pixel of the shared image.
	unsigned long GetData(int i, int j, double& x, double& y, double& z,
		double& R, double& G, double& B);

	/// Display image with cartesian coordinates (i.e. from SwissRanger camera) with openCV.
	/// Image color is determined based on to the depth value. The closer the object resides relative
	/// to the camera, the darker its color value will be.
	/// @param WinName The name of the openCV window.
	/// @param Save Saving image to disk on/off.
	/// @param DrawFeatures Enable/disable graphical representation of features within the picture.
	/// @param DrawAsPoints Enable/disable graphical representation of features as points.
	void DisplayCoord(std::string WinName, bool Save=false);

	/// Display shared image with openCV.
	/// Image color is determined based on to pictures RGB values.
	/// @param WinName The name of the openCV window.
	/// @param Save Saving image to disk on/off.
	/// @param DrawFeatures Enable/disable graphical representation of features within the picture.
	/// @param DrawAsPoints Enable/disable graphical representation of features as points.
	void DisplayShared(std::string WinName, bool Save=false);
	void DisplayInten(std::string WinName, bool Save=false);

	void ExportToPointCloud(ipa_utils::DblMatrix& M); ///< export as colored point cloud (6xN matrix)
	void ImportFromPointCloud(ipa_utils::DblMatrix& M); ///< import from colored point cloud (6xN matrix)	

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
	
	#ifdef __USE_SENSORS__
	unsigned long GetImagesFromSensors(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam);

	/// Gets the raw images from the sensors
	unsigned long GetRawImagesFromSensors(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam);
	#endif // __USE_SENSORS__
	
	/// Performs range segmentation on the shared image.
	/// Determines for every point of the range image if it resides within the sphere of
	/// radius 'Radius' and center 'Center'. If so, the point stays within m_CoordImage and 
	/// m_SharedImage, if not, the point is 'removed' from the images.
	/// 'ZOff' is used to avoid cut off edges in z-direction.
	/// @param Center The segmentation center.
	/// @param Radius The segmentation radius around the center.
	/// @param ZOff The maximal z-distance a segmented point may reside apart from the 'Center' point.
	void DoRangeSegmentation(ipa_utils::Frame& F, double Radius, double zMin, double zMax, int Cut=0);

	/// Cuts a square out of the image.
	/// The square is centered around the segmented image center and is of size 'CutWidth' * 'CutWidth'.
	/// @param CutWidth The width and height of the resulting image.
	void CutImageBorder(int CutWidth);//, int MaskVal=0);

	void OrientAlongPrincipalAxises();
	unsigned long GetCorrespondences(SharedImage& si, std::vector<PixelNeighborStruct>& corrs, int kernel=30, double colDistThresh=5.0);
	unsigned long GetTransformation(SharedImage& si, Mat3d* rot, Vec3d* trans, double min=0.20, double max=0.95);
	unsigned long GetTransformation(SharedImage& si, SharedImage& si1, Mat3d* rot, Vec3d* trans, double min, double max); 

	void ApplyTransformationToCoordImage(SharedImage& si, Mat3d& rot, Vec3d& trans);

	void GetClosestUV(double x, double y, double z, int& u, int& v);

private:
	
	IplImage* m_CoordImage;
	IplImage* m_SharedImage;
	IplImage* m_IntenImage;
	ipa_utils::Point3Dbl m_Min;
	ipa_utils::Point3Dbl m_Max;

public:

	static std::string m_CoordExtension;
	static std::string m_CoordExtensionX;
	static std::string m_CoordExtensionY;
	static std::string m_CoordExtensionZ;
	static std::string m_SharedExtension;
	static std::string m_IntenExtension;
	static unsigned int m_CoordDepth;
	static unsigned int m_SharedDepth;
	static unsigned int m_IntenDepthSource;
	static unsigned int m_IntenDepthUsed;
	static unsigned int m_CoordNChannels;
	static unsigned int m_SharedNChannels;
	static unsigned int m_IntenNChannels;
	static std::string m_SaveSharedDisplay;
	static std::string m_SaveCoordDisplay;
	static std::string m_SaveIntenDisplay;

	static SharedImageParams m_Parameters;
};

#endif // __SHAREDIMAGEJBK_H__
