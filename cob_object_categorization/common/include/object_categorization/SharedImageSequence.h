#ifndef __SHAREDIMAGESEQUENCE_H__
#define __SHAREDIMAGESEQUENCE_H__

#include "object_categorization/SharedImageJBK.h"
#include "object_categorization/SharedImageGlobalConstants.h"
#include "object_categorization/JBKUtils.h"

//#include "Vision/CameraSensors/AbstractRangeImagingSensor.h"
//#include "Vision/CameraSensors/AbstractColorCamera.h"

#include "boost/progress.hpp"

#ifdef __USE_POWERCUBE__
#include "Vision/ObjectDetection/Utilities/PowerCube.h" 
#endif

class SharedImageSequence : public std::list<SharedImage>
{
public:
	SharedImageSequence();
	~SharedImageSequence(void);

	void DeleteSharedImageSequence(const std::string& Name);

	/// Stores a sequence of shared images to hard disk.
	/// @param Name The prefix of the filename for each image.
	int SaveSharedImageSequence(const std::string& Name);

	/// Loads a sequence of shared images from hard disk.
	/// @param Name The prefix of the filename for each image.
	/// @param Limit The maximal number of images to be loaded.
	int LoadSharedImageSequence(const std::string& Name, unsigned int Limit=INT_MAX, int k=1);

	/// Export to a sequence of PointClouds.
	void ExportSequenceAsPointClouds(const std::string& Name);
	
	void SegmentImageSequence(ipa_utils::Frame& F, double Radius, double zMin, double zMax, int Cut=0);

	/// Displays all elements of the class.
	/// All images that represented by the class are diplayed in successive order like
	/// a small slide show.
	void SlideShow();
	
	/// Reads a series of pictures from the camera system.
	/// Creates a specified number of shared images from the camera system. The pictures
	/// are segmented on demand by a sphere that is defined through its midpoint and radius.
	/// @param RangeCam SwissRanger camera instance.
	/// @param ColorCam Color camera instance.
	/// @param SharedImageSize Size of the shared image
	/// @param DegreeOffset Step size in degrees for powercube module	void GetSharedImageSequencePowerCube(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam, CvSize SharedimageSize, int DegreeOffset=5);
	
	#ifdef __USE_SENSORS__
	
	void GetSharedImageSequenceManually(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam, const CvSize& SharedimageSize);
	void GetSharedImageSequencePowerCube(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam, const CvSize& SharedimageSize, int DegreeOffset=5);
	
	/// Gets an image sequence of the raw camera images
	void GetRawImageSequence(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam);
	
	#endif // __USE_SENSORS

	
	// Converts a "raw" sequence of color and range images to a shared image sequence
	//void SequenceConversion(std::vector<IplImage*> ColorImgs, std::vector<IplImage*> RangeImgs);

	/// Cuts a square out of all images within the image sequence.
	/// The square is centered around the segmented image center and is of size 'CutWidth' * 'CutWidth'.
	/// @param CutWidth The width and height of the resulting image.
	void CutImageBorder(int CutWidth=0);

	void OrientAlongPrincipalAxises(bool global=false);
	void AlignSequence(int noItMax=1000, bool circular=true);

	static std::string m_InfFileAttachment; ///< Filename extension of info file that hold the number of images that have been stored on disk
	static std::string m_Spacing;
	static std::string m_PCloudAttachement;
	static std::string m_ColorWinName;
	static std::string m_CoordWinName;
};


#endif // __SHAREDIMAGE_H__
