/// @file BlobFeature.h
/// Latest updates: November 2008.

#ifndef BLOBFEATURE_H
#define BLOBFEATURE_H

#ifdef __LINUX__
	#include "object_categorization/JBKUtils.h"
	#include "object_categorization/ThreeDUtils.h"
#else
	#include "JBKUtils.h"
	#include "ThreeDUtils.h"
#endif

/// Class to represent blobs in general images (color, gray or range).
/// This class represents blobs in an image of type through their position, scale and angle.
class BlobFeature
{
public:

	/// Constructor.
	/// @param Id ID of the feature (i.e. to identify the channel (R,G,B) from which the feature aroze).
	/// @param x X-coordinate of the feature.
	/// @param y Y-coordinate of the feature.
	/// @param r Radius of the outer blob feature box (width=height=2*r).
	/// @param Res Blob feature response @see BlobResponse.
	/// @param Avg The avergage intensity value of the inner blob area.
	/// @param Mag The gradient magnitude
	/// @param Phi The gradient angle
	/// @param Label ?
	BlobFeature(int Id=0, int x=0, int y=0, int r=0, double Res=0.0, /*double Avg=0.0, double Mag=0.0, */double Phi=0.0, int Label=-1);
	~BlobFeature(){};
	
	int m_Id;	///< An identifier that can be used if several blob detectors are employed.
	int m_x;	///< The image position's x-component.
	int m_y;	///< The image position's y-component.
	int m_r;	///< The image position's radius.
	double m_Res; ///< The blob response.
	//double m_Avg; ///< The avergage value in the blob area.
	//double m_Mag; ///< The gradient magnitude.
	double m_Phi; ///< The gradient angle.
	ipa_utils::IpaVector<float> m_D; ///< The descriptor.
	ipa_utils::Frame m_Frame;		///< Coordinate Frame.				/�nderung RiB/

	///< Content: A 1 for each descriptor box that has an accumulated intensity value greater than
	///<          the median intensity value of all descriptor boxes. A -1 otherwise.
	//unsigned int m_DesWidth; ///< Number of descriptor's 0.5*rings.
	//unsigned int m_NoEdges; ///< Boxes per ring in descriptor.
	//unsigned int m_Label; ///< The associated label of the blob feature point.
	///< The label value is calculated by adding 2^k for each descriptor box that has an accumulated 
	///< intensity value greater than the median intensity value of all descriptor boxes, where k is
	///< the index of the corresponding descriptor box.
	std::string Str() const; ///< String conversion.

	/// Drawing function.
	/// This function implements a drawing function to draw a blob feature point into an IplImage.
	/// @param Src the source image in which the feature point will be drawn.
	/// @param Color the color (type CvScalar, see the OpenCV documents).
	/// @param Mode the drawing mode. \n
	/// 0: Draw cirecles around the feature location and a line for the gradient direction.
	/// 1: Draw points at the feature location with fixed colors.
	/// 2: Draw points at the feature location woth colors corresponding to the labels.
	/// @param Thickness the thicknes of the drawings.
	void DrawInIplImage(IplImage* Src, int Mode=0, bool UseKeyColor=true);
};

#endif // BLOBFEATURE_H
