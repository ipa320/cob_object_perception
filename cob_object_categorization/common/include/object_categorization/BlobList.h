/// @file Bloblist.h
/// Latest updates: November 2008.

#ifndef BLOBLIST_H
#define BLOBLIST_H

#ifdef __LINUX__
	#include "object_categorization/BlobFeature.h"
#else
	#include "BlobFeature.h"
#endif

#include <list>

class BlobList : public std::list<BlobFeature>
{
public:

	std::string Str(); ///< String conversion.

	/// Drawing function.
	/// This function implements a drawing function to draw a blob feature point list into an IplImage.
	/// @param Src the source image in which the feature point will be drawn.
	/// @param Color the color (type CvScalar, see the OpenCV documents).
	/// @param Mode the drawing mode. \n
	/// 0: Draw cirecles around the feature location and a line for the gradient direction.
	/// 1: Draw points at the feature location with fixed colors.
	/// 2: Draw points at the feature location woth colors corresponding to the labels.

	/// @param Thickness the thicknes of the drawings.
	void DrawListInIplImage(IplImage* Src, int Mode=0, bool UseKeyColor=true);

	/// Load a blob list.
	int Load(std::string Name, bool Append=false);

	/// Load a blob list from a ifstream.
	int Load(std::ifstream* Name, bool Append=false);
	
	/// Save a blob list.
	int Save(std::string Name);

	/// Save a blob list in a ofstream.
	int Save(std::ofstream* Name);

	/// Remove points close to the border.
	void DeleteBorderPoints(IplImage* ImageMask, int MaskVal=0);

	/// Filter a certain scale inteval.
	void FilterScaleInterval(int ScaleMin, int ScaleMax);

	/// Filter on threshold.
	void FilterThreshold(int Threshold, bool SmallerThan=false);
};

#endif // BLOBLIST_H
