/// @file AbstractBlobDetector.h
/// Latest updates: November 2008.

#ifndef ABSTRACTBLOBDETECTOR_H
#define ABSTRACTBLOBDETECTOR_H

#include "object_categorization/BlobList.h"

/// Abstract blob detector.
class AbstractBlobDetector
{
public:

	enum SearchMode {MODE_FAST, MODE_ALL};

	/// Destructor.
	virtual ~AbstractBlobDetector(){};
	
	/// Initialize.
	//virtual unsigned long Init(){return RET_OK};
	/// Basic feature detection.
	/// This function implements the fast blob detection.
	/// @param Src source image (must be a one channel image) i.e. Src->nChannels==0.
	/// @param Out The output list should be of type BlobList<T>
	/// @param Mode the operation mode @see enum SearchMode.
	/// If Mode==MODE_ALL a brute force search over the full scale space will be computed.
	/// If Mode==MODE_FAST then a fast recursive search is performed which gives less blob features.
	/// @return The number of detected features
	virtual int DetectFeatures(IplImage* Src, BlobList& Out, SearchMode Mode=MODE_FAST)=0;

	bool m_UseDescriptor;

	int m_Id;						///< Identifier. Can be overwritten by the class user.

};

#endif // ABSTRACTBLOBDETECTOR_H
