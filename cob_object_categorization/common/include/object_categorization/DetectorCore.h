#ifndef __DETECTORCORE_H__
#define __DETECTORCORE_H__

#include "object_categorization/SharedImageJBK.h"
#include "object_categorization/SharedImageSequence.h"

#include <opencv/ml.h>

/// Type definition to hold detection results.
struct t_DetectionResult
{
	double m_tx;		///< Translation in x direction
	double m_ty;		///< Translation in y direction
	double m_tz;		///< Translation in z direction
	double m_rx;		///< Rotation in x direction
	double m_ry;		///< Rotation in y direction
	double m_rz;		///< Rotation in z direction
	unsigned int m_u;	///< Image coordinate in u direction
	unsigned int m_v;	///< Image coordinate in v direction

	double m_DetectionQuality;	///< Detection qualitiy expressed with a probability ranging from [0,1]
};

//static const int DESCR_AREAS = 4;
//static const int DESCR_BINS = 8;

/// Definition of a sequence of detectione results
typedef std::vector<t_DetectionResult> t_DetectionResultSequence;

class DetectorCore
{
public:

	DetectorCore();
	~DetectorCore(){};
	
	typedef enum {MODE_LEARN, MODE_DETECT} tMode; ///< mode to select for learning or detection

	/// Get a feature frame cloud sequence from a shared image sequence (for learning).
	//void GetObjectModel(SharedImageSequence& simgs, ObjectModel& model);
	//void GetDescriptorRate(SharedImageSequence& simgs, ObjectModel& model, CvRect rect, bool saveOutput);
	//void DrawSegmentedFeatures(SharedImage& img, ObjectModel& model);

	/// Get a feature frame cloud from a shared image (for learning and detection).
	//void GetFPCloudFromSharedImage(SharedImage& si, ipa_utils::FPCloud2& fpc, tMode mode=MODE_DETECT, ObjectModel* model=0);
	
	/// Fuse feature frame cloud sequence into object model
	//void GetObjectModelFPCFromFPCloudSequence(ipa_utils::FPCloud2Sequence& fpcs, ipa_utils::FPCloud2& fpc, ipa_utils::Frame* tableFrame=0, double degOffset=0);

	/// Get the object model
	//void GetObjectModel(SharedImageSequence& simgs, ObjectModel& model);


	/// Detect model
	//void DetectModel(SharedImage& simg, ObjectModel& model, t_DetectionResultSequence& DetectionResults, int numResults);

	/// Draw a feature frame into the output image
	void DrawFeatureFrame(IplImage* img, ipa_utils::Frame& f, ipa_utils::IntVector* key=0);
	
	/// Draw feature frames into image
	//void DrawFeatureFrames(IplImage* img, ipa_utils::FPCloud2& fpc);


	/// helping functions
	static std::vector< std::vector<int> > defaultParameterIntMatrix;
	void GetFeaturePoints(SharedImage& si, std::vector<CvSURFPoint>& pts, ipa_utils::IntVector& id, ipa_utils::DblMatrix* descriptors=0, int pMaskMode=0, IplImage** pMask=NULL, int pMode=0, std::vector< std::vector<int> >& pTablePoints = defaultParameterIntMatrix, std::string pSpecialTreatment="", double pHessianThreshold=1250);
	bool GetFeaturePointData(SharedImage& si, CvSURFPoint& feature, int id, ipa_utils::Frame& f, ipa_utils::IntVector& k, int pMode=0);
	//void GetFPCloud(SharedImage& si, ObjectModel& model, ipa_utils::FPCloud2& fpc);
	//bool GetDescriptor(SharedImage& si, CvSURFPoint& feature, ipa_utils::Frame& f, ipa_utils::DblVector& D);

protected:

	/// surf/fpcloud params
	int m_extended;
	double m_surfThresh;
	CvSURFParams m_params;
	int m_noDims;
	double m_radDiv;
	double m_threshPhi;
	double m_foc;
	//double m_threshCluster;

	/// registration
	int m_NoItMax;
	double m_EpsCorrs;
	double m_EpsShrink;

	/// clustering
	double m_EpsCluster;
	int m_MinClusterMems;

	//CvSVMParams m_svmParams;

	bool m_UseKeys;
	double m_edgeThresh;
	bool m_UseOwnDescr;

	bool m_Draw;
};

#endif // __DETECTORCORE_H__
