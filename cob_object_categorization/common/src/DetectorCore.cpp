#include "object_categorization/DetectorCore.h"

//ipa_utils::FPCloud2Sequence* pFPCS;

using namespace std;
using namespace ipa_utils;

DetectorCore::DetectorCore()
{
	m_extended = 0; ///< 64 descriptor
	double m_surfThresh = 1250;//2000; // originally 500
	m_params = cvSURFParams(m_surfThresh, m_extended);
	if(m_extended==0) m_noDims = 64;
	else if(m_extended==1) m_noDims = 128;
	else assert(false);
	m_radDiv = 50;	///< for the discrete size computation
	m_threshPhi = 0.75*THE_PI_DEF;
	SharedImage si;
	m_foc=si.m_Parameters.m_f;

	m_NoItMax = 1000;
	m_EpsCorrs = 100.0;
	m_EpsShrink = 1.0;
	m_EpsCluster = 20.0;
	m_MinClusterMems = 1;

	m_UseKeys = true;
	m_edgeThresh = 50.0;
	m_UseOwnDescr = false;
	m_Draw = false;
};

//void DetectorCore::GetObjectModel(SharedImageSequence& simgs, ObjectModel& model)
//{
//	/// init (just in case)
//	model.m_cls.clear();
//	
//	//*************** SVM
//
//	std::map<IntVector, DblMatrix> clusters;
//
//	/// get features and get clusters
//	SharedImageSequence::iterator it;
//	for(it=simgs.begin(); it!=simgs.end(); it++)
//	{ 
//		std::vector<CvSURFPoint> features;
//		IntVector IDs;
//		DblMatrix descriptors;
//		
//		//if(m_UseOwnDescr) GetFeaturePoints(*it, features, IDs, 0);
//		//else
//		GetFeaturePoints(*it, features, IDs, &descriptors);
//		
//		/// get clusters and feature frame clouds
//		for(unsigned int i=0; i<features.size(); i++)
//		{
//			Frame f; IntVector k;
//			if(!GetFeaturePointData(*it, features[i], IDs[i], f, k)) continue;
//			else
//			{
//				//if(m_UseOwnDescr)
//				//{
//				//	DblVector descr;
//				//	if(!GetDescriptor(*it, features[i], f, descr)) continue;
//				//	else model.m_cls.AppendEntry(k, descr);
//				//}
//				//else
//				//{
//				model.m_cls.AppendEntry(k, descriptors[i]);
//				//}
//
//				//if(m_Draw)
//				//	DrawFeatureFrame(it->Shared(), f);
//			}
//		}
//		//if(m_Draw)
//		//	cvSaveImage("featureFramesOut.bmp", it->Shared());
//	}
//
//	model.m_cls.Train();
//	
//	//*************** FPC
//
//	model.m_fpc.Clear();
//	FPCloud2Sequence fpcs;
//	for(it=simgs.begin(); it!=simgs.end(); it++)
//	{ 
//		/// get feature frame cloud
//		std::vector<CvSURFPoint> features;
//		IntVector IDs;
//		DblMatrix descriptors;
//		//if(m_UseOwnDescr) GetFeaturePoints(*it, features, IDs, 0);
//		//else
//		GetFeaturePoints(*it, features, IDs, &descriptors);
//		
//		FPCloud2 fpc;
//		for(unsigned int i=0; i<features.size(); i++)
//		{
//			Frame f; IntVector k;
//			if(!GetFeaturePointData(*it, features[i], IDs[i], f, k)) continue;
//			else
//			{
//				int label = -1;
//				//	if(m_UseOwnDescr)
//				//	{
//				//		DblVector descr;
//				//		if(!GetDescriptor(*it, features[i], f, descr)) continue;
//				//		else label = model.m_cls.IsInCluster(k, descr);
//				//	}
//				//	else
//				//	{
//				label = model.m_cls.IsInCluster(k, descriptors[i]);
//				//	}
//
//				k.push_back(label);
//				fpc.SetMember(k, f);
//			}
//		}
//		//fpc.Centralize3();
//		fpcs.push_back(fpc);
//	}
//
//	/// registration
//	fpcs.RegisterFPCloudsIt(model.m_fpc, m_NoItMax, m_EpsCorrs, m_EpsShrink);
//	//pFPCS->insert(pFPCS->end(), fpcs.begin(), fpcs.end());
//	//model.m_fpc.QTClustering(m_EpsCluster, m_MinClusterMems);
//	//model.m_fpc.Centralize3();
//	pFPCS->push_back(model.m_fpc);
//}
//
//void DetectorCore::GetDescriptorRate(SharedImageSequence& simgs, ObjectModel& model, CvRect rect, bool saveOutput)
//{
//	SharedImageSequence::iterator it;
//	double r_hits=0.0,  r_false_alarms=0.0, cnt_pos=0.0, cnt_neg=0.0, cnt_nones=0, cnt_all=0;
//	double r_miss=0.0, r_corr_rej=0.0;
//	
//	for(it=simgs.begin(); it!=simgs.end(); it++)
//	{
//		std::vector<CvSURFPoint> features;
//		IntVector IDs;
//		DblMatrix descriptors;
//		
//		if(m_UseOwnDescr) GetFeaturePoints(*it, features, IDs, 0);
//		else GetFeaturePoints(*it, features, IDs, &descriptors);
//		
//		cnt_all = features.size();
//
//		/// get clusters
//		for(unsigned int i=0; i<features.size(); i++)
//		{
//			Frame f; IntVector k;
//			if(!GetFeaturePointData(*it, features[i], IDs[i], f, k))
//			{
//				cnt_nones++;
//				continue;
//			}
//
//			/// get cluster num
//			int res = -1;
//		//	if(m_UseOwnDescr)
//		//	{
//		//		DblVector descr;
//		//		if(!GetDescriptor(*it, features[i], f, descr))
//		//		{
//		//			cnt_nones++;
//		//			continue;
//		//		}
//		//		res = model.m_cls.IsInCluster(k, descr);
//		//	}
//		//	else
//		//	{
//				res = model.m_cls.IsInCluster(k, descriptors[i]);
//		//	}
//
//			if(res<0)
//			{
//				cnt_nones++;
//			}
//
//			if(	features[i].pt.x > rect.x &&
//				features[i].pt.x < rect.x+rect.width &&
//				features[i].pt.y > rect.y &&
//				features[i].pt.y < rect.y+rect.height)
//			{
//				cnt_pos++;
//				if(res>=0)
//				{
//					r_hits++;
//				}
//				else
//				{
//					r_miss++;
//				}
//			}
//			else
//			{
//				cnt_neg++;
//				if(res>=0)
//				{
//					r_false_alarms++;
//				}
//				else
//				{
//					r_corr_rej++;
//				}
//				
//			}
//		}
//	}
//
//	double r_h = 1;
//	if(cnt_pos>0) r_h = r_hits/cnt_pos;
//	double r_fa = 0;
//	if(cnt_neg>0) r_fa = r_false_alarms/cnt_neg;
//	double err = (r_miss+r_false_alarms)/(cnt_pos+cnt_neg);
//	std::cout << "error      " << err << std::endl;
//	std::cout << "hits       " << r_h << std::endl;
//	std::cout << "false al.  " << r_fa << std::endl;
//	std::cout << "misses     " << r_miss/(cnt_pos+cnt_neg) << std::endl;
//	std::cout << "corr. rej. " << r_corr_rej/(cnt_pos+cnt_neg) << std::endl;
//	std::cout << "all        " << cnt_all << std::endl;
//	std::cout << "nones      " << cnt_nones/(cnt_all) << std::endl;
//}	
//
//void DetectorCore::DrawSegmentedFeatures(SharedImage& img, ObjectModel& model)
//{
//	std::vector<CvSURFPoint> features;
//	IntVector IDs;
//	DblMatrix descriptors;	
//	if(m_UseOwnDescr) GetFeaturePoints(img, features, IDs, 0);
//	else GetFeaturePoints(img, features, IDs, &descriptors);
//	std::vector<CvPoint> points;
//
//	for(unsigned int i=0; i<features.size(); i++)
//	{
//		Frame f; IntVector k;
//		if(!GetFeaturePointData(img, features[i], IDs[i], f, k)) continue;
//
//		/// get cluster num
//		int res = -1;
//		//if(m_UseOwnDescr)
//		//{
//		//	DblVector descr;
//		//	if(!GetDescriptor(img, features[i], f, descr)) continue;
//		//	res = model.m_cls.IsInCluster(k, descr);
//		//}
//		//else
//		//{
//			res = model.m_cls.IsInCluster(k, descriptors[i]);
//		//}
//
//		//CvPoint point = cvPoint(cvRound(features[i].pt.x), cvRound(features[i].pt.y));
//		//if(res>-1)
//		//{
//		//	points.push_back(point);
//		//}
//		//else
//		//{
//		//	cvCircle(img.Shared(), point, 2, CV_RGB(255,0,0), 2);
//		//}
//	}
//
//	//for(unsigned int k=0; k<points.size(); k++)
//	//{
//	//	cvCircle(img.Shared(), points[k], 2, CV_RGB(0,255,0), 2);
//	//}
//}

//void DetectorCore::DetectModel(SharedImage& simg, ObjectModel& model, t_DetectionResultSequence& DetectionResults, int numResults)
//{
//	/// get feature frames
//	FPCloud2 scene;
//	//GetFPCloudFromSharedImage(simg, scene, &model.m_cls, MODE_DETECT);
//
//	VotingList VL;
//	double cubeSize = 0.5 * model.m_fpc.m_Norm;
//	double vicinity =  0.5 * cubeSize;
//	double threshHits = 0.1;
//	scene.GetApprFramesFast2(model.m_fpc, VL, numResults, threshHits,
//							THE_PI_DEF, cubeSize, vicinity);
//
//	/// Copy to the final results
//	DetectionResults.clear();
//	for(unsigned int i=0; i<VL.size(); i++)
//	{
//		/// get frame
//		t_DetectionResult DetectionResult;
//		DetectionResult.m_tx = VL[i].m_F[0];
//		DetectionResult.m_ty = VL[i].m_F[1];
//		DetectionResult.m_tz = VL[i].m_F[2];
//		DetectionResult.m_rx = VL[i].m_F[3];
//		DetectionResult.m_ry = VL[i].m_F[4];
//		DetectionResult.m_rz = VL[i].m_F[5];
//		
//		/// convert m_tx, m_ty, and m_tz to u and v using the ranger transform 
//		DetectionResult.m_u = cvRound((DetectionResult.m_tx/DetectionResult.m_tz) * m_foc) + simg.Shared()->width/2;
//		DetectionResult.m_v = cvRound((DetectionResult.m_ty/DetectionResult.m_tz) * m_foc) + simg.Shared()->height/2;
//		
//		/// push to result list
//		DetectionResults.push_back(DetectionResult);
//	}
//}

void DetectorCore::DrawFeatureFrame(IplImage* img,  Frame& f, IntVector* key)
{
	int u = (int)floor(m_foc * f[0]/f[2] + img->width/2.0);
	int v = (int)floor(m_foc * f[1]/f[2] + img->height/2.0);
	
	Point3Dbl ex, ey, ez;
	f.eX(ex); f.eY(ey); f.eZ(ez);
	double size = 100.0;
	ex.ScalarMul(size); ey.ScalarMul(size); ez.ScalarMul(size); 
	Point3Dbl center(f[0], f[1], f[2]);
	ex.AddVec(center); ey.AddVec(center); ez.AddVec(center);

	int ux = (int)floor(m_foc * ex.m_x/ex.m_z + img->width/2.0);
	int vx = (int)floor(m_foc * ex.m_y/ex.m_z + img->height/2.0);
	int uy = (int)floor(m_foc * ey.m_x/ey.m_z + img->width/2.0);
	int vy = (int)floor(m_foc * ey.m_y/ey.m_z + img->height/2.0);
	int uz = (int)floor(m_foc * ez.m_x/ez.m_z + img->width/2.0);
	int vz = (int)floor(m_foc * ez.m_y/ez.m_z + img->height/2.0);

	cvLine(img, cvPoint(u, v), cvPoint(ux, vx), CV_RGB(0,0,0), 3);
	cvLine(img, cvPoint(u, v), cvPoint(uy, vy), CV_RGB(0,0,0), 3);
	cvLine(img, cvPoint(u, v), cvPoint(uz, vz), CV_RGB(0,0,0), 3);

	cvLine(img, cvPoint(u, v), cvPoint(ux, vx), CV_RGB(255,0,0), 1);
	cvLine(img, cvPoint(u, v), cvPoint(uy, vy), CV_RGB(0,255,0), 1);
	cvLine(img, cvPoint(u, v), cvPoint(uz, vz), CV_RGB(0,0,255), 1);
	
	if(key!=0)
	{
		int r=0, g=0, b=0;
		IntToCol(key->Sum(), r, g, b);
		cvCircle(img, cvPoint(u, v), 2, CV_RGB(r, g, b), 1);
	}
}

//void DetectorCore::DrawFeatureFrames(IplImage* img, ipa_utils::FPCloud2& fpc)
//{
//	FPCloudMap::iterator it;
//	FrameList::iterator itf;
//	for(it=fpc.GetMapPtr()->begin(); it!=fpc.GetMapPtr()->end(); it++)
//	{
//		IntVector key = it->first;
//		for(itf=it->second.begin(); itf!=it->second.end(); itf++)
//		{
//			DrawFeatureFrame(img, *itf, &key);
//		}
//	}
//}


/// helping functions

struct voxelGridCompare {
	bool operator() (const cv::Scalar& lhs, const cv::Scalar& rhs) const
	{
		if (lhs.val[0] == rhs.val[0])
		{
			if (lhs.val[1] == rhs.val[1])
			{
				if  (lhs.val[2] == rhs.val[2])
					return (lhs.val[3]<rhs.val[3]);
				else
					return (lhs.val[2]<rhs.val[2]);
			}
			else
				return (lhs.val[1]<rhs.val[1]);
		}
		else
			return (lhs.val[0]<rhs.val[0]);
	}
};


void DetectorCore::GetFeaturePoints(SharedImage& si, std::vector<CvSURFPoint>& pts, IntVector& id, DblMatrix* descriptors, int pMaskMode, IplImage** pMask, int pMode, std::vector< std::vector<int> >& pTablePoints, std::string pSpecialTreatment, double pHessianThreshold)
{
	//pMode: 0=RGBI+channel of origin , 1=RGBI , 2=Intensity from color image , 3=HSVI+channel of origin, 4=HSVI

	/// init image channels
	vector<IplImage*> imgs;
	if (pMode==0 || pMode==1)
	{
		imgs.push_back(cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 1));
		imgs.push_back(cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 1));
		imgs.push_back(cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 1));
		cvSplit(si.Shared(), imgs[2], imgs[1], imgs[0], 0);
		imgs.push_back(cvCloneImage(si.Inten()));
		//IplImage* Z = cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 1);
	}
	else if (pMode==2)
	{
		imgs.push_back(cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 1));
		cvCvtColor(si.Shared(), imgs[0], CV_BGR2GRAY);
	}
	else if (pMode==3 || pMode==4)
	{
		imgs.push_back(cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 1));
		imgs.push_back(cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 1));
		imgs.push_back(cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 1));
		IplImage* hsvShared = cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 3);
		cvCvtColor(si.Shared(), hsvShared, CV_BGR2HSV);
		cvSplit(hsvShared, imgs[0], imgs[1], imgs[2], 0);
		cvReleaseImage(&hsvShared);
		imgs.push_back(cvCloneImage(si.Inten()));
	}

	//for(int j=0; j<si.Coord()->height; j++) 
	//{	
	//	for(int i=0; i<si.Coord() ->width; i++)
	//	{	
	//		float z = ((float*)(si.Coord()->imageData + j * si.Coord()->widthStep))[i * si.Coord()->nChannels+2];
	//		uchar intenZ = (cvRound(z)-500)/10; // one eight bit color level corresponds to one cm, starting at 50 cm min
	//		intenZ = MIN(255, MAX(intenZ, 0)); // cropping
	//		((uchar*)(Z->imageData + j * Z->widthStep))[i] = intenZ;
	//	}
	//}
	//imgs.push_back(Z);

	/// Mask everything outside the region of interest (only applicable in training mode and on pictures taken at the IPA rotating plate)			/�nderung RiB/
	IplImage* Mask;

	if (pMaskMode==1)
	{	// Use given mask
		Mask = *pMask;
	}
	else
	{	// Use no mask (MaskMode=0) or create mask (MaskMode=2)
		if (pMaskMode == 2)
		{
			Mask = cvCreateImage(cvGetSize(si.Coord()), IPL_DEPTH_8U, 1);
			cvSet(Mask, cvScalar(255,255,255,255));
		}
	}

	if (pMaskMode==2 && pMask!=NULL)
	{
		// find 3 points on the table and build the table normal (and plane)
		double MaxZ = 0;		// maximum z-distance where the image is not masked
		ipa_utils::Point3Dbl Point1, Point2, Point3;
		CvScalar Point;
		Point = cvGet2D(si.Coord(), pTablePoints[0][1], pTablePoints[0][0]);	//400, 90);
		Point1 = ipa_utils::Point3Dbl(Point.val[0], Point.val[1], Point.val[2]);
		Point = cvGet2D(si.Coord(), pTablePoints[1][1], pTablePoints[1][0]);	// 350, 440);
		Point2 = ipa_utils::Point3Dbl(Point.val[0], Point.val[1], Point.val[2]);
		Point = cvGet2D(si.Coord(), pTablePoints[2][1], pTablePoints[2][0]);	//315, 122);
		Point3 = ipa_utils::Point3Dbl(Point.val[0], Point.val[1], Point.val[2]);
		MaxZ = Point.val[2];
		ipa_utils::Point3Dbl Diff1, Diff2, Normal;
		Diff1.SubVec(Point2, Point1);
		Diff2.SubVec(Point3, Point1);
		Normal.VectorProd(Diff1, Diff2);
		Normal.Normalize();

		// mask every pixel on the left and right side as well as on the top
		int LeftPixels = cvRound(si.Coord()->width*0.15);		//*0.28);
		int RightPixels = cvRound(si.Coord()->width*0.15);				//*0.24);
		int TopPixels = cvRound(si.Coord()->height*0.09);
		CvScalar Zero = cvScalar(0,0,0,0);
		
		cvSet(Mask, cvScalar(255,255,255,255));
		cvSetImageROI(Mask, cvRect(0, 0, LeftPixels, Mask->height));
		cvSet(Mask, Zero);
		cvResetImageROI(Mask);
		cvSetImageROI(Mask, cvRect(Mask->width-RightPixels-1, 0, RightPixels, Mask->height));
		cvSet(Mask, Zero);
		cvResetImageROI(Mask);
		cvSetImageROI(Mask, cvRect(0, 0, Mask->width, TopPixels));
		cvSet(Mask, Zero);
		cvResetImageROI(Mask);

		// estimate plane normal
		// 1. collect all potential inliers
		double Distance=0;
		std::vector<cv::Scalar> planePoints;
		for (int u=LeftPixels; u<(si.Coord()->width-RightPixels); u++)
		{
			for (int v=TopPixels; v<(si.Coord()->height); v++)
			{
				Point = cvGet2D(si.Coord(), v, u);
				// point to far away -> =background
				if (Point.val[2] > MaxZ)
				{
					cvSetReal2D(Mask, v, u, 0);
					continue;
				}
				// point on table surface
				Diff1.SubVec(ipa_utils::Point3Dbl(Point.val[0], Point.val[1], Point.val[2]), Point1);
				Distance = Normal.ScalarProd(Diff1);
				if (fabs(Distance)<0.025)
				{
					planePoints.push_back(Point);
					continue;
				}
			}
		}
		// 2. convert into regression problem and estimate normal
		cv::Mat A(planePoints.size(), 4, CV_64FC1);
		for (int i=0; i<(int)planePoints.size(); i++)
		{
			for (int j=0; j<3; j++)
				A.at<double>(i, j) = planePoints[i].val[j];
			A.at<double>(i, 3) = -1.0;
		}
		cv::SVD svd(A);
		Normal.m_x = svd.vt.at<double>(svd.vt.rows-1, 0);
		Normal.m_y = svd.vt.at<double>(svd.vt.rows-1, 1);
		Normal.m_z = svd.vt.at<double>(svd.vt.rows-1, 2);
		// alignment of normal with (0, -1, 0) (scalar product of both vectors > 0)
		if (Normal.m_y > 0)
		{
			Normal.m_x *= -1;
			Normal.m_y *= -1;
			Normal.m_z *= -1;
		}
		Normal.Normalize();
		// compute plane point from Hesse normal form
		Point1.m_x = 0;
		Point1.m_y = svd.vt.at<double>(svd.vt.rows-1, 3) / svd.vt.at<double>(svd.vt.rows-1, 1);
		Point1.m_z = 0;

		// mask table pixels and very distant pixels
		Distance=0;
		for (int u=LeftPixels; u<(si.Coord()->width-RightPixels); u++)
		{
			for (int v=TopPixels; v<(si.Coord()->height); v++)
			{
				Point = cvGet2D(si.Coord(), v, u);
				// point to far away -> =background
				if (Point.val[2] > MaxZ)
				{
					cvSetReal2D(Mask, v, u, 0);
					continue;
				}
				// point on table surface
				Diff1.SubVec(ipa_utils::Point3Dbl(Point.val[0], Point.val[1], Point.val[2]), Point1);
				Distance = Normal.ScalarProd(Diff1);

				double tableDistanceThreshold = 0.022;
				if (pSpecialTreatment == "scissors")
					tableDistanceThreshold = 0.07;
				else if (pSpecialTreatment == "silverware")
					tableDistanceThreshold = 0.06;
				else if (pSpecialTreatment == "pen")
					tableDistanceThreshold = 0.058;
				else if (pSpecialTreatment == "pen_highbase")
					tableDistanceThreshold = 0.07;
				else if (pSpecialTreatment == "dishes")
					tableDistanceThreshold = 0.05;
				if (fabs(Distance)<tableDistanceThreshold || Distance<0.f)			// normal: 0.022, scissors: 0.06, silverware: 0.055, pen: 0.058, dishes: 0.05
				{
					cvSetReal2D(Mask, v, u, 0);
					continue;
				}
			}
		}
		int erodeRadius = 6;
		if (pSpecialTreatment == "scissors")
			erodeRadius = 0;
		else if (pSpecialTreatment == "silverware")
			erodeRadius = 0;
		else if (pSpecialTreatment == "pen" || pSpecialTreatment == "pen_highbase")
			erodeRadius = 0;
		else if (pSpecialTreatment == "dishes")
			erodeRadius = 6;
		cvErode(Mask, Mask, 0, erodeRadius);    // normal: 6, scissors: 0, silverware: 0, pen: 0, dishes: 6

		// voxelize the point cloud and unmask voxels with low point density --> removes sensor noise at object borders
		std::map<cv::Scalar, std::vector<cv::Scalar>, voxelGridCompare> voxelGrid;
		std::map<cv::Scalar, std::vector<cv::Scalar>, voxelGridCompare>::iterator itVoxelGrid;
		cv::Scalar voxelCellSize(0.02, 0.02, 0.01, 0);
		if (pSpecialTreatment == "tetrapaks")
		{ voxelCellSize[0]=0.01; voxelCellSize[1]=0.01; voxelCellSize[2]=0.005; }
		// assign 2d depth image coordinates to 3d voxel cells
		for (int u=LeftPixels; u<(si.Coord()->width-RightPixels); u++)
		{
			for (int v=TopPixels; v<(si.Coord()->height); v++)
			{
				if (cvGetReal2D(Mask, v, u) == 255)
				{
					Point = cvGet2D(si.Coord(), v, u);
					cv::Scalar voxelCell(floor(Point.val[0]/voxelCellSize.val[0] + 0.5)*voxelCellSize.val[0], floor(Point.val[1]/voxelCellSize.val[1] + 0.5)*voxelCellSize.val[1], floor(Point.val[2]/voxelCellSize.val[2] + 0.5)*voxelCellSize.val[2], 0);
					cv::Scalar imageCoordinate(u, v, 0, 0);
					voxelGrid[voxelCell].push_back(imageCoordinate);
				}
			}
		}
		// compute average density in all cells
		double averagePointDensity = 0;
		for (itVoxelGrid = voxelGrid.begin(); itVoxelGrid != voxelGrid.end(); itVoxelGrid++)
			averagePointDensity += (double)itVoxelGrid->second.size();
		averagePointDensity /= (double)voxelGrid.size();
		std::cout << "avgDensity = " << averagePointDensity << std::endl;
		// unmask all pixels that have too low density in their voxel and no high density voxel within their 6-neighborhood in 3d
		double densityThresholdMultiplier = 1.5;
		if (pSpecialTreatment == "scissors")
			densityThresholdMultiplier = 1.8;
		else if (pSpecialTreatment == "silverware")
			densityThresholdMultiplier = 1.5;
		else if (pSpecialTreatment == "pen" || pSpecialTreatment == "pen_highbase")
			densityThresholdMultiplier = 4.8;
		else if (pSpecialTreatment == "dishes")
			densityThresholdMultiplier = 0.5;
		else if (pSpecialTreatment == "bottle")
			densityThresholdMultiplier = 3.2;
		else if (pSpecialTreatment == "can")
			densityThresholdMultiplier = 0.8;
		else if (pSpecialTreatment == "coffeepot")
			densityThresholdMultiplier = 2.4;
		else if (pSpecialTreatment == "mouse")
			densityThresholdMultiplier = 0.8;
		else if (pSpecialTreatment == "screens")
			densityThresholdMultiplier = 1.0;
		else if (pSpecialTreatment == "tetrapaks")
			densityThresholdMultiplier = 1.2;

		unsigned int densityThreshold = (unsigned int)floor(densityThresholdMultiplier*averagePointDensity); //hier die stddev berücksichtigen ?
		for (itVoxelGrid = voxelGrid.begin(); itVoxelGrid != voxelGrid.end(); itVoxelGrid++)
		{
			if (itVoxelGrid->second.size() < densityThreshold)
			{	// voxel has a low point density
				bool hasHighDensityNeighbor = false;
				//for (int dim=0; dim<3; dim++)
				//{
				//	for (int offset = -1; offset <= 1; offset+=2)
				//	{
				//		Point = itVoxelGrid->first;
				//		Point.val[dim] += offset*voxelCellSize.val[dim];
				//		if (voxelGrid.find(Point) != voxelGrid.end() && voxelGrid.find(Point)->second.size() >= densityThreshold)
				//			hasHighDensityNeighbor = true;
				//	}
				//	if (hasHighDensityNeighbor == true)
				//		break;
				//}
				if (hasHighDensityNeighbor == false)
				{	// voxel has no neighboring high density voxel --> noise --> unmask all points assigned to that voxel
					for (size_t i=0; i<itVoxelGrid->second.size(); i++)
						cvSetReal2D(Mask, itVoxelGrid->second[i].val[1], itVoxelGrid->second[i].val[0], 0);
				}
			}
		}
		// ... and erode to close gaps


		/*	for (int u=LeftPixels; u<(si.Coord()->width-RightPixels); u++)
			{
				for (int v=0; v<(si.Coord()->height); v++)
				{
					if (cvGetReal2D(Mask, v, u) != 0) cvSetReal2D(Mask, v, u, cvGetReal2D(imgs[0],v,u));
				}
			}
			cvSaveImage("Mask.bmp", Mask);
		*/

		//// output the point cloud to file
		//std::cout << "Coordinates masked:\n";
		//std::ofstream fout("coordinates_masked.txt");
		//for (int u=LeftPixels; u<(si.Coord()->width-RightPixels); u++)
		//{
		//	for (int v=TopPixels; v<(si.Coord()->height); v++)
		//	{
		//		if (cvGetReal2D(Mask, v, u) == 255)
		//		{
		//			Point = cvGet2D(si.Coord(), v, u);
		//			fout << Point.val[0] << " \t" << Point.val[1] << " \t" << Point.val[2] << std::endl;
		//		}
		//	}
		//}
		//fout.close();
		//cv::Mat maskDisp(Mask);
		//cv::imshow("mask", maskDisp);
		//cv::waitKey();
		//std::cout << "press any key\n";
		//getchar();
	}
	/// End masking pixels out


	/// extract feature points
	m_params.hessianThreshold = pHessianThreshold;
	unsigned int j;
	for(j=0; j<imgs.size(); j++)
	{
		CvSeq *keys = 0, *des = 0;
		CvMemStorage* storage = cvCreateMemStorage(0);
	
		if (pMaskMode==0) Mask = cvCloneImage(imgs[j]);
		if(descriptors==0) cvExtractSURF(imgs[j], Mask, &keys, 0, storage, m_params );
		else cvExtractSURF(imgs[j], Mask, &keys, &des, storage, m_params );

		CvSeqReader key_reader, des_reader;
		if (keys!=0 && keys->total>0) cvStartReadSeq( keys, &key_reader );
		if(!m_UseOwnDescr && descriptors!=0 && des!=0 && des->total>0) cvStartReadSeq( des, &des_reader );
		
		if (keys!=0)
		{
			for(int i=0; i<keys->total; i++)
			{
				const CvSURFPoint* key = (const CvSURFPoint*)key_reader.ptr;
				CV_NEXT_SEQ_ELEM( key_reader.seq->elem_size, key_reader );
				pts.push_back(*key);
				id.push_back(j);

				if(descriptors!=0)
				{
					const float* des = (const float*)des_reader.ptr;
					CV_NEXT_SEQ_ELEM( des_reader.seq->elem_size, des_reader );
					descriptors->push_back(DblVector());
					for(int i=0; i<m_noDims; i++)
						descriptors->back().push_back(des[i]);
				}

				if(m_Draw)
				{
					float r = 0.5f * key->size;
					float phi = 3.1415f*key->dir/180.0f; ///< div by 2 pi
					int dx=cvRound(r*cos(phi));
					int dy=cvRound(r*sin(phi));
					int u=cvRound(key->pt.x);
					int v=cvRound(key->pt.y);	
					cvCircle(imgs[j], cvPoint(u, v), intmax(1, cvRound(r)), cvScalarAll(0), 3);
					cvLine(imgs[j], cvPoint(u, v), cvPoint(u+dx, v+dy), cvScalarAll(0), 3);
					cvCircle(imgs[j], cvPoint(u, v), intmax(1, cvRound(r)), cvScalarAll(255), 1);
					cvLine(imgs[j], cvPoint(u, v), cvPoint(u+dx, v+dy), cvScalarAll(255), 1);
				}
			}
		}
		if (keys!=0) cvRelease((void **)&keys);
		if (des!=0) cvRelease((void **)&des);
		cvReleaseMemStorage(&storage);
	}

	for(j=0; j<imgs.size(); j++)
	{
		if(m_Draw)
		{
			std::stringstream sstr;
			sstr << "featOut" << j << ".bmp";
			cvSaveImage(sstr.str().c_str(), imgs[j]);
		}
		cvReleaseImage(&(imgs[j]));
	}

	if (pMask != NULL) *pMask = Mask;
	else cvReleaseImage(&Mask);	
}


bool DetectorCore::GetFeaturePointData(SharedImage& si, CvSURFPoint& feature, int id, Frame& f, IntVector& k, int pMode)
{
	//pMode: 0=RGBI+channel of origin , 1=RGBI , 2=Intensity from color image , 3=HSVI+channel of origin, 4=HSVI

	/// get the center, continue if masked
	unsigned int u = intmin(cvRound(feature.pt.x), si.Shared()->width-1);
	unsigned int v = intmin(cvRound(feature.pt.y), si.Shared()->height-1);
	//uchar val = ((uchar*)(si.Shared()->imageData + v * si.Shared()->widthStep))[u];
	//if(val==0) return false;

	/// get the orbit point, continue if masked
	float r = 0.5f * feature.size;
	float phi = 3.1415f*(float)feature.dir/180.0f; ///< div by 2 pi
	int dx=cvRound(r*cos(phi));
	int dy=cvRound(r*sin(phi));
	int u_o = u+dx;
	int v_o = v+dy;
	if (u_o < 0 || u_o >= si.Coord()->width || v_o < 0 || v_o >= si.Coord()->height)
		return false;
	////uchar val_o = ((uchar*)(si.Shared()->imageData + v_o * si.Shared()->widthStep))[u_o];
	//if(val_o==0) return false;
	
	/// get the discrete key entries
	double x_c = ((float*)(si.Coord()->imageData + v * si.Coord()->widthStep))[u * si.Coord()->nChannels];	// central pixel
	double y_c = ((float*)(si.Coord()->imageData + v * si.Coord()->widthStep))[u * si.Coord()->nChannels+1];
	double z_c = ((float*)(si.Coord()->imageData + v * si.Coord()->widthStep))[u * si.Coord()->nChannels+2];
	if(x_c==0 && y_c==0 && z_c==0) return false; 
	double x_o = ((float*)(si.Coord()->imageData + v_o * si.Coord()->widthStep))[u_o * si.Coord()->nChannels];	// perimeter pixel
	double y_o = ((float*)(si.Coord()->imageData + v_o * si.Coord()->widthStep))[u_o * si.Coord()->nChannels+1];
	double z_o = ((float*)(si.Coord()->imageData + v_o * si.Coord()->widthStep))[u_o * si.Coord()->nChannels+2];
	if(x_o==0 && y_o==0 && z_o==0) return false; 
	double realRadius = sqrt(dblsqr(x_o-x_c) + dblsqr(y_o-y_c) + dblsqr(z_o-z_c));
	
	/// get/correct normal, check angle
	Point3Dbl zAxis;
	if(GetNormal(si.Coord(), u, v, zAxis)==RET_FAILED) return false;
	if(zAxis.m_z>0) zAxis.Negative();	///< correction the the normals look to the range sensor
	Point3Dbl  C(x_c, y_c, z_c);
	double phiN = zAxis.GetAngle(C);		
	if(phiN < m_threshPhi) return false;	// Ortsvektor C und Normale zAxis zeigen in verschiedene Richtungen, C von Kamera weg, zAxis zur Kamera hin, daher sind Winkel < 135� zwischen ihnen schlecht bestimmte Normalen

	/// get the local frame
	Point3Dbl xAxis(x_o-x_c, y_o-y_c, z_o-z_c);
	if(f.MakeFrameOriginDirectionsXZ(C, xAxis, zAxis)==RET_FAILED) return false;

	// make key

	if(m_UseKeys)
	{
		if (pMode==0 || pMode==3) k.push_back(id);				///< image id = channel of origin of feature point
		k.push_back(feature.laplacian);	///< laplace value (hill/hole/even) 1/-1/0
		k.push_back(cvRound(realRadius/m_radDiv));
	}
	else
	{	
		k.push_back(0);
	}

	/// draw
	//CvScalar col;
	//if(j==0) col = CV_RGB(255, 0, 0);
	//if(j==1) col = CV_RGB(0, 255, 0);
	//if(j==2) col = CV_RGB(0, 0, 255);

	return true;
}

//void DetectorCore::GetFPCloud(SharedImage& si, ObjectModel& model, ipa_utils::FPCloud2& fpc)
//{
//	/// init image channels
//	vector<IplImage*> imgs;
//	imgs.push_back(cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 1));
//	imgs.push_back(cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 1));
//	imgs.push_back(cvCreateImage(cvGetSize(si.Shared()), IPL_DEPTH_8U, 1));
//	cvSplit(si.Shared(), imgs[2], imgs[1], imgs[0], 0);
//
//	/// get features
//	std::vector<CvSURFPoint> features;
//	IntVector IDs;
//	DblMatrix descriptors;
//	GetFeaturePoints(si, features, IDs, &descriptors);
//
////	unsigned int j;
////	for(j=0; j<imgs.size(); j++)
////	{
////		CvSeq *keypoints = 0, *descriptors = 0;
////		CvMemStorage* storage = cvCreateMemStorage(0);
////		
////		cvExtractSURF(imgs[j], imgs[j], &keypoints, &descriptors, storage, m_params );
////
////		CvSeqReader key_reader, des_reader;
////		cvStartReadSeq( keypoints, &key_reader );
////		cvStartReadSeq( descriptors, &des_reader );
////		
////		for(int i=0; i<keypoints->total; i++)
////		{
////			const CvSURFPoint* key = (const CvSURFPoint*)key_reader.ptr;
////			const float* des = (const float*)des_reader.ptr;
////			CV_NEXT_SEQ_ELEM( key_reader.seq->elem_size, key_reader );
////			CV_NEXT_SEQ_ELEM( des_reader.seq->elem_size, des_reader );
////	   
////			/// get the image coordinates
////			unsigned int u = cvRound(key->pt.x);
////			unsigned int v = cvRound(key->pt.y);;
////			if(u>=w || v>=h) continue;
////
////			
////			/// retrieve the SURF descriptor
////			DblVector descrVec;
////			for(int d=0; d<m_noDims; d++)
////				descrVec.push_back(des[d]);
////			
////			/// check/append descriptors
////			if(mode==MODE_LEARN)
////			{
////				//assert(clusters!=0);
////				//clusters->push_back(descrVec);
////			}
////			else if(mode==MODE_DETECT)
////			{
////				assert(svm!=0);
////				/// get final discrete key entry
////				CvMat* _desc = 0;
////				int K = 0;//(int)svm->predict(_desc);
////				//std::cout << K << std::endl;
////				if(K>-1)
////				{
////					dkey.push_back(K);
////				}
////				else continue;
////			}
////			else assert(false);
////
////			
////			/// draw
////			cvCircle(si.Shared(), cvPoint(u, v), intmax(1, cvRound(r)), col, 1);
////			cvLine(si.Shared(), cvPoint(u, v), cvPoint(u+dx, v+dy), col, 1);
////
////			/// feature frame cloud entry
////			fpc.SetMember(dkey, f);
////		}
////
////		cvReleaseMemStorage(&storage);
////	}
////	cvSaveImage("featout.png", si.Shared());
//
//	for(unsigned int j=0; j<imgs.size(); j++)
//		cvReleaseImage(&(imgs[j]));
//}
