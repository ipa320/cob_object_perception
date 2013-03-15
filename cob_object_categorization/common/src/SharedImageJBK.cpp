#include "object_categorization/SharedImageJBK.h"

using namespace ipa_utils;

std::string SharedImage::m_CoordExtension = "_Coord";
std::string SharedImage::m_CoordExtensionX = "_X.png";
std::string SharedImage::m_CoordExtensionY = "_Y.png";
std::string SharedImage::m_CoordExtensionZ = "_Z.png";
std::string SharedImage::m_SharedExtension = "_Shared.png";
std::string SharedImage::m_IntenExtension = "_Inten.png";
std::string SharedImage::m_SaveSharedDisplay = "_SharedDisp.png";
std::string SharedImage::m_SaveCoordDisplay = "_CoordDisp.png";
std::string SharedImage::m_SaveIntenDisplay = "_IntenDisp.png";
unsigned int SharedImage::m_CoordDepth = IPL_DEPTH_32F;
unsigned int SharedImage::m_SharedDepth = IPL_DEPTH_8U;
unsigned int SharedImage::m_IntenDepthSource = IPL_DEPTH_32F;
unsigned int SharedImage::m_IntenDepthUsed = IPL_DEPTH_8U;
unsigned int SharedImage::m_CoordNChannels = 3;
unsigned int SharedImage::m_SharedNChannels = 3;
unsigned int SharedImage::m_IntenNChannels = 1;
SharedImageParams SharedImage::m_Parameters;

SharedImageParams::SharedImageParams()
{
	m_Initialized = true;

	m_ColCamIntrinsic =  (CvMat*)cvLoad((calibParameterPath+IntrinsicMatrixCCName).c_str());
	m_ColCamDistortion =  (CvMat*) cvLoad((calibParameterPath+DistortionCoeffsCCName).c_str());
	m_ExtrTranslationVector =  (CvMat*) cvLoad((calibParameterPath+ExtrinsicTrafoTranslationVectorName).c_str());
	m_ExtrRotationMatrix = (CvMat*) cvLoad((calibParameterPath+ExtrinsicTrafoRotationMatrixName).c_str());
	m_SRMapX = (IplImage*) cvLoad((calibParameterPath+UndistortXMapName).c_str());
	m_SRMapY = (IplImage*) cvLoad((calibParameterPath+UndistortYMapName).c_str());

	CvMat* intrinsic_matrix_SR = (CvMat*)cvLoad((calibParameterPath+IntrinsicMatrixSRName).c_str());
	CvMat* klm = (CvMat*)cvLoad((calibParameterPath+DistanceParamsName).c_str());
	
	IplImage* unDisX = (IplImage*)cvLoad((calibParameterPath+UndistortXMapName).c_str());
	IplImage* unDisY = (IplImage*)cvLoad((calibParameterPath+UndistortYMapName).c_str());

	if(m_ColCamIntrinsic==0 || m_ColCamDistortion==0 || m_ExtrTranslationVector==0 ||
		m_ExtrRotationMatrix==0 || m_SRMapX==0 || m_SRMapY==0 || unDisX==0 || unDisY==0)
	{
		m_Initialized=false;
		return;
	}
	else
	{
		if(intrinsic_matrix_SR!=0)
		{
			m_f = cvGetReal2D(intrinsic_matrix_SR, 0, 0);
			cvReleaseMat(&intrinsic_matrix_SR);

		}

		if(klm!=0)
		{
			m_k = cvGetReal1D(klm, 0);
			m_l = cvGetReal1D(klm, 1);
			m_m = cvGetReal1D(klm, 2);
			cvReleaseMat(&klm);
		}
	}
}

SharedImageParams::~SharedImageParams()
{
	if(m_ColCamIntrinsic!=0) cvReleaseMat(&m_ColCamIntrinsic);
	if(m_ColCamDistortion!=0) cvReleaseMat(&m_ColCamDistortion);
	if(m_ExtrTranslationVector!=0) cvReleaseMat(&m_ExtrTranslationVector);
	if(m_ExtrRotationMatrix!=0) cvReleaseMat(&m_ExtrRotationMatrix);
	if(m_SRMapX!=0) cvReleaseImage(&m_SRMapX);
	if(m_SRMapY!=0) cvReleaseImage(&m_SRMapY);
}

SharedImage::SharedImage()
{
	m_CoordImage = 0;
	m_SharedImage = 0;
	m_IntenImage = 0;
	m_Min = Point3Dbl(0);
	m_Max = Point3Dbl(0);
}

SharedImage::SharedImage(const SharedImage& si)
{
	if(si.m_CoordImage != 0)
	{
		m_CoordImage = cvCreateImage(cvGetSize(si.m_CoordImage), si.m_CoordImage->depth, si.m_CoordImage->nChannels);
		cvCopyImage(si.m_CoordImage, m_CoordImage);
	} else m_CoordImage = 0;

	if(si.m_SharedImage != 0)
	{
		m_SharedImage = cvCreateImage(cvGetSize(si.m_SharedImage), si.m_SharedImage->depth, si.m_SharedImage->nChannels);
		cvCopyImage(si.m_SharedImage, m_SharedImage);
	} else m_SharedImage = 0;

	if(si.m_IntenImage != 0)
	{
		m_IntenImage = cvCreateImage(cvGetSize(si.m_IntenImage), si.m_IntenImage->depth, si.m_IntenImage->nChannels);
		cvCopyImage(si.m_IntenImage, m_IntenImage);
	} else m_IntenImage = 0;

	m_Min = si.m_Min;
	m_Max = si.m_Max;
}

SharedImage& SharedImage::operator=(const SharedImage& si)
{
	/// Check for self-assignment
	if (this==&si)
	{
		 return *this;
	}

	if(si.m_CoordImage != 0)
	{
		m_CoordImage = cvCreateImage(cvGetSize(si.m_CoordImage), si.m_CoordImage->depth, si.m_CoordImage->nChannels);
		cvCopyImage(si.m_CoordImage, m_CoordImage);
	} else m_CoordImage = 0;

	if(si.m_SharedImage != 0)
	{
		m_SharedImage = cvCreateImage(cvGetSize(si.m_SharedImage), si.m_SharedImage->depth, si.m_SharedImage->nChannels);
		cvCopyImage(si.m_SharedImage, m_SharedImage);
	} else m_SharedImage = 0;

	if(si.m_IntenImage != 0)
	{
		m_IntenImage = cvCreateImage(cvGetSize(si.m_IntenImage), si.m_IntenImage->depth, si.m_IntenImage->nChannels);
		cvCopyImage(si.m_IntenImage, m_IntenImage);
	} else m_IntenImage = 0;

	m_Min = si.m_Min;
	m_Max = si.m_Max;

	return *this;
}

void SharedImage::Release(void)
{
	if(m_CoordImage)
	{
		cvReleaseImage(&m_CoordImage);
		m_CoordImage = 0;
	}
	if(m_SharedImage)
	{
		cvReleaseImage(&m_SharedImage);
		m_SharedImage = 0;
	}
	if(m_IntenImage)
	{
		cvReleaseImage(&m_IntenImage);
		m_IntenImage = 0;
	}
}

SharedImage::~SharedImage(void)
{
	Release();
}


int SharedImage::AllocateImages()
{
	m_CoordImage = cvCreateImage(SharedImageSize, m_CoordDepth, m_CoordNChannels);
	m_SharedImage = cvCreateImage(SharedImageSize, m_SharedDepth, m_SharedNChannels);
	m_IntenImage = cvCreateImage(SharedImageSize, m_IntenDepthUsed, m_IntenNChannels);

	cvSetZero(m_CoordImage);
	cvSetZero(m_SharedImage);
	cvSetZero(m_IntenImage);

	return RET_OK;
}

int SharedImage::LoadCoordinateImage(const std::string& Name)
{
	IplImage* tmpX = cvLoadImage((Name+m_CoordExtensionX).c_str());
	IplImage* tmpY = cvLoadImage((Name+m_CoordExtensionY).c_str());
	IplImage* tmpZ = cvLoadImage((Name+m_CoordExtensionZ).c_str());

	if(tmpX==0 || tmpY==0 || tmpZ==0) return RET_FAILED;

	m_CoordImage = cvCreateImage(cvGetSize(m_SharedImage), m_CoordDepth, m_CoordNChannels);

	for(int j=0; j<m_SharedImage->height; j++)
	{
		for(int i=0; i<m_SharedImage->width; i++)
		{
			CvScalar x = cvGet2D(tmpX, j, i);
			CvScalar y = cvGet2D(tmpY, j, i);
			CvScalar z = cvGet2D(tmpZ, j, i);

			short X, Y, Z;
			Convert3x8BitToInt((BYTE)x.val[0], (BYTE)x.val[1], (BYTE)x.val[2], X);
			Convert3x8BitToInt((BYTE)y.val[0], (BYTE)y.val[1], (BYTE)y.val[2], Y);
			Convert3x8BitToInt((BYTE)z.val[0], (BYTE)z.val[1], (BYTE)z.val[2], Z);
			cvSet2D(m_CoordImage, j, i, cvScalar((float)X, (float)Y, (float)Z));
		}
	}
	
	cvReleaseImage(&tmpX);
	cvReleaseImage(&tmpY);
	cvReleaseImage(&tmpZ);

	return RET_OK;
}

int SharedImage::LoadSharedImage(const std::string& Name)
{
	Release();
	std::stringstream n0, n1, n2;
	n1 << Name << m_SharedExtension;
	m_SharedImage = cvLoadImage(n1.str().c_str());
	n2 << Name << m_IntenExtension;
	m_IntenImage = cvLoadImage(n2.str().c_str(), 0);
	n0 << Name << m_CoordExtension;
	LoadCoordinateImage(n0.str().c_str());
	return RET_OK;
}

int SharedImage::DeleteSharedImage(const std::string& Name)
{
	std::string n0, n1, n2, n3, n4;
	n0 = Name + m_CoordExtension;
	remove((n0+m_CoordExtensionX).c_str());
	remove((n0+m_CoordExtensionY).c_str());
	remove((n0+m_CoordExtensionZ).c_str());
	n2 = Name + m_SharedExtension;
	remove(n2.c_str());
	n3 = Name + m_IntenExtension;
	remove(n3.c_str());
	return RET_OK;
}

int SharedImage::SaveCoordinateImage(const std::string& Name)
{
	IplImage* tmpX = cvCreateImage(cvGetSize(m_CoordImage), IPL_DEPTH_8U, 3);
	IplImage* tmpY = cvCreateImage(cvGetSize(m_CoordImage), IPL_DEPTH_8U, 3);
	IplImage* tmpZ = cvCreateImage(cvGetSize(m_CoordImage), IPL_DEPTH_8U, 3);

	for(int j=0; j<m_CoordImage->height; j++)
	{
		for(int i=0; i<m_CoordImage->width; i++)
		{
			CvScalar p = cvGet2D(m_CoordImage, j, i);
			short int x = (short int)cvRound(p.val[0]);
			BYTE lx, hx, sx;
			ConvertIntTo3x8Bit(x, lx, hx, sx);
			cvSet2D(tmpX, j, i, cvScalar(lx, hx, sx));

			if(m_CoordImage->nChannels>1)
			{
				BYTE ly, hy, sy, lz, hz, sz;
				short int y = (short int)cvRound(p.val[1]);
				short int z = (short int)cvRound(p.val[2]);
				ConvertIntTo3x8Bit(y, ly, hy, sy);
				ConvertIntTo3x8Bit(z, lz, hz, sz);
				cvSet2D(tmpY, j, i, cvScalar(ly, hy, sy));
				cvSet2D(tmpZ, j, i, cvScalar(lz, hz, sz));
			}
		}
	}
	
	std::string nameX = Name + m_CoordExtensionX;
	std::string nameY = Name + m_CoordExtensionY;
	std::string nameZ = Name + m_CoordExtensionZ;
	
	cvSaveImage(nameX.c_str(), tmpX);
	cvSaveImage(nameY.c_str(), tmpY);
	cvSaveImage(nameZ.c_str(), tmpZ);

	cvReleaseImage(&tmpX);
	cvReleaseImage(&tmpY);
	cvReleaseImage(&tmpZ);

	return RET_OK;
}

int SharedImage::SaveSharedImage(const std::string& Name)
{
	std::string n0, n1, n2, n3, n4;
	if(m_CoordImage != 0)
	{
		n0 = Name + m_CoordExtension;
		SaveCoordinateImage(n0);
	}
	else
		return RET_FAILED;
	
	if(m_SharedImage != 0)
	{
		n2 = Name + m_SharedExtension;
		cvSaveImage(n2.c_str(), m_SharedImage);
	} else return RET_FAILED;

	if(m_IntenImage != 0)
	{
		n3 = Name + m_IntenExtension;
		cvSaveImage(n3.c_str(), m_IntenImage);
	}
	else return RET_FAILED;
	return RET_OK;
}

unsigned long SharedImage::GetData(int i, int j, double& x, double& y, double& z,
		double& R, double& G, double& B)
{
	if(m_CoordImage==NULL || m_SharedImage==NULL) return RET_FAILED;
	CvScalar p = cvGet2D(m_CoordImage, j, i);
	CvScalar c = cvGet2D(m_SharedImage, j, i);
	x = p.val[0]; y = p.val[1]; z = p.val[2];
	R = c.val[2]; G = c.val[1]; B = c.val[0];
	return RET_OK;
}

void SharedImage::DisplayCoord(std::string WinName, bool Save)
{
	if(m_CoordImage!=NULL)
	{
		IplImage* ShowCoordImage = cvCreateImage(cvGetSize(m_CoordImage), IPL_DEPTH_8U, 3);
		CoordDisplayImageConversionOnlyZSpectral(m_CoordImage, ShowCoordImage); 
		cvShowImage(WinName.c_str(), ShowCoordImage);
		if(Save)
			cvSaveImage(m_SaveCoordDisplay.c_str(), ShowCoordImage);
		cvReleaseImage(&ShowCoordImage);
	}
}

void SharedImage::DisplayShared(std::string WinName, bool Save)
{
	if(m_SharedImage!=NULL)
	{
		cvShowImage(WinName.c_str(), m_SharedImage);
		if(Save) cvSaveImage(m_SaveSharedDisplay.c_str(), m_SharedImage);
	}
}

void SharedImage::DisplayInten(std::string WinName, bool Save)
{
	if(m_IntenImage!=NULL)
	{
		cvShowImage(WinName.c_str(), m_IntenImage);
		if(Save) cvSaveImage(m_SaveIntenDisplay.c_str(), m_IntenImage);
	}
}

void SharedImage::ExportToPointCloud(DblMatrix& M)
{
	//if(m_CoordImage==NULL ||m_SharedImage==NULL) return;
	//for(int j=0; j<m_CoordImage->height; j++)
	//{	
	//	for(int i=0; i<m_CoordImage->width; i++)
	//	{
	//		DblVector V;
	//		V.push_back(i);
	//		V.push_back(j);
	//		CvScalar v = cvGet2D(m_CoordImage, j, i);
	//		V.push_back(v.val[0]);
	//		V.push_back(v.val[1]);
	//		V.push_back(v.val[2]);
	//		CvScalar c = cvGet2D(m_SharedImage, j, i);
	//		V.push_back(c.val[0]);
	//		V.push_back(c.val[1]);
	//		V.push_back(c.val[2]);
	//		M.push_back(V);
	//	}
	//}
}

void SharedImage::ImportFromPointCloud(DblMatrix& M)
{
	//// if (!m_initialized) return;
	//// implement: get image sizes automatically
	//if(m_CoordImage==NULL ||m_SharedImage==NULL) return;
	//m_Min = Point3Dbl(DBL_MAX);
	//m_Max = Point3Dbl(-DBL_MAX);
	//int I, J;
	//for(int j=0; j<M.GetHeight(); j++)
	//{	
	//	I = (int)M[j][0];
	//	J = (int)M[j][1];
	//	cvSet2D(m_CoordImage, J, I, cvScalar(M[j][2], M[j][3], M[j][4]));
	//	if(M[j][2]<m_Min.m_x) m_Min.m_x = M[j][2];
	//	if(M[j][3]<m_Min.m_y) m_Min.m_y = M[j][3];
	//	if(M[j][4]<m_Min.m_z) m_Min.m_z = M[j][4];
	//	if(M[j][2]>m_Max.m_x) m_Max.m_x = M[j][2];
	//	if(M[j][3]>m_Max.m_y) m_Max.m_y = M[j][3];
	//	if(M[j][4]>m_Max.m_z) m_Max.m_z = M[j][4];
	//	cvSet2D(m_SharedImage, J, I, cvScalar(M[j][5], M[j][6], M[j][7]));
	//}
}

#ifdef __USE_SENSORS__

unsigned long SharedImage::GetImagesFromSensors(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam)
{
	if(m_CoordImage==NULL && m_SharedImage==NULL && m_IntenImage==NULL) AllocateImages();
	
	/// Temporal images
	IplImage* IntenImage = cvCreateImage(SharedImageSize, m_IntenDepthSource, m_IntenNChannels);
	IplImage* RangeImage = cvCreateImage(SharedImageSize, IPL_DEPTH_32F, 1);
	
	/// Get ranger images
	if(RangeCam->AcquireImages(RangeImage, IntenImage) & RET_FAILED)
	{
			std::cout << "SharedImage::GetImagesFromSensors: Capturing of range image failed.\n";
			return RET_FAILED;
	}

	/// Undistort images
	UndistortRangerImages(RangeImage, IntenImage, m_Parameters.m_SRMapX, m_Parameters.m_SRMapY);
	IntenDisplayImageConversion(IntenImage, m_IntenImage);

	/// Get xyz image
	GetXYZImageFromUndistortedRangeImage(	RangeImage, m_Parameters.m_f, m_Parameters.m_k,
											m_Parameters.m_l, m_Parameters.m_m, m_CoordImage);

	IplImage* ColorImage = cvCreateImage(ColorImageSize, IPL_DEPTH_8U, 3);
	if(ColorCam->GetColorImage(ColorImage) & RET_FAILED)
	{
		std::cout << "SharedImage::GetImagesFromSensors: Capturing of color image failed.\n";
		return RET_FAILED;
	}

	IplImage* zImg = cvCreateImage(OriginalColorImageSize, IPL_DEPTH_32F, 1);	///< init z-values image
	IplImage* uvImg = cvCreateImage(OriginalColorImageSize, IPL_DEPTH_16U, 2);	///< init pixel positions image
	cvSet(zImg, cvScalarAll(Z_MAX)); ///< Swissranger won't retrieve a value greater than 10 meter;
	cvSetZero(m_SharedImage); ///< init shared image
	
	//m_Min = Point3Dbl(DBL_MAX);		///< set mins
	//m_Max = Point3Dbl(-DBL_MAX);	///< set maxs
	//CvScalar val;
	double x, y, z, uc, vc;
	float X, Y, Z;
	unsigned int U, V, I, J, Udot, Vdot, old_i, old_j, old_I, old_J, R, G, B, JS, IS;
	//boost::progress_timer t(std::clog);
	for(int j=0; j<m_SharedImage->height; j++)
	{
		J = j * m_CoordImage->widthStep;
		JS = j * m_SharedImage->widthStep;
		for(int i=0; i<m_SharedImage->width; i++)
		{
			I = i * m_CoordImage->nChannels;
			//CvScalar val = cvGet2D(m_CoordImage, j, i);
			X = ((float*)(m_CoordImage->imageData + J))[I];
			Y = ((float*)(m_CoordImage->imageData + J))[I+1];
			Z = ((float*)(m_CoordImage->imageData + J))[I+2];
			//X = val.val[0]; Y = val.val[1]; Z = val.val[2]; 

			/// get color cam image pixel
			x = y = z = uc = vc = 0.0;
			TransformPoint(X, Y, Z, x, y, z, m_Parameters.m_ExtrRotationMatrix, m_Parameters.m_ExtrTranslationVector);
			PerspectiveProjection(x, y, z, uc, vc, m_Parameters.m_ColCamIntrinsic, m_Parameters.m_ColCamDistortion);
			U=cvRound(uc);
			V=cvRound(vc);
			
			// check if valid coordinates
			if(U<1 || U>=(unsigned int)ColorImage->width-1 || V<1 || V>=(unsigned int)ColorImage->height-1) continue;
			
			/// get color value
			Vdot = V * ColorImage->widthStep;
			Udot = U * ColorImage->nChannels;
			B = ((uchar *)(ColorImage->imageData + Vdot))[Udot];
			G = ((uchar *)(ColorImage->imageData + Vdot))[Udot + 1];
			R = ((uchar *)(ColorImage->imageData + Vdot))[Udot + 2];
		
			/// check if already occupied 
			float Z_old = ((float*)(zImg->imageData + V*zImg->widthStep))[U*zImg->nChannels];
			
			if(Z_old + Z_OFFSET <= Z) continue;

			/// set shared image color
			IS = i * m_SharedImage->nChannels;
			((uchar*)(m_SharedImage->imageData + JS))[IS]=B;
			((uchar*)(m_SharedImage->imageData + JS))[IS+1]=G;
			((uchar*)(m_SharedImage->imageData + JS))[IS+2]=R;

			if(Z < Z_old - Z_OFFSET)
			{
				/// set old pixel (and neighbors) to unvalid (black)
				old_j = ((int*)(uvImg->imageData + V * uvImg->widthStep))[U * uvImg->nChannels];
				old_i = ((int*)(uvImg->imageData + V * uvImg->widthStep))[U * uvImg->nChannels + 1];
				
				old_J = old_j * m_SharedImage->widthStep;
				old_I = old_i * m_SharedImage->nChannels;
				((uchar*)(m_SharedImage->imageData + old_J))[old_I]=0;
				((uchar*)(m_SharedImage->imageData + old_J))[old_I+1]=0;
				((uchar*)(m_SharedImage->imageData + old_J))[old_I+2]=0;
			}

			/// set z-value and pixel coordinates
			((float*)(zImg->imageData + V * zImg->widthStep))[U * zImg->nChannels] = Z;
			((int*)(uvImg->imageData + V * uvImg->widthStep))[U * uvImg->nChannels] = j;
			((int*)(uvImg->imageData + V * uvImg->widthStep))[U * uvImg->nChannels + 1] = i;
		}
	}
	

	/// Release temp images
	cvReleaseImage(&RangeImage);
	cvReleaseImage(&ColorImage);
	cvReleaseImage(&zImg);
	cvReleaseImage(&uvImg);

	return RET_OK;
}

unsigned long SharedImage::GetRawImagesFromSensors(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam)
{
	/// Release previously used images
	Release();
	
	/// Create raw range and color image stores and initialize new images
	CvSize RangeSize = SharedImageSize;
	CvSize ColorSize = ColorImageSize;
	m_CoordImage = cvCreateImage(RangeSize, m_CoordDepth, 1);
	m_SharedImage = cvCreateImage(ColorSize, m_SharedDepth, m_SharedNChannels);
	IplImage* IntenImage = cvCreateImage(RangeSize, m_IntenDepthSource, m_IntenNChannels);

	/// Acquire images
	if(RangeCam->AcquireImages(m_CoordImage, IntenImage, false) & RET_FAILED) return RET_FAILED;
	if(ColorCam->GetColorImage(m_SharedImage) & RET_FAILED) return RET_FAILED;
	//cvSaveImage("test.bmp", m_SharedImage);
	m_IntenImage = cvCreateImage(RangeSize, m_IntenDepthUsed, m_IntenNChannels);
	IntenDisplayImageConversion(IntenImage, m_IntenImage);

	return RET_OK;
}

#endif // __USE_SENSORS__

void SharedImage::DoRangeSegmentation(Frame& F, double Rad, double zMin, double zMax, int Cut)
{
	if(m_CoordImage==NULL || m_SharedImage==NULL) return;

	IplImage* SegmentedImage = cvCreateImage(cvGetSize(m_SharedImage), m_SharedImage->depth, m_SharedImage->nChannels); 
	cvSetZero(SegmentedImage);
	IplImage* NewCoordImage = cvCloneImage(m_CoordImage);
	int i, j;
	for(j=0; j<m_CoordImage->height; j++)
	{
		for(i=0; i<m_CoordImage->width; i++)
		{
			// write into the local frame
			CvScalar p = cvGet2D(m_CoordImage, j, i);
			Point3Dbl P(p.val[0], p.val[1], p.val[2]);
			Point3Dbl q;
			F.ToFrame(P, q);
			
			bool flag = false;
			if(!(i==0 || j==0 || i==m_CoordImage->width-1 || j==m_CoordImage->height-1))
			{
				// evaluate cylinder memership
				double r = sqrt(q.m_x*q.m_x + q.m_y*q.m_y);
				if(r<=Rad && q.m_z>=zMin && q.m_z<=zMax)
				{
					// evaluate surface normal
					Point3Dbl n;
					GetNormal(m_CoordImage, i ,j, n);
					if(n.m_z>0) n.Negative();
					double phi = n.GetAngle(P);
					double thresh = 0.6*THE_PI_DEF;

					if(phi > thresh)
					{
						flag = true;
					}
					else
					{
						flag = false;
					}
				}
			}
			
			if(flag==true)
			{
				CvScalar col = cvGet2D(m_SharedImage, j, i);
				cvSet2D(SegmentedImage, j, i, col);
			}
			else
			{
				cvSet2D(NewCoordImage, j, i, cvScalarAll(0));
				cvSetReal2D(m_IntenImage, j, i, 0);
			}
		}
	}

	cvCopy(SegmentedImage, m_SharedImage);
	cvCopy(NewCoordImage, m_CoordImage);
	cvReleaseImage(&SegmentedImage);
	cvReleaseImage(&NewCoordImage);

	if(Cut>0)
		CutImageBorder(Cut);

	//cvReleaseImage(&MaskImage);
	//t1.SetNow();
	//std::cout << "t " << t1-t0 << std::endl;
}

void SharedImage::CutImageBorder(int CutWidth)//, int MaskVal)
{
	/// Get the center
	double cx=0, cy=0, cnt=0.0;
	for(int j=0; j<m_CoordImage->height; j++)
	{
		for(int i=0; i<m_CoordImage->width; i++)
		{
			CvScalar val = cvGet2D(m_SharedImage, j, i);
			//std::cout << val.val[0];
			if(val.val[0]+val.val[1]+val.val[2] != 0)
			{	
				cx *= cnt;
				cx += i;
				cy *= cnt;
				cy += j;
				
				cnt++;
				
				cx /=cnt;
				cy /=cnt;
			}
		}
	}
	
	/// Get the new smaller image	
	IplImage* NewCoord=cvCreateImage(cvSize(CutWidth,CutWidth), m_CoordDepth, m_CoordNChannels);
	IplImage* NewShared=cvCreateImage(cvSize(CutWidth,CutWidth), m_SharedDepth, m_SharedNChannels);
	IplImage* NewInten=cvCreateImage(cvSize(CutWidth,CutWidth), m_IntenDepthUsed, m_IntenNChannels);

	/// Get center and copy area 
	///*int cx= m_CoordImage->width/2, cy= m_CoordImage->height/2;
	//if(Flag==4)
	//{
	//	cx = (r+l)/2;
	//	cy = (b+t)/2;
	//}*/

	cvSetZero(NewCoord);
	cvSetZero(NewShared);
	cvSetZero(NewInten);
	
	for(int j=0; j<CutWidth; j++)
	{
		for(int i=0; i<CutWidth; i++)
		{
			int I = cvRound(cx)-CutWidth/2+i;
			int J = cvRound(cy)-CutWidth/2+j;
			if(I<0 || I>=m_CoordImage->width || J<0 || J>=m_CoordImage->height) continue;
			cvSet2D(NewCoord, j, i, cvGet2D(m_CoordImage, J, I));
			cvSet2D(NewShared, j, i, cvGet2D(m_SharedImage, J, I));
			cvSet2D(NewInten, j, i, cvGet2D(m_IntenImage, J, I));
		}
	}
	
	//cvSetImageROI(m_CoordImage, cvRect(cx-CutWidth/2, cy-CutWidth/2, CutWidth, CutWidth));
	//cvSetImageROI(m_SharedImage, cvRect(cx-CutWidth/2, cy-CutWidth/2, CutWidth, CutWidth));
	//cvSetImageROI(m_IntenImage, cvRect(cx-CutWidth/2, cy-CutWidth/2, CutWidth, CutWidth));
	//cvCopy(m_CoordImage, NewCoord);
	//cvCopy(m_SharedImage, NewShared);
	//cvCopy(m_IntenImage, NewInten);

	/// Replace images
	cvReleaseImage(&m_CoordImage);
	cvReleaseImage(&m_SharedImage);
	cvReleaseImage(&m_IntenImage);
	m_CoordImage=cvCloneImage(NewCoord);
	m_SharedImage=cvCloneImage(NewShared);
	m_IntenImage=cvCloneImage(NewInten);
	//m_SharedImageSize.width = CutWidth;
	//m_SharedImageSize.height = CutWidth;

	cvReleaseImage(&NewCoord);
	cvReleaseImage(&NewShared);
	cvReleaseImage(&NewInten);

}

void SharedImage::OrientAlongPrincipalAxises()
{
	/// Get the points
	std::vector<SharedImagePoint> points;
	int i, j, k;
	for(j=0; j<SharedImageSize.height; j++)
	{
		for(i=0; i<SharedImageSize.width; i++)
		{
			CvScalar p = cvGet2D(Coord(), j, i);
			if(p.val[0]==0 && p.val[1]==0 && p.val[2]==0) continue;
			else
			{
				points.push_back(SharedImagePoint(p.val[0], p.val[1], p.val[2], i, j));
			}
		}
	}
	
	/// Build data arrays
	CvMat* data = cvCreateMat((int)points.size(), 3, CV_32FC1);
	CvMat* result = cvCreateMat((int)points.size(), 3, CV_32FC1);
	CvMat* avg =  cvCreateMat(1, 3, CV_32FC1);
	CvMat* eigenvalues = cvCreateMat(1, 3, CV_32FC1);
	CvMat* eigenvectors = cvCreateMat(3, 3, CV_32FC1);
	for(k=0; k<(int)points.size(); k++)
	{
		cvSetReal2D(data, k, 0, points[k].x);
		cvSetReal2D(data, k, 1, points[k].y);
		cvSetReal2D(data, k, 2, points[k].z);
	}

	/// Get PCA
	cvCalcPCA(data, avg, eigenvalues, eigenvectors, CV_PCA_DATA_AS_ROW);

	/// Apply transformation
	cvProjectPCA(data, avg, eigenvectors, result);

	for(k=0; k<(int)points.size(); k++)
	{
		double x = cvGetReal2D(result, k, 0);
		double y = cvGetReal2D(result, k, 1);
		double z = cvGetReal2D(result, k, 2);
		cvSet2D(Coord(), points[k].j, points[k].i, cvScalar(x, y, z));
	}

	cvReleaseMat(&data);
	cvReleaseMat(&result);
	cvReleaseMat(&avg);
	cvReleaseMat(&eigenvalues);
	cvReleaseMat(&eigenvectors);
}

unsigned long SharedImage::GetCorrespondences(SharedImage& si, std::vector<PixelNeighborStruct>& corrs, int kernel, double colDistThresh)
{
	int i, j, k, l;
	for(j=kernel; j<m_CoordImage->height-kernel-1; j++)
	{
		for(i=kernel; i<m_CoordImage->width-kernel-1; i++)
		{
			/// Get source color and coordinates
			CvScalar colS = cvGet2D(Shared(), j, i);
			CvScalar pS = cvGet2D(Coord(), j, i);
			
			/// Check if source pixel is masked out
			if(colS.val[0]==0 && colS.val[1]==0 && colS.val[2]==0) continue;
					
			double bestDist=DBL_MAX;
			//Point3Dbl bestPoint;
			int bI, bJ, bK, bL; 
			for(l=j-kernel; l<=j+kernel; l++)
			{
				for(k=i-kernel; k<=i+kernel; k++)
				{
					/// Get target color and coordinates
					CvScalar colT = cvGet2D(si.Shared(), l, k);
					CvScalar pT = cvGet2D(si.Coord(), l, k);

					/// Check if target pixel is masked out
					if(colT.val[0]==0 && colT.val[1]==0 && colT.val[2]==0) continue;
			
					/// Update color distance
					double dist = sqrt(dblsqr(colS.val[0]-colT.val[0])+dblsqr(colS.val[1]-colT.val[1])+dblsqr(colS.val[2]-colT.val[2]));
					double distMetric =	sqrt(dblsqr(pS.val[0]-pT.val[0])+dblsqr(pS.val[1]-pT.val[1])+dblsqr(pS.val[2]-pT.val[2]));

					if(dist<=colDistThresh && distMetric<bestDist)
					{
						bestDist=distMetric;
						//bestPoint = Point3Dbl(pT.val[0], pT.val[1], pT.val[2]);
						bI = i; bJ = j; bK=k; bL=l;
					}
				}	
			}

			if(bestDist<DBL_MAX)
			{
				PixelNeighborStruct p;
				p.i = bI; p.j=bJ; p.k = bK; p.l = bL; p.dCoor = bestDist;
				corrs.push_back(p);
			}
		}
	}
	return RET_OK;
}

unsigned long SharedImage::GetTransformation(SharedImage& si, Mat3d* rot, Vec3d* trans, double min, double max)
{ 

	/// Get and sort the correspondences
	std::vector<PixelNeighborStruct> corrs;
	GetCorrespondences(si, corrs);
	std::sort(corrs.begin(), corrs.end(), SortPixelNeighborStruct);
	if(corrs.size()<3) return RET_FAILED;

	/// Build the point sets
	static const int maxPoints = 10000;
	Vec3d sourcePoints[maxPoints];
	Vec3d targetPoints[maxPoints];
	int k, l=0;
	for(k=(int)((double)corrs.size()*min); k<intmin((int)corrs.size(), intmin(maxPoints, (int)ceil((double)corrs.size()*max))); k++, l++)
	{
		CvScalar pS = cvGet2D(Coord(), corrs[k].j, corrs[k].i);
		CvScalar pT = cvGet2D(si.Coord(), corrs[k].l, corrs[k].k);
		sourcePoints[l].x=(float)pS.val[0];
		sourcePoints[l].y=(float)pS.val[1];
		sourcePoints[l].z=(float)pS.val[2];
		targetPoints[l].x=(float)pT.val[0];
		targetPoints[l].y=(float)pT.val[1];
		targetPoints[l].z=(float)pT.val[2];
	}
	if(l<2) return RET_FAILED;

	/// Get the relative frame
	CICP* getFrame = new CICP();
	int ret=0;
	if(getFrame->CalculateOptimalTransformation(targetPoints, sourcePoints, l, *rot, *trans)) ret = RET_OK;
	else ret = RET_FAILED;
	delete getFrame;
	return ret;
}

unsigned long SharedImage::GetTransformation(SharedImage& si, SharedImage& si1, Mat3d* rot, Vec3d* trans, double min, double max)
{ 

	/// Get and sort the correspondences
	std::vector<PixelNeighborStruct> corrs;
	GetCorrespondences(si, corrs);
	std::sort(corrs.begin(), corrs.end(), SortPixelNeighborStruct);
	if(corrs.size()<3) return RET_FAILED;

	/// Build the point sets
	static const int maxPoints = 10000;
	Vec3d sourcePoints[maxPoints];
	Vec3d targetPoints[maxPoints];
	int k, l=0;
	for(k=(int)((double)corrs.size()*min); k<intmin((int)corrs.size(), intmin(maxPoints, (int)ceil((double)corrs.size()*max))); k++, l++)
	{
		CvScalar pS = cvGet2D(Coord(), corrs[k].j, corrs[k].i);
		CvScalar pT = cvGet2D(si.Coord(), corrs[k].l, corrs[k].k);
		sourcePoints[l].x=(float)pS.val[0];
		sourcePoints[l].y=(float)pS.val[1];
		sourcePoints[l].z=(float)pS.val[2];
		targetPoints[l].x=(float)pT.val[0];
		targetPoints[l].y=(float)pT.val[1];
		targetPoints[l].z=(float)pT.val[2];
	}
	if(l<2) return RET_FAILED;

	/// Get the relative frame
	CICP* getFrame = new CICP();
	int ret = 0;
	if(getFrame->CalculateOptimalTransformation(targetPoints, sourcePoints, l, *rot, *trans)) ret = RET_OK;
	else ret = RET_FAILED;
	delete getFrame;
	return ret;
}

void SharedImage::ApplyTransformationToCoordImage(SharedImage& si, Mat3d& rot, Vec3d& trans)
{
	/// Convert the target cloud
	int i, j;
	for(j=0; j<m_CoordImage->height; j++)
	{
		for(i=0; i<m_CoordImage->width; i++)
		{
			/// Get target point
			CvScalar pT = cvGet2D(si.Coord(), j, i);
					
			/// Check if target point is masked out
			if(pT.val[0]==0 && pT.val[1]==0 && pT.val[2]==0) continue;
			
			/// Convert and execute transformation
			Vec3d vT, vT2; vT.x = (float)pT.val[0]; vT.y = (float)pT.val[1]; vT.z = (float)pT.val[2];  
			Math3d::MulMatVec(rot, vT, trans, vT2);

			/// Overwrite coordinates
			cvSet2D(si.Coord(), j, i, cvScalar(vT2.x, vT2.y, vT2.z));
		}
	}
}

//void SharedImage::AlignImages(SharedImage& si, int noIt, int kernel, double distThresh, double colDistThresh, double eps)
//{
//	Mat3d rot;
//	Vec3d trans;
//	double mean=0.0, sigma=DBL_MAX;
//
//	for(int i=0; i<noIt; i++)
//	{
//		int noPoints = GetTransformation(si, rot, trans, mean, sigma, distThresh, kernel, colDistThresh, eps);
//		if(noPoints>3)
//			ApplyTransformationToCoordImage(si, rot, trans);
//		else break;
//	}
//}

void SharedImage::GetClosestUV(double x, double y, double z, int& u, int& v)
{
	int i, j;
	double minDist = DBL_MAX;
	for(j=0; j<m_CoordImage->height; j++)
	{
		for(i=0; i<m_CoordImage->width; i++)
		{
			CvScalar p = cvGet2D(m_CoordImage, j, i);
			double dist = sqrt(dblsqr(p.val[0]-x)+dblsqr(p.val[1]-y));//+dblsqr(p.val[2]-z));
			if(dist<minDist)
			{
				minDist = dist;
				u = i;
				v = j;
			}
		}
	}
}
//void SharedImage::AlignColorICP(SharedImage& si, int itMax, int noCorr)
//{
//	double bound = 100.0;
//	double epsMean = 5.0;
//	for(int it=0; it<itMax; it++)
//	{
//		/// For each pixel in source get the nearest color pixel in target
//		double mean, min, max;
//		int noPoints=0;
//		std::vector<PixelNeighborStruct> Neighbors;
//		noPoints = GetNearestColorNeigbors(si, Neighbors, bound, mean, min, max);
//		if(noPoints<noCorr || mean<epsMean) return;
//
//		/// Sort according to distances (larger distances first)
//		std::sort(Neighbors.begin(), Neighbors.end(), SortPixelNeighborStruct);
//
//		/// Take the first noCorr tuples and build a correspondence set
//		static const int maxPoints = 10000;
//		Vec3d sourcePoints[maxPoints];
//		Vec3d targetPoints[maxPoints];
//		CvScalar pS, pT;
//		for(int i=0; i<noCorr; i++)
//		{
//			pS = cvGet2D(Coord(), Neighbors[i].j, Neighbors[i].i);
//			pT = cvGet2D(si.Coord(), Neighbors[i].l, Neighbors[i].k);
//			sourcePoints[i].x=(float)pS.val[0];
//			sourcePoints[i].y=(float)pS.val[1];
//			sourcePoints[i].z=(float)pS.val[2];
//			targetPoints[i].x=(float)pT.val[0];
//			targetPoints[i].y=(float)pT.val[1];
//			targetPoints[i].z=(float)pT.val[2];		
//		}
//
//		/// Estimate and apply transformation to si
//		CICP getFrame;
//		Mat3d rot;
//		Vec3d trans;
//		getFrame.CalculateOptimalTransformation(targetPoints, sourcePoints, noPoints, rot, trans);
//		ApplyTransformationToCoordImage(si, rot, trans); 
//
//		///// Update to tighter bound
//		bound = mean;
//	}
//}
//
bool SortPixelNeighborStruct(const PixelNeighborStruct& s0, const PixelNeighborStruct& s1)
{
	return s0.dCoor>s1.dCoor;
}

//int SharedImage::GetNearestColorNeigbors(SharedImage& si, std::vector<PixelNeighborStruct>& Neighbors, double bound, double& mean, double& min, double& max)
//{
//	double count=0.0;
//	mean=0.0; min=DBL_MAX; max=0.0; 
//	int winSize = 19;
//	for(int j=0; j<SharedImageSize.height; j++)
//	{
//		for(int i=0; i<SharedImageSize.width; i++)
//		{
//			/// Get source color
//			CvScalar sourceCol = cvGet2D(Shared(), j, i);
//
//			/// Mask zero pixels
//			if(sourceCol.val[0]==0 && sourceCol.val[1]==0 && sourceCol.val[2]==0) continue;
//			
//			/// Get source coordinates
//			CvScalar sourceCoord = cvGet2D(Coord(), j, i);
//
//			/// Loop over target image
//			double distColMin = DBL_MAX;
//			PixelNeighborStruct bestCorr;
//			bestCorr.i = i;
//			bestCorr.j = j;
//			bool flag = false;
//			for(int l=j-winSize; l<=j+winSize; l++)
//			{
//				for(int k=i-winSize; k<=i+winSize; k++)
//				{
//					/// Check for valid location
//					if(k<0 || k>=SharedImageSize.width || l<0 || l>=SharedImageSize.height) continue;
//
//					/// Get the target coordinates
//					CvScalar targetCoord = cvGet2D(si.Coord(), l, k);
//					
//					/// Mask zero pixels
//					if(targetCoord.val[0]==0 && targetCoord.val[1]==0 && targetCoord.val[2]==0) continue;
//				
//					/// Terminate in distance too large
//					double distCoord =  sqrt(dblsqr(sourceCoord.val[0]-targetCoord.val[0])+dblsqr(sourceCoord.val[1]-targetCoord.val[1])+dblsqr(sourceCoord.val[2]-targetCoord.val[2]));
//
//					/// Get the color distance
//					CvScalar sourceCol = cvGet2D(Shared(), l, k);
//					CvScalar targetCol = cvGet2D(si.Shared(), l, k);
//					double distCol = sqrt(dblsqr(sourceCol.val[0]-targetCol.val[0])+dblsqr(sourceCol.val[1]-targetCol.val[1])+dblsqr(sourceCol.val[2]-targetCol.val[2]));
//					if(distCoord>bound) continue;
//
//					/// If the color distance is smallest so far, then overwrite best
//					if(distCol<distColMin || (distCol==distColMin && bestCorr.dCoor>distCoord))
//					{
//						bestCorr.k = k;
//						bestCorr.l = l;
//						bestCorr.dCol = distCol;
//						bestCorr.dCoor = distCoord;
//						distColMin = distCol;
//						flag=true;
//					}
//				}
//			}
//
//			/// Add new correspondence to the list
//			if(flag==true)
//			{
//				Neighbors.push_back(bestCorr);
//				
//				/// Update statistics
//				mean += bestCorr.dCoor;
//				if(bestCorr.dCoor > max) max = bestCorr.dCoor;
//				if(bestCorr.dCoor < min) min = bestCorr.dCoor;
//				count++;
//				//std::cout << ".";
//			}	
//		}
//	}
//	//std::cout << std::endl;
//
//	/// Get mean
//	mean/=count;
//
//	return (int)count;
//}
//
//void SharedImage::ApplyTransformationToCoordImage(SharedImage& si, Mat3d& rot, Vec3d& trans)
//{
//	/// Convert the target cloud
//	int i, j;
//	for(j=0; j<SharedImageSize.height; j++)
//	{
//		for(i=0; i<SharedImageSize.width; i++)
//		{
//			/// Get target point
//			CvScalar pT = cvGet2D(si.Coord(), j, i);
//					
//			/// Check if target point is masked out
//			if(pT.val[0]==0 && pT.val[1]==0 && pT.val[2]==0) continue;
//			
//			/// Convert and execute transformation
//			Vec3d vT, vT2; vT.x = (float)pT.val[0]; vT.y = (float)pT.val[1]; vT.z = (float)pT.val[2];  
//			Math3d::MulMatVec(rot, vT, trans, vT2);
//
//			/// Overwrite coordinates
//			cvSet2D(si.Coord(), j, i, cvScalar(vT2.x, vT2.y, vT2.z));
//		}
//	}
//}

// get shared image from raw images
/*
void SharedImage::GetFromRawImages(IplImage* ColorImage, IplImage* RangeImage, bool RangeSegmentation, Point3& Center, double Radius)
{
	// get coordinate image
	IplImage* CoordImage = cvCreateImage(cvGetSize(RangeImage), IPL_DEPTH_32F, 3);
	SR31 Ranger;
	//Ranger.GetCoordImageFromRangeImage(RangeImage, CoordImage);
	assert(false);

	// copy/resize coordinate image
	cvResize(CoordImage, m_CoordImage);
	
	// get shared image
	IplImage* SmoothedImage = cvCreateImage(cvGetSize(ColorImage), IPL_DEPTH_8U, 3);
	cvSmooth(ColorImage, SmoothedImage, CV_GAUSSIAN, 0, 0, SmoothingSigma);
	
	// set mins/maxs
	m_Min = Set(DBL_MAX);
	m_Max = Set(-DBL_MAX);

	// get data
	CvScalar Val;
	for(int j=0; j<SharedImageHeight; j++)
		for(int i=0; i<SharedImageWidth; i++)
		{
			Val = cvGet2D(m_CoordImage, j, i);

			double uc, vc;
			m_Trafo.GetImageCoordinates(Set(Val.val[0], Val.val[1], Val.val[2]), uc, vc);
			int UC = cvRound(uc); int VC = cvRound(vc);
			
			// check if valid coordinates
			if(UC<0 || UC>=ColorImageWidth || VC<0 || VC>=ColorImageHeight) continue;
			
			// get color
			CvScalar Color = cvGet2D(ColorImage, VC, UC);
			
			if(Val.val[0]<m_Min.x) m_Min.x = Val.val[0];
			else if(Val.val[0]>m_Max.x) m_Max.x = Val.val[0];

			if(Val.val[1]<m_Min.y) m_Min.y = Val.val[1];
			else if(Val.val[1]>m_Max.y) m_Max.y = Val.val[1];
			
			if(Val.val[2]<m_Min.z) m_Min.z = Val.val[2];
			else if(Val.val[2]>m_Max.z) m_Max.z = Val.val[2];

			cvSet2D(m_SharedImage, j, i, Color);
		}
	
	if(RangeSegmentation) DoRangeSegmentation(Center, Radius);
	cvReleaseImage(&SmoothedImage);
	cvReleaseImage(&CoordImage);
}
*/

//SharedImageSequence::SharedImageSequence(std::string Name)
//{
//	m_Name=Name;
//}
