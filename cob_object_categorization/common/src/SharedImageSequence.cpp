#include "object_categorization/SharedImageSequence.h"

using namespace ipa_utils;

std::string SharedImageSequence::m_InfFileAttachment = "_info.txt";
std::string SharedImageSequence::m_Spacing = "_";
std::string SharedImageSequence::m_PCloudAttachement = ".pcl";
std::string SharedImageSequence::m_ColorWinName = "Color";
std::string SharedImageSequence::m_CoordWinName = "Coordinates";

SharedImageSequence::SharedImageSequence()
{
}

SharedImageSequence::~SharedImageSequence()
{

}

void SharedImageSequence::DeleteSharedImageSequence(const std::string& Name)
{
	// load old info header
	std::stringstream FileNameStream;
	FileNameStream << Name << m_InfFileAttachment;
	int s=0;
	std::ifstream f(FileNameStream.str().c_str());
	if(!f.is_open()) return;

	f >> s;
	SharedImage Dummy;
	for(int i=0; i<s; i++)
	{
		std::stringstream FileNameStream2;
		FileNameStream2 << Name << m_Spacing << i;
		Dummy.DeleteSharedImage(FileNameStream2.str());
		i++;
	}
	f.close();

	std::string name = FileNameStream.str();
	removeFile(name);
}

int SharedImageSequence::SaveSharedImageSequence(const std::string& Name)
{
	// delete old sequence
	DeleteSharedImageSequence(Name);
	
	// save info header
	std::cout << "SharedImageSequence::SaveSharedImageSequence: Saving " << Name << "\n";
	std::stringstream FileNameStream;
	FileNameStream << Name << m_InfFileAttachment;
	//std::cout << FileNameStream; 
	std::ofstream f((FileNameStream.str()).c_str());
	if(!f.is_open()) return RET_FAILED;
	f << (int)this->size() << std::endl;
	SharedImageSequence::iterator it;
	int i=0;
	for(it=begin(); it!=end(); it++)
	{
		std::stringstream FileNameStream2;
		FileNameStream2 << Name << m_Spacing << i;
		it->SaveSharedImage(FileNameStream2.str());
		std::cout << i << " ";
		i++;
	}
	return RET_OK;
}

int SharedImageSequence::LoadSharedImageSequence(const std::string& filename, unsigned int Limit, int k)
{
	clear();
	std::cout << "Loading image sequence " << filename << "\n";
	std::stringstream FileNameStream;
	FileNameStream << filename << m_InfFileAttachment;
	std::ifstream f((FileNameStream.str()).c_str());
	if(!f.is_open())
	{
		std::cout << "SharedImageSequence::LoadSharedImageSequence: Error while opening file " 
			      << FileNameStream.str().c_str() << ".\n";
		return RET_FAILED;
	}

	/// Read number of images that are stored on disk
	int s=0; f >> s; s=intmin(Limit, s);

	std::cout << "SharedImageSequence::LoadSharedImageSequence: Loading ";
	std::list<SharedImage> listSI;
	int i;
	for(i=0; i<s; i+=k)
	{
		std::stringstream FileNameStream2;
		FileNameStream2 << filename << m_Spacing << i;
		
		SharedImage Tmp;
		
		/// Load the single images (coordinate and shared image) from disk
		if(Tmp.LoadSharedImage(FileNameStream2.str())==RET_FAILED) return RET_FAILED;
		push_back(Tmp);
		//listSI.push_back(Tmp);
		std::cout << i << " ";
	}

	//	std::list<SharedImage>::iterator it;
	//	(*this).assign(s, SharedImage());
	//	i=0;
	//	for(it=listSI.begin(); it!=listSI.end(); it++)
	//	{
	//		(*this)[i]=*it;
	//		i++;
	//	}

	std::cout << "\n";

	return RET_OK;
}

void SharedImageSequence::ExportSequenceAsPointClouds(const std::string& Name)
{
	SharedImageSequence::iterator it;
	int i=0;
	for(it=begin(); it!=end(); it++)
	{
		std::stringstream FileNameStream;
		FileNameStream << Name << i << m_PCloudAttachement;
		DblMatrix M;
		it->ExportToPointCloud(M);
		M.Save(FileNameStream.str());
		i++;
	}
}

void SharedImageSequence::SegmentImageSequence(Frame& F, double Radius, double zMin, double zMax, int Cut)
{
	SharedImageSequence::iterator it;
	for(it=begin(); it!=end(); it++)
	{
		it->DoRangeSegmentation(F, Radius, zMin, zMax, Cut);
	}
}

void SharedImageSequence::SlideShow()
{
	cvNamedWindow(m_ColorWinName.c_str());
	cvNamedWindow(m_CoordWinName.c_str());
	SharedImageSequence::iterator it;
	for(it=begin(); it!=end(); it++)
	{
		it->DisplayCoord(m_CoordWinName);
		it->DisplayShared(m_ColorWinName);
		char c = cvWaitKey(1000);
		if(c=='q') break;
	}
	cvDestroyAllWindows();
}

#ifdef __USE_SENSORS__

void SharedImageSequence::GetSharedImageSequenceManually(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam, const CvSize& SharedImageSize)
{

	cvNamedWindow(m_CoordWinName.c_str());
	cvNamedWindow(m_ColorWinName.c_str());

	char c=-1;
	SharedImage SImg;
	SImg.GetImagesFromSensors(RangeCam, ColorCam, SharedImageDefaultSize);

	SImg.DisplayCoord(m_CoordWinName);
	SImg.DisplayShared(m_ColorWinName);

	int cnt=0;
	while(cvGetWindowHandle(m_ColorWinName.c_str()) && cvGetWindowHandle(m_CoordWinName.c_str()))
	{

		std::cout << "SharedImageSequence::GetSharedImageSequence: Press 'n' to take a training image, 's' to save the image, or 'q' to quit.\n";

		c = cvWaitKey();

		std::cout << "SharedImageSequence::GetSharedImageSequence: " << c << ".\n";
		if(c=='q' || c=='Q')
		{
			break;
		}
		else if (c=='n' || c=='N')
		{
		#ifndef __USE_SHAREDIMAGE_JBK__
			SImg.GetImagesFromSensors(RangeCam, ColorCam, SharedImageDefaultSize);
		#endif
		#ifdef __USE_SHAREDIMAGE_JBK__
			SImg.GetImagesFromSensors(RangeCam, ColorCam);
		#endif
			SImg.DisplayCoord(m_CoordWinName);
			SImg.DisplayShared(m_ColorWinName);
                }
		else if (c=='s' || c=='S')
		{
			push_back(SImg);
			std::cout << "SharedImageSequence::GetSharedImageSequence: ... one image sucessfully acquired." << std::endl;
			cnt++;
		}
	}
	cvDestroyAllWindows();
}

void SharedImageSequence::GetSharedImageSequencePowerCube(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam, const CvSize& SharedImageSize, int DegreeOffset)
{

#ifndef __USE_POWERCUBE__
	std::cout << "Error: you have to enable the preprocessor symbol __USE_POWERCUBE__" << std::endl;
#endif

#ifdef __USE_POWERCUBE__
	cvNamedWindow(m_CoordWinName.c_str());
	cvNamedWindow(m_ColorWinName.c_str());
	char c=-1;
	ipa_utils::PowerCube powercube;
	powercube.Init();
	powercube.Open();
	powercube.DoHoming();
			
	// Rotate and capture

	unsigned int rotationIncrement = DegreeOffset;
	for(unsigned int degree = 0; degree < 360; degree += rotationIncrement)
	{
		
		if(!cvGetWindowHandle(m_CoordWinName.c_str()) || !cvGetWindowHandle(m_ColorWinName.c_str()))
		{
			break;
		}
		
		powercube.Rotate(rotationIncrement);
		
		SharedImage SImg;//(m_SharedImageSize);//, m_CameraSensorsIniDirectory);
		//SImg.Init(SharedImageSize);
	#ifndef __USE_SHAREDIMAGE_JBK__
		SImg.GetImagesFromSensors(RangeCam, ColorCam, SharedImageDefaultSize);
	#endif
	#ifdef __USE_SHAREDIMAGE_JBK__
		SImg.GetImagesFromSensors(RangeCam, ColorCam);
	#endif
		SImg.DisplayCoord(m_CoordWinName);
		SImg.DisplayShared(m_ColorWinName);
		push_back(SImg);

		c = cvWaitKey(100);
		if(c=='q' || c=='Q')
		{
			break;
		}

		std::cout << "SharedImageSequence::GetSharedImageSequence: ... one image sucessfully acquired." << std::endl;
	}
	cvDestroyAllWindows();
	powercube.Close();
#endif // __USE_POWERCUBE__
}

void SharedImageSequence::GetRawImageSequence(libCameraSensors::AbstractRangeImagingSensor* RangeCam, libCameraSensors::AbstractColorCamera* ColorCam)
{
	cvNamedWindow(m_CoordWinName.c_str());
	cvNamedWindow(m_ColorWinName.c_str());

	char c=-1;
	SharedImage SImg;
	SImg.GetRawImagesFromSensors(RangeCam, ColorCam);
	SImg.DisplayInten(m_CoordWinName);
	SImg.DisplayShared(m_ColorWinName);

	int cnt=0;
	while(cvGetWindowHandle(m_ColorWinName.c_str()) && cvGetWindowHandle(m_CoordWinName.c_str()))
	{

		std::cout << "SharedImageSequence::GetSharedImageSequence: Press 'n' to take a training image, 's' to save the image, or 'q' to quit.\n";

		c = cvWaitKey();

		SImg.GetRawImagesFromSensors(RangeCam, ColorCam);

		SImg.DisplayInten(m_CoordWinName);
		SImg.DisplayShared(m_ColorWinName);

		std::cout << "SharedImageSequence::GetSharedImageSequence: " << c << ".\n";
		if(c=='q' || c=='Q')
		{
			break;
		}
		else if (c=='n' || c=='N')
		{
			SImg.GetRawImagesFromSensors(RangeCam, ColorCam);
			SImg.DisplayInten(m_CoordWinName);
			SImg.DisplayShared(m_ColorWinName);
		}
		else if (c=='s' || c=='S')
		{
			push_back(SImg);
			std::cout << "SharedImageSequence::GetSharedImageSequence: ... one image sucessfully acquired." << std::endl;
			cnt++;
		}
	}
	cvDestroyAllWindows();
}

#endif // __USE_SENSORS__

void SharedImageSequence::CutImageBorder(int CutWidth)
{
	SharedImageSequence::iterator it;
	for(it=begin(); it!=end(); it++)
	{
		it->CutImageBorder(CutWidth);
	}
}

void SharedImageSequence::OrientAlongPrincipalAxises(bool global)
{
	if(!global)
	{
		SharedImageSequence::iterator it;
		for(it=begin(); it!=end(); it++)
		{
			it->OrientAlongPrincipalAxises();
		}
	}
	else
	{
		/// Get the points
		SharedImageSequence::iterator it;
		int k=0;
		std::vector<SharedImagePoint> points;
		for(it=begin(); it!=end(); it++, k++)
		{
			int i, j;
			for(j=0; j<it->Coord()->height; j++)
			{
				for(i=0; i<it->Coord()->width; i++)
				{
					CvScalar p = cvGet2D(it->Coord(), j, i);
					if(p.val[0]==0 && p.val[1]==0 && p.val[2]==0) continue;
					else
					{
						points.push_back(SharedImagePoint(p.val[0], p.val[1], p.val[2], i, j, k));
					}
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
			it=begin();
			for(int l=0; l<points[k].k; l++, it++);
			double x = cvGetReal2D(result, k, 0);
			double y = cvGetReal2D(result, k, 1);
			double z = cvGetReal2D(result, k, 2);
			cvSet2D(it->Coord(), points[k].j, points[k].i, cvScalar(x, y, z));
		}

		cvReleaseMat(&data);
		cvReleaseMat(&result);
		cvReleaseMat(&avg);
		cvReleaseMat(&eigenvalues);
		cvReleaseMat(&eigenvectors);
	}
}

void SharedImageSequence::AlignSequence(int noItMax, bool circular)
{
	boost::progress_timer t(std::clog);
	SharedImageSequence::iterator it0, it1, it2, it3, it4;
	Mat3d rot; Vec3d trans;
	DblVector maxRots;
	DblVector maxTranss;

	OrientAlongPrincipalAxises(true);

	/// Main iteration
	for(int i=0; i<noItMax; i++)
	{
		double maxRot=0, maxTrans=0, maxNorm=0, normSum = 0.0;
		//std::cout << "\nIt " << i << std::endl;
		int k=0, l=0;
		bool flag=false;
		for(it0=begin(); it0!=end(); it0++)
		{
			/// Update iterators
			it1=it0;
			it1++;
			l=k+1;
			if(it1==end())
			{
				OrientAlongPrincipalAxises(true);
				if(circular)
				{
					it1=begin();
					l=0;
					flag = true;
				}
				else break;
			}
			
			/// Find the transformation between neighbors
			if(it0->GetTransformation(*it1, &rot, &trans)==RET_FAILED) continue;
			
			/// Output the strength of transformatiom
			Frame F;
			ConvertFrame(rot, trans, F);
			//std::cout << "Views " << k << " " << l << std::endl;
			double normTrans = Point3Dbl(F[0], F[1], F[2]).AbsVal();
			double degRot = F[3]*180.0/THE_PI_DEF;
			double norm = normTrans + F[3] * ROT_FACTOR;
			normSum += norm;
			if(norm>maxNorm) maxNorm = norm;
			if(maxRot<degRot) maxRot=degRot;
			if(maxTrans<normTrans) maxTrans=normTrans;
			//std::cout << "Strength (rot, trans): " << degRot << " (deg) " << normTrans << " (mm) " << std::endl;
			////std::cout << "Statistics (count, mean, sigma): " << noPoints << " " << mean[k] << " " << sigma[k] << std::endl;

			/// Apply transformation to the tail and update means
			if(!flag)
			{
				for(it2=it1; it2!=end(); it2++)
				{	
					it2->ApplyTransformationToCoordImage(*it2, rot, trans);
				}
			}
			else
			{
				double n=0;
				for(it2=it1; it2!=end(); it2++, n++)
				{	
				
					/// Get weight of transformation
					Frame G=F;
					Mat3d wRot; Vec3d wTrans;
				
					double weight = 1.0/exp((double)n);//((double)size()-n)/(double)size();
					
					/// Alter frame
					
					G[0]*=weight; G[1]*=weight;
					G[2]*=weight; G[3]*=weight;	

					ConvertFrameBack(G, wRot, wTrans);
					it2->ApplyTransformationToCoordImage(*it2, wRot, wTrans);
				}
			}
			//std::cout << std::endl;
			k++;
		}

		//std::cout << "Max rot  : " << maxRot << " (deg) " << std::endl;
		//std::cout << "Max trans: " << maxTrans << " (mm) " << std::endl;
		//std::cout << "Max norm: " << maxNorm  << std::endl;
		double avgNorm = normSum/(double)k;
		//std::cout << "Avg norm: " << avgNorm  << std::endl;
		maxRots.push_back(maxRot);
		maxTranss.push_back(maxTrans);
	}
	
	OrientAlongPrincipalAxises(true);

	std::string rotStr = maxRots.ExportForPSTricks();
	std::ofstream rotf("maxRotations.txt");
	rotf << rotStr;
	std::string transStr = maxTranss.ExportForPSTricks();
	std::ofstream transf("maxTranslations.txt");
	transf << transStr;
}

/*
void SharedImageSequence::AlignSequence(double thresh0, double thresh1, int noItMax, bool circular)
{
	boost::progress_timer t(std::clog);
	SharedImageSequence::iterator it, it2, it3, it4, it5;
	Mat3d rot; Vec3d trans;
	DblVector mean;
	DblVector sigma;
	mean.Assign((int)size(), DBL_MAX/(thresh1+THE_EPS_DEF));
	sigma.Assign((int)size(), DBL_MAX);
	DblVector maxRots;
	DblVector maxTranss;
	for(int i=0; i<noItMax; i++)
	{
		double maxRot=0, maxTrans=0, maxNorm=0, normSum = 0.0;;
		std::cout << "\nIt " << i << std::endl;
		int k=0, l=0;
		for(it=begin(); it!=end(); it++)
		{
			it2=it;
			it2++;
			l=k+1;
			if(it2==end())
			{
				if(circular && noItMax>1)
				{
					it2=begin();
					l=0;
				}
				else break;
			}
			
			/// Find the transformation between neighbors
			double meank = mean[k];
			int noPoints = it->GetTransformation(*it2, &rot, &trans, mean[k], sigma[k], thresh1*meank);

			if(noPoints>=3)
			{
				/// Output the strength of transformatiom
				Frame F;
				it->ConvertFrame(rot, trans, F);
				std::cout << "Views " << k << " " << l << std::endl;
				double normTrans = Point3Dbl(F[0], F[1], F[2]).AbsVal();
				double degRot = F[3]*180.0/THE_PI_DEF;
				double norm = normTrans + F[3] * ROT_FACTOR;
				normSum += norm;
				if(norm>maxNorm) maxNorm = norm;
				if(maxRot<degRot) maxRot=degRot;
				if(maxTrans<normTrans) maxTrans=normTrans;
				std::cout << "Strength (rot, trans): " << degRot << " (deg) " << normTrans << " (mm) " << std::endl;
				std::cout << "Statistics (count, mean, sigma): " << noPoints << " " << mean[k] << " " << sigma[k] << std::endl;

				/// In the loop-closure case: back-propagate error frame
				//if(circular && it2==begin())
				//{
				//	it4=it5=it2;
				//	it4++;
				//	int n=1;
				//	Mat3d wRot;
				//	Vec3d wTrans;
				//	std::cout << "Propagating error frame" << std::endl;
				//	for(; it4!=end(); it4++, it5++, n++)
				//	{
				//		 Get weight of transformation
				//		double weight = ((double)size()-n)/(double)size();

				//		 Alter frame
				//		Frame G=F;
				//		G[0]*=weight;
				//		G[1]*=weight;
				//		G[2]*=weight;
				//		G[3]*=weight;
				//		it->ConvertFrameBack(G, wRot, wTrans);
				//		it->ApplyTransformationToCoordImage(*it4, wRot, wTrans);
				//		std::cout << ".";
				//		
				//		 Apply the transformation to the tail of the sequence
				//		for(it3=it2; it3!=end(); it3++)
				//		{
				//			it->ApplyTransformationToCoordImage(*it3, wRot, wTrans);
				//		}
				//	}
				//	std::cout << std::endl;
				//	int K=0;
				//	it4=it5=it2;
				//	it4++;
				//	std::cout << "Updating means" << std::endl;
				//	for(; it4!=end(); it4++, it5++, K++)
				//	{
				//		 Update means
				//		it5->GetTransformation(*it4, NULL, NULL, mean[K], sigma[K], DBL_MAX);
				//		std::cout << ".";
				//	}
				//	std::cout << std::endl;
				//}
				//else
				//{
					/// Apply the transformation to the tail of the sequence
					int n=1;
					for(it3=it2; it3!=end(); it3++)
					{
						Mat3d wRot;
						Vec3d wTrans;
						
						/// Get weight of transformation
						double weight = ((double)size()-n)/(double)size();

						/// Alter frame
						Frame G=F;
						G[0]*=weight;
						G[1]*=weight;
						G[2]*=weight;
						G[3]*=weight;
						it3->ConvertFrameBack(G, wRot, wTrans);
						it3->ApplyTransformationToCoordImage(*it3, wRot, wTrans);
						std::cout << ".";
						n++;

						/// Get new statistics
						it4=it3;
						it4--;
						it4->GetTransformation(*it3, NULL, NULL, mean[n-1], sigma[n-1], DBL_MAX);
					}
				//}
			}
			else
			{	
				std::cout << "Error: not enough points between views " << k << " and " << l << std::endl;
			}

			//std::cout << ".";
			k++;
		}

		/// End if overall mean is lower than thresh0 and worst case mean <= thresh1 * overall mean
		double meanAll=DBL_MAX, sigmaAll, meanMin, meanMax;
		int meanMinPos, meanMaxPos;
		mean.GetStatistics(meanAll, sigmaAll);
		mean.MinMax(meanMin, meanMax, meanMinPos, meanMaxPos);
		if(meanAll<thresh0 && meanMax < thresh1*meanAll)
		{
			break;
		}

		std::cout << "Max rot  : " << maxRot << " (deg) " << std::endl;
		std::cout << "Max trans: " << maxTrans << " (mm) " << std::endl;
		std::cout << "Max norm: " << maxNorm  << std::endl;
		double avgNorm = normSum/(double)k;
		std::cout << "Avg norm: " << avgNorm  << std::endl;
		maxRots.push_back(maxRot);
		maxTranss.push_back(maxTrans);
	}

	std::string rotStr = maxRots.ExportForPSTricks();
	std::ofstream rotf("maxRotations.txt");
	rotf << rotStr;
	std::string transStr = maxTranss.ExportForPSTricks();
	std::ofstream transf("maxTranslations.txt");
	transf << transStr;
}
*/
