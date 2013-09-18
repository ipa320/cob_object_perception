#include "object_categorization/ObjectClassifier.h"
#include "object_categorization/timer.h"

//#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
namespace fs = boost::filesystem;

#ifdef PCL_VERSION_COMPARE //fuerte
	#include <pcl/point_types.h>
#else
	#include <pcl/point_types.h>
#endif
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/rsd.h>
#include <pcl/features/fpfh.h>
#ifndef __LINUX__
	#include <pcl/features/gfpfh.h>
#endif
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifdef PCL_VERSION_COMPARE //fuerte
	#ifndef __LINUX__
		#include <pcl/io/openni_grabber.h>
	#endif
	#define pcl_search pcl::search::KdTree
#else
	#define pcl_search pcl::KdTreeFLANN
#endif


ObjectClassifier::ObjectClassifier(std::string pEMClusterFilename, std::string pGlobalClassifierPath)
{
	if (pEMClusterFilename != "" && pGlobalClassifierPath != "")
	{
		mData.LoadLocalFeatureClusterer(pEMClusterFilename);
		mData.LoadGlobalClassifiers(pGlobalClassifierPath, CLASSIFIER_RTC);
		std::cout << "ObjectClassifier::ObjectClassifier: Global classifiers loaded." << std::endl;
	}
}


int ObjectClassifier::LoadCINDatabase(std::string pAnnotationFileName, std::string pDatabasePath, int pMode, ClusterMode pClusterMode, GlobalFeatureParams& pGlobalFeatureParams, std::string pLocalFeatureFileName, std::string pGlobalFeatureFileName, std::string pCovarianceMatrixFileName, std::string pLocalFeatureClustererPath, std::string pTimingLogFileName, MaskMode pMaskMode)
{
	/// Load class labels from pAnnotationFileName
	std::map<int, ObjectStruct> ObjectStructMap;
	std::map<std::string, std::vector<int> > ObjectCategoryMap;
	std::map<std::string, std::vector<int> > ObjectGeneralCategoryMap;
	ObjectStruct ObjectInformation;

	SimpleStopWatch sw;
	sw.start();
	if (pMode != 2)
	{
		/// Read class label file
		std::ifstream File((pAnnotationFileName).c_str());
		if(!File.is_open())
		{
			std::cout << "Error: could not open " << pAnnotationFileName << "\n";
			return 1;
		}

		std::stringstream ss;
		while(!File.eof())
		{	
			char c = File.get();
			ss << c;
		}

		/// fill ObjectMap with object number and class label
		for (int i=1; i<1001; i++)
		{
			ss >> ObjectInformation.ObjectNum;
			ss >> ObjectInformation.CategoryName;
			ss >> ObjectInformation.GeneralCategoryName;

			ObjectStructMap[ObjectInformation.ObjectNum] = ObjectInformation;
		}

		/// fill ObjectCategoryMap with class label and corresponding object numbers
		std::map<int,ObjectStruct>::iterator ItObjectStructMap;
		for (ItObjectStructMap = ObjectStructMap.begin(); ItObjectStructMap != ObjectStructMap.end(); ItObjectStructMap++)
		{
			ObjectCategoryMap[ItObjectStructMap->second.CategoryName].push_back(ItObjectStructMap->first);
			ObjectGeneralCategoryMap[ItObjectStructMap->second.GeneralCategoryName].push_back(ItObjectStructMap->first);
		}

		std::cout << ObjectCategoryMap.size() << " object categories and " << ObjectGeneralCategoryMap.size() << " general object categories found.\n";
	}

	/// Fill mData with data by opening images classwise and extracting features

	/// 1. Local features
	if (pMode < 1)
	{
		std::cout << "\n\nLocal feature extraction\n\n";

		int ObjectCounter = 0;
		std::map<std::string, std::vector<int> >::iterator ItObjectCategoryMap;
		std::vector<int>::iterator ItSampleList;
		for (ItObjectCategoryMap = ObjectCategoryMap.begin(); ItObjectCategoryMap != ObjectCategoryMap.end(); ItObjectCategoryMap++)
		{
			int ClassObjectCounter = 0;
			for (ItSampleList = ItObjectCategoryMap->second.begin(); ItSampleList != ItObjectCategoryMap->second.end(); ItSampleList++, ClassObjectCounter++)
			{
				std::cout << "\n\nFeature extraction in class " << ItObjectCategoryMap->first << " on object " << *ItSampleList << " (" << ++ObjectCounter << ". object overall).\n";

				std::stringstream FileName;
				SharedImageSequence SharedImgSeq;
				SharedImageSequence::iterator ItSharedImgSeq;
				FileName << pDatabasePath << ItObjectCategoryMap->first << "_" << ClassObjectCounter+1;
				SharedImgSeq.LoadSharedImageSequence((FileName.str()).c_str());

				int ViewCounter = 0;
				for (ItSharedImgSeq = SharedImgSeq.begin(); ItSharedImgSeq != SharedImgSeq.end(); ItSharedImgSeq++, ViewCounter++)
				{
					BlobListStruct TempBlobListStruct;
		
					std::stringstream ViewFileName;
					ViewFileName << FileName.str() << SharedImgSeq.m_Spacing << ViewCounter;
					TempBlobListStruct.FileName = ViewFileName.str();
					ExtractLocalFeatures(&(*ItSharedImgSeq), TempBlobListStruct.BlobFPs, pClusterMode, pMaskMode, ViewFileName.str(), CIN); /*, MASK_SAVE, ViewFileName.str() ... remove comment in order to create new masks*/

					(mData.mLocalFeaturesMap[ItObjectCategoryMap->first])[ClassObjectCounter].push_back(TempBlobListStruct);
				}
			}
		}
		SaveFPDataLocal(pLocalFeatureFileName);
	}
	else
	{
		LoadFPDataLocal(pLocalFeatureFileName);
	}
	std::cout << "Processing time for local feature extraction: " << sw.stop() << "s.\n";


	/// 2. Global Features
	sw.start();
	if (abs(pMode) < 2)
	{
		/// Clustering
		int MaxClusters = cvRound(mData.GetNumberLocalFeaturePoints()/10);
		const int MAXCLUSTERS = pGlobalFeatureParams.vocabularySize;//200;//250;//46;//100;		//300;
		const int MINCLUSTERS = pGlobalFeatureParams.vocabularySize;//200;//250;//10;
		if (MaxClusters > MAXCLUSTERS) MaxClusters=MAXCLUSTERS;
//MaxClusters=11;
		ClusterLocalFeatures(pCovarianceMatrixFileName, pLocalFeatureClustererPath, MINCLUSTERS, MaxClusters, cvRound((MaxClusters-MINCLUSTERS)/10), 10, 0.005); //0.005

		std::cout << "\n\nGlobal feature extraction\n\n";

		//std::cout << "Strike any key when ready...";
		//getchar();

		LocalFeaturesMap::iterator ItLocalFeaturesMap;
		ObjectMap::iterator ItObjectMap;
		std::vector<BlobListStruct>::iterator ItBlobListStructs;

		/// Find number of local features
		int NumberLocalFeatures=mData.GetNumberLocalFeatures();
		if (NumberLocalFeatures <= 0) return ipa_utils::RET_FAILED;
		int NumberGlobalFeatures = 0;

		/// Iterate over classes
		for (ItLocalFeaturesMap = mData.mLocalFeaturesMap.begin(); ItLocalFeaturesMap != mData.mLocalFeaturesMap.end(); ItLocalFeaturesMap++)
		{
			//if (ItLocalFeaturesMap->first != "tetrapack") continue;
			std::cout << "\n\nExtracting global Features for class " << ItLocalFeaturesMap->first << ".\n";
			
			/// Iterate over objects of a class
			int ObjectCounter = 0;
			for (ItObjectMap = ItLocalFeaturesMap->second.begin(); ItObjectMap != ItLocalFeaturesMap->second.end(); ItObjectMap++, ObjectCounter++)
			{
				//if (ItObjectMap->first < 1) continue;
				CvMat* GlobalFeatures = NULL;

				/// Iterate over pictures of an object
				int BlobListCounter=0;
				int Rows = ItObjectMap->second.size();
				CvMat* Features = NULL;
				for (ItBlobListStructs = ItObjectMap->second.begin(); ItBlobListStructs != ItObjectMap->second.end(); ItBlobListStructs++)
				{
					SharedImage SourceImage;
					SourceImage.LoadSharedImage(ItBlobListStructs->FileName.c_str());
					IplImage* Mask = cvLoadImage((ItBlobListStructs->FileName+"_Mask.png").c_str(), 0);
					std::cout << ItBlobListStructs->FileName << "\n";
					// check whether features were extracted																	// disabled image output
					if (ExtractGlobalFeatures(&(ItBlobListStructs->BlobFPs), &Features, pClusterMode, pGlobalFeatureParams, CIN, SourceImage.Coord(), Mask/*, cvCloneImage(SourceImage.Shared())*/, 0, false, pTimingLogFileName) == ipa_utils::RET_OK)
					{
						// initialize GlobalFeatures and determine number of global features
						if (!GlobalFeatures)
						{
							NumberGlobalFeatures = Features->cols;
							GlobalFeatures = cvCreateMat(Rows, NumberGlobalFeatures, CV_32FC1);
						}

						for (int j=0; j<NumberGlobalFeatures; j++) cvmSet(GlobalFeatures, BlobListCounter, j, cvGetReal1D(Features, j));
						BlobListCounter++;
					}
					else
					{
						if (GlobalFeatures)
						{
							// no features available -> reshape
							CvMat* Temp = cvCreateMat(GlobalFeatures->rows-1, GlobalFeatures->cols, GlobalFeatures->type);
							for (int i=0; i<BlobListCounter; i++) for (int j=0; j<Temp->width; j++) cvmSet(Temp, i, j, cvmGet(GlobalFeatures, i, j));
							cvReleaseMat(&GlobalFeatures);
							GlobalFeatures = Temp;
						}
						else
						{
							Rows--;
						}
					}
					cvReleaseImage(&Mask);
				}
				(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter] = GlobalFeatures;

				/// Output
				/*std::cout << ItLocalFeaturesMap->first << " " << ObjectCounter << "\n";
				for (int i=0; i<(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter]->height; i++)
				{
					for (int j=0; j<(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter]->width; j++) std::cout << cvGetReal2D((mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter], i, j) << "\t";
					std::cout << "\n";
				}*/
			}
		}
		SaveFPDataGlobal(pGlobalFeatureFileName);
	}
	else
	{
		LoadFPDataGlobal(pGlobalFeatureFileName);
	}
	std::cout << "Processing time for global feature extraction: " << sw.stop() << "s.\n";

	return ipa_utils::RET_OK;
}


struct SampleObject
{
	std::string categoryName;
	std::string path;
};

unsigned long LoadMat(cv::Mat& mat, std::string filename)
{
	size_t file_length = 0;
	char *c_string = 0;

	std::ifstream file(filename.c_str(), std::ios_base::binary|std::ios_base::in|std::ios_base::ate);
	if(!file.is_open())
	{
		std::cerr << "ERROR - ipa_Utils::LoadMat:" << std::endl;
		std::cerr << "\t ... Could not open " << filename << " \n";
		return ipa_utils::RET_FAILED;
	}

	file_length = file.tellg();
	file.seekg(0, std::ios_base::beg);
	file.clear();

	c_string = new char[file_length];
	file.read(c_string, file_length);

	unsigned int rows, cols;
	int channels;
	rows = ((int*)c_string)[0];
	cols = ((int*)c_string)[1];
	channels = ((int*)c_string)[2];

	mat.create(rows, cols, CV_32FC(channels));
	float* f_ptr;
	char* c_ptr;

	f_ptr = mat.ptr<float>(0);
	c_ptr = &c_string[3 * sizeof(int)];

	memcpy(f_ptr, c_ptr,  channels * mat.cols * mat.rows * sizeof(float));

	file.close();

	delete[] c_string;

	return ipa_utils::RET_OK;
}

int ObjectClassifier::LoadCIN2Database(std::string pAnnotationFileName, std::string pDatabasePath, int pMode, ClusterMode pClusterMode, GlobalFeatureParams& pGlobalFeatureParams, LocalFeatureParams& pLocalFeatureParams, std::string pLocalFeatureFileName,
										std::string pGlobalFeatureFileName, std::string pCovarianceMatrixFileName, std::string pLocalFeatureClustererPath, std::string pTimingLogFileName, std::ofstream& pScreenLogFile, MaskMode pMaskMode)
{
	SimpleStopWatch sw;
	sw.start();

	const int numberOfViewsPerObject = 36;


	// read in object classes available in pDatabasePath (each class has its own folder)
	std::cout << "Reading data from directory: '" << pDatabasePath << "'" << std::endl;
	pScreenLogFile << "Reading data from directory: '" << pDatabasePath << "'" << std::endl;

	std::vector<SampleObject> objectCollection;

	if (pMode != 2)
	{
		for (fs::directory_iterator classDirIter(pDatabasePath); classDirIter!=fs::directory_iterator(); ++classDirIter)
		{
			std::string path = classDirIter->path().string();
			std::string className = fs::basename(path);

			for (fs::directory_iterator objectDirIter(path); objectDirIter!=fs::directory_iterator(); ++objectDirIter)
			{
				// Extract path
				std::string pathAndName = objectDirIter->path().string();
				std::string objectName = fs::basename(pathAndName);
				if (objectName == "setup" || objectName == "Setup" || objectName == "Mice2")
					continue;

				SampleObject s;
				s.path = pathAndName;
				s.categoryName = className;
				objectCollection.push_back(s);
			}
		}
	}

	// read in mapping from original classes (those in the folder) to those classes defined for the task
	// and assign the found data to the desired categories
	std::map<std::string, std::string> classToCategoryMapping;		// maps classToCategoryMapping[originalClassName] = category defined for that class
																	// if a mapping does not exist for originalClassName then this object will be ignored
	std::map<std::string, std::vector<std::string> > ObjectCategoryMap;		// maps a category (as defined for the task at hand) to a collection of paths where instances of that class are located
	
	if (pMode != 2)
	{
		// read in the mappings to categories
		std::ifstream file((pAnnotationFileName).c_str());
		if(!file.is_open())
		{
			std::cout << "Error: could not open " << pAnnotationFileName << std::endl;
			pScreenLogFile << "Error: could not open " << pAnnotationFileName << std::endl;
			return 1;
		}

		while (!file.eof())
		{
			std::string from, to;
			file >> from;
			file >> to;
			classToCategoryMapping[from] = to;
		}

		// assign the mappings to the found objects in the database
		for (unsigned int i=0; i<objectCollection.size(); i++)
		{
			if (classToCategoryMapping.find(objectCollection[i].categoryName) != classToCategoryMapping.end())
			{
				ObjectCategoryMap[classToCategoryMapping[objectCollection[i].categoryName]].push_back(objectCollection[i].path);
			}
		}

		std::cout << ObjectCategoryMap.size() << " object categories found." << std::endl;
		pScreenLogFile << ObjectCategoryMap.size() << " object categories found." << std::endl;
	}


	/// Fill mData with data by opening images classwise and extracting features

	/// 1. Local features
	if (pMode < 1)
	{
		std::cout << "\n\nLocal feature extraction\n\n";
		pScreenLogFile << "\n\nLocal feature extraction\n\n";

		int ObjectCounter = 0;
		std::map<std::string, std::vector<std::string> >::iterator ItObjectCategoryMap;
		for (ItObjectCategoryMap = ObjectCategoryMap.begin(); ItObjectCategoryMap != ObjectCategoryMap.end(); ItObjectCategoryMap++)
		{
//			if (ItObjectCategoryMap->first != "coffeepot") continue;

			int ClassObjectCounter = 0;
			for (unsigned int sampleIndex = 0; sampleIndex < ItObjectCategoryMap->second.size(); sampleIndex++, ClassObjectCounter++)
			{
//				if (sampleIndex < 1) continue;

				if (ItObjectCategoryMap->first == "pen" && sampleIndex == 4) continue;	// very bad data quality

				std::cout << "\n\nFeature extraction in class " << ItObjectCategoryMap->first << " on object " << sampleIndex << " (" << ++ObjectCounter << ". object overall) in path " << ItObjectCategoryMap->second[sampleIndex] << std::endl;
				pScreenLogFile << "\n\nFeature extraction in class " << ItObjectCategoryMap->first << " on object " << sampleIndex << " (" << ObjectCounter << ". object overall) in path " << ItObjectCategoryMap->second[sampleIndex] << std::endl;
				std::string directory = ItObjectCategoryMap->second[sampleIndex] + "/";

				// iterate through all views
				for (int imageIndex = 0; imageIndex < numberOfViewsPerObject; imageIndex++)
				{
					// load shared images
					double exp = 0;
					if (imageIndex > 0)
						exp = std::log10((double)imageIndex);
					std::stringstream indexFormatted;
					for (int i=0; i<4-floor(exp); i++)
						indexFormatted << "0";
					indexFormatted << imageIndex;

					// color (CV_8UC3)
					std::string inputFilename = directory + "sharedImage_color_" + indexFormatted.str() + ".png";
					cv::Mat colorImage = cv::imread(inputFilename);
					IplImage colorImageIpl = (IplImage)colorImage;
					IplImage* colorImageIplCopy = cvCreateImage(cvSize(colorImageIpl.width, colorImageIpl.height), colorImageIpl.depth, colorImageIpl.nChannels);
					cvCopyImage(&colorImageIpl, colorImageIplCopy);

					// xyz (CV_32FC3)
					inputFilename = directory + "sharedImage_xyz_" + indexFormatted.str() + ".bin";
					cv::Mat xyzImage;
					LoadMat(xyzImage, inputFilename);
					IplImage xyzImageIpl = (IplImage)xyzImage;
					IplImage* xyzImageIplCopy = cvCreateImage(cvSize(xyzImageIpl.width, xyzImageIpl.height), xyzImageIpl.depth, xyzImageIpl.nChannels);
					cvCopyImage(&xyzImageIpl, xyzImageIplCopy);

					// intensity (CV_32FC1)
					inputFilename = directory + "sharedImage_inten_" + indexFormatted.str() + ".bin";
					cv::Mat intenImage;
					//LoadMat(intenImage, inputFilename);	// intensity images are buggy
					cv::cvtColor(colorImage, intenImage, CV_BGR2GRAY);
					IplImage intenImageIpl = (IplImage)intenImage;
					IplImage* intenImageIplCopy = cvCreateImage(cvSize(intenImageIpl.width, intenImageIpl.height), intenImageIpl.depth, intenImageIpl.nChannels);
					cvCopyImage(&intenImageIpl, intenImageIplCopy);

					SharedImage si;
					si.setCoord(xyzImageIplCopy);
					si.setShared(colorImageIplCopy);
					si.setInten(intenImageIplCopy);

					//if (imageIndex>29)
					//{
					//	std::string fnam = directory + "color.bmp";
					//	cv::imwrite(fnam, colorImage);
					//	fnam = directory + "coordinates.txt";
					//	std::ofstream ofi(fnam.c_str());
					//	for (int v=0; v<xyzImage.rows; v++)
					//		for (int u=0; u<xyzImage.cols; u++)
					//			ofi << xyzImage.at<cv::Point3f>(v, u).x << "\t" << xyzImage.at<cv::Point3f>(v, u).y << "\t" << xyzImage.at<cv::Point3f>(v, u).z << "\n";
					//	ofi.close();
					//	getchar();
					//}


					BlobListStruct TempBlobListStruct;
					TempBlobListStruct.FileName = directory + "sharedImage_" + indexFormatted.str();		// should not have an extension
					std::string classString = ItObjectCategoryMap->first;
					if (pLocalFeatureParams.useFeature.compare("surf") == 0)
					{
						if (classString == "pen" && (sampleIndex == 6 || sampleIndex == 7 || sampleIndex == 8 || sampleIndex == 10))
							classString = "pen_highbase";
						ExtractLocalFeatures(&si, TempBlobListStruct.BlobFPs, pClusterMode, pMaskMode, TempBlobListStruct.FileName, CIN2, classString); /*, MASK_SAVE, ViewFileName.str() ... remove comment in order to create new masks*/
					}
					else if (pLocalFeatureParams.useFeature.compare("rsd") == 0)
					{
						ExtractLocalRSDorFPFHFeatures(&si, TempBlobListStruct.BlobFPs, pLocalFeatureParams, pMaskMode, TempBlobListStruct.FileName, CIN2);
					}
					else if (pLocalFeatureParams.useFeature.compare("fpfh") == 0)
					{
						ExtractLocalRSDorFPFHFeatures(&si, TempBlobListStruct.BlobFPs, pLocalFeatureParams, pMaskMode, TempBlobListStruct.FileName, CIN2);
					}
					else
					{
						std::cout << "ObjectClassifier::LoadCIN2Database: Error: Local feature type " << pLocalFeatureParams.useFeature << " unknown" << std::endl;
						return ipa_utils::RET_FAILED;
					}

					si.Release();

					(mData.mLocalFeaturesMap[ItObjectCategoryMap->first])[ClassObjectCounter].push_back(TempBlobListStruct);
				}
			}
		}
		SaveFPDataLocal(pLocalFeatureFileName);
	}
	else
	{
		LoadFPDataLocal(pLocalFeatureFileName);
		for (LocalFeaturesMap::iterator ItLocalFeaturesMap = mData.mLocalFeaturesMap.begin(); ItLocalFeaturesMap != mData.mLocalFeaturesMap.end(); ItLocalFeaturesMap++)
		{
			if (ObjectCategoryMap.find(ItLocalFeaturesMap->first) == ObjectCategoryMap.end())
			{
				LocalFeaturesMap::iterator ItTemp = ItLocalFeaturesMap;
				ItLocalFeaturesMap++;
				mData.mLocalFeaturesMap.erase(ItTemp);
			}
		}
		std::cout << "Found the following classes:" << std::endl;
		for (LocalFeaturesMap::iterator ItLocalFeaturesMap = mData.mLocalFeaturesMap.begin(); ItLocalFeaturesMap != mData.mLocalFeaturesMap.end(); ItLocalFeaturesMap++)
		{
			std::cout << ItLocalFeaturesMap->first << std::endl;
		}
	}
	std::cout << "Processing time for local feature extraction: " << sw.stop() << "s.\n";
	pScreenLogFile << "Processing time for local feature extraction: " << sw.stop() << "s.\n";



	/// 2. Global Features
	sw.start();
	if (abs(pMode) < 2)
	{
		/// Clustering
		int MaxClusters = pGlobalFeatureParams.vocabularySize; //cvRound(mData.GetNumberLocalFeaturePoints()/10);
		const int MAXCLUSTERS = pGlobalFeatureParams.vocabularySize;//250;//46;//100;		//300;
		const int MINCLUSTERS = pGlobalFeatureParams.vocabularySize;//250;//10;
		if (MaxClusters > MAXCLUSTERS) MaxClusters=MAXCLUSTERS;
//MaxClusters=11;
		int trials = 1; //10
		double minRelativeDeviationChange = 5; //0.005
		if ((pGlobalFeatureParams.useFeature.find("bow") != pGlobalFeatureParams.useFeature.end() && pGlobalFeatureParams.useFeature["bow"] == true) ||
			(pGlobalFeatureParams.useFeature.find("gfpfh") != pGlobalFeatureParams.useFeature.end() && pGlobalFeatureParams.useFeature["gfpfh"] == true))
			ClusterLocalFeatures(pCovarianceMatrixFileName, pLocalFeatureClustererPath, MINCLUSTERS, MaxClusters, cvRound((MaxClusters-MINCLUSTERS)/10), trials, minRelativeDeviationChange , &pScreenLogFile); //0.005

		std::cout << "Processing time for clustering: " << sw.stop() << "s.\n";
		pScreenLogFile << "Processing time for clustering: " << sw.stop() << "s.\n";
		sw.start();

		std::cout << "\n\nGlobal feature extraction\n\n";
		pScreenLogFile << "\n\nGlobal feature extraction\n\n";

		LocalFeaturesMap::iterator ItLocalFeaturesMap;
		ObjectMap::iterator ItObjectMap;
		std::vector<BlobListStruct>::iterator ItBlobListStructs;

		/// Find number of local features
		int NumberLocalFeatures=mData.GetNumberLocalFeatures();
		if (NumberLocalFeatures <= 0) return ipa_utils::RET_FAILED;
		int NumberGlobalFeatures = 0;

		/// Iterate over classes
		for (ItLocalFeaturesMap = mData.mLocalFeaturesMap.begin(); ItLocalFeaturesMap != mData.mLocalFeaturesMap.end(); ItLocalFeaturesMap++)
		{
//			if (ItLocalFeaturesMap->first != "coffeepot") continue;
			std::cout << "\n\nExtracting global Features for class " << ItLocalFeaturesMap->first << ".\n";
			pScreenLogFile << "\n\nExtracting global Features for class " << ItLocalFeaturesMap->first << ".\n";
			
			/// Iterate over objects of a class
			int ObjectCounter = 0;
			for (ItObjectMap = ItLocalFeaturesMap->second.begin(); ItObjectMap != ItLocalFeaturesMap->second.end(); ItObjectMap++, ObjectCounter++)
			{
//				if (ItObjectMap->first < 1) continue;
				CvMat* GlobalFeatures = NULL;

				/// Iterate over pictures of an object
				int BlobListCounter=0;
				int numberOfTiltAngles = 1 + pGlobalFeatureParams.additionalArtificialTiltedViewAngle.size();
				int Rows = ItObjectMap->second.size() * numberOfTiltAngles;
				CvMat* Features = NULL;
				for (ItBlobListStructs = ItObjectMap->second.begin(); ItBlobListStructs != ItObjectMap->second.end(); ItBlobListStructs++)
				{					
					std::string inputFilename = ItBlobListStructs->FileName + ".bin";
					size_t pos = inputFilename.find("sharedImage_");
					if (pos != std::string::npos)
						inputFilename.replace(pos, 12, "sharedImage_xyz_");
					else
					{
						std::cout << "Error: LoadCIN2Database: Filename " << inputFilename << " does not contain sharedImage_ ." << std::endl;
						pScreenLogFile << "Error: LoadCIN2Database: Filename " << inputFilename << " does not contain sharedImage_ ." << std::endl;
						return ipa_utils::RET_FAILED;
					}
					cv::Mat xyzImage;
					LoadMat(xyzImage, inputFilename);
					IplImage xyzImageIpl = (IplImage)xyzImage;
					
					IplImage* Mask = cvLoadImage((ItBlobListStructs->FileName+"_Mask.png").c_str(), 0);
					std::cout << ItBlobListStructs->FileName << "\n";
					pScreenLogFile << ItBlobListStructs->FileName << "\n";
					// check whether features were extracted																	// disabled image output
					if (ExtractGlobalFeatures(&(ItBlobListStructs->BlobFPs), &Features, pClusterMode, pGlobalFeatureParams, CIN2, &xyzImageIpl, Mask/*, cvCloneImage(SourceImage.Shared())*/, 0, false, pTimingLogFileName, &pScreenLogFile) == ipa_utils::RET_OK)
					{
						// initialize GlobalFeatures and determine number of global features
						if (!GlobalFeatures)
						{
							NumberGlobalFeatures = Features->cols;
							GlobalFeatures = cvCreateMat(Rows, NumberGlobalFeatures, CV_32FC1);
						}

						for (int j=0; j<NumberGlobalFeatures; j++)
						{
							for (int i=0; i<numberOfTiltAngles; i++)
								cvmSet(GlobalFeatures, i*Rows/numberOfTiltAngles + BlobListCounter, j, cvmGet(Features, i, j));
						}
						BlobListCounter++;
					}
					else
					{
						if (GlobalFeatures)
						{
							// no features available -> reshape
							CvMat* Temp = cvCreateMat(GlobalFeatures->rows-numberOfTiltAngles, GlobalFeatures->cols, GlobalFeatures->type);
							for (int i=0; i<BlobListCounter; i++)
							{
								for (int j=0; j<Temp->width; j++)
								{
									for (int a=0; a<numberOfTiltAngles; a++)
										cvmSet(Temp, a*(Rows-numberOfTiltAngles)/numberOfTiltAngles + i, j, cvmGet(GlobalFeatures, a*Rows/numberOfTiltAngles + i, j));
								}
							}
							cvReleaseMat(&GlobalFeatures);
							GlobalFeatures = Temp;
						}
						Rows -= numberOfTiltAngles;
					}
					cvReleaseImage(&Mask);
				}
				(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter] = GlobalFeatures;

				/// Output
				/*std::cout << ItLocalFeaturesMap->first << " " << ObjectCounter << "\n";
				for (int i=0; i<(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter]->height; i++)
				{
					for (int j=0; j<(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter]->width; j++) std::cout << cvGetReal2D((mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter], i, j) << "\t";
					std::cout << "\n";
				}*/
			}
		}
		SaveFPDataGlobal(pGlobalFeatureFileName);
	}
	else
	{
		LoadFPDataGlobal(pGlobalFeatureFileName);
	}
	std::cout << "Processing time for global feature extraction: " << sw.stop() << "s.\n";
	pScreenLogFile << "Processing time for global feature extraction: " << sw.stop() << "s.\n";

	return ipa_utils::RET_OK;
}


//////////////////////////////////////////////////////////////////////////
// Loading function for Washington dataset
//////////////////////////////////////////////////////////////////////////

struct PointXYZRGBIM
{
  union
  {
    struct
    {
      float x;
      float y;
      float z;
      float rgb;
      float imX;
      float imY;
    };
    float data[6];
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBIM,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, rgb, rgb)
                                    (float, imX, imX)
                                    (float, imY, imY)
)


int ObjectClassifier::LoadWashingtonDatabase(std::string pAnnotationFileName, std::string pDatabasePath, int pMode, ClusterMode pClusterMode, GlobalFeatureParams& pGlobalFeatureParams, std::string pLocalFeatureFileName,
										std::string pGlobalFeatureFileName, std::string pCovarianceMatrixFileName, std::string pLocalFeatureClustererPath, std::string pTimingLogFileName, std::ofstream& pScreenLogFile, MaskMode pMaskMode, bool useIPA3Database)
{
	SimpleStopWatch sw;
	sw.start();

	// no fixed number of views, we take every fifth frame as in the original paper
	// const int numberOfViewsPerObject = 3*50;	// contains 3 sequences with 200-250 images


	// read in object classes available in pDatabasePath (each class has its own folder)
	std::cout << "Reading data from directory: '" << pDatabasePath << "'" << std::endl;
	pScreenLogFile << "Reading data from directory: '" << pDatabasePath << "'" << std::endl;

	std::vector<SampleObject> objectCollection;

	for (fs::directory_iterator classDirIter(pDatabasePath); classDirIter!=fs::directory_iterator(); ++classDirIter)
	{
		// path = path to object
		std::string path = classDirIter->path().string();
		std::string className = fs::basename(path);

#if !defined(BOOST_FILESYSTEM_VERSION) || BOOST_FILESYSTEM_VERSION<3
		if (classDirIter->path().filename() == "README.txt" || classDirIter->path().filename() == "rgbd-dataset_pcd" || classDirIter->path().filename() == "rgbd-dataset_pcd.tar")
			continue;
#else
		if (classDirIter->path().filename().string() == "README.txt" || classDirIter->path().filename().string() == "rgbd-dataset_pcd" || classDirIter->path().filename().string() == "rgbd-dataset_pcd.tar")
			continue;
#endif

		for (fs::directory_iterator objectDirIter(path); objectDirIter!=fs::directory_iterator(); ++objectDirIter)
		{
			// Extract instance path
			std::string pathAndName = objectDirIter->path().string();
			std::string objectName = fs::basename(pathAndName);

			SampleObject s;
			s.path = pathAndName;
			s.categoryName = className;
			objectCollection.push_back(s);
		}

		//std::cout << className << "\t" << className << std::endl;
	}

	// read in mapping from original classes (those in the folder) to those classes defined for the task
	// and assign the found data to the desired categories
	std::map<std::string, std::string> classToCategoryMapping;		// maps classToCategoryMapping[originalClassName] = category defined for that class
																	// if a mapping does not exist for originalClassName then this object will be ignored
	std::map<std::string, std::vector<std::string> > ObjectCategoryMap;		// maps a category (as defined for the task at hand) to a collection of paths where instances of that class are located
	
	if (pMode != 2)
	{
		// read in the mappings to categories
		std::ifstream file((pAnnotationFileName).c_str());
		if(!file.is_open())
		{
			std::cout << "Error: could not open " << pAnnotationFileName << std::endl;
			pScreenLogFile << "Error: could not open " << pAnnotationFileName << std::endl;
			return 1;
		}

		while (!file.eof())
		{
			std::string from, to;
			file >> from;
			file >> to;
			classToCategoryMapping[from] = to;
		}

		// assign the mappings to the found objects in the database
		for (unsigned int i=0; i<objectCollection.size(); i++)
		{
			if (classToCategoryMapping.find(objectCollection[i].categoryName) != classToCategoryMapping.end())
			{
				ObjectCategoryMap[classToCategoryMapping[objectCollection[i].categoryName]].push_back(objectCollection[i].path);
			}
		}

		std::cout << ObjectCategoryMap.size() << " object categories found." << std::endl;
		pScreenLogFile << ObjectCategoryMap.size() << " object categories found." << std::endl;
	}


	/// Fill mData with data by opening images classwise and extracting features

	/// 1. Local features
	if (pMode < 1)
	{
		std::cout << "\n\nLocal feature extraction\n\n";
		pScreenLogFile << "\n\nLocal feature extraction\n\n";

		int ObjectCounter = 0;
		std::map<std::string, std::vector<std::string> >::iterator ItObjectCategoryMap;
		for (ItObjectCategoryMap = ObjectCategoryMap.begin(); ItObjectCategoryMap != ObjectCategoryMap.end(); ItObjectCategoryMap++)
		{
			//if (ItObjectCategoryMap->first != "tetrapaks") continue;

			int ClassObjectCounter = 0;
			for (unsigned int sampleIndex = 0; sampleIndex < ItObjectCategoryMap->second.size(); sampleIndex++, ClassObjectCounter++)
			{
				//if (sampleIndex < 0) continue;
				
				std::cout << "\n\nFeature extraction in class " << ItObjectCategoryMap->first << " on object " << sampleIndex << " (" << ++ObjectCounter << ". object overall) in path " << ItObjectCategoryMap->second[sampleIndex] << std::endl;
				pScreenLogFile << "\n\nFeature extraction in class " << ItObjectCategoryMap->first << " on object " << sampleIndex << " (" << ObjectCounter << ". object overall) in path " << ItObjectCategoryMap->second[sampleIndex] << std::endl;
				std::string directory = ItObjectCategoryMap->second[sampleIndex] + "/";

				// open every fifth file per object 
				// count number of views in each sequence, sequences are numbered 1,2,4
				int maxSequence = (useIPA3Database==true) ? 1 : 4;
				for (int sequence=1; sequence<=maxSequence; sequence+=sequence)
				{
					int numberViews = (useIPA3Database==true) ? 62 : 0;

					// count number of views in each sequence
					if (useIPA3Database==false)
					{
						for (fs::directory_iterator viewDirIter(directory); viewDirIter!=fs::directory_iterator(); ++viewDirIter)
						{
							std::stringstream ss;
	#if !defined(BOOST_FILESYSTEM_VERSION) || BOOST_FILESYSTEM_VERSION<3
							std::string filename = viewDirIter->path().filename();
							ss << viewDirIter->path().parent_path().filename() << "_" << sequence << "_";
	#else
							std::string filename = viewDirIter->path().filename().string();
							ss << viewDirIter->path().parent_path().filename().string() << "_" << sequence << "_";
	#endif
							std::string namePrefix = ss.str();

							// if this file originates from the sequence at hand
							if (filename.find(namePrefix) != filename.npos)
							{
								// get view index and compare with numberViews
								filename.erase(0, namePrefix.length());
								filename.erase(filename.length()-4, 4);
								std::stringstream ssViewIndex;
								ssViewIndex << filename;
								int viewIndex = 0;
								ssViewIndex >> viewIndex;
								if (viewIndex > numberViews)
									numberViews = viewIndex;
							}
						}
					}

					fs::directory_iterator viewDirIter(directory);
					std::stringstream ssNamePrefix;
#if !defined(BOOST_FILESYSTEM_VERSION) || BOOST_FILESYSTEM_VERSION<3
					ssNamePrefix << viewDirIter->path().parent_path().filename() << "_";
					if (useIPA3Database==false)
						ssNamePrefix << sequence << "_";
#else
					ssNamePrefix << viewDirIter->path().parent_path().filename().string() << "_";
					if (useIPA3Database==false)
						ssNamePrefix << sequence << "_";
#endif
					std::string namePrefix = ssNamePrefix.str();

					// iterate through all views taking every fifth
					int startImageIndex = (useIPA3Database==true ? 0 : 1);
					int stepImageIndex = (useIPA3Database==true ? 1 : 5);
					for (int imageIndex = startImageIndex; imageIndex < numberViews; imageIndex+=stepImageIndex)
					{
						// assemble filename
						std::stringstream ss;
						ss << directory << namePrefix << imageIndex << ".pcd";
						std::string filename = ss.str();

						//if (fs::exists(filename) == false)
						//	std::cout << filename << " does not exist." << std::endl;
						
						// load pcd file and write images
						IplImage* colorImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 3);
						cvSetZero(colorImage);
						IplImage* intenImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
						cvSetZero(intenImage);
						IplImage* xyzImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F, 3);
						cvSetZero(xyzImage);

						pcl::PointCloud<PointXYZRGBIM>::Ptr cloud (new pcl::PointCloud<PointXYZRGBIM>);

						if (pcl::io::loadPCDFile<PointXYZRGBIM> (filename, *cloud) == -1) //* load the file
						{
							std::cout << "Couldn't read file " << filename << "." << std::endl;
							return ipa_utils::RET_FAILED;
						}
						//std::cout << "Loaded " << cloud->width * cloud->height << " data points from test_pcd.pcd with the following fields: " << std::endl;

						for (size_t i = 0; i < cloud->points.size (); ++i)
						{
							uint32_t rgb = *reinterpret_cast<int*>(&cloud->points[i].rgb);
							uint8_t r = (rgb >> 16) & 0x0000ff;
							uint8_t g = (rgb >> 8)  & 0x0000ff;
							uint8_t b = (rgb)       & 0x0000ff;
							int u = cloud->points[i].imX;
							int v = cloud->points[i].imY;
							float x = cloud->points[i].x;
							float y = -cloud->points[i].z;
							float z = cloud->points[i].y;

							cvSet2D(colorImage, v, u, cvScalar(b, g, r, 0));
							cvSet2D(xyzImage, v, u, cvScalar(x, y, z, 0));
						}

						cvCvtColor(colorImage, intenImage, CV_BGR2GRAY);

						// create shared image
						SharedImage si;
						si.setCoord(xyzImage);
						si.setShared(colorImage);
						si.setInten(intenImage);

						//cvNamedWindow("color");
						//cvShowImage("color", colorImage);
						//cvNamedWindow("xyz");
						//cvShowImage("xyz", xyzImage);
						//cvNamedWindow("inten");
						//cvShowImage("inten", intenImage);
						//cvWaitKey(10);
						//cvWaitKey();

						BlobListStruct TempBlobListStruct;
						TempBlobListStruct.FileName = filename;
						std::string classString = ItObjectCategoryMap->first;
						ExtractLocalFeatures(&si, TempBlobListStruct.BlobFPs, pClusterMode, pMaskMode, TempBlobListStruct.FileName, WASHINGTON, classString); //, MASK_SAVE, ViewFileName.str() ... remove comment in order to create new masks

						si.Release();

						if (useIPA3Database==true && TempBlobListStruct.BlobFPs.size()==0)
						{
							BlobFeature blob;
							blob.m_D.push_back(0);
							TempBlobListStruct.BlobFPs.push_back(blob);
						}
						(mData.mLocalFeaturesMap[ItObjectCategoryMap->first])[ClassObjectCounter].push_back(TempBlobListStruct);
		
						std::cout << ".";
					}
				}
				std::cout << std::endl;
			}
		}
		SaveFPDataLocal(pLocalFeatureFileName);
	}
	else
	{
		LoadFPDataLocal(pLocalFeatureFileName);
		for (LocalFeaturesMap::iterator ItLocalFeaturesMap = mData.mLocalFeaturesMap.begin(); ItLocalFeaturesMap != mData.mLocalFeaturesMap.end(); )
		{
			if (ObjectCategoryMap.find(ItLocalFeaturesMap->first) == ObjectCategoryMap.end())
			{
				mData.mLocalFeaturesMap.erase(ItLocalFeaturesMap);
				ItLocalFeaturesMap = mData.mLocalFeaturesMap.begin();
			}
			else
				ItLocalFeaturesMap++;
		}
		std::cout << "Found the following classes:" << std::endl;
		pScreenLogFile << "Found the following classes:" << std::endl;
		for (LocalFeaturesMap::iterator ItLocalFeaturesMap = mData.mLocalFeaturesMap.begin(); ItLocalFeaturesMap != mData.mLocalFeaturesMap.end(); ItLocalFeaturesMap++)
		{
			std::cout << ItLocalFeaturesMap->first << std::endl;
			pScreenLogFile << ItLocalFeaturesMap->first << std::endl;
		}
	}
	std::cout << "Processing time for local feature extraction: " << sw.stop() << "s.\n";
	pScreenLogFile << "Processing time for local feature extraction: " << sw.stop() << "s.\n";



	/// 2. Global Features
	sw.start();
	if (abs(pMode) < 2)
	{
		/// Clustering
		int MaxClusters = cvRound(mData.GetNumberLocalFeaturePoints()/10);
		const int MAXCLUSTERS = pGlobalFeatureParams.vocabularySize;//250;//46;//100;		//300;
		const int MINCLUSTERS = pGlobalFeatureParams.vocabularySize;//250;//10;
		if (MaxClusters > MAXCLUSTERS) MaxClusters=MAXCLUSTERS;
//MaxClusters=11;
		ClusterLocalFeatures(pCovarianceMatrixFileName, pLocalFeatureClustererPath, MINCLUSTERS, MaxClusters, std::max(1,cvRound((MaxClusters-MINCLUSTERS)/10)), 10, 0.005, &pScreenLogFile); //0.005

		std::cout << "\n\nGlobal feature extraction\n\n";
		pScreenLogFile << "\n\nGlobal feature extraction\n\n";

		LocalFeaturesMap::iterator ItLocalFeaturesMap;
		ObjectMap::iterator ItObjectMap;
		std::vector<BlobListStruct>::iterator ItBlobListStructs;

		/// Find number of local features
		int NumberLocalFeatures=mData.GetNumberLocalFeatures();
		if (NumberLocalFeatures <= 0) return ipa_utils::RET_FAILED;
		int NumberGlobalFeatures = 0;

		/// Iterate over classes
		for (ItLocalFeaturesMap = mData.mLocalFeaturesMap.begin(); ItLocalFeaturesMap != mData.mLocalFeaturesMap.end(); ItLocalFeaturesMap++)
		{
			//if (ItLocalFeaturesMap->first != "bottle") continue;
			std::cout << "\n\nExtracting global Features for class " << ItLocalFeaturesMap->first << ".\n";
			pScreenLogFile << "\n\nExtracting global Features for class " << ItLocalFeaturesMap->first << ".\n";
			
			/// Iterate over objects of a class
			int ObjectCounter = 0;
			for (ItObjectMap = ItLocalFeaturesMap->second.begin(); ItObjectMap != ItLocalFeaturesMap->second.end(); ItObjectMap++, ObjectCounter++)
			{
				//if (ItObjectMap->first < 2) continue;
				CvMat* GlobalFeatures = NULL;

				/// Iterate over pictures of an object
				int BlobListCounter=0;
				int Rows = ItObjectMap->second.size();
				CvMat* Features = NULL;
				for (ItBlobListStructs = ItObjectMap->second.begin(); ItBlobListStructs != ItObjectMap->second.end(); ItBlobListStructs++)
				{
					IplImage* maskImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_8U, 1);
					cvSetZero(maskImage);
					IplImage* xyzImage = cvCreateImage(cvSize(640, 480), IPL_DEPTH_32F, 3);
					cvSetZero(xyzImage);

					pcl::PointCloud<PointXYZRGBIM>::Ptr cloud (new pcl::PointCloud<PointXYZRGBIM>);

					if (pcl::io::loadPCDFile<PointXYZRGBIM> (ItBlobListStructs->FileName, *cloud) == -1) //* load the file
					{
						std::cout << "Couldn't read file " << ItBlobListStructs->FileName << "." << std::endl;
						return ipa_utils::RET_FAILED;
					}

					for (size_t i = 0; i < cloud->points.size (); ++i)
					{
						int u = cloud->points[i].imX;
						int v = cloud->points[i].imY;
						float x = cloud->points[i].x;
						float y = -cloud->points[i].z;
						float z = cloud->points[i].y;

						cvSet2D(xyzImage, v, u, cvScalar(x, y, z, 0));
						cvSetReal2D(maskImage, v, u, 255);
					}

					std::cout << ItBlobListStructs->FileName << "\n";
					pScreenLogFile << ItBlobListStructs->FileName << "\n";
					// check whether features were extracted																	// disabled image output
					if (ExtractGlobalFeatures(&(ItBlobListStructs->BlobFPs), &Features, pClusterMode, pGlobalFeatureParams, CIN2, xyzImage, maskImage/*, cvCloneImage(SourceImage.Shared())*/, 0, false, pTimingLogFileName, &pScreenLogFile) == ipa_utils::RET_OK)
					{
						// initialize GlobalFeatures and determine number of global features
						if (!GlobalFeatures)
						{
							NumberGlobalFeatures = Features->cols;
							GlobalFeatures = cvCreateMat(Rows, NumberGlobalFeatures, CV_32FC1);
						}

						for (int j=0; j<NumberGlobalFeatures; j++) cvmSet(GlobalFeatures, BlobListCounter, j, cvGetReal1D(Features, j));
						BlobListCounter++;
					}
					else
					{
						if (GlobalFeatures)
						{
							// no features available -> reshape
							CvMat* Temp = cvCreateMat(GlobalFeatures->rows-1, GlobalFeatures->cols, GlobalFeatures->type);
							for (int i=0; i<BlobListCounter; i++) for (int j=0; j<Temp->width; j++) cvmSet(Temp, i, j, cvmGet(GlobalFeatures, i, j));
							cvReleaseMat(&GlobalFeatures);
							GlobalFeatures = Temp;
						}
						else
						{
							Rows--;
						}
					}
					cvReleaseImage(&maskImage);
					cvReleaseImage(&xyzImage);
				}
				(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter] = GlobalFeatures;

				/// Output
				/*std::cout << ItLocalFeaturesMap->first << " " << ObjectCounter << "\n";
				for (int i=0; i<(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter]->height; i++)
				{
					for (int j=0; j<(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter]->width; j++) std::cout << cvGetReal2D((mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter], i, j) << "\t";
					std::cout << "\n";
				}*/
			}
		}
		SaveFPDataGlobal(pGlobalFeatureFileName);
	}
	else
	{
		LoadFPDataGlobal(pGlobalFeatureFileName);
	}
	std::cout << "Processing time for global feature extraction: " << sw.stop() << "s.\n";
	pScreenLogFile << "Processing time for global feature extraction: " << sw.stop() << "s.\n";

	return ipa_utils::RET_OK;
}


int ObjectClassifier::LoadALOIDatabase(std::string pAnnotationFileName, int pMode, ClusterMode pClusterMode, GlobalFeatureParams& pGlobalFeatureParams, std::string pLocalFeatureFileName, std::string pGlobalFeatureFileName, std::string pCovarianceMatrixFileName, std::string pLocalFeatureClustererPath)
{
	/// Load class labels from pAnnotationFileName
	std::map<int, ObjectStruct> ObjectStructMap;
	std::map<std::string, std::vector<int> > ObjectCategoryMap;
	std::map<std::string, std::vector<int> > ObjectGeneralCategoryMap;
	ObjectStruct ObjectInformation;

	if (pMode != 2)
	{
		/// Read class label file
		std::ifstream File((pAnnotationFileName).c_str());
		if(!File.is_open())
		{
			std::cout << "Error: could not open " << pAnnotationFileName << "\n";
			return 1;
		}

		std::stringstream ss;
		while(!File.eof())
		{	
			char c = File.get();
			ss << c;
		}

		/// fill ObjectMap with object number and class label
		for (int i=1; i<1001; i++)
		{
			ss >> ObjectInformation.ObjectNum;
			ss >> ObjectInformation.CategoryName;
			ss >> ObjectInformation.GeneralCategoryName;

			ObjectStructMap[ObjectInformation.ObjectNum] = ObjectInformation;
		}

		/// fill ObjectCategoryMap with class label and corresponding object numbers
		std::map<int,ObjectStruct>::iterator ItObjectStructMap = ObjectStructMap.begin();
		for (; ItObjectStructMap != ObjectStructMap.end(); ItObjectStructMap++)
		{
			ObjectCategoryMap[ItObjectStructMap->second.CategoryName].push_back(ItObjectStructMap->first);
			ObjectGeneralCategoryMap[ItObjectStructMap->second.GeneralCategoryName].push_back(ItObjectStructMap->first);
		}

		std::cout << ObjectCategoryMap.size() << " object categories and " << ObjectGeneralCategoryMap.size() << " general object categories found.\n";
	}

	/// Fill mData with data by opening images classwise and extracting features

	/// 1. Local features
	if (pMode < 1)
	{
		int ObjectCounter = 0;
		std::map<std::string, std::vector<int> >::iterator ItObjectCategoryMap;
		std::vector<int>::iterator ItSampleList;
		for (ItObjectCategoryMap = ObjectCategoryMap.begin(); ItObjectCategoryMap != ObjectCategoryMap.end(); ItObjectCategoryMap++)
		{
			int ClassObjectCounter = 0;
			for (ItSampleList = ItObjectCategoryMap->second.begin(); ItSampleList != ItObjectCategoryMap->second.end(); ItSampleList++, ClassObjectCounter++)
			{
				std::cout << "\n\nFeature extraction in class " << ItObjectCategoryMap->first << " on object " << *ItSampleList << " (" << ++ObjectCounter << ". object overall).\n";
				for (int i=0; i<360; i+=5)
				{
					BlobListStruct TempBlobListStruct;
					std::stringstream FileName;
					FileName << "../../Datenbanken/png4/" << *ItSampleList << "/" << *ItSampleList << "_r" << i << ".png";
					
					TempBlobListStruct.FileName = FileName.str();
					ExtractLocalFeatures(TempBlobListStruct.FileName, TempBlobListStruct.BlobFPs, pClusterMode);

					(mData.mLocalFeaturesMap[ItObjectCategoryMap->first])[ClassObjectCounter].push_back(TempBlobListStruct);
				}
			}
		}
		SaveFPDataLocal(pLocalFeatureFileName);
	}
	else
	{
		LoadFPDataLocal(pLocalFeatureFileName);
	}


	/// 2. Global Features
	if (abs(pMode) < 2)
	{
		/// Clustering
		ClusterLocalFeatures(pCovarianceMatrixFileName, pLocalFeatureClustererPath, 10, 250, 10, 1);

		LocalFeaturesMap::iterator ItLocalFeaturesMap;
		ObjectMap::iterator ItObjectMap;
		std::vector<BlobListStruct>::iterator ItBlobListStructs;

		/// Find number of local features
		int NumberLocalFeatures=mData.GetNumberLocalFeatures();
		if (NumberLocalFeatures <= 0) return ipa_utils::RET_FAILED;
		int NumberGlobalFeatures = 0;

		/// Iterate over classes
		for (ItLocalFeaturesMap = mData.mLocalFeaturesMap.begin(); ItLocalFeaturesMap != mData.mLocalFeaturesMap.end(); ItLocalFeaturesMap++)
		{
			std::cout << "Extracting global Features for class " << ItLocalFeaturesMap->first << ".\n";
			
			/// Iterate over objects of a class
			int ObjectCounter = 0;
			for (ItObjectMap = ItLocalFeaturesMap->second.begin(); ItObjectMap != ItLocalFeaturesMap->second.end(); ItObjectMap++, ObjectCounter++)
			{
				CvMat* GlobalFeatures = NULL;

				/// Iterate over pictures of an object
				int BlobListCounter=0;
				for (ItBlobListStructs = ItObjectMap->second.begin(); ItBlobListStructs != ItObjectMap->second.end(); ItBlobListStructs++)
				{
					CvMat* Features = NULL;
					ExtractGlobalFeatures(&(ItBlobListStructs->BlobFPs), &Features, pClusterMode, pGlobalFeatureParams);
					
					// check whether features were extracted
					if (Features!=NULL)
					{
						// initialize GlobalFeatures and determine number of global features
						if (!GlobalFeatures)
						{
							NumberGlobalFeatures = Features->cols;
							GlobalFeatures = cvCreateMat(ItObjectMap->second.size(), NumberGlobalFeatures, CV_32FC1);
						}

						for (int j=0; j<NumberGlobalFeatures; j++) cvmSet(GlobalFeatures, BlobListCounter, j, cvmGet(Features, 0, j));
						BlobListCounter++;
					}
					else
					{
						if (GlobalFeatures)
						{
							// no features available -> reshape
							CvMat* Temp = cvCreateMat((GlobalFeatures)->rows-1, (GlobalFeatures)->cols, (GlobalFeatures)->type);
							for (int i=0; i<BlobListCounter; i++)
							{
								for (int j=0; j<Temp->width; j++) cvmSet(Temp, i, j, cvmGet(GlobalFeatures, i, j));
							}
							cvReleaseMat(&GlobalFeatures);
							GlobalFeatures = Temp;
						}
					}
				}
				(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter] = GlobalFeatures;

				/// Output
			/*	std::cout << ItLocalFeaturesMap->first << " " << ObjectCounter << "\n";
				for (int i=0; i<(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter]->height; i++)
				{
					for (int j=0; j<(mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter]->width; j++) std::cout << cvGetReal2D((mData.mGlobalFeaturesMap[ItLocalFeaturesMap->first])[ObjectCounter], i, j) << "\t";
					std::cout << "\n";
				}*/
			}
		}
		SaveFPDataGlobal(pGlobalFeatureFileName);
	}
	else
	{
		LoadFPDataGlobal(pGlobalFeatureFileName);
	}

	return 0;
}


int ObjectClassifier::ClusterLocalFeatures(std::string pCovarianceMatrixFileName, std::string pLocalFeatureClustererPath, int pNumberClustersMin, int pNumberClustersMax, int pNumberClustersStep, int pNumberValidationRuns, double pMinRelativeDeviationChange, std::ofstream* pScreenLogFile)
{
	/// Clustering methode aufrufen auf allen lokalen Deskriptordaten
	/// 1. Ausfuehrliches K-Means (Problems+Solutions S. 479; pro cluster: cvCalcCovarMatrix, cvSVD -> groessten und mittleren Eigenwert messen)
	///    für Bestimmung der optimalen Clusteranzahl
	/// 2. mehrmals EM mit K-Means (u.U. bereits ausgewaehlt in mehreren Laeufen) Initialisierung
	/// 3. bestes Ergebnis speichern und zum Clustern verwenden

	/// Count local features and determine local feature descriptor dimension
	int NumberSamples = mData.GetNumberLocalFeaturePoints();
	if (NumberSamples < 1) return ipa_utils::RET_FAILED;
	int NumberFeatures = mData.GetNumberLocalFeatures();
	if (NumberFeatures < 1) return ipa_utils::RET_FAILED;


	/// Create matrix of all local features
	CvMat* AllLocalFeatures = mData.CreateAllLocalFeatureMatrix(NumberSamples, NumberFeatures);
	
	//std::ofstream fout("alllocalfeatures.txt", std::ios::out);
	//fout << NumberSamples << "\t" << NumberFeatures << std::endl;
	//for (int s=0; s<NumberSamples; s++)
	//{
	//	for (int f=0; f<NumberFeatures; f++) fout << cvGetReal2D(AllLocalFeatures, s, f) << "\t";
	//	fout << std::endl;
	//}
	//fout.close();
	//std::cout << "File written\n";

	/// Calculate covariance matrix and its squareroot
/*	CvArr** vect = new CvArr*;
	vect[0] = AllLocalFeatures;
	CvMat* CovarMatrix = cvCreateMat(NumberFeatures, NumberFeatures, CV_32FC1);
	CvMat* CovarMatrixSqrt = cvCreateMat(NumberFeatures, NumberFeatures, CV_32FC1);
	CvMat* FeatureAverages = cvCreateMat(1, NumberFeatures, CV_32FC1);
	CvMat* Eigenvalues = cvCreateMat(NumberFeatures, 1, CV_32FC1);
	CvMat* EigenvalueMatrixSqrt = cvCreateMat(NumberFeatures, NumberFeatures, CV_32FC1);
	cvSetZero(EigenvalueMatrixSqrt);
	CvMat* EigenvectorMatrix = cvCreateMat(NumberFeatures, NumberFeatures, CV_32FC1);

	std::cout << "calccovar\n";
	cvCalcCovarMatrix((const CvArr**)vect, 1, CovarMatrix, FeatureAverages,  CV_COVAR_NORMAL | CV_COVAR_SCALE | CV_COVAR_ROWS);

	std::cout << "\nCovar Matrix:\n";
	for (int i = 0; i<CovarMatrix->rows; i++)
	{
		for (int j=0; j<CovarMatrix->cols; j++) std::cout << cvGetReal2D(CovarMatrix, i, j) << "\t\t";
		std::cout << "\n";
	}

	cvInvert(CovarMatrix, CovarMatrix, CV_SVD_SYM);
	
	std::cout << "\nCovar Matrix Inverse:\n";
	for (int i = 0; i<CovarMatrix->rows; i++)
	{
		for (int j=0; j<CovarMatrix->cols; j++) std::cout << cvGetReal2D(CovarMatrix, i, j) << "\t\t";
		std::cout << "\n";
	}

	// get squareroot of inverse covariance matrix (via diagonalization)
	cvSVD(CovarMatrix, Eigenvalues, EigenvectorMatrix, NULL, CV_SVD_MODIFY_A | CV_SVD_U_T);
	for (int i=0; i<NumberFeatures; i++)
	{
		cvSetReal2D(EigenvalueMatrixSqrt, i, i, sqrt(cvGetReal1D(Eigenvalues, i)));
	}
	// CovarMatrixSqrt = EigenvectorMatrix * EigenvalueMatrixSqrt * transpose(EigenvectorMatrix)
	cvGEMM(EigenvectorMatrix, EigenvalueMatrixSqrt, 1, NULL, 0, CovarMatrixSqrt, CV_GEMM_A_T);
	cvGEMM(CovarMatrixSqrt, EigenvectorMatrix, 1, NULL, 0, CovarMatrixSqrt, 0);

	std::cout << "\nCovar Matrix Inverse Sqrt:\n";
	for (int i = 0; i<CovarMatrixSqrt->rows; i++)
	{
		for (int j=0; j<CovarMatrixSqrt->cols; j++) std::cout << cvGetReal2D(CovarMatrixSqrt, i, j) << "\t\t";
		std::cout << "\n";
	}

	std::cout << "\nEigenvector Matrix:\n";
	for (int i = 0; i<EigenvectorMatrix->rows; i++)
	{
		for (int j=0; j<EigenvectorMatrix->cols; j++) std::cout << cvGetReal2D(EigenvectorMatrix, i, j) << "\t\t";
		std::cout << "\n";
	}

	std::cout << "\nEigenvalue Matrix:\n";
	for (int i = 0; i<EigenvalueMatrixSqrt->rows; i++)
	{
		for (int j=0; j<EigenvalueMatrixSqrt->cols; j++) std::cout << cvGetReal2D(EigenvalueMatrixSqrt, i, j) << "\t\t";
		std::cout << "\n";
	}

	/// Save squareroot of inverted covariance matrix
	cvReleaseMat(&(mData.mSqrtInverseCovarianceMatrix));
	mData.mSqrtInverseCovarianceMatrix = CovarMatrixSqrt;
	SaveSqrtInverseCovarianceMatrix(pCovarianceMatrixFileName);


	cvReleaseMat(&CovarMatrix);
	cvReleaseMat(&FeatureAverages);
	cvReleaseMat(&Eigenvalues);
	cvReleaseMat(&EigenvalueMatrixSqrt);
	cvReleaseMat(&EigenvectorMatrix);

	/// Remove variance from input data (AllLocalFeatures[New] = AllLocalFeatures * CovarMatrix^(-0.5))
	// cvGEMM(AllLocalFeatures, CovarMatrixSqrt, 1, NULL, 0, AllLocalFeatures, 0);
	CvMat* Temp = cvCreateMat(AllLocalFeatures->rows, AllLocalFeatures->cols, AllLocalFeatures->type);
	Temp = cvCloneMat(AllLocalFeatures);
	for (int i = 0; i<AllLocalFeatures->rows; i++)
	{
		for (int j=0; j<AllLocalFeatures->cols; j++)
		{
			double sum=0;
			for (int k=0; k<AllLocalFeatures->cols; k++) sum += cvGetReal2D(Temp, i, k)*cvGetReal2D(CovarMatrixSqrt, k, j);
			cvSetReal2D(AllLocalFeatures, i, j, sum);
		}
	}
	cvReleaseMat(&Temp);
*/


	/// Find optimal number of clusters (i.e. the number from which an increase in the number of clusters does not improve the result very much)
	int NumberClustersOptimal = pNumberClustersMax;
	CvMat* ClusterCentersOptimal = NULL;
	double LastMinDeviationFromCenter = DBL_MAX;
	for (int K = pNumberClustersMin; K <= pNumberClustersMax; K += pNumberClustersStep)
	{
		// try several differently (randomly) initilized K-Means runs and determine the mean distance of data points from the cluster center
		// choose the shortest mean distance as comparing criterion
		double MinDeviationFromCenter = DBL_MAX;
		std::cout << "\nK = " << K << "\nCluster Centers:\n";
		if (pScreenLogFile) *pScreenLogFile << "\nK = " << K << "\nCluster Centers:\n";
		for (int RunCounter=0; RunCounter<pNumberValidationRuns; RunCounter++)
		{
			CvMat* ClusterLabels = cvCreateMat(NumberSamples, 1, CV_32SC1);
			CvMat* ClusterCenters = cvCreateMat(K, NumberFeatures, CV_32FC1);
			cvKMeans2(AllLocalFeatures, K, ClusterLabels, cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0), 1, 0, cv::KMEANS_PP_CENTERS, ClusterCenters);

			// calculate cluster centers - unnecessary since KMeans2 can return them
			////CvMat* ClusterCentersSampleCounter = cvCreateMat(K, 1, CV_32FC1);
			//unsigned int* ClusterCentersSampleCounter = new unsigned int[K];
			//cvSetZero(ClusterCenters);
			////cvSetZero(ClusterCentersSampleCounter);
			//for (int i=0; i<K; i++) ClusterCentersSampleCounter[i] = 0.f;
			//for (int SampleCounter = 0; SampleCounter<NumberSamples; SampleCounter++)
			//{
			//	int ClusterLabel = (int)cvGetReal1D(ClusterLabels, SampleCounter);
			//	for (int j=0; j<NumberFeatures; j++)
			//		cvSetReal2D(ClusterCenters, ClusterLabel, j, cvGetReal2D(ClusterCenters, ClusterLabel, j)+cvGetReal2D(AllLocalFeatures, SampleCounter, j));
			//	//cvSetReal1D(ClusterCentersSampleCounter, ClusterLabel, cvGetReal1D(ClusterCentersSampleCounter, ClusterLabel)+1);
			//	ClusterCentersSampleCounter[ClusterLabel]++;
			//}
			//for (int i=0; i<ClusterCenters->rows; i++)
			//	for (int j=0; j<NumberFeatures; j++)
			//		//cvSetReal2D(ClusterCenters, i, j, cvGetReal2D(ClusterCenters, i, j)/cvGetReal1D(ClusterCentersSampleCounter, i));
			//		cvSetReal2D(ClusterCenters, i, j, cvGetReal2D(ClusterCenters, i, j)/(double)ClusterCentersSampleCounter[i]);

			////cvReleaseMat(&ClusterCentersSampleCounter);
			//delete[] ClusterCentersSampleCounter;

			
			//for (int i=0; i<ClusterCenters->rows; i++)
			//{
			//	for (int j=0; j<ClusterCenters->cols; j++) std::cout << cvGetReal2D(ClusterCenters, i, j) << "\t";
			//	std::cout << "\n";
			//}

			// calculate mean deviation from centers
			double Deviation = 0;
			for (int SampleCounter = 0; SampleCounter<NumberSamples; SampleCounter++)
			{
				double Distance = 0;
				int ClusterLabel = (int)cvGetReal1D(ClusterLabels, SampleCounter);
				for (int j=0; j<NumberFeatures; j++)
				{
					double dist = cvmGet(AllLocalFeatures, SampleCounter, j)-cvmGet(ClusterCenters, ClusterLabel, j);
					Distance += dist*dist;
				}
				Deviation += sqrt(Distance);
			}
			Deviation /= NumberSamples;

			std::cout << "Deviation: " << Deviation << "\n";
			if (pScreenLogFile) *pScreenLogFile << "Deviation: " << Deviation << "\n";

			// better result found inside the runs for K clusters?
			if (Deviation < MinDeviationFromCenter)
			{
				MinDeviationFromCenter = Deviation;

				// better result than with fewer clusters?
				if (Deviation < LastMinDeviationFromCenter)
				{
					cvReleaseMat(&ClusterCentersOptimal);
					ClusterCentersOptimal = cvCloneMat(ClusterCenters);
					NumberClustersOptimal = K;
				}
			}
			cvReleaseMat(&ClusterCenters);
			cvReleaseMat(&ClusterLabels);
		}

		double RelativeDeviationChange = (LastMinDeviationFromCenter-MinDeviationFromCenter)/MinDeviationFromCenter;

		std::cout << "RelativeDeviationChange: " << RelativeDeviationChange << "\n";
		if (pScreenLogFile) *pScreenLogFile << "RelativeDeviationChange: " << RelativeDeviationChange << "\n";
		if (RelativeDeviationChange < pMinRelativeDeviationChange)
		{
			//NumberClustersOptimal = K;
			break;
		}

		LastMinDeviationFromCenter = MinDeviationFromCenter;
	}

	
	/// Perform EM
#if (CV_MAJOR_VERSION<=2 && CV_MINOR_VERSION<=3)
	CvEMParams EMParams = CvEMParams(NumberClustersOptimal, CvEM::COV_MAT_SPHERICAL, CvEM::START_AUTO_STEP, cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, FLT_EPSILON), NULL, NULL, NULL, NULL);
	if (ClusterCentersOptimal != NULL)
	{
		EMParams.means = ClusterCentersOptimal;
		EMParams.start_step = CvEM::START_E_STEP;
		std::cout << "\nNumberClustersOptimal: " << NumberClustersOptimal << "   ClusterCentersOptimal: " << ClusterCentersOptimal->rows << " x " << ClusterCentersOptimal->cols << "\n";
		if (pScreenLogFile) *pScreenLogFile << "\nNumberClustersOptimal: " << NumberClustersOptimal << "   ClusterCentersOptimal: " << ClusterCentersOptimal->rows << " x " << ClusterCentersOptimal->cols << "\n";
	}
//	CvMat* ClusterLabels = cvCreateMat(NumberSamples, 1, CV_32SC1);
	cv::Mat allLocalFeatures(AllLocalFeatures, true);
	mData.mLocalFeatureClusterer->train(allLocalFeatures, cv::Mat(), EMParams, NULL);
	std::cout << "First train done (spherical). LogLikelihood: " << mData.mLocalFeatureClusterer->get_log_likelihood() << "\n";
	if (pScreenLogFile) *pScreenLogFile << "First train done (spherical). LogLikelihood: " << mData.mLocalFeatureClusterer->get_log_likelihood() << "\n";

	EMParams.cov_mat_type = CvEM::COV_MAT_DIAGONAL;
	EMParams.start_step = CvEM::START_E_STEP;
	CvMat** Covs = new CvMat*[NumberClustersOptimal];
	for (int i=0; i<NumberClustersOptimal; i++) Covs[i] = cvCloneMat((mData.mLocalFeatureClusterer->get_covs()[i]));
	EMParams.covs = (const CvMat**)Covs;
	CvMat* Means = cvCloneMat(mData.mLocalFeatureClusterer->get_means());
	EMParams.means = Means;
	CvMat* Weights = cvCloneMat(mData.mLocalFeatureClusterer->get_weights());
	EMParams.weights = Weights;
	std::cout << (EMParams.covs[0])->rows << "   " << (EMParams.covs[0])->cols << "\n";
	std::cout << EMParams.means->rows << "   " << EMParams.means->cols << "\n";
	std::cout << EMParams.weights->rows << "   " << EMParams.weights->cols << "\n";
	if (pScreenLogFile) *pScreenLogFile << (EMParams.covs[0])->rows << "   " << (EMParams.covs[0])->cols << "\n" << EMParams.means->rows << "   " << EMParams.means->cols << "\n" << EMParams.weights->rows << "   " << EMParams.weights->cols << "\n";
	mData.mLocalFeatureClusterer->train(AllLocalFeatures, NULL, EMParams, NULL);
	std::cout << "Second train done (diagonal). LogLikelihood: " << mData.mLocalFeatureClusterer->get_log_likelihood() << "\n";
	if (pScreenLogFile) *pScreenLogFile << "Second train done (diagonal). LogLikelihood: " << mData.mLocalFeatureClusterer->get_log_likelihood() << "\n";
#else
	if (mData.mLocalFeatureClusterer != 0)
			delete mData.mLocalFeatureClusterer;
	mData.mLocalFeatureClusterer = new cv::EM(NumberClustersOptimal, cv::EM::COV_MAT_SPHERICAL, cv::TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, FLT_EPSILON));
	cv::Mat allLocalFeatures(AllLocalFeatures, true);
	if (ClusterCentersOptimal != NULL)
	{
		cv::Mat clusterCentersOptimalMat(ClusterCentersOptimal);
		mData.mLocalFeatureClusterer->trainE(allLocalFeatures, clusterCentersOptimalMat);
		std::cout << "\nNumberClustersOptimal: " << NumberClustersOptimal << "   ClusterCentersOptimal: " << ClusterCentersOptimal->rows << " x " << ClusterCentersOptimal->cols << "\n";
		if (pScreenLogFile) *pScreenLogFile << "\nNumberClustersOptimal: " << NumberClustersOptimal << "   ClusterCentersOptimal: " << ClusterCentersOptimal->rows << " x " << ClusterCentersOptimal->cols << "\n";
	}
	else
	{
		mData.mLocalFeatureClusterer->train(allLocalFeatures);
	}
	std::cout << "First train done (spherical).\n";
	if (pScreenLogFile) *pScreenLogFile << "First train done (spherical).\n";

	mData.mLocalFeatureClusterer->set("covMatType", cv::EM::COV_MAT_DIAGONAL);
	cv::Mat means = mData.mLocalFeatureClusterer->get<cv::Mat>("means");
	std::vector<cv::Mat> covs = mData.mLocalFeatureClusterer->get<std::vector<cv::Mat> >("covs");
	cv::Mat weights = mData.mLocalFeatureClusterer->get<cv::Mat>("weights");
	std::cout << covs[0].rows << "   " << covs[0].cols << "\n";
	std::cout << means.rows << "   " << means.cols << "\n";
	std::cout << weights.rows << "   " << weights.cols << "\n";
	if (pScreenLogFile) *pScreenLogFile << covs[0].rows << "   " << covs[0].cols << "\n" << means.rows << "   " << means.cols << "\n" << weights.rows << "   " << weights.cols << "\n";
	mData.mLocalFeatureClusterer->trainE(allLocalFeatures, means, covs, weights);
	std::cout << "Second train done (diagonal).\n";
	if (pScreenLogFile) *pScreenLogFile << "Second train done (diagonal).\n";
#endif

	/// Save EM
	std::stringstream FileName;
	FileName << pLocalFeatureClustererPath << "/EMClusterer" << NumberClustersOptimal << ".txt";
	SaveLocalFeatureClusterer(FileName.str());
	//mData.mLocalFeatureClusterer->save();

	/// Output EM centers
/*	CvMat* ClusterCenters = cvCloneMat(mData.mLocalFeatureClusterer->get_means());
	std::cout << "\nEM ClusterCenters: " << ClusterCenters->rows << " x " << ClusterCenters->cols << "\n";
	for (int i=0; i<ClusterCenters->rows; i++)
	{
		for (int j=0; j<ClusterCenters->cols; j++) std::cout << cvmGet(ClusterCenters, i, j) << "\t";
		std::cout << "\n";
	}
	cvReleaseMat(&ClusterCenters);
*///	cvReleaseMat(&ClusterLabels);

#if (CV_MAJOR_VERSION<=2 && CV_MINOR_VERSION<=3)
	for (int i=0; i<NumberClustersOptimal; i++) cvReleaseMat(&(Covs[i]));
	cvReleaseMat(&Means);
	cvReleaseMat(&Weights);
#endif

	cvReleaseMat(&AllLocalFeatures);
	// cvReleaseMat(&CovarMatrixSqrt);  --> pointer taken from mData.mSqrtInverseCovarianceMatrix
	if (ClusterCentersOptimal) cvReleaseMat(&ClusterCentersOptimal);

	return ipa_utils::RET_OK;
}


int ObjectClassifier::TrainLocal(std::string pClassifierSavePath, float pFactorCorrect, float pFactorIncorrect, ClassifierType pClassifierType)
{
	std::cout << "\n\nTraining " << ClassifierLabel(pClassifierType) << " classifiers.\n";

	/// Train local classifiers
	
	// iterate over all classes
	LocalFeaturesMap::iterator ItLocalFeaturesMap;
	int Counter = 1;
	for (ItLocalFeaturesMap = mData.mLocalFeaturesMap.begin(); ItLocalFeaturesMap != mData.mLocalFeaturesMap.end(); ItLocalFeaturesMap++, Counter++)
	{
		std::cout << "Training class " << ItLocalFeaturesMap->first << " (" << Counter << "th out of " << mData.mLocalFeaturesMap.size() << ")" << ".\n";
		// get training data matrix
		CvMat* TrainingFeatureMatrix = NULL;
		int NumberCorrectSamples = 0;
		if (mData.GetLocalFeatureMatrix(ItLocalFeaturesMap->first, &TrainingFeatureMatrix, pFactorCorrect, pFactorIncorrect, NumberCorrectSamples) == ipa_utils::RET_FAILED) continue;

		// create correct response matrix
		CvMat* TrainingCorrectResponses = cvCreateMat(TrainingFeatureMatrix->rows, 1, CV_32FC1);
		for (int i=0; i<NumberCorrectSamples; i++) cvSetReal1D(TrainingCorrectResponses, i, 1.0);
		for (int i=NumberCorrectSamples; i<TrainingCorrectResponses->rows; i++) cvSetReal1D(TrainingCorrectResponses, i, 0.0);

		TrainLocal(pClassifierType, ItLocalFeaturesMap->first, TrainingFeatureMatrix, TrainingCorrectResponses);
		
		cvReleaseMat(&TrainingFeatureMatrix);
		cvReleaseMat(&TrainingCorrectResponses);
	}

	// save classifiers
	mData.SaveLocalClassifiers(pClassifierSavePath, pClassifierType);

	return ipa_utils::RET_OK;
}


int ObjectClassifier::TrainLocal(ClassifierType pClassifierType, std::string pClass, int pNumberSamples, int pNumberFeatures, LocalFeaturesMap::iterator pItLocalFeaturesClass, std::list<int>* pIndicesTrainCorrect, std::list<int>* pIndicesTrainIncorrect, CvMat* pNegativeSamplesMatrix)
{
	// create training data matrix and response matrix
	CvMat* TrainingFeatureMatrix = cvCreateMat(pNumberSamples, pNumberFeatures, pNegativeSamplesMatrix->type);
	CvMat* TrainingFeatureResponseMatrix = cvCreateMat(pNumberSamples, 1, pNegativeSamplesMatrix->type);

	// fill training data matrix with correct samples
	std::list<int>::iterator ItIndices;
	LocalFeaturesMap::value_type::second_type::value_type::second_type::iterator ItBlobListStructs;
	BlobListRiB::iterator ItBlobFPs;
	int SampleIndex = 0;
	for (ItIndices = pIndicesTrainCorrect->begin(); ItIndices != pIndicesTrainCorrect->end(); ItIndices++)
	{
		for (ItBlobListStructs = pItLocalFeaturesClass->second[*ItIndices].begin(); ItBlobListStructs != pItLocalFeaturesClass->second[*ItIndices].end(); ItBlobListStructs++)
		{
			for (ItBlobFPs = ItBlobListStructs->BlobFPs.begin(); ItBlobFPs != ItBlobListStructs->BlobFPs.end(); ItBlobFPs++, SampleIndex++)
			{
				for (int j=0; j<pNumberFeatures; j++)
				{
					cvmSet(TrainingFeatureMatrix, SampleIndex, j, ItBlobFPs->m_D[j]);
					cvSetReal1D(TrainingFeatureResponseMatrix, SampleIndex, 1.0);
				}
			}
		}
	}

	// fill training data matrix with incorrect samples
	for (ItIndices = pIndicesTrainIncorrect->begin(); ItIndices != pIndicesTrainIncorrect->end(); ItIndices++, SampleIndex++)
	{
		for (int j=0; j<pNumberFeatures; j++)
		{
			cvmSet(TrainingFeatureMatrix, SampleIndex, j, cvmGet(pNegativeSamplesMatrix, *ItIndices, j));
			cvSetReal1D(TrainingFeatureResponseMatrix, SampleIndex, 0.0);
		}
	}

	// train classifier
	TrainLocal(pClassifierType, pClass, TrainingFeatureMatrix, TrainingFeatureResponseMatrix);

	cvReleaseMat(&TrainingFeatureMatrix);
	cvReleaseMat(&TrainingFeatureResponseMatrix);
	
	return ipa_utils::RET_OK;
}


int ObjectClassifier::TrainLocal(ClassifierType pClassifierType, std::string pClass, CvMat* pTrainingFeatureMatrix, CvMat* pTrainingCorrectResponses)
{
	// clean if old classifier is found
	LocalClassifierMap::iterator ItLocalClassifierMap;
	if ((ItLocalClassifierMap=mData.mLocalClassifierMap.find(pClass)) != mData.mLocalClassifierMap.end())
	{
		ItLocalClassifierMap->second->clear();
	}

	switch (pClassifierType)
	{
		case CLASSIFIER_RTC:
			{
				// create new classifier
				mData.mLocalClassifierMap[pClass] = new CvRTrees;
				CvRTrees* RTC = NULL;
				RTC = dynamic_cast<CvRTrees*>(mData.mLocalClassifierMap[pClass]);

				// train classifier
				CvMat* VarType = cvCreateMat(1,(pTrainingFeatureMatrix->width+1),CV_8UC1);
		//		if (ClassifierMode == CLASSIFICATION)
		//		{
		//			for (int i=0; i<(pTrainingFeatureMatrix->width+1); i++) cvSetReal1D(VarType, i, CV_VAR_CATEGORICAL);
		//		}
		//		else
		//		{
					for (int i=0; i<(pTrainingFeatureMatrix->width+1); i++) cvSetReal1D(VarType, i, CV_VAR_ORDERED);
					//cvSetReal1D(VarType, pTrainingFeatureMatrix->width, CV_VAR_CATEGORICAL);
		//		}
				CvRTParams RTParams = CvRTParams();
				RTParams.calc_var_importance = true;
				RTParams.nactive_vars = (int)sqrt((float)pTrainingFeatureMatrix->width);
				RTParams.term_crit = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 50, 0.05);
				float Priors[] = {1.0, 1.0};
				RTParams.priors = Priors;
				//delete Priors;
				RTParams.truncate_pruned_tree=true;
				RTC->train(pTrainingFeatureMatrix, CV_ROW_SAMPLE, pTrainingCorrectResponses, 0, 0, VarType, 0, RTParams);

				cvReleaseMat(&VarType);
				break;
			}
		case CLASSIFIER_SVM:
			{
				// create new classifier
				mData.mLocalClassifierMap[pClass] = new CvSVM;
				CvSVM* SVM = dynamic_cast<CvSVM*>(mData.mLocalClassifierMap[pClass]);

				// train classifier
				CvSVMParams SVMParams = CvSVMParams(CvSVM::NU_SVR, CvSVM::RBF, 0, 2.0, 0, 1.0, 0.2, 0, 0, cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 50, 0.05));
				SVM->train(pTrainingFeatureMatrix, pTrainingCorrectResponses, 0, 0, SVMParams);
				break;
			}
		case CLASSIFIER_BOOST:
			{
				// create new classifier
				mData.mLocalClassifierMap[pClass] = new CvBoost;
				CvBoost* Boost = dynamic_cast<CvBoost*>(mData.mLocalClassifierMap[pClass]);

				// train classifier
				CvMat* VarType = cvCreateMat(1,(pTrainingFeatureMatrix->width+1),CV_8UC1);
				for (int i=0; i<(pTrainingFeatureMatrix->width); i++) cvSetReal1D(VarType, i, CV_VAR_ORDERED);
				cvSetReal1D(VarType, pTrainingFeatureMatrix->width, CV_VAR_CATEGORICAL);
				
				CvBoostParams BoostParams = CvBoostParams(CvBoost::GENTLE, 1000, 0.95, 1, false, NULL);
				Boost->train(pTrainingFeatureMatrix, CV_ROW_SAMPLE, pTrainingCorrectResponses, 0, 0, VarType, 0, BoostParams);

				cvReleaseMat(&VarType);
				break;
			}
		case CLASSIFIER_KNN:
			{
				// create new classifier
				mData.mLocalClassifierMap[pClass] = new CvKNearest;
				CvKNearest* KNN = dynamic_cast<CvKNearest*>(mData.mLocalClassifierMap[pClass]);

				// train classifier
				KNN->train(pTrainingFeatureMatrix, pTrainingCorrectResponses, 0, true);

				break;
			}
	}
	return ipa_utils::RET_OK;
}


int ObjectClassifier::TrainGlobal(std::string pPath, float pFactorCorrect, float pFactorIncorrect, ClassifierType pClassifierType)
{
	std::cout << "\n\nTraining " << ClassifierLabel(pClassifierType) << " classifiers.\n";
	
	/// Train global classifiers
	
	// iterate over all classes
	GlobalFeaturesMap::iterator ItGlobalFeaturesMap;
	int Counter = 1;
	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++, Counter++)
	{
		std::cout << "Training class " << ItGlobalFeaturesMap->first << " (" << Counter << "th out of " << mData.mGlobalFeaturesMap.size() << ")" << ".\n";
		// get training data matrix
		CvMat* TrainingFeatureMatrix = NULL;
		int NumberCorrectSamples = 0;
		if (mData.GetGlobalFeatureMatrix(ItGlobalFeaturesMap->first, &TrainingFeatureMatrix, pFactorCorrect, pFactorIncorrect, NumberCorrectSamples) == ipa_utils::RET_FAILED) continue;

		// create correct response matrix
		CvMat* TrainingCorrectResponses = cvCreateMat(TrainingFeatureMatrix->rows, 1, CV_32FC1);
		for (int i=0; i<NumberCorrectSamples; i++) cvmSet(TrainingCorrectResponses, i, 0, 1.0);
		for (int i=NumberCorrectSamples; i<TrainingCorrectResponses->rows; i++) cvmSet(TrainingCorrectResponses, i, 0, 0.0);

		TrainGlobal(pClassifierType, ItGlobalFeaturesMap->first, TrainingFeatureMatrix, TrainingCorrectResponses);
		
		cvReleaseMat(&TrainingFeatureMatrix);
		cvReleaseMat(&TrainingCorrectResponses);
	}

	// save: file name: ClassName_Classifier_Loc/Glob.txt
	mData.SaveGlobalClassifiers(pPath, pClassifierType);

	return ipa_utils::RET_OK;
}


int ObjectClassifier::TrainGlobalSampleRange(ClassifierType pClassifierType, std::string pClass, int pNumberSamples, int pNumberFeatures, GlobalFeaturesMap::iterator pItGlobalFeaturesClass, std::list<int>* pIndicesTrainCorrect, std::list<int>* pIndicesTrainIncorrect, CvMat* pNegativeSamplesMatrix, double rangeStartFactor, double rangeEndFactor)
{
	// create training data matrix and response matrix
	CvMat* TrainingFeatureMatrix = cvCreateMat(pNumberSamples, pNumberFeatures, pItGlobalFeaturesClass->second[0]->type);
	CvMat* TrainingFeatureResponseMatrix = cvCreateMat(pNumberSamples, 1, pItGlobalFeaturesClass->second[0]->type);

	// fill training data matrix with correct samples
	std::list<int>::iterator ItIndices;
	int SampleIndex = 0;
	for (ItIndices = pIndicesTrainCorrect->begin(); ItIndices != pIndicesTrainCorrect->end(); ItIndices++)
	{
		for (double dSample=pItGlobalFeaturesClass->second[*ItIndices]->rows*rangeStartFactor; dSample<pItGlobalFeaturesClass->second[*ItIndices]->rows * rangeEndFactor; dSample+=1.)
		{
			int i = (int)dSample;
			//for (int i=0; i<pItGlobalFeaturesClass->second[*ItIndices]->rows; i++, SampleIndex++)
			//{
			for (int j=0; j<pNumberFeatures; j++)
			{
				cvmSet(TrainingFeatureMatrix, SampleIndex, j, cvmGet(pItGlobalFeaturesClass->second[*ItIndices], i, j));
			}
			cvmSet(TrainingFeatureResponseMatrix, SampleIndex, 0, 1.0);
			SampleIndex++;
		}
	}

	// fill training data matrix with incorrect samples
	for (ItIndices = pIndicesTrainIncorrect->begin(); ItIndices != pIndicesTrainIncorrect->end(); ItIndices++, SampleIndex++)
	{
		for (int j=0; j<pNumberFeatures; j++)
		{
			cvmSet(TrainingFeatureMatrix, SampleIndex, j, cvmGet(pNegativeSamplesMatrix, *ItIndices, j));
		}
		cvmSet(TrainingFeatureResponseMatrix, SampleIndex, 0, 0.0);
	}

	// train classifier
	TrainGlobal(pClassifierType, pClass, TrainingFeatureMatrix, TrainingFeatureResponseMatrix);
	cvReleaseMat(&TrainingFeatureMatrix);
	cvReleaseMat(&TrainingFeatureResponseMatrix);

	return ipa_utils::RET_OK;
}


int ObjectClassifier::TrainGlobal(ClassifierType pClassifierType, std::string pClass, int pNumberSamples, int pNumberFeatures, GlobalFeaturesMap::iterator pItGlobalFeaturesClass, std::list<int>* pIndicesTrainCorrect, std::list<int>* pIndicesTrainIncorrect, CvMat* pNegativeSamplesMatrix, int pViewsPerObject)
{
	// create training data matrix and response matrix
	CvMat* TrainingFeatureMatrix = cvCreateMat(pNumberSamples, pNumberFeatures, pItGlobalFeaturesClass->second[0]->type);
	CvMat* TrainingFeatureResponseMatrix = cvCreateMat(pNumberSamples, 1, pItGlobalFeaturesClass->second[0]->type);

	// fill training data matrix with correct samples
	std::list<int>::iterator ItIndices;
	int SampleIndex = 0;
	for (ItIndices = pIndicesTrainCorrect->begin(); ItIndices != pIndicesTrainCorrect->end(); ItIndices++)
	{
		for (double dSample=0; dSample<pItGlobalFeaturesClass->second[*ItIndices]->rows; (pViewsPerObject==-1) ? dSample+=1. : dSample+=max(1., (double)pItGlobalFeaturesClass->second[*ItIndices]->rows/(double)pViewsPerObject))
		{
			int i = (int)dSample;
		//for (int i=0; i<pItGlobalFeaturesClass->second[*ItIndices]->rows; i++, SampleIndex++)
		//{
			for (int j=0; j<pNumberFeatures; j++)
			{
				cvmSet(TrainingFeatureMatrix, SampleIndex, j, cvmGet(pItGlobalFeaturesClass->second[*ItIndices], i, j));
			}
			cvmSet(TrainingFeatureResponseMatrix, SampleIndex, 0, 1.0);
			SampleIndex++;
		}
	}

	// fill training data matrix with incorrect samples
	for (ItIndices = pIndicesTrainIncorrect->begin(); ItIndices != pIndicesTrainIncorrect->end(); ItIndices++, SampleIndex++)
	{
		for (int j=0; j<pNumberFeatures; j++)
		{
			cvmSet(TrainingFeatureMatrix, SampleIndex, j, cvmGet(pNegativeSamplesMatrix, *ItIndices, j));
		}
		cvmSet(TrainingFeatureResponseMatrix, SampleIndex, 0, 0.0);
	}

	// train classifier
	TrainGlobal(pClassifierType, pClass, TrainingFeatureMatrix, TrainingFeatureResponseMatrix);
	cvReleaseMat(&TrainingFeatureMatrix);
	cvReleaseMat(&TrainingFeatureResponseMatrix);
	
	return ipa_utils::RET_OK;
}

int ObjectClassifier::TrainGlobal(ClassifierType pClassifierType, std::string pClass, CvMat* pTrainingFeatureMatrix, CvMat* pTrainingCorrectResponses)
{
	// clean if old classifier is found
	GlobalClassifierMap::iterator ItGlobalClassifierMap;
	if ((ItGlobalClassifierMap=mData.mGlobalClassifierMap.find(pClass)) != mData.mGlobalClassifierMap.end())
	{
		ItGlobalClassifierMap->second->clear();
		delete ItGlobalClassifierMap->second;
	}

	switch (pClassifierType)
	{
		case CLASSIFIER_RTC:
			{
				// create new classifier
				mData.mGlobalClassifierMap[pClass] = new CvRTrees;
				CvRTrees* RTC = NULL;
				RTC = dynamic_cast<CvRTrees*>(mData.mGlobalClassifierMap[pClass]);

				// train classifier
				CvMat* VarType = cvCreateMat(1,(pTrainingFeatureMatrix->width+1),CV_8UC1);
		//		if (ClassifierMode == CLASSIFICATION)
		//		{
		//			for (int i=0; i<(pTrainingFeatureMatrix->width+1); i++) cvSetReal1D(VarType, i, CV_VAR_CATEGORICAL);
		//		}
		//		else
		//		{
					for (int i=0; i<(pTrainingFeatureMatrix->width+1); i++) cvSetReal1D(VarType, i, CV_VAR_ORDERED);
					//cvSetReal1D(VarType, pTrainingFeatureMatrix->width, CV_VAR_CATEGORICAL);
		//		}
				int maxDepth = 5;
				float priors[] = {1.0, 1.0};
				CvRTParams RTParams = CvRTParams(maxDepth, 10, 0.0f, false, 2, priors, true, (int)sqrt((float)pTrainingFeatureMatrix->width), 250, 0.0001f, CV_TERMCRIT_ITER | CV_TERMCRIT_EPS);
				//RTParams.calc_var_importance = true;
				//RTParams.nactive_vars = (int)sqrt((float)pTrainingFeatureMatrix->width);
				// the number of iterations corresponds with the max. number of trees in the forest, the eps criterion corresponds with the forest accuracy (using the internal cross-validation)
				//RTParams.term_crit = cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 250, 0.0001);
				//RTParams.priors = priors;
				//delete priors;
				//RTParams.truncate_pruned_tree=true;	// does not apply to Random Forests --> no effect
				RTC->train(pTrainingFeatureMatrix, CV_ROW_SAMPLE, pTrainingCorrectResponses, 0, 0, VarType, 0, RTParams);

				cvReleaseMat(&VarType);
				break;
			}
		case CLASSIFIER_SVM:
			{
				// create new classifier
				mData.mGlobalClassifierMap[pClass] = new CvSVM;
				CvSVM* SVM = dynamic_cast<CvSVM*>(mData.mGlobalClassifierMap[pClass]);

				// train classifier
				CvSVMParams SVMParams = CvSVMParams(CvSVM::NU_SVR, CvSVM::RBF, 0, 0.1, 0, 1.0, 0.7, 0, 0, cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 2500, 0.0001));
				SVM->train(pTrainingFeatureMatrix, pTrainingCorrectResponses, 0, 0, SVMParams);
				break;
			}
		case CLASSIFIER_BOOST:
			{
				// create new classifier
				mData.mGlobalClassifierMap[pClass] = new CvBoost;
				CvBoost* Boost = dynamic_cast<CvBoost*>(mData.mGlobalClassifierMap[pClass]);

				// train classifier
				CvMat* VarType = cvCreateMat(1,(pTrainingFeatureMatrix->width+1),CV_8UC1);
				for (int i=0; i<(pTrainingFeatureMatrix->width); i++) cvSetReal1D(VarType, i, CV_VAR_ORDERED);
				cvSetReal1D(VarType, pTrainingFeatureMatrix->width, CV_VAR_CATEGORICAL);
				
				CvBoostParams BoostParams = CvBoostParams(CvBoost::GENTLE, 2000, 0.95, 1, false, NULL);
				Boost->train(pTrainingFeatureMatrix, CV_ROW_SAMPLE, pTrainingCorrectResponses, 0, 0, VarType, 0, BoostParams);

				cvReleaseMat(&VarType);
				break;
			}
		case CLASSIFIER_KNN:
			{
				// create new classifier
				mData.mGlobalClassifierMap[pClass] = new CvKNearest;
				CvKNearest* KNN = dynamic_cast<CvKNearest*>(mData.mGlobalClassifierMap[pClass]);

				// train classifier
				KNN->train(pTrainingFeatureMatrix, pTrainingCorrectResponses, 0, true);

				break;
			}
	}
	return ipa_utils::RET_OK;
}


int ObjectClassifier::PredictLocal(ClassifierType pClassifierType, std::string pClass, CvMat* pFeatureData, double& pPredictionResponse)
{
	LocalClassifierMap::iterator ItLocalClassifierMap;
	if ((ItLocalClassifierMap = mData.mLocalClassifierMap.find(pClass)) == mData.mLocalClassifierMap.end())
	{
		std::cout << "ObjectClassifier::PredictLocal: No classifier found for class " << pClass << ".\n";
		return ipa_utils::RET_FAILED;
	}

	switch (pClassifierType)
	{
		case CLASSIFIER_RTC:
			{
				CvRTrees* RTC = dynamic_cast<CvRTrees*> (ItLocalClassifierMap->second);
				pPredictionResponse = RTC->predict(pFeatureData);
				break;
			}
		case CLASSIFIER_SVM:
			{
				CvSVM* SVM = dynamic_cast<CvSVM*> (ItLocalClassifierMap->second);
				pPredictionResponse = SVM->predict(pFeatureData);
				break;
			}
		case CLASSIFIER_BOOST:
			{
				CvBoost* Boost = dynamic_cast<CvBoost*> (ItLocalClassifierMap->second);
				pPredictionResponse = Boost->predict(pFeatureData, 0, 0, CV_WHOLE_SEQ, false, true);
				double posExp = exp(pPredictionResponse);
				pPredictionResponse = posExp/(posExp + exp(-pPredictionResponse));
				break;
			}
		case CLASSIFIER_KNN:
			{
				CvKNearest* KNN = dynamic_cast<CvKNearest*> (ItLocalClassifierMap->second);
				int k=1;
				pPredictionResponse = KNN->find_nearest(pFeatureData, k);
				break;
			}
		default:
			{
				std::cout << "ObjectClassifier::PredictLocal: Error: Classifier type unknown.\n";
				return ipa_utils::RET_FAILED;
			}
	};

//	std::cout << "Sample belongs to class " << pClass << " with degree " << pPredictionResponse << ".\n";

	return ipa_utils::RET_OK;
}


int ObjectClassifier::PredictGlobal(ClassifierType pClassifierType, std::string pClass, CvMat* pFeatureData, double& pPredictionResponse)
{
	GlobalClassifierMap::iterator ItGlobalClassifierMap;
	if ((ItGlobalClassifierMap = mData.mGlobalClassifierMap.find(pClass)) == mData.mGlobalClassifierMap.end())
	{
		std::cout << "ObjectClassifier::PredictGlobal: No classifier found for class " << pClass << ".\n";
		return ipa_utils::RET_FAILED;
	}

	switch (pClassifierType)
	{
		case CLASSIFIER_RTC:
			{
				CvRTrees* RTC = NULL;
				RTC = dynamic_cast<CvRTrees*> (ItGlobalClassifierMap->second);
				pPredictionResponse = RTC->predict(pFeatureData);
				break;
			}
		case CLASSIFIER_SVM:
			{
				CvSVM* SVM = dynamic_cast<CvSVM*> (ItGlobalClassifierMap->second);
				pPredictionResponse = SVM->predict(pFeatureData);
				break;
			}
		case CLASSIFIER_BOOST:
			{
				CvBoost* Boost = dynamic_cast<CvBoost*> (ItGlobalClassifierMap->second);
				pPredictionResponse = Boost->predict(pFeatureData, 0, 0, CV_WHOLE_SEQ, false, true);
				double posExp = exp(pPredictionResponse);
				pPredictionResponse = posExp/(posExp + exp(-pPredictionResponse));
				break;
			}
		case CLASSIFIER_KNN:
			{
				CvKNearest* KNN = dynamic_cast<CvKNearest*> (ItGlobalClassifierMap->second);
				int k=1;
				pPredictionResponse = KNN->find_nearest(pFeatureData, k);
				break;
			}
		default:
			{
				std::cout << "ObjectClassifier::PredictGlobal: Error: Classifier type unknown.\n";
				return ipa_utils::RET_FAILED;
			}
	};

//	std::cout << "Sample belongs to class " << pClass << " with degree " << pPredictionResponse << ".\n";

	return ipa_utils::RET_OK;
}


int ObjectClassifier::PredictGlobal(std::string pPath, std::string pClass, CvMat* pFeatureData, double& pPredictionResponse, ClassifierType pClassifierType)
{
	switch (pClassifierType)
	{
		case CLASSIFIER_RTC:
			{
				CvRTrees* RTC = new CvRTrees;
				std::stringstream ss;
				ss << pPath << pClass << "_" << "RTC" << "_glob.txt";
				RTC->load((ss.str()).c_str());

				pPredictionResponse = RTC->predict(pFeatureData);
				break;
			}
		case CLASSIFIER_SVM:
			{
				CvSVM* SVM = new CvSVM;
				std::stringstream ss;
				ss << pPath << pClass << "_" << "SVM" << "_glob.txt";
				SVM->load((ss.str()).c_str());

				pPredictionResponse = SVM->predict(pFeatureData);
				break;
			}
		case CLASSIFIER_BOOST:
			{
				CvBoost* Boost = new CvBoost;
				std::stringstream ss;
				ss << pPath << pClass << "_" << "Boost" << "_glob.txt";
				Boost->load((ss.str()).c_str());

				pPredictionResponse = Boost->predict(pFeatureData, 0, 0, CV_WHOLE_SEQ, false, true);
				double posExp = exp(pPredictionResponse);
				pPredictionResponse = posExp/(posExp + exp(-pPredictionResponse));
				break;
			}
		case CLASSIFIER_KNN:
			{
				CvKNearest* KNN = new CvKNearest;
				std::stringstream ss;
				ss << pPath << pClass << "_" << "KNN" << "_glob.txt";
				KNN->load((ss.str()).c_str());

				int k=1;
				pPredictionResponse = KNN->find_nearest(pFeatureData, k);
				break;
			}
		default:
			{
				std::cout << "ObjectClassifier::PredictGlobal: Error: Classifier type unknown.\n";
				return ipa_utils::RET_FAILED;
			}
	};

	std::cout << "Sample belongs to class " << pClass << " with degree " << pPredictionResponse << ".\n";

	return ipa_utils::RET_OK;
}


int ObjectClassifier::ValidateLocal(ClassifierType pClassifierType, std::string pClass, int pNumberFeatures, LocalFeaturesMap::iterator pItLocalFeaturesClass,
									std::list<int>* pIndicesValidationCorrect, std::list<int>* pIndicesValidationIncorrect, CvMat* pNegativeSamplesMatrix,
									ClassifierOutputCollection& pOutputStorage)
{
	// validate classifier
	CvMat* SampleMat;
	std::list<int>::iterator ItIndices;
	LocalFeaturesMap::value_type::second_type::value_type::second_type::iterator ItBlobListStructs;
	BlobListRiB::iterator ItBlobFPs;
	double ClassificationResult=0;
	SampleMat = cvCreateMat(1, pNumberFeatures, pNegativeSamplesMatrix->type);
	for (ItIndices = pIndicesValidationCorrect->begin(); ItIndices != pIndicesValidationCorrect->end(); ItIndices++)
	{	// positive samples
		for (ItBlobListStructs = pItLocalFeaturesClass->second[*ItIndices].begin(); ItBlobListStructs != pItLocalFeaturesClass->second[*ItIndices].end(); ItBlobListStructs++)
		{
			for (ItBlobFPs = ItBlobListStructs->BlobFPs.begin(); ItBlobFPs != ItBlobListStructs->BlobFPs.end(); ItBlobFPs++)
			{
				for (int j=0; j<pNumberFeatures; j++)
				{
					cvSetReal1D(SampleMat, j, ItBlobFPs->m_D[j]);
				}
				PredictLocal(pClassifierType, pClass, SampleMat, ClassificationResult);

				pOutputStorage.positiveSampleResponses.push_back(ClassificationResult);
			}
		}
	}
	for (ItIndices = pIndicesValidationIncorrect->begin(); ItIndices != pIndicesValidationIncorrect->end(); ItIndices++)
	{	// Incorrect Samples
		cvGetRow(pNegativeSamplesMatrix, SampleMat, *ItIndices);
		PredictLocal(pClassifierType, pClass, SampleMat, ClassificationResult);

		pOutputStorage.negativeSampleResponses.push_back(ClassificationResult);
	}
	cvReleaseMat(&SampleMat);

	return ipa_utils::RET_OK;
}


int ObjectClassifier::ValidateGlobal(ClassifierType pClassifierType, std::string pClass, int pNumberFeatures, GlobalFeaturesMap::iterator pItGlobalFeaturesClass,
									 std::list<int>* pIndicesValidationCorrect, std::list<int>* pIndicesValidationIncorrect, CvMat* pNegativeSamplesMatrix,
									 std::vector<std::string>& pNegativeSamplesLabels, ClassifierOutputCollection& pOutputStorage)
{
	// validate classifier
	CvMat* SampleMat;
	std::list<int>::iterator ItIndices;
	double ClassificationResult=0;
	SampleMat = cvCreateMat(1, pNumberFeatures, pItGlobalFeaturesClass->second[0]->type);
	for (ItIndices = pIndicesValidationCorrect->begin(); ItIndices != pIndicesValidationCorrect->end(); ItIndices++)
	{	// positive samples
		for (int SampleNumber=0; SampleNumber<pItGlobalFeaturesClass->second[*ItIndices]->rows; SampleNumber++)
		{
			cvGetRow(pItGlobalFeaturesClass->second[*ItIndices], SampleMat, SampleNumber);
			PredictGlobal(pClassifierType, pClass, SampleMat, ClassificationResult);

			pOutputStorage.positiveSampleResponses.push_back(ClassificationResult);
		}
	}
	for (ItIndices = pIndicesValidationIncorrect->begin(); ItIndices != pIndicesValidationIncorrect->end(); ItIndices++)
	{	// negative Samples
		cvGetRow(pNegativeSamplesMatrix, SampleMat, *ItIndices);
		PredictGlobal(pClassifierType, pClass, SampleMat, ClassificationResult);

		pOutputStorage.negativeSampleResponses.push_back(ClassificationResult);
		pOutputStorage.negativeSampleCorrectLabel.push_back(pNegativeSamplesLabels[*ItIndices]);
	}
	cvReleaseMat(&SampleMat);

	return ipa_utils::RET_OK;
}


int ObjectClassifier::ValidateGlobalSampleRange(ClassifierType pClassifierType, std::string pClass, int pNumberFeatures, GlobalFeaturesMap::iterator pItGlobalFeaturesClass,
	std::list<int>* pIndicesValidationCorrect, std::list<int>* pIndicesValidationIncorrect, CvMat* pNegativeSamplesMatrix,
	std::vector<std::string>& pNegativeSamplesLabels, ClassifierOutputCollection& pOutputStorage, double rangeStartFactor, double rangeEndFactor)
{
	// validate classifier
	CvMat* SampleMat;
	std::list<int>::iterator ItIndices;
	double ClassificationResult=0;
	SampleMat = cvCreateMat(1, pNumberFeatures, pItGlobalFeaturesClass->second[0]->type);
	for (ItIndices = pIndicesValidationCorrect->begin(); ItIndices != pIndicesValidationCorrect->end(); ItIndices++)
	{	// positive samples
		for (int SampleNumber=pItGlobalFeaturesClass->second[*ItIndices]->rows*rangeStartFactor; SampleNumber<pItGlobalFeaturesClass->second[*ItIndices]->rows*rangeEndFactor; SampleNumber++)
		{
			cvGetRow(pItGlobalFeaturesClass->second[*ItIndices], SampleMat, SampleNumber);
			PredictGlobal(pClassifierType, pClass, SampleMat, ClassificationResult);

			pOutputStorage.positiveSampleResponses.push_back(ClassificationResult);
		}
	}
	for (ItIndices = pIndicesValidationIncorrect->begin(); ItIndices != pIndicesValidationIncorrect->end(); ItIndices++)
	{	// negative Samples
		cvGetRow(pNegativeSamplesMatrix, SampleMat, *ItIndices);
		PredictGlobal(pClassifierType, pClass, SampleMat, ClassificationResult);

		pOutputStorage.negativeSampleResponses.push_back(ClassificationResult);
		pOutputStorage.negativeSampleCorrectLabel.push_back(pNegativeSamplesLabels[*ItIndices]);
	}
	cvReleaseMat(&SampleMat);

	return ipa_utils::RET_OK;
}


//int ObjectClassifier::ClassifierStatisticsGlobal(std::string pPath, ClassifierType pClassifierType, float pClassificationThreshold, double& pRightPositives, double& pRightNegatives, bool pLoadClassifiers)
//{
//	if (pLoadClassifiers) LoadClassifiersGlobal(pPath, pClassifierType);
//
//	GlobalFeaturesMap::iterator ItGlobalFeaturesMap;
//	GlobalFeaturesMap::value_type::second_type::iterator ItObjectMap;
//	GlobalClassifierMap::iterator ItGlobalClassifierMap;
//	CvMat* SampleMat=NULL;
//	unsigned long int TotalNumberSamples = 0;
//
//	// initialize StatisticsMap
//	for (ItGlobalClassifierMap = mData.mGlobalClassifierMap.begin(); ItGlobalClassifierMap != mData.mGlobalClassifierMap.end(); ItGlobalClassifierMap++)
//	{
//		mData.mStatisticsMap[ItGlobalClassifierMap->first].RightNegatives = 0;
//		mData.mStatisticsMap[ItGlobalClassifierMap->first].RightPositives = 0;
//		mData.mStatisticsMap[ItGlobalClassifierMap->first].WrongNegatives = 0;
//		mData.mStatisticsMap[ItGlobalClassifierMap->first].WrongPositives = 0;
//	}
//
//	// iterate over all samples from all classes
//	int Counter=0;
//	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
//	{
//		std::cout << "Processing class " << ItGlobalFeaturesMap->first << " (" << ++Counter << "th out of " << mData.mGlobalFeaturesMap.size() << ").\n";
//		for (ItObjectMap = ItGlobalFeaturesMap->second.begin(); ItObjectMap != ItGlobalFeaturesMap->second.end(); ItObjectMap++)
//		{
//			int NumberSamples = ItObjectMap->second->rows;
//			TotalNumberSamples += (unsigned long int)NumberSamples;
//
//			double ClassificationResult=0;
//			SampleMat = cvCreateMat(1, ItObjectMap->second->cols, ItObjectMap->second->type);
//			for (int SampleNumber=0; SampleNumber<NumberSamples; SampleNumber++)
//			{
//				cvGetRow(ItObjectMap->second, SampleMat, SampleNumber);
//
//				// try each classifier with each sample
//				for (ItGlobalClassifierMap = mData.mGlobalClassifierMap.begin(); ItGlobalClassifierMap != mData.mGlobalClassifierMap.end(); ItGlobalClassifierMap++)
//				{
//					PredictGlobal(pClassifierType, ItGlobalClassifierMap->first, SampleMat, ClassificationResult);
//
//					if (ClassificationResult >= pClassificationThreshold)
//					{
//						if (ItGlobalClassifierMap->first == ItGlobalFeaturesMap->first)
//						{ mData.mStatisticsMap[ItGlobalClassifierMap->first].RightPositives++; }
//						else
//						{ mData.mStatisticsMap[ItGlobalClassifierMap->first].WrongPositives++; }
//					}
//					else
//					{
//						if (ItGlobalClassifierMap->first == ItGlobalFeaturesMap->first)
//						{ mData.mStatisticsMap[ItGlobalClassifierMap->first].WrongNegatives++; }
//						else
//						{ mData.mStatisticsMap[ItGlobalClassifierMap->first].RightNegatives++; }
//					}
//				}
//			}
//			cvReleaseMat(&SampleMat);
//		}
//	}
//
//	// output and file storage
//	std::stringstream FileName;
//	FileName << pPath << "Statistics/Statistics_" << ClassifierLabel(pClassifierType) << "_" << pClassificationThreshold << ".txt";
//	std::ofstream f((FileName.str()).c_str(), std::fstream::out);
//	if(!f.is_open())
//	{
//		std::cout << "ObjectClassifier::ClassifierStatistics: Could not open '" << FileName.str() << "'" << std::endl;
//		return ipa_utils::RET_FAILED;
//	}
//
//	f << "NumberOfClasses\t" << mData.mGlobalClassifierMap.size() << "\n";
//	f << "TotalNumberOfSamples\t" << TotalNumberSamples << "\n";
//
//	std::cout << "\nTotal number samples: " << TotalNumberSamples << "\n";
//	std::cout << "Threshold: " << pClassificationThreshold << "\n";
//
//	// print StatisticsMap
//	std::cout << "\n\nClass\t\tObjects\t\tRightPos\tWrongPos\tWrongNeg\tRightNeg\tRP%\t\tRN%\n";
//	double RP=0;	// right positives
//	double RN=0;	// right negatives
//	for (ItGlobalClassifierMap = mData.mGlobalClassifierMap.begin(); ItGlobalClassifierMap != mData.mGlobalClassifierMap.end(); ItGlobalClassifierMap++)
//	{
//		int NumberClassSamples=0;
//		ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.find(ItGlobalClassifierMap->first);
//		for (ItObjectMap = ItGlobalFeaturesMap->second.begin(); ItObjectMap != ItGlobalFeaturesMap->second.end(); ItObjectMap++)
//		{ NumberClassSamples+= ItObjectMap->second->rows; }
//		double RPP = (double)mData.mStatisticsMap[ItGlobalClassifierMap->first].RightPositives/(double)NumberClassSamples;
//		double RNP = (double)mData.mStatisticsMap[ItGlobalClassifierMap->first].RightNegatives/(double)(TotalNumberSamples-(unsigned long int)NumberClassSamples);
//		std::cout << ItGlobalClassifierMap->first << "\t\t" << ItGlobalFeaturesMap->second.size() << "\t\t" <<
//			mData.mStatisticsMap[ItGlobalClassifierMap->first].RightPositives << "\t\t" <<
//			mData.mStatisticsMap[ItGlobalClassifierMap->first].WrongPositives << "\t\t" <<
//			mData.mStatisticsMap[ItGlobalClassifierMap->first].WrongNegatives << "\t\t" <<
//			mData.mStatisticsMap[ItGlobalClassifierMap->first].RightNegatives << "\t\t" << RPP << "\t" << RNP << "\n";
//		f << ItGlobalClassifierMap->first << "\t" << ItGlobalFeaturesMap->second.size() << "\t" <<
//			mData.mStatisticsMap[ItGlobalClassifierMap->first].RightPositives << "\t" <<
//			mData.mStatisticsMap[ItGlobalClassifierMap->first].WrongPositives << "\t" <<
//			mData.mStatisticsMap[ItGlobalClassifierMap->first].WrongNegatives << "\t" <<
//			mData.mStatisticsMap[ItGlobalClassifierMap->first].RightNegatives << "\t" << RPP << "\t" << RNP << "\n";
//		RP += RPP;
//		RN += RNP;
//	}
//	RP /= mData.mGlobalClassifierMap.size();
//	RN /= mData.mGlobalClassifierMap.size();
//
//	std::cout << "\nOverall\t\t RP = " << RP << "\t\t RN = " << RN << "\n";
//	
//	f << "OverallRP\t" << RP << "\n" << "OverallRN\t" << RN << "\n\n";
//	pRightPositives = RP;
//	pRightNegatives = RN;
//
//	f.close();
//
//	return ipa_utils::RET_OK;
//}


int ObjectClassifier::ROCCurve(std::string pPath, ClassifierType pClassifierType)
{
	std::cout << "\n\nCalculating ROCCurve.\n";

	LoadClassifiersGlobal(pPath, pClassifierType);

	std::stringstream FileName;
	FileName << pPath << "Statistics/ROCCurve_" << ClassifierLabel(pClassifierType) << ".txt";
	std::ofstream f((FileName.str()).c_str(), std::fstream::out);
	if(!f.is_open())
	{
		std::cout << "ObjectClassifier::ConfusionDiagram: Could not open '" << FileName.str() << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	f << "0.0\t1.0\t0.0\n";
	for (float Threshold=(float)0.05; Threshold<1.0; Threshold+=(float)0.05)
	{
		double RP=0, RN=0;
		//ClassifierStatisticsGlobal(pPath, pClassifierType, Threshold, RP, RN, false);
		f << Threshold << "\t" << RP << "\t" << RN << "\n";
	}
	f << "1.0\t0.0\t1.0\n";
	f.close();
	return ipa_utils::RET_OK;
}


int ObjectClassifier::CrossValidationLocal(ClassifierType pClassifierType, std::string pClass, int pNumberCycles, float pFactorIncorrect,
										   ClassifierOutputCollection& pValidationSetOutput, ClassifierOutputCollection& pTestSetOutput,
										   float pPercentTest, float pPercentValidation, const CvMat** pVariableImportance)
{
	// Local features
	LocalFeaturesMap::iterator ItLocalFeaturesMap;
	LocalFeaturesMap::iterator ItLocalFeaturesClass;
	LocalFeaturesMap::value_type::second_type::iterator ItObjectMap;
	ObjectMap::value_type::second_type::iterator ItBlobListStructs;
	LocalClassifierMap::iterator ItLocalClassifierMap;

	if ((ItLocalFeaturesClass = mData.mLocalFeaturesMap.find(pClass)) == mData.mLocalFeaturesMap.end())
	{
		std::cout << "ObjectClassifier::CrossValidationLocal: No local feature data found for class " << pClass << ".\n";
		return ipa_utils::RET_FAILED;
	}

	int NumberObjectsCorrect = ItLocalFeaturesClass->second.size();			// number of objects from pClass
	int NumberObjectsCorrectTest=0;											// number of objects from pClass used for testing only
	int NumberObjectsCorrectTrain=0;										// number of objects from pClass used for training
	int NumberObjectsCorrectValidation=0;									// number of objects from pClass used for validation after training

	// check for a large enough number of objects
	if (((NumberObjectsCorrectTest=cvRound((double)NumberObjectsCorrect*pPercentTest)) < 1) && (pPercentTest > 0.0))
	{
		NumberObjectsCorrectTest = 1;
		//std::cout << "ObjectClassifier::CrossValidationLocal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or increase the percentage of the test set.\n";
		//return ipa_utils::RET_FAILED;
	}
	if ((NumberObjectsCorrectValidation=cvRound((double)NumberObjectsCorrect*pPercentValidation)) < 1)
	{
		NumberObjectsCorrectValidation = 1;
		//std::cout << "ObjectClassifier::CrossValidationLocal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or increase the percentage of the validation set.\n";
		//return ipa_utils::RET_FAILED;
	}
	if ((NumberObjectsCorrectTrain = NumberObjectsCorrect-NumberObjectsCorrectTest-NumberObjectsCorrectValidation) < 1)
	{
		std::cout << "ObjectClassifier::CrossValidationLocal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or let more percentage for the train set.\n";
		return ipa_utils::RET_FAILED;
	}

	// index lists
	std::list<int> IndicesTrainCorrect;			// index with object numbers
	std::list<int> IndicesValidationCorrect;	// index with object numbers
	std::list<int> IndicesTestCorrect;			// index with object numbers
	std::list<int> IndicesTrainIncorrect;		// index with matrix row numbers
	std::list<int> IndicesValidationIncorrect;	// index with matrix row numbers
	std::list<int> IndicesTestIncorrect;		// index with matrix row numbers
	std::list<int>::iterator ItIndices;
	// determine total number of positive training samples and set number of negative samples
	int NumberSamplesCorrect = mData.GetNumberLocalFeaturePoints(pClass);
	int NumberSamplesIncorrect = cvRound((double)NumberSamplesCorrect*pFactorIncorrect);
	int NumberFeatures = mData.GetNumberLocalFeatures();
	
	// fill index list for training with all positive class objects and remove test and validation set later
	for (int i=0; i<NumberObjectsCorrect; i++) IndicesTrainCorrect.push_back(i);

	// prepare index list for positive test set
	int NumberSamplesCorrectTest = 0;
	for (int i=0; i<NumberObjectsCorrectTest; i++)
	{
		int Index = int(IndicesTrainCorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
		ItIndices = IndicesTrainCorrect.begin();
		for (int k=0; k<Index; k++, ItIndices++);
		for (ItBlobListStructs = ItLocalFeaturesClass->second[*ItIndices].begin(); ItBlobListStructs != ItLocalFeaturesClass->second[*ItIndices].end(); ItBlobListStructs++)
			NumberSamplesCorrectTest += ItBlobListStructs->BlobFPs.size();
		IndicesTestCorrect.push_back(*ItIndices);
		IndicesTrainCorrect.remove(*ItIndices);
	}

	// create matrix with incorrect samples (drawn by chance from all other classes)
	CvMat* NegativeSamplesMatrix = cvCreateMat(NumberSamplesIncorrect, NumberFeatures, CV_32FC1);
	mData.GetNegativeSamplesMatrixLocal(pClass, NegativeSamplesMatrix, 0, NumberSamplesIncorrect);

	// fill index list for training with all non-class objects and remove test and validation set later
	for (int i=0; i<NumberSamplesIncorrect; i++) IndicesTrainIncorrect.push_back(i);

	int NumberSamplesIncorrectTest = cvRound((double)NumberSamplesIncorrect*pPercentTest);
	int NumberSamplesIncorrectValidation = 0;
	int NumberSamplesIncorrectTrain = 0;

	// prepare index list for non-class test set
	for (int i=0; i<NumberSamplesIncorrectTest; i++)
	{
		int Index = int(IndicesTrainIncorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
		ItIndices = IndicesTrainIncorrect.begin();
		for (int k=0; k<Index; k++, ItIndices++);
		IndicesTestIncorrect.push_back(*ItIndices);
		IndicesTrainIncorrect.remove(*ItIndices);
	}
	
	// cross-validation cycles
	for (int Counter=0; Counter<pNumberCycles; Counter++)
	{
		std::cout << "Cycle " << Counter << "\n";
		// prepare train and validation index list for objects from the class
		for (int i=0; i<NumberObjectsCorrectValidation; i++)
		{
			int Index = int(IndicesTrainCorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
			ItIndices = IndicesTrainCorrect.begin();
			for (int k=0; k<Index; k++, ItIndices++);
			IndicesValidationCorrect.push_back(*ItIndices);
			IndicesTrainCorrect.remove(*ItIndices);
		}

		// count correct training samples and determine needed incorrect samples
		int NumberSamplesCorrectTrain = 0;
		for (ItIndices = IndicesTrainCorrect.begin(); ItIndices != IndicesTrainCorrect.end(); ItIndices++)
		{
			for (ItBlobListStructs = ItLocalFeaturesClass->second[*ItIndices].begin(); ItBlobListStructs != ItLocalFeaturesClass->second[*ItIndices].end(); ItBlobListStructs++)
				NumberSamplesCorrectTrain += ItBlobListStructs->BlobFPs.size();
		}
		NumberSamplesIncorrectTrain = cvRound((double)NumberSamplesCorrectTrain*pFactorIncorrect);
		NumberSamplesIncorrectValidation = NumberSamplesIncorrect - NumberSamplesIncorrectTest - NumberSamplesIncorrectTrain;

		// prepare train and validation index list for non-class objects
		for (int i=0; i<NumberSamplesIncorrectValidation; i++)
		{
			int Index = int(IndicesTrainIncorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
			ItIndices = IndicesTrainIncorrect.begin();
			for (int k=0; k<Index; k++, ItIndices++);
			IndicesValidationIncorrect.push_back(*ItIndices);
			IndicesTrainIncorrect.remove(*ItIndices);
		}

		// train classifiers
		TrainLocal(pClassifierType, pClass, NumberSamplesCorrectTrain+NumberSamplesIncorrectTrain, NumberFeatures, ItLocalFeaturesClass, &IndicesTrainCorrect, &IndicesTrainIncorrect, NegativeSamplesMatrix);

		// validate classifier
		ValidateLocal(pClassifierType, pClass, NumberFeatures, ItLocalFeaturesClass, &IndicesValidationCorrect, &IndicesValidationIncorrect, NegativeSamplesMatrix, pValidationSetOutput);

		// write back the validation indices into the train index set for objects from class
		for (ItIndices = IndicesValidationCorrect.begin(); ItIndices != IndicesValidationCorrect.end(); ItIndices = IndicesValidationCorrect.begin())
		{
			IndicesTrainCorrect.push_back(*ItIndices);
			IndicesValidationCorrect.remove(*ItIndices);
		}
		IndicesTrainCorrect.sort();

		// write back the validation indices into the train index set for non-class objects
		for (ItIndices = IndicesValidationIncorrect.begin(); ItIndices != IndicesValidationIncorrect.end(); ItIndices = IndicesValidationIncorrect.begin())
		{
			IndicesTrainIncorrect.push_back(*ItIndices);
			IndicesValidationIncorrect.remove(*ItIndices);
		}
		IndicesTrainIncorrect.sort();
	}
	
	// Test classifier with the unseen test set.
	if (NumberObjectsCorrectTest > 0)
	{
		// train classifier with whole train data
		TrainLocal(pClassifierType, pClass, NumberSamplesCorrect-NumberSamplesCorrectTest+NumberSamplesIncorrect-NumberSamplesIncorrectTest, NumberFeatures, ItLocalFeaturesClass, &IndicesTrainCorrect, &IndicesTrainIncorrect, NegativeSamplesMatrix);

		// test classifier
		ValidateLocal(pClassifierType, pClass, NumberFeatures, ItLocalFeaturesClass, &IndicesTestCorrect, &IndicesTestIncorrect, NegativeSamplesMatrix, pTestSetOutput);
	}

	// variable importance
	if (pVariableImportance != NULL) GetVariableImportance(mData.mLocalClassifierMap.find(pClass)->second, pClassifierType, pVariableImportance);

	cvReleaseMat(&NegativeSamplesMatrix);

	return ipa_utils::RET_OK;
}


int ObjectClassifier::CrossValidationLocal(std::string pStatisticsPath, std::string pFileDescription, ClassifierType pClassifierType, int pNumberCycles, float pFactorIncorrect, float pPercentTest, float pPercentValidation)
{
	LocalFeaturesMap::iterator ItLocalFeaturesMap;
	ClassifierOutputCollection ValidationSetOutput;
	ClassifierOutputCollection TestSetOutput;
	ClassifierPerformanceStruct TestSetPerformance;
	ClassifierPerformanceStruct ValidationSetPerformance;

	std::cout << "\n\n Local cross-validation:\n";
	/// Iterate over all classes
	for (ItLocalFeaturesMap = mData.mLocalFeaturesMap.begin(); ItLocalFeaturesMap != mData.mLocalFeaturesMap.end(); ItLocalFeaturesMap++)
	{
		std::string Class = ItLocalFeaturesMap->first;
		std::cout << "\n\n\n-------------------------------------\n" << Class << ":\n-------------------------------------\n";
		
		/// Statistics file for every object
		std::stringstream FileName;
		FileName << pStatisticsPath << "ROCCurve_" << pFileDescription << "_" << ClassifierLabel(pClassifierType) << "_" << Class << "_loc.txt";
		std::ofstream f((FileName.str()).c_str(), std::fstream::out);
		if(!f.is_open())
		{
			std::cout << "ObjectClassifier::CrossValidationLocal: Could not open '" << FileName.str() << "'" << std::endl;
			return ipa_utils::RET_FAILED;
		}

		// clear response buffer
		ValidationSetOutput.clear();
		TestSetOutput.clear();

		// do a cross-validation and determine the classifier responses
		const CvMat** SingleRunVariableImportance = new const CvMat*;
		*SingleRunVariableImportance = NULL;
		CrossValidationLocal(pClassifierType, Class, pNumberCycles, pFactorIncorrect, ValidationSetOutput, TestSetOutput, pPercentTest, pPercentValidation, SingleRunVariableImportance);

		// variable importance
		std::vector<CvMat*> VariableImportance;
		std::vector<CvMat*>::iterator ItVariableImportance;
		VariableImportance.push_back(cvCloneMat(*SingleRunVariableImportance));

		/// Calculate ROC curve, variable importance
		double NearestDistance = 2.0;
		float BestThreshold = 0;
		bool printHeader = true;
		for (float Threshold=(float)0.05; Threshold<1.0; Threshold+=(float)0.05)
		{
			//std::cout << "\n\nThreshold: " << Threshold << "\n";
			f << Threshold << "\t";

			// initialize StatisticsMap
			ValidationSetPerformance.clear();
			TestSetPerformance.clear();

			// generate statistics
			StatisticsFromOutput(ValidationSetOutput, ValidationSetPerformance, Threshold);
			StatisticsFromOutput(TestSetOutput, TestSetPerformance, Threshold);

			// output (and save) results
			double Distance = 2.0;
			CrossValidationStatisticsOutput(Class, ValidationSetPerformance, TestSetPerformance, f, pFactorIncorrect, Distance, Threshold, printHeader);
			printHeader = false;

			// find best threshold
			if (Distance < NearestDistance)
			{
				BestThreshold = Threshold;
				NearestDistance = Distance;
			}
		}
		f.close();
		
		// save best threshold
		mData.mLocalClassifierThresholdMap[Class] = BestThreshold;

		// save variable importance
		std::stringstream VarImpFileName;
		VarImpFileName << pStatisticsPath << "VarImportance_Loc_" << pFileDescription << "_" << ClassifierLabel(pClassifierType) << "_" << Class << ".txt";
		f.open((VarImpFileName.str()).c_str(), std::fstream::out);
		if(!f.is_open())
		{
			std::cout << "ObjectClassifier::CrossValidationLocal: Could not open '" << VarImpFileName.str() << "'" << std::endl;
			return ipa_utils::RET_FAILED;
		}
		for (ItVariableImportance = VariableImportance.begin(); ItVariableImportance != VariableImportance.end(); ItVariableImportance++)
		{
			for (int j=0; j<(*ItVariableImportance)->width; j++) f << cvGetReal1D(*ItVariableImportance, j) << "\t";
			f << "\n";
			cvReleaseMat(&(*ItVariableImportance));
		}
		f.close();
	}

	return ipa_utils::RET_OK;
}


////////////////////////////////////////////////////////////////////////////////// ----

int ObjectClassifier::CrossValidationGlobalSampleRange(ClassifierType pClassifierType, std::string pClass, int pNumberCycles, float pFactorIncorrect,
	ClassifierOutputCollection& pValidationSetOutput, ClassifierOutputCollection& pTestSetOutput,
	float pPercentTest, float pPercentValidation, const CvMat** pVariableImportance, int pViewsPerObject, double pFactorSamplesTrainData)
{
	bool drawValidationSetRandomized = false;
	double factorSamplesTrainData = pFactorSamplesTrainData;		// for each object, this ratio of samples should go to the training set, the rest is for testing

	// Global features
	GlobalFeaturesMap::iterator ItGlobalFeaturesMap;
	GlobalFeaturesMap::iterator ItGlobalFeaturesClass;
	GlobalFeaturesMap::value_type::second_type::iterator ItObjectMap;
	GlobalClassifierMap::iterator ItGlobalClassifierMap;

	if ((ItGlobalFeaturesClass = mData.mGlobalFeaturesMap.find(pClass)) == mData.mGlobalFeaturesMap.end())
	{
		std::cout << "ObjectClassifier::CrossValidation: No global feature data found for class " << pClass << ".\n";
		return ipa_utils::RET_FAILED;
	}

	int NumberObjectsCorrect = ItGlobalFeaturesClass->second.size();
	int NumberObjectsCorrectTest=0;
	int NumberObjectsCorrectTrain=0;
	int NumberObjectsCorrectValidation=0;

	// check for a large enough number of objects
	if (((NumberObjectsCorrectTest=cvRound((double)NumberObjectsCorrect*pPercentTest)) < 1) && (pPercentTest > 0.0))
	{
		NumberObjectsCorrectTest = 1;
		//std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or increase the percentage of the test set.\n";
		//return ipa_utils::RET_FAILED;
	}
	if ((NumberObjectsCorrectValidation=cvRound((double)NumberObjectsCorrect*pPercentValidation)) < 1)
	{
		NumberObjectsCorrectValidation = 1;
		//std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or increase the percentage of the validation set.\n";
		//return ipa_utils::RET_FAILED;
	}
	if ((NumberObjectsCorrectTrain = NumberObjectsCorrect-NumberObjectsCorrectTest-NumberObjectsCorrectValidation) < 1)
	{
		std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or let more percentage for the train set.\n";
		return ipa_utils::RET_FAILED;
	}

	// index lists
	std::list<int> IndicesTrainCorrect;
	std::list<int> IndicesValidationCorrect;
	std::list<int> IndicesTestCorrect;
	std::list<int> IndicesTrainIncorrect;
	std::list<int> IndicesValidationIncorrect;
	std::list<int> IndicesTestIncorrect;
	std::list<int>::iterator ItIndices;
	int NumberSamplesCorrect = 0;
	int NumberSamplesIncorrect = 0;
	int NumberFeatures = ItGlobalFeaturesClass->second[0]->cols;

	// fill index list for training with all positive class objects and remove test and validation set later
	for (int i=0; i<NumberObjectsCorrect; i++) IndicesTrainCorrect.push_back(i);

	// determine total number of positive training samples and set number of negative samples
	for (ItIndices = IndicesTrainCorrect.begin(); ItIndices != IndicesTrainCorrect.end(); ItIndices++)
	{
//		if (pViewsPerObject == -1)
			NumberSamplesCorrect += ItGlobalFeaturesClass->second[*ItIndices]->rows * factorSamplesTrainData;
		//else
		//	NumberSamplesCorrect += min(pViewsPerObject, ItGlobalFeaturesClass->second[*ItIndices]->rows);
	}
	NumberSamplesIncorrect = cvRound((double)NumberSamplesCorrect*pFactorIncorrect);

	// prepare index list for positive test set
	int NumberSamplesCorrectTest = 0;
	for (int i=0; i<NumberObjectsCorrectTest; i++)
	{
		int Index = int(IndicesTrainCorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
		ItIndices = IndicesTrainCorrect.begin();
		for (int k=0; k<Index; k++, ItIndices++);
		NumberSamplesCorrectTest += ItGlobalFeaturesClass->second[*ItIndices]->rows * (1-factorSamplesTrainData);
		IndicesTestCorrect.push_back(*ItIndices);
		IndicesTrainCorrect.remove(*ItIndices);
	}

	// create matrix with incorrect samples (drawn by chance from all other classes)
	CvMat* NegativeSamplesMatrixTrain = cvCreateMat(NumberSamplesIncorrect, NumberFeatures, ItGlobalFeaturesClass->second[0]->type);
	std::vector<std::string> NegativeSamplesLabelsTrain(NumberSamplesIncorrect);
	mData.GetNegativeSamplesMatrixGlobalSampleRange(pClass, NegativeSamplesMatrixTrain, NegativeSamplesLabelsTrain, 0, NumberSamplesIncorrect, 0., factorSamplesTrainData);

	CvMat* NegativeSamplesMatrixTestValidation = cvCreateMat(NumberSamplesIncorrect, NumberFeatures, ItGlobalFeaturesClass->second[0]->type);
	std::vector<std::string> NegativeSamplesLabelsTestValidation(NumberSamplesIncorrect);
	mData.GetNegativeSamplesMatrixGlobalSampleRange(pClass, NegativeSamplesMatrixTestValidation, NegativeSamplesLabelsTestValidation, 0, NumberSamplesIncorrect, factorSamplesTrainData, 1.);

	// fill index list for training with all non-class objects and remove test and validation set later
	for (int i=0; i<NumberSamplesIncorrect; i++) IndicesTrainIncorrect.push_back(i);

	int NumberSamplesIncorrectTest = cvRound((double)NumberSamplesIncorrect*pPercentTest);
	int NumberSamplesIncorrectValidation = 0;
	int NumberSamplesIncorrectTrain = 0;


	// prepare index list for non-class test set
	for (int i=0; i<NumberSamplesIncorrectTest; i++)
	{
		int Index = int(IndicesTrainIncorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
		ItIndices = IndicesTrainIncorrect.begin();
		for (int k=0; k<Index; k++, ItIndices++);
		IndicesTestIncorrect.push_back(*ItIndices);
		IndicesTrainIncorrect.remove(*ItIndices);
	}

	// cross-validation cycles
	std::cout << "Cycle ";
	for (int Counter=0; Counter<pNumberCycles; Counter++)
	{
		std::cout << Counter << " " << std::flush;
		// prepare train and validation index list for objects from the class
		for (int i=0; i<NumberObjectsCorrectValidation; i++)
		{
			int Index = 0;
			if (drawValidationSetRandomized == true || Counter>=(int)IndicesTrainCorrect.size())
				Index = int(IndicesTrainCorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
			else
				Index = (Counter+i)%IndicesTrainCorrect.size();
			ItIndices = IndicesTrainCorrect.begin();
			for (int k=0; k<Index; k++, ItIndices++);
			IndicesValidationCorrect.push_back(*ItIndices);
			IndicesTrainCorrect.remove(*ItIndices);
		}

		// count correct training samples and determine needed incorrect samples
		int NumberSamplesCorrectTrain = 0;
		for (ItIndices = IndicesTrainCorrect.begin(); ItIndices != IndicesTrainCorrect.end(); ItIndices++)
		{
//			if (pViewsPerObject == -1)
				NumberSamplesCorrectTrain += ItGlobalFeaturesClass->second[*ItIndices]->rows * factorSamplesTrainData;
			//else
			//	NumberSamplesCorrectTrain += min(pViewsPerObject, ItGlobalFeaturesClass->second[*ItIndices]->rows);
		}
		NumberSamplesIncorrectTrain = cvRound((double)NumberSamplesCorrectTrain*pFactorIncorrect);
		NumberSamplesIncorrectValidation = NumberSamplesIncorrect - NumberSamplesIncorrectTest - NumberSamplesIncorrectTrain;

		// prepare train and validation index list for non-class objects
		for (int i=0; i<NumberSamplesIncorrectValidation; i++)
		{
			int Index = int(IndicesTrainIncorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
			ItIndices = IndicesTrainIncorrect.begin();
			for (int k=0; k<Index; k++, ItIndices++);
			IndicesValidationIncorrect.push_back(*ItIndices);
			IndicesTrainIncorrect.remove(*ItIndices);
		}

		// train classifiers
		TrainGlobalSampleRange(pClassifierType, pClass, NumberSamplesCorrectTrain+NumberSamplesIncorrectTrain, NumberFeatures, ItGlobalFeaturesClass, &IndicesTrainCorrect, &IndicesTrainIncorrect, NegativeSamplesMatrixTrain, 0., factorSamplesTrainData);

		// validate classifier
		ValidateGlobalSampleRange(pClassifierType, pClass, NumberFeatures, ItGlobalFeaturesClass, &IndicesValidationCorrect, &IndicesValidationIncorrect, NegativeSamplesMatrixTestValidation, NegativeSamplesLabelsTestValidation, pValidationSetOutput, factorSamplesTrainData, 1.);

		// write back the validation indices into the train index set for objects from class
		for (ItIndices = IndicesValidationCorrect.begin(); ItIndices != IndicesValidationCorrect.end(); ItIndices = IndicesValidationCorrect.begin())
		{
			IndicesTrainCorrect.push_back(*ItIndices);
			IndicesValidationCorrect.remove(*ItIndices);
		}
		IndicesTrainCorrect.sort();

		// write back the validation indices into the train index set for non-class objects
		for (ItIndices = IndicesValidationIncorrect.begin(); ItIndices != IndicesValidationIncorrect.end(); ItIndices = IndicesValidationIncorrect.begin())
		{
			IndicesTrainIncorrect.push_back(*ItIndices);
			IndicesValidationIncorrect.remove(*ItIndices);
		}
		IndicesTrainIncorrect.sort();
	}
	std::cout << std::endl;

	/// Test classifier with the unseen test set.
	if (NumberObjectsCorrectTest > 0)
	{
		// train classifier with whole train data
		TrainGlobalSampleRange(pClassifierType, pClass, NumberSamplesCorrect-NumberSamplesCorrectTest+NumberSamplesIncorrect-NumberSamplesIncorrectTest, NumberFeatures, ItGlobalFeaturesClass, &IndicesTrainCorrect, &IndicesTrainIncorrect, NegativeSamplesMatrixTrain, 0., factorSamplesTrainData);

		// test classifier
		ValidateGlobalSampleRange(pClassifierType, pClass, NumberFeatures, ItGlobalFeaturesClass, &IndicesTestCorrect, &IndicesTestIncorrect, NegativeSamplesMatrixTrain, NegativeSamplesLabelsTrain, pTestSetOutput, factorSamplesTrainData, 1.);
	}
	// variable importance
	if (*pVariableImportance != NULL) GetVariableImportance(mData.mGlobalClassifierMap.find(pClass)->second, pClassifierType, pVariableImportance);

	cvReleaseMat(&NegativeSamplesMatrixTrain);
	cvReleaseMat(&NegativeSamplesMatrixTestValidation);
	return ipa_utils::RET_OK;
}


int ObjectClassifier::CrossValidationGlobalSampleRange(std::string pStatisticsPath, std::string pFileDescription, ClassifierType pClassifierType, int pNumberCycles, float pFactorIncorrect,
	float pPercentTest, float pPercentValidation, int pViewsPerObject, std::ofstream* pScreenLogFile, double pFactorSamplesTrainData)
{
	GlobalFeaturesMap::iterator ItGlobalFeaturesMap, ItGlobalFeaturesMap2;
	ClassifierOutputCollection ValidationSetOutput;
	ClassifierOutputCollection TestSetOutput;
	ClassifierPerformanceStruct TestSetPerformance;
	ClassifierPerformanceStruct ValidationSetPerformance;
	double averageDistance = 0.0;
	int objectNumber = 0;
	std::map<std::string, double> bestDistancePerClass;	// stores best distance per class

	std::cout << "\n\n Global cross-validation:\n";
	if (pScreenLogFile) *pScreenLogFile << "\n\n Global cross-validation:\n";
	/// Iterate over all classes
	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
	{
		std::string Class = ItGlobalFeaturesMap->first;
		std::cout << "\n\n\n-------------------------------------\n" << Class << ":\n-------------------------------------\n";
		if (pScreenLogFile) *pScreenLogFile << "\n\n\n-------------------------------------\n" << Class << ":\n-------------------------------------\n";

		// classifier accuracy statistics
		for (ItGlobalFeaturesMap2 = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap2 != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap2++)
			mData.mGlobalClassifierAccuracy[Class][ItGlobalFeaturesMap2->first] = 0.0001;

		// Statistics file for every object
		std::stringstream FileName;
		FileName << pStatisticsPath << "ROCCurve_" << pFileDescription << "_" << ClassifierLabel(pClassifierType) << "_" << Class << ".txt";
		std::ofstream f((FileName.str()).c_str(), std::fstream::out);
		if(!f.is_open())
		{
			std::cout << "ObjectClassifier::CrossValidationGlobal: Could not open '" << FileName.str() << "'" << std::endl;
			if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::CrossValidationGlobal: Could not open '" << FileName.str() << "'" << std::endl;
			return ipa_utils::RET_FAILED;
		}

		// clear response buffer
		ValidationSetOutput.clear();
		TestSetOutput.clear();

		// do a cross-validation and determine the classifier responses
		const CvMat** SingleRunVariableImportance = new const CvMat*;
		*SingleRunVariableImportance = NULL;
		CrossValidationGlobalSampleRange(pClassifierType, Class, pNumberCycles, pFactorIncorrect, ValidationSetOutput, TestSetOutput, pPercentTest, pPercentValidation, SingleRunVariableImportance, pViewsPerObject, pFactorSamplesTrainData);

		// variable importance
		std::vector<CvMat*> VariableImportance;
		std::vector<CvMat*>::iterator ItVariableImportance;
		if (*SingleRunVariableImportance != 0) VariableImportance.push_back(cvCloneMat(*SingleRunVariableImportance));

		/// Calculate ROC curve, variable importance
		double NearestDistance = 2.0;
		float BestThreshold = 0;
		bool printHeader = true;
		for (float Threshold=(float)0.00; Threshold<=1.0; Threshold+=(float)0.005)
		{
			//std::cout << "\n\nThreshold: " << Threshold << "\n";
			f << Threshold << "\t";

			// initialize StatisticsMap
			ValidationSetPerformance.clear();
			TestSetPerformance.clear();

			// generate statistics
			StatisticsFromOutput(ValidationSetOutput, ValidationSetPerformance, Threshold);
			StatisticsFromOutput(TestSetOutput, TestSetPerformance, Threshold);

			// output (and save) results
			double Distance = 2.0;
			CrossValidationStatisticsOutput(Class, ValidationSetPerformance, TestSetPerformance, f, pFactorIncorrect, Distance, Threshold, printHeader, pScreenLogFile);
			printHeader = false;

			// find best threshold
			if (Distance < NearestDistance)
			{
				BestThreshold = Threshold;
				NearestDistance = Distance;
			}
		}
		f.close();

		// average/best Distance
		objectNumber += mData.mGlobalFeaturesMap[Class].size();
		averageDistance += NearestDistance*(double)mData.mGlobalFeaturesMap[Class].size();
		bestDistancePerClass[Class] = NearestDistance;

		// save best threshold
		mData.mGlobalClassifierThresholdMap[Class] = BestThreshold;

		// prepare accuracy statistics corresponding to best threshold
		// positive samples
		for (unsigned int i=0; i<ValidationSetOutput.positiveSampleResponses.size(); i++)
			if (ValidationSetOutput.positiveSampleResponses[i] >= BestThreshold) mData.mGlobalClassifierAccuracy[Class][Class]+=1.0;
		mData.mGlobalClassifierAccuracy[Class][Class] /= (double)ValidationSetOutput.positiveSampleResponses.size();
		// negative samples
		for (unsigned int i=0; i<ValidationSetOutput.negativeSampleResponses.size(); i++)
			if (ValidationSetOutput.negativeSampleResponses[i] >= BestThreshold) mData.mGlobalClassifierAccuracy[Class][ValidationSetOutput.negativeSampleCorrectLabel[i]]+=1.0/(double)ValidationSetOutput.negativeSampleResponses.size();


		// save variable importance
		if (VariableImportance.size() > 0)
		{
			std::stringstream VarImpFileName;
			VarImpFileName << pStatisticsPath << "VarImportance_Glob_" << pFileDescription << "_" << ClassifierLabel(pClassifierType) << "_" << Class << ".txt";
			f.open((VarImpFileName.str()).c_str(), std::fstream::out);
			if(!f.is_open())
			{
				std::cout << "ObjectClassifier::CrossValidationGlobal: Could not open '" << VarImpFileName.str() << "'" << std::endl;
				if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::CrossValidationGlobal: Could not open '" << VarImpFileName.str() << "'" << std::endl;
				return ipa_utils::RET_FAILED;
			}
			for (ItVariableImportance = VariableImportance.begin(); ItVariableImportance != VariableImportance.end(); ItVariableImportance++)
			{
				for (int j=0; j<(*ItVariableImportance)->width; j++) f << cvGetReal1D(*ItVariableImportance, j) << "\t";
				f << "\n";
				cvReleaseMat(&(*ItVariableImportance));
			}
			f.close();
		}
	}

	// normalize and save classifier accuracy statistics corresponding to best threshold
	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
	{
		std::string groundTruthLabel = ItGlobalFeaturesMap->first;
		double sum = 0.0;
		for (ItGlobalFeaturesMap2 = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap2 != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap2++)
		{
			std::string outputLabel = ItGlobalFeaturesMap2->first;
			if (outputLabel == groundTruthLabel) continue;
			sum += mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel];
		}
		for (ItGlobalFeaturesMap2 = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap2 != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap2++)
		{
			std::string outputLabel = ItGlobalFeaturesMap2->first;
			if (outputLabel == groundTruthLabel) continue;
			mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel] = (1-mData.mGlobalClassifierAccuracy[groundTruthLabel][groundTruthLabel]) * mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel]/sum;
		}
	}

	// distance statistics
	for (std::map<std::string, double>::iterator itBestDistancePerClass = bestDistancePerClass.begin(); itBestDistancePerClass!=bestDistancePerClass.end(); itBestDistancePerClass++)
	{
		std::cout << itBestDistancePerClass->first << "\t" << itBestDistancePerClass->second << "\n";
		if (pScreenLogFile) *pScreenLogFile <<  itBestDistancePerClass->first << "\t" << itBestDistancePerClass->second << "\n";
	}
	averageDistance /= (double)objectNumber;
	std::cout << "\n\nAverage Distance: " << averageDistance << "\n";
	if (pScreenLogFile) *pScreenLogFile << "\n\nAverage Distance: " << averageDistance << "\n";

	return ipa_utils::RET_OK;
}

////////////////////////////////////////////////////////////////////////////////// ----



int ObjectClassifier::CrossValidationGlobal(ClassifierType pClassifierType, std::string pClass, int pNumberCycles, float pFactorIncorrect,
											ClassifierOutputCollection& pValidationSetOutput, ClassifierOutputCollection& pTestSetOutput,
											float pPercentTest, float pPercentValidation, const CvMat** pVariableImportance, int pViewsPerObject)
{
	bool drawValidationSetRandomized = false;

	// Global features
	GlobalFeaturesMap::iterator ItGlobalFeaturesMap;
	GlobalFeaturesMap::iterator ItGlobalFeaturesClass;
	GlobalFeaturesMap::value_type::second_type::iterator ItObjectMap;
	GlobalClassifierMap::iterator ItGlobalClassifierMap;

	if ((ItGlobalFeaturesClass = mData.mGlobalFeaturesMap.find(pClass)) == mData.mGlobalFeaturesMap.end())
	{
		std::cout << "ObjectClassifier::CrossValidation: No global feature data found for class " << pClass << ".\n";
		return ipa_utils::RET_FAILED;
	}

	int NumberObjectsCorrect = ItGlobalFeaturesClass->second.size();
	int NumberObjectsCorrectTest=0;
	int NumberObjectsCorrectTrain=0;
	int NumberObjectsCorrectValidation=0;

	// check for a large enough number of objects
	if (((NumberObjectsCorrectTest=cvRound((double)NumberObjectsCorrect*pPercentTest)) < 1) && (pPercentTest > 0.0))
	{
		NumberObjectsCorrectTest = 1;
		//std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or increase the percentage of the test set.\n";
		//return ipa_utils::RET_FAILED;
	}
	if ((NumberObjectsCorrectValidation=cvRound((double)NumberObjectsCorrect*pPercentValidation)) < 1)
	{
		NumberObjectsCorrectValidation = 1;
		//std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or increase the percentage of the validation set.\n";
		//return ipa_utils::RET_FAILED;
	}
	if ((NumberObjectsCorrectTrain = NumberObjectsCorrect-NumberObjectsCorrectTest-NumberObjectsCorrectValidation) < 1)
	{
		std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or let more percentage for the train set.\n";
		return ipa_utils::RET_FAILED;
	}

	// index lists
	std::list<int> IndicesTrainCorrect;
	std::list<int> IndicesValidationCorrect;
	std::list<int> IndicesTestCorrect;
	std::list<int> IndicesTrainIncorrect;
	std::list<int> IndicesValidationIncorrect;
	std::list<int> IndicesTestIncorrect;
	std::list<int>::iterator ItIndices;
	int NumberSamplesCorrect = 0;
	int NumberSamplesIncorrect = 0;
	int NumberFeatures = ItGlobalFeaturesClass->second[0]->cols;
	
	// fill index list for training with all positive class objects and remove test and validation set later
	for (int i=0; i<NumberObjectsCorrect; i++) IndicesTrainCorrect.push_back(i);

	// determine total number of positive training samples and set number of negative samples
	for (ItIndices = IndicesTrainCorrect.begin(); ItIndices != IndicesTrainCorrect.end(); ItIndices++)
	{
		if (pViewsPerObject == -1)
			NumberSamplesCorrect += ItGlobalFeaturesClass->second[*ItIndices]->rows;
		else
			NumberSamplesCorrect += min(pViewsPerObject, ItGlobalFeaturesClass->second[*ItIndices]->rows);
	}
	NumberSamplesIncorrect = cvRound((double)NumberSamplesCorrect*pFactorIncorrect);

	// prepare index list for positive test set
	int NumberSamplesCorrectTest = 0;
	for (int i=0; i<NumberObjectsCorrectTest; i++)
	{
		int Index = int(IndicesTrainCorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
		ItIndices = IndicesTrainCorrect.begin();
		for (int k=0; k<Index; k++, ItIndices++);
		NumberSamplesCorrectTest += ItGlobalFeaturesClass->second[*ItIndices]->rows;
		IndicesTestCorrect.push_back(*ItIndices);
		IndicesTrainCorrect.remove(*ItIndices);
	}

	// create matrix with incorrect samples (drawn by chance from all other classes)
	CvMat* NegativeSamplesMatrix = cvCreateMat(NumberSamplesIncorrect, NumberFeatures, ItGlobalFeaturesClass->second[0]->type);
	std::vector<std::string> NegativeSamplesLabels(NumberSamplesIncorrect);
	mData.GetNegativeSamplesMatrixGlobal(pClass, NegativeSamplesMatrix, NegativeSamplesLabels, 0, NumberSamplesIncorrect, pViewsPerObject);

	// fill index list for training with all non-class objects and remove test and validation set later
	for (int i=0; i<NumberSamplesIncorrect; i++) IndicesTrainIncorrect.push_back(i);

	int NumberSamplesIncorrectTest = cvRound((double)NumberSamplesIncorrect*pPercentTest);
	int NumberSamplesIncorrectValidation = 0;
	int NumberSamplesIncorrectTrain = 0;


	// prepare index list for non-class test set
	for (int i=0; i<NumberSamplesIncorrectTest; i++)
	{
		int Index = int(IndicesTrainIncorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
		ItIndices = IndicesTrainIncorrect.begin();
		for (int k=0; k<Index; k++, ItIndices++);
		IndicesTestIncorrect.push_back(*ItIndices);
		IndicesTrainIncorrect.remove(*ItIndices);
	}
	
	// cross-validation cycles
	std::cout << "Cycle ";
	for (int Counter=0; Counter<pNumberCycles; Counter++)
	{
		std::cout << Counter << " " << std::flush;
		// prepare train and validation index list for objects from the class
		for (int i=0; i<NumberObjectsCorrectValidation; i++)
		{
			int Index = 0;
			if (drawValidationSetRandomized == true || Counter>=(int)IndicesTrainCorrect.size())
				Index = int(IndicesTrainCorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
			else
				Index = (Counter+i)%IndicesTrainCorrect.size();
			ItIndices = IndicesTrainCorrect.begin();
			for (int k=0; k<Index; k++, ItIndices++);
			IndicesValidationCorrect.push_back(*ItIndices);
			IndicesTrainCorrect.remove(*ItIndices);
		}

		// count correct training samples and determine needed incorrect samples
		int NumberSamplesCorrectTrain = 0;
		for (ItIndices = IndicesTrainCorrect.begin(); ItIndices != IndicesTrainCorrect.end(); ItIndices++)
		{
			if (pViewsPerObject == -1)
				NumberSamplesCorrectTrain += ItGlobalFeaturesClass->second[*ItIndices]->rows;
			else
				NumberSamplesCorrectTrain += min(pViewsPerObject, ItGlobalFeaturesClass->second[*ItIndices]->rows);
		}
		NumberSamplesIncorrectTrain = cvRound((double)NumberSamplesCorrectTrain*pFactorIncorrect);
		NumberSamplesIncorrectValidation = NumberSamplesIncorrect - NumberSamplesIncorrectTest - NumberSamplesIncorrectTrain;

		// prepare train and validation index list for non-class objects
		for (int i=0; i<NumberSamplesIncorrectValidation; i++)
		{
			int Index = int(IndicesTrainIncorrect.size()*((double)rand()/((double)RAND_MAX+1.0)));
			ItIndices = IndicesTrainIncorrect.begin();
			for (int k=0; k<Index; k++, ItIndices++);
			IndicesValidationIncorrect.push_back(*ItIndices);
			IndicesTrainIncorrect.remove(*ItIndices);
		}

		// train classifiers
		TrainGlobal(pClassifierType, pClass, NumberSamplesCorrectTrain+NumberSamplesIncorrectTrain, NumberFeatures, ItGlobalFeaturesClass, &IndicesTrainCorrect, &IndicesTrainIncorrect, NegativeSamplesMatrix, pViewsPerObject);

		// validate classifier
		ValidateGlobal(pClassifierType, pClass, NumberFeatures, ItGlobalFeaturesClass, &IndicesValidationCorrect, &IndicesValidationIncorrect, NegativeSamplesMatrix, NegativeSamplesLabels, pValidationSetOutput);

		// write back the validation indices into the train index set for objects from class
		for (ItIndices = IndicesValidationCorrect.begin(); ItIndices != IndicesValidationCorrect.end(); ItIndices = IndicesValidationCorrect.begin())
		{
			IndicesTrainCorrect.push_back(*ItIndices);
			IndicesValidationCorrect.remove(*ItIndices);
		}
		IndicesTrainCorrect.sort();

		// write back the validation indices into the train index set for non-class objects
		for (ItIndices = IndicesValidationIncorrect.begin(); ItIndices != IndicesValidationIncorrect.end(); ItIndices = IndicesValidationIncorrect.begin())
		{
			IndicesTrainIncorrect.push_back(*ItIndices);
			IndicesValidationIncorrect.remove(*ItIndices);
		}
		IndicesTrainIncorrect.sort();
	}
	std::cout << std::endl;
	
	/// Test classifier with the unseen test set.
	// train classifier with whole train data	// hack: this is normally contained in the following if statement
	TrainGlobal(pClassifierType, pClass, NumberSamplesCorrect-NumberSamplesCorrectTest+NumberSamplesIncorrect-NumberSamplesIncorrectTest, NumberFeatures, ItGlobalFeaturesClass, &IndicesTrainCorrect, &IndicesTrainIncorrect, NegativeSamplesMatrix, pViewsPerObject);
	if (NumberObjectsCorrectTest > 0)
	{
		// test classifier
		ValidateGlobal(pClassifierType, pClass, NumberFeatures, ItGlobalFeaturesClass, &IndicesTestCorrect, &IndicesTestIncorrect, NegativeSamplesMatrix, NegativeSamplesLabels, pTestSetOutput);
	}
	// variable importance
	if (*pVariableImportance != NULL) GetVariableImportance(mData.mGlobalClassifierMap.find(pClass)->second, pClassifierType, pVariableImportance);

	cvReleaseMat(&NegativeSamplesMatrix);
	return ipa_utils::RET_OK;
}


int ObjectClassifier::CrossValidationGlobal(std::string pStatisticsPath, std::string pFileDescription, ClassifierType pClassifierType, int pNumberCycles, float pFactorIncorrect,
											float pPercentTest, float pPercentValidation, int pViewsPerObject, std::ofstream* pScreenLogFile)
{
	GlobalFeaturesMap::iterator ItGlobalFeaturesMap, ItGlobalFeaturesMap2;
	ClassifierOutputCollection ValidationSetOutput;
	ClassifierOutputCollection TestSetOutput;
	ClassifierPerformanceStruct TestSetPerformance;
	ClassifierPerformanceStruct ValidationSetPerformance;
	double averageDistance = 0.0;
	int objectNumber = 0;
	std::map<std::string, double> bestDistancePerClass;	// stores best distance per class

	std::cout << "\n\n Global cross-validation:\n";
	if (pScreenLogFile) *pScreenLogFile << "\n\n Global cross-validation:\n";
	/// Iterate over all classes
	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
	{
		std::string Class = ItGlobalFeaturesMap->first;
		std::cout << "\n\n\n-------------------------------------\n" << Class << ":\n-------------------------------------\n";
		if (pScreenLogFile) *pScreenLogFile << "\n\n\n-------------------------------------\n" << Class << ":\n-------------------------------------\n";

		// classifier accuracy statistics
		for (ItGlobalFeaturesMap2 = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap2 != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap2++)
			mData.mGlobalClassifierAccuracy[Class][ItGlobalFeaturesMap2->first] = 0.0001;
		
		// Statistics file for every object
		std::stringstream FileName;
		FileName << pStatisticsPath << "ROCCurve_" << pFileDescription << "_" << ClassifierLabel(pClassifierType) << "_" << Class << ".txt";
		std::ofstream f((FileName.str()).c_str(), std::fstream::out);
		if(!f.is_open())
		{
			std::cout << "ObjectClassifier::CrossValidationGlobal: Could not open '" << FileName.str() << "'" << std::endl;
			if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::CrossValidationGlobal: Could not open '" << FileName.str() << "'" << std::endl;
			return ipa_utils::RET_FAILED;
		}

		// clear response buffer
		ValidationSetOutput.clear();
		TestSetOutput.clear();

		// do a cross-validation and determine the classifier responses
		const CvMat** SingleRunVariableImportance = new const CvMat*;
		*SingleRunVariableImportance = NULL;
		CrossValidationGlobal(pClassifierType, Class, pNumberCycles, pFactorIncorrect, ValidationSetOutput, TestSetOutput, pPercentTest, pPercentValidation, SingleRunVariableImportance, pViewsPerObject);

		// variable importance
		std::vector<CvMat*> VariableImportance;
		std::vector<CvMat*>::iterator ItVariableImportance;
		if (*SingleRunVariableImportance != 0) VariableImportance.push_back(cvCloneMat(*SingleRunVariableImportance));

		/// Calculate ROC curve, variable importance
		double NearestDistance = 2.0;
		float BestThreshold = 0;
		bool printHeader = true;
		for (float Threshold=(float)0.00; Threshold<=1.0; Threshold+=(float)0.005)
		{
			//std::cout << "\n\nThreshold: " << Threshold << "\n";
			f << Threshold << "\t";

			// initialize StatisticsMap
			ValidationSetPerformance.clear();
			TestSetPerformance.clear();

			// generate statistics
			StatisticsFromOutput(ValidationSetOutput, ValidationSetPerformance, Threshold);
			StatisticsFromOutput(TestSetOutput, TestSetPerformance, Threshold);

			// output (and save) results
			double Distance = 2.0;
			CrossValidationStatisticsOutput(Class, ValidationSetPerformance, TestSetPerformance, f, pFactorIncorrect, Distance, Threshold, printHeader, pScreenLogFile);
			printHeader = false;

			// find best threshold
			if (Distance < NearestDistance)
			{
				BestThreshold = Threshold;
				NearestDistance = Distance;
			}
		}
		f.close();

		// average/best Distance
		objectNumber += mData.mGlobalFeaturesMap[Class].size();
		averageDistance += NearestDistance*(double)mData.mGlobalFeaturesMap[Class].size();
		bestDistancePerClass[Class] = NearestDistance;

		// save best threshold
		mData.mGlobalClassifierThresholdMap[Class] = BestThreshold;

		// prepare accuracy statistics corresponding to best threshold
		// positive samples
		for (unsigned int i=0; i<ValidationSetOutput.positiveSampleResponses.size(); i++)
			if (ValidationSetOutput.positiveSampleResponses[i] >= BestThreshold) mData.mGlobalClassifierAccuracy[Class][Class]+=1.0;
		mData.mGlobalClassifierAccuracy[Class][Class] /= (double)ValidationSetOutput.positiveSampleResponses.size();
		// negative samples
		for (unsigned int i=0; i<ValidationSetOutput.negativeSampleResponses.size(); i++)
			if (ValidationSetOutput.negativeSampleResponses[i] >= BestThreshold) mData.mGlobalClassifierAccuracy[Class][ValidationSetOutput.negativeSampleCorrectLabel[i]]+=1.0/(double)ValidationSetOutput.negativeSampleResponses.size();


		// save variable importance
		if (VariableImportance.size() > 0)
		{
			std::stringstream VarImpFileName;
			VarImpFileName << pStatisticsPath << "VarImportance_Glob_" << pFileDescription << "_" << ClassifierLabel(pClassifierType) << "_" << Class << ".txt";
			f.open((VarImpFileName.str()).c_str(), std::fstream::out);
			if(!f.is_open())
			{
				std::cout << "ObjectClassifier::CrossValidationGlobal: Could not open '" << VarImpFileName.str() << "'" << std::endl;
				if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::CrossValidationGlobal: Could not open '" << VarImpFileName.str() << "'" << std::endl;
				return ipa_utils::RET_FAILED;
			}
			for (ItVariableImportance = VariableImportance.begin(); ItVariableImportance != VariableImportance.end(); ItVariableImportance++)
			{
				for (int j=0; j<(*ItVariableImportance)->width; j++) f << cvGetReal1D(*ItVariableImportance, j) << "\t";
				f << "\n";
				cvReleaseMat(&(*ItVariableImportance));
			}
			f.close();
		}
	}

	// normalize and save classifier accuracy statistics corresponding to best threshold
	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
	{
		std::string groundTruthLabel = ItGlobalFeaturesMap->first;
		double sum = 0.0;
		for (ItGlobalFeaturesMap2 = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap2 != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap2++)
		{
			std::string outputLabel = ItGlobalFeaturesMap2->first;
			if (outputLabel == groundTruthLabel) continue;
			sum += mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel];
		}
		for (ItGlobalFeaturesMap2 = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap2 != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap2++)
		{
			std::string outputLabel = ItGlobalFeaturesMap2->first;
			if (outputLabel == groundTruthLabel) continue;
			mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel] = (1-mData.mGlobalClassifierAccuracy[groundTruthLabel][groundTruthLabel]) * mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel]/sum;
		}
	}

	// distance statistics
	for (std::map<std::string, double>::iterator itBestDistancePerClass = bestDistancePerClass.begin(); itBestDistancePerClass!=bestDistancePerClass.end(); itBestDistancePerClass++)
	{
		std::cout << itBestDistancePerClass->first << "\t" << itBestDistancePerClass->second << "\n";
		if (pScreenLogFile) *pScreenLogFile <<  itBestDistancePerClass->first << "\t" << itBestDistancePerClass->second << "\n";
	}
	averageDistance /= (double)objectNumber;
	std::cout << "\n\nAverage Distance: " << averageDistance << "\n";
	if (pScreenLogFile) *pScreenLogFile << "\n\nAverage Distance: " << averageDistance << "\n";

	return ipa_utils::RET_OK;
}

int ObjectClassifier::CrossValidationGlobalMultiClass(std::string pStatisticsPath, std::string pFileDescription, ClassifierType pClassifierType, int pFold, float pFactorIncorrect, float pPercentTest,
														float pPercentValidation, int pViewsPerObject, std::ofstream* pScreenLogFile)
{
	bool drawValidationSetRandomized = false;

	if (pPercentTest > 0.)
	{
		std::cout << "Error: CrossValidationGlobalMultiClass: A test set is not supported at the moment. Please use only the training and validation sets." << std::endl;
		if (pScreenLogFile) *pScreenLogFile << "Error: CrossValidationGlobalMultiClass: A test set is not supported at the moment. Please use only the training and validation sets." << std::endl;
		return ipa_utils::RET_FAILED;
	}

	// determine number of objects available for training/validation/test set for each class
	GlobalFeaturesMap::iterator ItGlobalFeaturesMap, ItGlobalFeaturesMap2;
	std::map<std::string, int> numberObjects;		// number of objects available for each class
	std::map<std::string, int> numberObjectsTest;	// number of objects used in test set for each class
	std::map<std::string, int> numberObjectsTrain;	// number of objects used in training set for each class
	std::map<std::string, int> numberObjectsValidation;	// number of objects used in validation set for each class
	int numberFeatures = 0;		// dimension of the feature vectors
	std::map<std::string, int> objectClassNumberMapping;	// maps each object class name to a unique number
	std::map<int, std::string> numberObjectClassMapping;	// maps a unique number to an object class name
	int uniqueID = 0;
	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
	{
		std::string label = ItGlobalFeaturesMap->first;
		objectClassNumberMapping[label] = uniqueID;
		numberObjectClassMapping[uniqueID] = label;
		uniqueID++;
		numberObjects[label] = ItGlobalFeaturesMap->second.size();
		
		if (((numberObjectsTest[label]=cvRound((double)numberObjects[label]*pPercentTest)) < 1) && (pPercentTest > 0.0))	// check for a large enough number of objects
		{
			numberObjectsTest[label] = 1;
			//std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or increase the percentage of the test set.\n";
			//return ipa_utils::RET_FAILED;
		}
		if ((numberObjectsValidation[label]=cvRound((double)numberObjects[label]*pPercentValidation)) < 1)
		{
			numberObjectsValidation[label] = 1;
			//std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or increase the percentage of the validation set.\n";
			//return ipa_utils::RET_FAILED;
		}
		if ((numberObjectsTrain[label] = numberObjects[label]-numberObjectsTest[label]-numberObjectsValidation[label]) < 1)
		{
			std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << label << ". Use more examples or leave more percentage for the train set.\n";
			if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << label << ". Use more examples or leave more percentage for the train set.\n";
			return ipa_utils::RET_FAILED;
		}

		if (numberFeatures == 0)
			numberFeatures = ItGlobalFeaturesMap->second[0]->cols;
	}

	// index lists of object indices (to put whole object datasets in individual sets, i.e. no data of object x can be split into validation and training set, it all goes into one set)
	std::map<std::string, std::list<int> > indicesTrain;	// object indices for each set
	std::map<std::string, std::list<int> > indicesValidation;
	std::map<std::string, std::list<int> > indicesTest;
	std::list<int>::iterator ItIndices;
	std::map<std::string, int> numberSamples;		// total number of feature vectors available for each class
	std::map<std::string, int> numberSamplesTest;	// number of feature vectors available for the test set of each class
	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
	{
		std::string label = ItGlobalFeaturesMap->first;
		// fill index lists for training with all objects of each class and remove indices of objects for test and validation set later
		for (int i=0; i<numberObjects[label]; i++) indicesTrain[label].push_back(i);

		// note: was not used anywhere
		//// determine total number of available feature vectors for this class
		//numberSamples[label] = 0;
		//for (ItIndices = indicesTrain[label].begin(); ItIndices != indicesTrain[label].end(); ItIndices++)
		//	numberSamples[label] += ItGlobalFeaturesMap->second[*ItIndices]->rows;

		// prepare index list for test set
		numberSamplesTest[label] = 0;
		for (int i=0; i<numberObjectsTest[label]; i++)
		{
			int index = int(indicesTrain[label].size()*((double)rand()/((double)RAND_MAX+1.0)));
			ItIndices = indicesTrain[label].begin();
			for (int k=0; k<index; k++, ItIndices++);
			numberSamplesTest[label] += ItGlobalFeaturesMap->second[*ItIndices]->rows;
			indicesTest[label].push_back(*ItIndices);
			indicesTrain[label].remove(*ItIndices);
		}
	}


	// cross validation cycles
	std::map<std::string, std::map<std::string, int> > multiclassStatistics;   // statistics: tp=true positive, fp=false positive, fn=false negative
	std::map<std::string, std::map<std::string, int> > multiclassStatisticsBinary;   // statistics: tp=true positive, fp=false positive, fn=false negative
	std::map<std::string, std::map< int, std::vector< std::string > > > individualResults;	// prediction results, indices: classname - object number - view index - prediction

	// prepare statistics
	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
	{
		std::string label = ItGlobalFeaturesMap->first;
		multiclassStatistics[label]["tp"] = 0;
		multiclassStatistics[label]["fp"] = 0;
		multiclassStatistics[label]["fn"] = 0;
		multiclassStatisticsBinary[label]["tp"] = 0;
		multiclassStatisticsBinary[label]["fp"] = 0;
		multiclassStatisticsBinary[label]["fn"] = 0;
	}
	std::vector<std::map<std::string, std::map<std::string, int> > > singleFoldMulticlassStatistics(pFold, multiclassStatistics);   // statistics: tp=true positive, fp=false positive, fn=false negative
	std::vector<std::map<std::string, std::map<std::string, int> > > singleFoldMulticlassStatisticsBinary(pFold, multiclassStatisticsBinary);   // statistics: tp=true positive, fp=false positive, fn=false negative

	for (int fold=0; fold<pFold; fold++)
	{
		int numberSamplesTrain = 0;	// number of feature vectors available for the training set
		int numberSamplesValidation = 0;	// number of feature vectors available for the validation set
		std::map<std::string, int> numberSamplesTrainClasswise;	// number of feature vectors available for the training set in each class
		std::map<std::string, int> numberSamplesValidationClasswise;
		for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
		{
			std::string label = ItGlobalFeaturesMap->first;
			// prepare index list for validation and training set
			for (int i=0; i<numberObjectsValidation[label]; i++)
			{
				int index = 0;
				if (drawValidationSetRandomized == true || fold>=(int)indicesTrain[label].size())
					index = int(indicesTrain[label].size()*((double)rand()/((double)RAND_MAX+1.0)));
				else
					index = (fold+i)%indicesTrain[label].size();
				ItIndices = indicesTrain[label].begin();
				for (int k=0; k<index; k++, ItIndices++);
				indicesValidation[label].push_back(*ItIndices);
				indicesTrain[label].remove(*ItIndices);
			}

			// count number of feature vectors for each set
			numberSamplesTrainClasswise[label] = 0;
			for (ItIndices = indicesTrain[label].begin(); ItIndices != indicesTrain[label].end(); ItIndices++)
			{
				if (pViewsPerObject == -1)
				{
					numberSamplesTrain += ItGlobalFeaturesMap->second[*ItIndices]->rows;
					numberSamplesTrainClasswise[label] += ItGlobalFeaturesMap->second[*ItIndices]->rows;
				}
				else
				{
					numberSamplesTrain += min(pViewsPerObject, ItGlobalFeaturesMap->second[*ItIndices]->rows);
					numberSamplesTrainClasswise[label] += min(pViewsPerObject, ItGlobalFeaturesMap->second[*ItIndices]->rows);
				}
			}
			numberSamplesValidationClasswise[label] = 0;
			for (ItIndices = indicesValidation[label].begin(); ItIndices != indicesValidation[label].end(); ItIndices++)
			{
				numberSamplesValidation += ItGlobalFeaturesMap->second[*ItIndices]->rows;
				numberSamplesValidationClasswise[label] += ItGlobalFeaturesMap->second[*ItIndices]->rows;
			}
		}

		// construct training data and label matrices
		cv::Mat TrainingFeatureMatrix(numberSamplesTrain, numberFeatures, CV_32FC1);	// contains all training data, suited for multiclass classifiers
		cv::Mat TrainingFeatureResponseMatrix(numberSamplesTrain, 1, CV_32SC1);
		std::map<std::string, cv::Mat> TrainingFeatureMatricesBinary;		// contains binary training data (class samples and non-class samples), suited for binary classifiers
		std::map<std::string, cv::Mat> TrainingFeatureResponseMatricesBinary;
		int insertPosition = 0;
		for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
		{
			std::string label = ItGlobalFeaturesMap->first;
			int insertPositionBinary = 0;
			TrainingFeatureMatricesBinary[label] = cv::Mat(cvRound((double)numberSamplesTrainClasswise[label]*(1.0+pFactorIncorrect)), numberFeatures, CV_32FC1);
			TrainingFeatureResponseMatricesBinary[label] = cv::Mat(TrainingFeatureMatricesBinary[label].rows, 1, CV_32FC1);
			// class samples
			for (ItIndices = indicesTrain[label].begin(); ItIndices != indicesTrain[label].end(); ItIndices++)
			{
				for (double dSample=0; dSample<ItGlobalFeaturesMap->second[*ItIndices]->rows; (pViewsPerObject==-1) ? dSample+=1. : dSample+=max(1., (double)ItGlobalFeaturesMap->second[*ItIndices]->rows/(double)pViewsPerObject))
				{
					int sample = (int)dSample;
					for (int j=0; j<numberFeatures; j++)
					{
						TrainingFeatureMatrix.at<float>(insertPosition, j) = (float)cvmGet(ItGlobalFeaturesMap->second[*ItIndices], sample, j);
						TrainingFeatureMatricesBinary[label].at<float>(insertPositionBinary, j) = (float)cvmGet(ItGlobalFeaturesMap->second[*ItIndices], sample, j);
					}
					TrainingFeatureResponseMatrix.at<int>(insertPosition, 0) = objectClassNumberMapping[label];
					TrainingFeatureResponseMatricesBinary[label].at<float>(insertPositionBinary, 0) = 1.0;
					insertPosition++;
					insertPositionBinary++;
				}
			}
			// non-class samples for the matrices suited for binary classifiers
			while (insertPositionBinary < TrainingFeatureMatricesBinary[label].rows)
			{
				// pick random class
				int index =	int(mData.mGlobalFeaturesMap.size()*((double)rand()/((double)RAND_MAX+1.0)));
				ItGlobalFeaturesMap2 = mData.mGlobalFeaturesMap.begin();
				for (int k=0; k<index; k++, ItGlobalFeaturesMap2++);
				std::string labelIncorrect = ItGlobalFeaturesMap2->first;
				if (labelIncorrect == label) continue;		// do not pick samples from correct class
				
				// pick random training object
				index = int(indicesTrain[labelIncorrect].size()*((double)rand()/((double)RAND_MAX+1.0)));
				ItIndices = indicesTrain[labelIncorrect].begin();
				for (int k=0; k<index; k++, ItIndices++);

				// pick random sample from that object
				if (pViewsPerObject == -1)
					index = int(ItGlobalFeaturesMap2->second[*ItIndices]->rows*((double)rand()/((double)RAND_MAX+1.0)));
				else
				{
					double step=(double)ItGlobalFeaturesMap2->second[*ItIndices]->rows/(double)pViewsPerObject;
					do
					{
						index = int(ItGlobalFeaturesMap2->second[*ItIndices]->rows*((double)rand()/((double)RAND_MAX+1.0)));
					} while (index != int(step * int((double)index/step)));
				}
				for (int j=0; j<numberFeatures; j++)
					TrainingFeatureMatricesBinary[label].at<float>(insertPositionBinary, j) = (float)cvmGet(ItGlobalFeaturesMap2->second[*ItIndices], index, j);
				TrainingFeatureResponseMatricesBinary[label].at<float>(insertPositionBinary, 0) = 0.0;
				insertPositionBinary++;
			}
		}

		
		if (pClassifierType == CLASSIFIER_KNN)
		{
			// train KNN multi classifier
			CvKNearest* KNN = new CvKNearest;
			CvMat trainMat = (CvMat)TrainingFeatureMatrix;
			CvMat labelMat = (CvMat)TrainingFeatureResponseMatrix;
			bool trainResult = KNN->train(&trainMat, &labelMat, 0, false);
			if (!trainResult) std::cout << "Training Multiclass failed." << std::endl;

			// validate KNN multi classifier
			//statistics
			for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
			{
				std::string label = ItGlobalFeaturesMap->first;
				for (ItIndices = indicesValidation[label].begin(); ItIndices != indicesValidation[label].end(); ItIndices++)
				{
					for (int sample=0; sample<ItGlobalFeaturesMap->second[*ItIndices]->rows; sample++)
					{
						CvMat* featureVector = cvCreateMat(1, ItGlobalFeaturesMap->second[*ItIndices]->cols, ItGlobalFeaturesMap->second[*ItIndices]->type);
						cvGetRow(ItGlobalFeaturesMap->second[*ItIndices], featureVector, sample);

						// validate multi classifier
						int k=1;
						float result = KNN->find_nearest(featureVector, k);
						if (result == (float)objectClassNumberMapping[label])
						{
							// correct classification
							multiclassStatistics[label]["tp"]++;
							singleFoldMulticlassStatistics[fold][label]["tp"]++;
						}
						else
						{
							// false classification
							multiclassStatistics[label]["fn"]++;
							multiclassStatistics[numberObjectClassMapping[(int)result]]["fp"]++;
							singleFoldMulticlassStatistics[fold][label]["fn"]++;
							singleFoldMulticlassStatistics[fold][numberObjectClassMapping[(int)result]]["fp"]++;
						}
						
						individualResults[label][*ItIndices].push_back(numberObjectClassMapping[(int)result]);

						cvReleaseMat(&featureVector);
					}
				}
			}
		}
		else
		{
			CvSVM* SVM = new CvSVM;
			if (pClassifierType == CLASSIFIER_SVM)
			{
				// train multi classifier
				CvSVMParams SVMParams = CvSVMParams(CvSVM::NU_SVC, CvSVM::RBF, 0, 0.007, 0, 1.0, 0.09, 0, 0, cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 2500, 0.0001));
				CvMat trainMat = (CvMat)TrainingFeatureMatrix;
				CvMat labelMat = (CvMat)TrainingFeatureResponseMatrix;
				//bool trainResult = SVM->train(&trainMat, &labelMat, 0, 0, SVMParams);
				CvParamGrid cGrid(0, 1, 0);
				CvParamGrid gammaGrid(0.00021875, 3.0, 2.0);//CvParamGrid gammaGrid(0.00021875, 5.0, 2.0);  //CvParamGrid gammaGrid(0.000875, 1.0, 2.0);  CvParamGrid gammaGrid(0.00175, 0.00176, 2.0);
				CvParamGrid pGrid(0, 1, 0);
				CvParamGrid nuGrid(0.01125, 0.3, 2.0); //CvParamGrid nuGrid(0.01125, 0.3, 2.0);  CvParamGrid nuGrid(0.0225, 0.091, 2.0);
				CvParamGrid coeffGrid(0, 1, 0);
				CvParamGrid degreeGrid(0, 1, 0);
				bool trainResult = SVM->train_auto(&trainMat, &labelMat, 0, 0, SVMParams, 10, cGrid, gammaGrid, pGrid, nuGrid, coeffGrid, degreeGrid);
				CvSVMParams optimalParams = SVM->get_params();
				std::cout << "\nOptimal params: gamma=" << optimalParams.gamma << "  nu=" << optimalParams.nu << "  C=" << optimalParams.C << "  p=" << optimalParams.p << "  coeff=" << optimalParams.coef0 << "  degree=" << optimalParams.degree << std::endl;
				*pScreenLogFile << "\nOptimal params: gamma=" << optimalParams.gamma << "  nu=" << optimalParams.nu << "  C=" << optimalParams.C << "  p=" << optimalParams.p << "  coeff=" << optimalParams.coef0 << "  degree=" << optimalParams.degree << std::endl;
				if (trainResult) std::cout << "Training Multiclass finished successfully." << std::endl;
				else std::cout << "Training Multiclass failed." << std::endl;
			}
			else
			{
				// train binary classifiers
				for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
				{
					std::string label = ItGlobalFeaturesMap->first;
					CvMat trainMatBinary = TrainingFeatureMatricesBinary[label];
					CvMat labelMatBinary = TrainingFeatureResponseMatricesBinary[label];
					TrainGlobal(pClassifierType, label, &trainMatBinary, &labelMatBinary);
				}
			}

			// validate multi classifier and binary classifiers
			//compute marginals p(o_k) for output o_k, assuming p(c_i) uniformly distributed
			std::map<std::string, double> p_ok;
			for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
			{
				std::string outputLabel = ItGlobalFeaturesMap->first;
				p_ok[outputLabel] = 0.0;
				for (ItGlobalFeaturesMap2 = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap2 != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap2++)
				{
					std::string groundTruthLabel = ItGlobalFeaturesMap2->first;
					p_ok[outputLabel] += mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel];
				}
				p_ok[outputLabel] /= (double)mData.mGlobalFeaturesMap.size();
			}
			//statistics
			for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
			{
				std::string label = ItGlobalFeaturesMap->first;
				for (ItIndices = indicesValidation[label].begin(); ItIndices != indicesValidation[label].end(); ItIndices++)
				{
					for (int sample=0; sample<ItGlobalFeaturesMap->second[*ItIndices]->rows; sample++)
					{
						CvMat* featureVector = cvCreateMat(1, ItGlobalFeaturesMap->second[*ItIndices]->cols, ItGlobalFeaturesMap->second[*ItIndices]->type);
						cvGetRow(ItGlobalFeaturesMap->second[*ItIndices], featureVector, sample);

						if (pClassifierType == CLASSIFIER_SVM)
						{
							// validate multi classifier
							float result = SVM->predict(featureVector);
							if (result == (float)objectClassNumberMapping[label])
							{
								// correct classification
								multiclassStatistics[label]["tp"]++;
								singleFoldMulticlassStatistics[fold][label]["tp"]++;
							}
							else
							{
								// false classification
								multiclassStatistics[label]["fn"]++;
								multiclassStatistics[numberObjectClassMapping[(int)result]]["fp"]++;
								singleFoldMulticlassStatistics[fold][label]["fn"]++;
								singleFoldMulticlassStatistics[fold][numberObjectClassMapping[(int)result]]["fp"]++;
							}

							individualResults[label][*ItIndices].push_back(numberObjectClassMapping[(int)result]);
						}
						else if (pClassifierType == CLASSIFIER_RTC)
						{
							// validate binary classifiers
							std::map<std::string, double> classProbabilities;	// outputs p(o_k|x) of the different binary classifiers given the sample x
							double maxAPrioriProbability = -1.0;
							std::string maxAPrioriLabel = "";
							for (GlobalClassifierMap::iterator ItGlobalClassifierMap=mData.mGlobalClassifierMap.begin(); ItGlobalClassifierMap!=mData.mGlobalClassifierMap.end(); ItGlobalClassifierMap++)
							{
								double prediction = 0.0, th = mData.mGlobalClassifierThresholdMap[ItGlobalClassifierMap->first];
								PredictGlobal(pClassifierType, ItGlobalClassifierMap->first, featureVector, prediction);
								double mappedPrediction = 0;
								if (prediction>=th) mappedPrediction = 2*(prediction-th)/(1.0-th);
								else mappedPrediction = 2*(prediction-th)/th;
								classProbabilities[ItGlobalClassifierMap->first] = exp(mappedPrediction)/(exp(mappedPrediction)+exp(-mappedPrediction));
								//double temp = exp(mappedPrediction);

								if (classProbabilities[ItGlobalClassifierMap->first] > maxAPrioriProbability)
								{
									maxAPrioriProbability = classProbabilities[ItGlobalClassifierMap->first];
									maxAPrioriLabel = ItGlobalClassifierMap->first;
								}
							}
							// max a posteriori label
							std::map<std::string, double> p_ci_x;	// probability distribution for the actual object class given measurement x
							std::map<std::string, double>::iterator ItGroundTruthClass, ItOutputClass;
							for (ItGroundTruthClass = classProbabilities.begin(); ItGroundTruthClass != classProbabilities.end(); ItGroundTruthClass++)		// heuristic approach, light/fast probabilistic approach
							{
								std::string groundTruthLabel = ItGroundTruthClass->first;
								p_ci_x[groundTruthLabel] = 0.0;
								for (ItOutputClass = classProbabilities.begin(); ItOutputClass != classProbabilities.end(); ItOutputClass++)
								{
									std::string outputLabel = ItOutputClass->first;
									p_ci_x[groundTruthLabel] += mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel]/(p_ok[outputLabel]*classProbabilities.size()) * classProbabilities[outputLabel];
								}
							}
							//for (ItGroundTruthClass = classProbabilities.begin(); ItGroundTruthClass != classProbabilities.end(); ItGroundTruthClass++)
							//	p_ci_x[ItGroundTruthClass->first] = 0.0;
							//unsigned long numberCombinations = 1<<classProbabilities.size();
							//for (unsigned long outputCombination = 0; outputCombination < numberCombinations; outputCombination++)
							//{
							//	double summandCommonPart=1;
							//	int classShift=0;
							//	for (ItOutputClass = classProbabilities.begin(); ItOutputClass != classProbabilities.end(); ItOutputClass++, classShift++)
							//	{
							//		std::string outputLabel = ItOutputClass->first;
							//		bool outputState = ((outputCombination>>classShift) & 0x01) == 1;	// if true: p(o_k=1) is considered, else p(o_k=0)
							//		if (outputState == true)
							//			summandCommonPart *= classProbabilities[outputLabel] / p_ok[outputLabel];
							//		else
							//			summandCommonPart *= (1.-classProbabilities[outputLabel]) / (1.-p_ok[outputLabel]);
							//	}
							//	summandCommonPart /= classProbabilities.size();	// uniform ground truth class prior

							//	for (ItGroundTruthClass = classProbabilities.begin(); ItGroundTruthClass != classProbabilities.end(); ItGroundTruthClass++)
							//	{
							//		std::string groundTruthLabel = ItGroundTruthClass->first;
							//		double summandGroundTruthClassSpecific=1;
							//		int classShift=0;
							//		for (ItOutputClass = classProbabilities.begin(); ItOutputClass != classProbabilities.end(); ItOutputClass++, classShift++)
							//		{
							//			std::string outputLabel = ItOutputClass->first;
							//			bool outputState = ((outputCombination>>classShift) & 0x01) == 1;	// if true: p(o_k=1) is considered, else p(o_k=0)
							//			if (outputState == true)
							//				summandGroundTruthClassSpecific *= mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel];
							//			else
							//				summandGroundTruthClassSpecific *= (1.-mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel]);
							//		}
							//		p_ci_x[groundTruthLabel] += summandGroundTruthClassSpecific * summandCommonPart;
							//	}
							//}

							double maxAPosterioriProbability = -1.0;
							std::string maxAPosterioriLabel = "";
							for (ItGroundTruthClass = classProbabilities.begin(); ItGroundTruthClass != classProbabilities.end(); ItGroundTruthClass++)
							{
								std::string groundTruthLabel = ItGroundTruthClass->first;
								if (p_ci_x[groundTruthLabel] > maxAPosterioriProbability)
								{
									maxAPosterioriProbability = p_ci_x[groundTruthLabel];
									maxAPosterioriLabel = groundTruthLabel;
								}
							}

							std::string maxLabel = maxAPosterioriLabel;		// or: maxAPrioriLabel
							if (maxLabel == label)
							{
								// correct classification
								multiclassStatisticsBinary[label]["tp"]++;
								singleFoldMulticlassStatisticsBinary[fold][label]["tp"]++;
							}
							else
							{
								// false classification
								multiclassStatisticsBinary[label]["fn"]++;
								multiclassStatisticsBinary[maxLabel]["fp"]++;
								singleFoldMulticlassStatisticsBinary[fold][label]["fn"]++;
								singleFoldMulticlassStatisticsBinary[fold][maxLabel]["fp"]++;
							}

							individualResults[label][*ItIndices].push_back(maxLabel);
						}

						cvReleaseMat(&featureVector);
					}
				}
			}
			SVM->clear();
			delete SVM;
		}

		// write back the validation indices into the train index set for objects from class
		for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
		{
			std::string label = ItGlobalFeaturesMap->first;
			for (ItIndices = indicesValidation[label].begin(); ItIndices != indicesValidation[label].end(); ItIndices = indicesValidation[label].begin())
			{
				indicesTrain[label].push_back(*ItIndices);
				indicesValidation[label].remove(*ItIndices);
			}
			indicesTrain[label].sort();
		}
		std::cout << ".";
		if (pScreenLogFile) *pScreenLogFile << ".";
	}
	std::cout << std::endl;
	if (pScreenLogFile) *pScreenLogFile << std::endl;
	
	/// Test classifier with the unseen test set.
	if (numberObjectsTest.begin()->second > 0)
	{
		// train classifier with whole train data
		//TrainGlobal(pClassifierType, pClass, NumberSamplesCorrect-NumberSamplesCorrectTest+NumberSamplesIncorrect-NumberSamplesIncorrectTest, NumberFeatures, ItGlobalFeaturesClass, &IndicesTrainCorrect, &IndicesTrainIncorrect, NegativeSamplesMatrix);

		// test classifier
		//ValidateGlobal(pClassifierType, pClass, NumberFeatures, ItGlobalFeaturesClass, &IndicesTestCorrect, &IndicesTestIncorrect, NegativeSamplesMatrix, pTestSetOutput);
	}
	// variable importance
	//if (*pVariableImportance != NULL) GetVariableImportance(mData.mGlobalClassifierMap.find(pClass)->second, pClassifierType, pVariableImportance);


/*	std::cout << std::endl << "Multiclass statistics:" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
	double avgRecall=0.0, avgPrecision=0.0;
	for (std::map<std::string, std::map<std::string, int> >::iterator ItStatistics = multiclassStatistics.begin(); ItStatistics != multiclassStatistics.end(); ItStatistics++)
	{
		double recall = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fn"]);
		double precision = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fp"]);
		avgRecall += recall;
		avgPrecision += precision;
		std::cout << ItStatistics->first << (ItStatistics->first.length()<8 ? "\t" : "") << "\t" << ItStatistics->second["tp"] << "\t" << ItStatistics->second["fn"] << "\t" << ItStatistics->second["fp"] << "\t" << recall << "\t" << precision << std::endl;
	}
	std::cout << "\t\t\t\t\t" << avgRecall/(double)multiclassStatistics.size() << "\t" << avgPrecision/(double)multiclassStatistics.size() << std::endl;
*/

	if (pClassifierType == CLASSIFIER_KNN || pClassifierType == CLASSIFIER_SVM)
	{
		// results for each single fold
		std::cout << std::endl << "Multiclass statistics from multiclass classifiers (for each of " << pFold << " folds):" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl << "Multiclass statistics from multiclass classifiers (for each of " << pFold << " folds):" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		for (int fold=0; fold<pFold; fold++)
		{
			for (std::map<std::string, std::map<std::string, int> >::iterator ItStatistics = singleFoldMulticlassStatistics[fold].begin(); ItStatistics != singleFoldMulticlassStatistics[fold].end(); ItStatistics++)
			{
				double recall = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fn"]);
				double precision = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fp"]);
				std::cout << ItStatistics->first << (ItStatistics->first.length()<8 ? "\t" : "") << "\t" << ItStatistics->second["tp"] << "\t" << ItStatistics->second["fn"] << "\t" << ItStatistics->second["fp"] << "\t" << recall << "\t" << precision << std::endl;
				if (pScreenLogFile) *pScreenLogFile << ItStatistics->first << (ItStatistics->first.length()<8 ? "" : "") << " " << ItStatistics->second["tp"] << " " << ItStatistics->second["fn"] << " " << ItStatistics->second["fp"] << " " << recall << " " << precision << std::endl;
			}
		}

		// average over all folds
		std::cout << std::endl << "\nMulticlass statistics from multiclass classifiers:" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl << "Multiclass statistics from multiclass classifiers:" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		double avgRecall=0.0, avgPrecision=0.0;
		for (std::map<std::string, std::map<std::string, int> >::iterator ItStatistics = multiclassStatistics.begin(); ItStatistics != multiclassStatistics.end(); ItStatistics++)
		{
			double recall = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fn"]);
			double precision = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fp"]);
			avgRecall += recall;
			avgPrecision += precision;
			std::cout << ItStatistics->first << (ItStatistics->first.length()<8 ? "\t" : "") << "\t" << ItStatistics->second["tp"] << "\t" << ItStatistics->second["fn"] << "\t" << ItStatistics->second["fp"] << "\t" << recall << "\t" << precision << std::endl;
			if (pScreenLogFile) *pScreenLogFile << ItStatistics->first << (ItStatistics->first.length()<8 ? "" : "") << " " << ItStatistics->second["tp"] << " " << ItStatistics->second["fn"] << " " << ItStatistics->second["fp"] << " " << recall << " " << precision << std::endl;
		}
		std::cout << "\t\t\t\t\t" << avgRecall/(double)multiclassStatistics.size() << "\t" << avgPrecision/(double)multiclassStatistics.size() << std::endl;
		if (pScreenLogFile) *pScreenLogFile << " " << avgRecall/(double)multiclassStatistics.size() << " " << avgPrecision/(double)multiclassStatistics.size() << std::endl;
	}
	else
	{
		// results for each single fold
		std::cout << std::endl << "Multiclass statistics from binary classifiers (for each of " << pFold << " folds):" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl << "Multiclass statistics from binary classifiers (for each of " << pFold << " folds):" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		for (int fold=0; fold<pFold; fold++)
		{
			for (std::map<std::string, std::map<std::string, int> >::iterator ItStatistics = singleFoldMulticlassStatisticsBinary[fold].begin(); ItStatistics != singleFoldMulticlassStatisticsBinary[fold].end(); ItStatistics++)
			{
				double recall = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fn"]);
				double precision = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fp"]);
				std::cout << ItStatistics->first << (ItStatistics->first.length()<8 ? "\t" : "") << "\t" << ItStatistics->second["tp"] << "\t" << ItStatistics->second["fn"] << "\t" << ItStatistics->second["fp"] << "\t" << recall << "\t" << precision << std::endl;
				if (pScreenLogFile) *pScreenLogFile << ItStatistics->first << (ItStatistics->first.length()<8 ? "" : "") << " " << ItStatistics->second["tp"] << " " << ItStatistics->second["fn"] << " " << ItStatistics->second["fp"] << " " << recall << " " << precision << std::endl;
			}
		}

		// average over all folds
		std::cout << std::endl << "\nMulticlass statistics from binary classifiers:" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl << "Multiclass statistics from binary classifiers:" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		double avgRecall=0.0, avgPrecision=0.0;
		for (std::map<std::string, std::map<std::string, int> >::iterator ItStatistics = multiclassStatisticsBinary.begin(); ItStatistics != multiclassStatisticsBinary.end(); ItStatistics++)
		{
			double recall = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fn"]);
			double precision = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fp"]);
			avgRecall += recall;
			avgPrecision += precision;
			std::cout << ItStatistics->first << (ItStatistics->first.length()<8 ? "\t" : "") << "\t" << ItStatistics->second["tp"] << "\t" << ItStatistics->second["fn"] << "\t" << ItStatistics->second["fp"] << "\t" << recall << "\t" << precision << std::endl;
			if (pScreenLogFile) *pScreenLogFile << ItStatistics->first << (ItStatistics->first.length()<8 ? "" : "") << " " << ItStatistics->second["tp"] << " " << ItStatistics->second["fn"] << " " << ItStatistics->second["fp"] << " " << recall << " " << precision << std::endl;
		}
		std::cout << "\t\t\t\t\t" << avgRecall/(double)multiclassStatisticsBinary.size() << "\t" << avgPrecision/(double)multiclassStatisticsBinary.size() << std::endl;
		if (pScreenLogFile) *pScreenLogFile << " " << avgRecall/(double)multiclassStatisticsBinary.size() << " " << avgPrecision/(double)multiclassStatisticsBinary.size() << std::endl;
	}

	std::cout << "\n\nClasswise errors per view:" << std::endl;
	if (pScreenLogFile) *pScreenLogFile << "\n\nClasswise errors per view:" << std::endl;
	std::string pathFile = pStatisticsPath + "individualResults.txt";
	std::ofstream fout(pathFile.c_str(), std::ios_base::out);
	std::map<std::string, std::vector<int> > classwiseViewErrors;
	for (std::map<std::string, std::map< int, std::vector< std::string > > >::iterator itClasses=individualResults.begin(); itClasses!=individualResults.end(); itClasses++)
	{
		std::string label=itClasses->first;
		int numberViewsPerObject = 36;
		classwiseViewErrors[label].resize(numberViewsPerObject, 0);		// todo: this is database-dependent on the views per object
		fout << label;

		for (std::map< int, std::vector< std::string > >::iterator itObjects=itClasses->second.begin(); itObjects!=itClasses->second.end(); itObjects++)
		{
			for (int view = 0; view<(int)itObjects->second.size(); view++)
			{
				if (view%numberViewsPerObject == 0) fout << std::endl << itObjects->first << "\t";
				if (itObjects->second[view] != label)
				{
					classwiseViewErrors[label][view%numberViewsPerObject]++;
				}
				fout << itObjects->second[view] << "\t";
			}
		}
		fout << std::endl;

		std::cout << label << std::endl;
		if (pScreenLogFile) *pScreenLogFile << label << std::endl;
		for (int view=0; view<(int)classwiseViewErrors[label].size(); view++)
		{
			std::cout << ((view<10)? "  " : " ") << view << "  ";
			if (pScreenLogFile) *pScreenLogFile << ((view<10)? "  " : " ") << view << "  ";
		}
		std::cout << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl;
		for (int view=0; view<(int)classwiseViewErrors[label].size(); view++)
		{
			std::cout << ((classwiseViewErrors[label][view]<10)? "  " : " ") << classwiseViewErrors[label][view] << "  ";
			if (pScreenLogFile) *pScreenLogFile << ((classwiseViewErrors[label][view]<10)? "  " : " ") << classwiseViewErrors[label][view] << "  ";
		}
		std::cout << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl;
	}
	fout.close();

	return ipa_utils::RET_OK;
}


///////////////////////////////////////////////////////////////////////////// ----

int ObjectClassifier::CrossValidationGlobalMultiClassSampleRange(std::string pStatisticsPath, std::string pFileDescription, ClassifierType pClassifierType, int pFold, float pFactorIncorrect, float pPercentTest,
														float pPercentValidation, int pViewsPerObject, std::ofstream* pScreenLogFile, double pFactorSamplesTrainData)
{
	bool drawValidationSetRandomized = false;
	double factorSamplesTrainData = pFactorSamplesTrainData;		// for each object, this ratio of samples should go to the training set, the rest is for testing
	
	if (pPercentTest > 0.)
	{
		std::cout << "Error: CrossValidationGlobalMultiClass: A test set is not supported at the moment. Please use only the training and validation sets." << std::endl;
		if (pScreenLogFile) *pScreenLogFile << "Error: CrossValidationGlobalMultiClass: A test set is not supported at the moment. Please use only the training and validation sets." << std::endl;
		return ipa_utils::RET_FAILED;
	}

	// determine number of objects available for training/validation/test set for each class
	GlobalFeaturesMap::iterator ItGlobalFeaturesMap, ItGlobalFeaturesMap2;
	std::map<std::string, int> numberObjects;		// number of objects available for each class
	std::map<std::string, int> numberObjectsTest;	// number of objects used in test set for each class
	std::map<std::string, int> numberObjectsTrain;	// number of objects used in training set for each class
	std::map<std::string, int> numberObjectsValidation;	// number of objects used in validation set for each class
	int numberFeatures = 0;		// dimension of the feature vectors
	std::map<std::string, int> objectClassNumberMapping;	// maps each object class name to a unique number
	std::map<int, std::string> numberObjectClassMapping;	// maps a unique number to an object class name
	int uniqueID = 0;
	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
	{
		std::string label = ItGlobalFeaturesMap->first;
		objectClassNumberMapping[label] = uniqueID;
		numberObjectClassMapping[uniqueID] = label;
		uniqueID++;
		numberObjects[label] = ItGlobalFeaturesMap->second.size();
		
		if (((numberObjectsTest[label]=cvRound((double)numberObjects[label]*pPercentTest)) < 1) && (pPercentTest > 0.0))	// check for a large enough number of objects
		{
			numberObjectsTest[label] = 1;
			//std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or increase the percentage of the test set.\n";
			//return ipa_utils::RET_FAILED;
		}
		if ((numberObjectsValidation[label]=cvRound((double)numberObjects[label]*pPercentValidation)) < 1)
		{
			numberObjectsValidation[label] = 1;
			//std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << pClass << ". Use more examples or increase the percentage of the validation set.\n";
			//return ipa_utils::RET_FAILED;
		}
		if ((numberObjectsTrain[label] = numberObjects[label]-numberObjectsTest[label]-numberObjectsValidation[label]) < 1)
		{
			std::cout << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << label << ". Use more examples or leave more percentage for the train set.\n";
			if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::CrossValidationGlobal: Not enough objects to do cross validation for class " << label << ". Use more examples or leave more percentage for the train set.\n";
			return ipa_utils::RET_FAILED;
		}

		if (numberFeatures == 0)
			numberFeatures = ItGlobalFeaturesMap->second[0]->cols;
	}

	// index lists of object indices (to put whole object datasets in individual sets, i.e. no data of object x can be split into validation and training set, it all goes into one set)
	std::map<std::string, std::list<int> > indicesTrain;	// object indices for each set
	std::map<std::string, std::list<int> > indicesValidation;
	std::map<std::string, std::list<int> > indicesTest;
	std::list<int>::iterator ItIndices;
	std::map<std::string, int> numberSamples;		// total number of feature vectors available for each class
	std::map<std::string, int> numberSamplesTest;	// number of feature vectors available for the test set of each class
	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
	{
		std::string label = ItGlobalFeaturesMap->first;
		// fill index lists for training with all objects of each class and remove indices of objects for test and validation set later
		for (int i=0; i<numberObjects[label]; i++) indicesTrain[label].push_back(i);

		// note: was not used anywhere
		//// determine total number of available feature vectors for this class
		//numberSamples[label] = 0;
		//for (ItIndices = indicesTrain[label].begin(); ItIndices != indicesTrain[label].end(); ItIndices++)
		//	numberSamples[label] += ItGlobalFeaturesMap->second[*ItIndices]->rows;

		// prepare index list for test set
		numberSamplesTest[label] = 0;
		for (int i=0; i<numberObjectsTest[label]; i++)
		{
			int index = int(indicesTrain[label].size()*((double)rand()/((double)RAND_MAX+1.0)));
			ItIndices = indicesTrain[label].begin();
			for (int k=0; k<index; k++, ItIndices++);
			numberSamplesTest[label] += ItGlobalFeaturesMap->second[*ItIndices]->rows * (1. - factorSamplesTrainData);
			indicesTest[label].push_back(*ItIndices);
			indicesTrain[label].remove(*ItIndices);
		}
	}


	// cross validation cycles
	std::map<std::string, std::map<std::string, int> > multiclassStatistics;   // statistics: tp=true positive, fp=false positive, fn=false negative
	std::map<std::string, std::map<std::string, int> > multiclassStatisticsBinary;   // statistics: tp=true positive, fp=false positive, fn=false negative
	std::map<std::string, std::map< int, std::vector< std::string > > > individualResults;	// prediction results, indices: classname - object number - view index - prediction

	// prepare statistics
	for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
	{
		std::string label = ItGlobalFeaturesMap->first;
		multiclassStatistics[label]["tp"] = 0;
		multiclassStatistics[label]["fp"] = 0;
		multiclassStatistics[label]["fn"] = 0;
		multiclassStatisticsBinary[label]["tp"] = 0;
		multiclassStatisticsBinary[label]["fp"] = 0;
		multiclassStatisticsBinary[label]["fn"] = 0;
	}
	std::vector<std::map<std::string, std::map<std::string, int> > > singleFoldMulticlassStatistics(pFold, multiclassStatistics);   // statistics: tp=true positive, fp=false positive, fn=false negative
	std::vector<std::map<std::string, std::map<std::string, int> > > singleFoldMulticlassStatisticsBinary(pFold, multiclassStatisticsBinary);   // statistics: tp=true positive, fp=false positive, fn=false negative

	for (int fold=0; fold<pFold; fold++)
	{
		int numberSamplesTrain = 0;	// number of feature vectors available for the training set
		int numberSamplesValidation = 0;	// number of feature vectors available for the validation set
		std::map<std::string, int> numberSamplesTrainClasswise;	// number of feature vectors available for the training set in each class
		std::map<std::string, int> numberSamplesValidationClasswise;
		for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
		{
			std::string label = ItGlobalFeaturesMap->first;
			// prepare index list for validation and training set
			for (int i=0; i<numberObjectsValidation[label]; i++)
			{
				int index = 0;
				if (drawValidationSetRandomized == true || fold>=(int)indicesTrain[label].size())
					index = int(indicesTrain[label].size()*((double)rand()/((double)RAND_MAX+1.0)));
				else
					index = (fold+i)%indicesTrain[label].size();
				ItIndices = indicesTrain[label].begin();
				for (int k=0; k<index; k++, ItIndices++);
				indicesValidation[label].push_back(*ItIndices);
				indicesTrain[label].remove(*ItIndices);
			}

			// count number of feature vectors for each set
			numberSamplesTrainClasswise[label] = 0;
			for (ItIndices = indicesTrain[label].begin(); ItIndices != indicesTrain[label].end(); ItIndices++)
			{
				if (pViewsPerObject == -1)
				{
					numberSamplesTrain += ItGlobalFeaturesMap->second[*ItIndices]->rows * factorSamplesTrainData;
					numberSamplesTrainClasswise[label] += ItGlobalFeaturesMap->second[*ItIndices]->rows * factorSamplesTrainData;
				}
				else
				{
					numberSamplesTrain += min(pViewsPerObject, int(ItGlobalFeaturesMap->second[*ItIndices]->rows * factorSamplesTrainData));
					numberSamplesTrainClasswise[label] += min(pViewsPerObject, int(ItGlobalFeaturesMap->second[*ItIndices]->rows * factorSamplesTrainData));
				}
			}
			numberSamplesValidationClasswise[label] = 0;
			for (ItIndices = indicesValidation[label].begin(); ItIndices != indicesValidation[label].end(); ItIndices++)
			{
				numberSamplesValidation += ItGlobalFeaturesMap->second[*ItIndices]->rows * (1.-factorSamplesTrainData);
				numberSamplesValidationClasswise[label] += ItGlobalFeaturesMap->second[*ItIndices]->rows * (1.-factorSamplesTrainData);
			}
		}

		// construct training data and label matrices
		cv::Mat TrainingFeatureMatrix(numberSamplesTrain, numberFeatures, CV_32FC1);	// contains all training data, suited for multiclass classifiers
		cv::Mat TrainingFeatureResponseMatrix(numberSamplesTrain, 1, CV_32SC1);
		std::map<std::string, cv::Mat> TrainingFeatureMatricesBinary;		// contains binary training data (class samples and non-class samples), suited for binary classifiers
		std::map<std::string, cv::Mat> TrainingFeatureResponseMatricesBinary;
		int insertPosition = 0;
		for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
		{
			std::string label = ItGlobalFeaturesMap->first;
			int insertPositionBinary = 0;
			TrainingFeatureMatricesBinary[label] = cv::Mat(cvRound((double)numberSamplesTrainClasswise[label]*(1.0+pFactorIncorrect)), numberFeatures, CV_32FC1);
			TrainingFeatureResponseMatricesBinary[label] = cv::Mat(TrainingFeatureMatricesBinary[label].rows, 1, CV_32FC1);
			// class samples
			for (ItIndices = indicesTrain[label].begin(); ItIndices != indicesTrain[label].end(); ItIndices++)
			{
				for (double dSample=0; dSample<ItGlobalFeaturesMap->second[*ItIndices]->rows*factorSamplesTrainData; (pViewsPerObject==-1) ? dSample+=1. : dSample+=max(1., (double)ItGlobalFeaturesMap->second[*ItIndices]->rows*factorSamplesTrainData/(double)pViewsPerObject))
				{
					int sample = (int)dSample;
					for (int j=0; j<numberFeatures; j++)
					{
						TrainingFeatureMatrix.at<float>(insertPosition, j) = (float)cvmGet(ItGlobalFeaturesMap->second[*ItIndices], sample, j);
						TrainingFeatureMatricesBinary[label].at<float>(insertPositionBinary, j) = (float)cvmGet(ItGlobalFeaturesMap->second[*ItIndices], sample, j);
					}
					TrainingFeatureResponseMatrix.at<int>(insertPosition, 0) = objectClassNumberMapping[label];
					TrainingFeatureResponseMatricesBinary[label].at<float>(insertPositionBinary, 0) = 1.0;
					insertPosition++;
					insertPositionBinary++;
				}
			}
			// non-class samples for the matrices suited for binary classifiers
			while (insertPositionBinary < TrainingFeatureMatricesBinary[label].rows)
			{
				// pick random class
				int index =	int(mData.mGlobalFeaturesMap.size()*((double)rand()/((double)RAND_MAX+1.0)));
				ItGlobalFeaturesMap2 = mData.mGlobalFeaturesMap.begin();
				for (int k=0; k<index; k++, ItGlobalFeaturesMap2++);
				std::string labelIncorrect = ItGlobalFeaturesMap2->first;
				if (labelIncorrect == label) continue;		// do not pick samples from correct class
				
				// pick random training object
				index = int(indicesTrain[labelIncorrect].size()*((double)rand()/((double)RAND_MAX+1.0)));
				ItIndices = indicesTrain[labelIncorrect].begin();
				for (int k=0; k<index; k++, ItIndices++);

				// pick random sample from that object
				if (pViewsPerObject == -1)
					index = int((ItGlobalFeaturesMap2->second[*ItIndices]->rows*factorSamplesTrainData)*((double)rand()/((double)RAND_MAX+1.0)));
				else
				{
					double step=(double)ItGlobalFeaturesMap2->second[*ItIndices]->rows*factorSamplesTrainData/(double)pViewsPerObject;
					do
					{
						index = int((ItGlobalFeaturesMap2->second[*ItIndices]->rows*factorSamplesTrainData)*((double)rand()/((double)RAND_MAX+1.0)));
					} while (index != int(step * int((double)index/step)));
				}
				for (int j=0; j<numberFeatures; j++)
					TrainingFeatureMatricesBinary[label].at<float>(insertPositionBinary, j) = (float)cvmGet(ItGlobalFeaturesMap2->second[*ItIndices], index, j);
				TrainingFeatureResponseMatricesBinary[label].at<float>(insertPositionBinary, 0) = 0.0;
				insertPositionBinary++;
			}
		}

		
		if (pClassifierType == CLASSIFIER_KNN)
		{
			// train KNN multi classifier
			CvKNearest* KNN = new CvKNearest;
			CvMat trainMat = (CvMat)TrainingFeatureMatrix;
			CvMat labelMat = (CvMat)TrainingFeatureResponseMatrix;
			bool trainResult = KNN->train(&trainMat, &labelMat, 0, false);
			if (!trainResult) std::cout << "Training Multiclass failed." << std::endl;

			// validate KNN multi classifier
			//statistics
			for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
			{
				std::string label = ItGlobalFeaturesMap->first;
				for (ItIndices = indicesValidation[label].begin(); ItIndices != indicesValidation[label].end(); ItIndices++)
				{
					for (int sample=ItGlobalFeaturesMap->second[*ItIndices]->rows*factorSamplesTrainData; sample<ItGlobalFeaturesMap->second[*ItIndices]->rows; sample++)
					{
						CvMat* featureVector = cvCreateMat(1, ItGlobalFeaturesMap->second[*ItIndices]->cols, ItGlobalFeaturesMap->second[*ItIndices]->type);
						cvGetRow(ItGlobalFeaturesMap->second[*ItIndices], featureVector, sample);

						// validate multi classifier
						int k=1;
						float result = KNN->find_nearest(featureVector, k);
						if (result == (float)objectClassNumberMapping[label])
						{
							// correct classification
							multiclassStatistics[label]["tp"]++;
							singleFoldMulticlassStatistics[fold][label]["tp"]++;
						}
						else
						{
							// false classification
							multiclassStatistics[label]["fn"]++;
							multiclassStatistics[numberObjectClassMapping[(int)result]]["fp"]++;
							singleFoldMulticlassStatistics[fold][label]["fn"]++;
							singleFoldMulticlassStatistics[fold][numberObjectClassMapping[(int)result]]["fp"]++;
						}
						
						individualResults[label][*ItIndices].push_back(numberObjectClassMapping[(int)result]);

						cvReleaseMat(&featureVector);
					}
				}
			}
		}
		else
		{
			CvSVM* SVM = new CvSVM;
			if (pClassifierType == CLASSIFIER_SVM)
			{
				// train multi classifier
				CvSVMParams SVMParams = CvSVMParams(CvSVM::NU_SVC, CvSVM::RBF, 0, 0.007, 0, 1.0, 0.09, 0, 0, cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 2500, 0.0001));
				CvMat trainMat = (CvMat)TrainingFeatureMatrix;
				CvMat labelMat = (CvMat)TrainingFeatureResponseMatrix;
				//bool trainResult = SVM->train(&trainMat, &labelMat, 0, 0, SVMParams);
				CvParamGrid cGrid(0, 1, 0);
				CvParamGrid gammaGrid(0.00021875, 5.0, 2.0);//CvParamGrid gammaGrid(0.00021875, 5.0, 2.0);  //CvParamGrid gammaGrid(0.000875, 1.0, 2.0);
				CvParamGrid pGrid(0, 1, 0);
				CvParamGrid nuGrid(0.01125, 0.3, 2.0); //CvParamGrid nuGrid(0.01125, 0.3, 2.0);
				CvParamGrid coeffGrid(0, 1, 0);
				CvParamGrid degreeGrid(0, 1, 0);
				bool trainResult = SVM->train_auto(&trainMat, &labelMat, 0, 0, SVMParams, 10, cGrid, gammaGrid, pGrid, nuGrid, coeffGrid, degreeGrid);
				CvSVMParams optimalParams = SVM->get_params();
				std::cout << "\nOptimal params: gamma=" << optimalParams.gamma << "  nu=" << optimalParams.nu << "  C=" << optimalParams.C << "  p=" << optimalParams.p << "  coeff=" << optimalParams.coef0 << "  degree=" << optimalParams.degree << std::endl;
				*pScreenLogFile << "\nOptimal params: gamma=" << optimalParams.gamma << "  nu=" << optimalParams.nu << "  C=" << optimalParams.C << "  p=" << optimalParams.p << "  coeff=" << optimalParams.coef0 << "  degree=" << optimalParams.degree << std::endl;
				if (trainResult) std::cout << "Training Multiclass finished successfully." << std::endl;
				else std::cout << "Training Multiclass failed." << std::endl;
			}
			else
			{
				// train binary classifiers
				for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
				{
					std::string label = ItGlobalFeaturesMap->first;
					CvMat trainMatBinary = TrainingFeatureMatricesBinary[label];
					CvMat labelMatBinary = TrainingFeatureResponseMatricesBinary[label];
					TrainGlobal(pClassifierType, label, &trainMatBinary, &labelMatBinary);
				}
			}

			// validate multi classifier and binary classifiers
			//compute marginals p(o_k) for output o_k, assuming p(c_i) uniformly distributed
			std::map<std::string, double> p_ok;
			for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
			{
				std::string outputLabel = ItGlobalFeaturesMap->first;
				p_ok[outputLabel] = 0.0;
				for (ItGlobalFeaturesMap2 = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap2 != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap2++)
				{
					std::string groundTruthLabel = ItGlobalFeaturesMap2->first;
					p_ok[outputLabel] += mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel];
				}
				p_ok[outputLabel] /= (double)mData.mGlobalFeaturesMap.size();
			}
			//statistics
			for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
			{
				std::string label = ItGlobalFeaturesMap->first;
				for (ItIndices = indicesValidation[label].begin(); ItIndices != indicesValidation[label].end(); ItIndices++)
				{
					for (int sample=ItGlobalFeaturesMap->second[*ItIndices]->rows*factorSamplesTrainData; sample<ItGlobalFeaturesMap->second[*ItIndices]->rows; sample++)
					{
						CvMat* featureVector = cvCreateMat(1, ItGlobalFeaturesMap->second[*ItIndices]->cols, ItGlobalFeaturesMap->second[*ItIndices]->type);
						cvGetRow(ItGlobalFeaturesMap->second[*ItIndices], featureVector, sample);

						if (pClassifierType == CLASSIFIER_SVM)
						{
							// validate multi classifier
							float result = SVM->predict(featureVector);
							if (result == (float)objectClassNumberMapping[label])
							{
								// correct classification
								multiclassStatistics[label]["tp"]++;
								singleFoldMulticlassStatistics[fold][label]["tp"]++;
							}
							else
							{
								// false classification
								multiclassStatistics[label]["fn"]++;
								multiclassStatistics[numberObjectClassMapping[(int)result]]["fp"]++;
								singleFoldMulticlassStatistics[fold][label]["fn"]++;
								singleFoldMulticlassStatistics[fold][numberObjectClassMapping[(int)result]]["fp"]++;
							}

							individualResults[label][*ItIndices].push_back(numberObjectClassMapping[(int)result]);
						}
						else if (pClassifierType == CLASSIFIER_RTC)
						{
							// validate binary classifiers
							std::map<std::string, double> classProbabilities;	// outputs p(o_k|x) of the different binary classifiers given the sample x
							double maxAPrioriProbability = -1.0;
							std::string maxAPrioriLabel = "";
							for (GlobalClassifierMap::iterator ItGlobalClassifierMap=mData.mGlobalClassifierMap.begin(); ItGlobalClassifierMap!=mData.mGlobalClassifierMap.end(); ItGlobalClassifierMap++)
							{
								double prediction = 0.0, th = mData.mGlobalClassifierThresholdMap[ItGlobalClassifierMap->first];
								PredictGlobal(pClassifierType, ItGlobalClassifierMap->first, featureVector, prediction);
								double mappedPrediction = 0;
								if (prediction>=th) mappedPrediction = 2*(prediction-th)/(1.0-th);
								else mappedPrediction = 2*(prediction-th)/th;
								classProbabilities[ItGlobalClassifierMap->first] = exp(mappedPrediction)/(exp(mappedPrediction)+exp(-mappedPrediction));
								//double temp = exp(mappedPrediction);

								if (classProbabilities[ItGlobalClassifierMap->first] > maxAPrioriProbability)
								{
									maxAPrioriProbability = classProbabilities[ItGlobalClassifierMap->first];
									maxAPrioriLabel = ItGlobalClassifierMap->first;
								}
							}
							// max a posteriori label
							std::map<std::string, double> p_ci_x;	// probability distribution for the actual object class given measurement x
							std::map<std::string, double>::iterator ItGroundTruthClass, ItOutputClass;
							for (ItGroundTruthClass = classProbabilities.begin(); ItGroundTruthClass != classProbabilities.end(); ItGroundTruthClass++)		// heuristic approach, light/fast probabilistic approach
							{
								std::string groundTruthLabel = ItGroundTruthClass->first;
								p_ci_x[groundTruthLabel] = 0.0;
								for (ItOutputClass = classProbabilities.begin(); ItOutputClass != classProbabilities.end(); ItOutputClass++)
								{
									std::string outputLabel = ItOutputClass->first;
									p_ci_x[groundTruthLabel] += mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel]/(p_ok[outputLabel]*classProbabilities.size()) * classProbabilities[outputLabel];
								}
							}
							//for (ItGroundTruthClass = classProbabilities.begin(); ItGroundTruthClass != classProbabilities.end(); ItGroundTruthClass++)
							//	p_ci_x[ItGroundTruthClass->first] = 0.0;
							//unsigned long numberCombinations = 1<<classProbabilities.size();
							//for (unsigned long outputCombination = 0; outputCombination < numberCombinations; outputCombination++)
							//{
							//	double summandCommonPart=1;
							//	int classShift=0;
							//	for (ItOutputClass = classProbabilities.begin(); ItOutputClass != classProbabilities.end(); ItOutputClass++, classShift++)
							//	{
							//		std::string outputLabel = ItOutputClass->first;
							//		bool outputState = ((outputCombination>>classShift) & 0x01) == 1;	// if true: p(o_k=1) is considered, else p(o_k=0)
							//		if (outputState == true)
							//			summandCommonPart *= classProbabilities[outputLabel] / p_ok[outputLabel];
							//		else
							//			summandCommonPart *= (1.-classProbabilities[outputLabel]) / (1.-p_ok[outputLabel]);
							//	}
							//	summandCommonPart /= classProbabilities.size();	// uniform ground truth class prior

							//	for (ItGroundTruthClass = classProbabilities.begin(); ItGroundTruthClass != classProbabilities.end(); ItGroundTruthClass++)
							//	{
							//		std::string groundTruthLabel = ItGroundTruthClass->first;
							//		double summandGroundTruthClassSpecific=1;
							//		int classShift=0;
							//		for (ItOutputClass = classProbabilities.begin(); ItOutputClass != classProbabilities.end(); ItOutputClass++, classShift++)
							//		{
							//			std::string outputLabel = ItOutputClass->first;
							//			bool outputState = ((outputCombination>>classShift) & 0x01) == 1;	// if true: p(o_k=1) is considered, else p(o_k=0)
							//			if (outputState == true)
							//				summandGroundTruthClassSpecific *= mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel];
							//			else
							//				summandGroundTruthClassSpecific *= (1.-mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel]);
							//		}
							//		p_ci_x[groundTruthLabel] += summandGroundTruthClassSpecific * summandCommonPart;
							//	}
							//}

							double maxAPosterioriProbability = -1.0;
							std::string maxAPosterioriLabel = "";
							for (ItGroundTruthClass = classProbabilities.begin(); ItGroundTruthClass != classProbabilities.end(); ItGroundTruthClass++)
							{
								std::string groundTruthLabel = ItGroundTruthClass->first;
								if (p_ci_x[groundTruthLabel] > maxAPosterioriProbability)
								{
									maxAPosterioriProbability = p_ci_x[groundTruthLabel];
									maxAPosterioriLabel = groundTruthLabel;
								}
							}

							std::string maxLabel = maxAPosterioriLabel;		// or: maxAPrioriLabel
							if (maxLabel == label)
							{
								// correct classification
								multiclassStatisticsBinary[label]["tp"]++;
								singleFoldMulticlassStatisticsBinary[fold][label]["tp"]++;
							}
							else
							{
								// false classification
								multiclassStatisticsBinary[label]["fn"]++;
								multiclassStatisticsBinary[maxLabel]["fp"]++;
								singleFoldMulticlassStatisticsBinary[fold][label]["fn"]++;
								singleFoldMulticlassStatisticsBinary[fold][maxLabel]["fp"]++;
							}

							individualResults[label][*ItIndices].push_back(maxLabel);
						}

						cvReleaseMat(&featureVector);
					}
				}
			}
			SVM->clear();
			delete SVM;
		}

		// write back the validation indices into the train index set for objects from class
		for (ItGlobalFeaturesMap = mData.mGlobalFeaturesMap.begin(); ItGlobalFeaturesMap != mData.mGlobalFeaturesMap.end(); ItGlobalFeaturesMap++)
		{
			std::string label = ItGlobalFeaturesMap->first;
			for (ItIndices = indicesValidation[label].begin(); ItIndices != indicesValidation[label].end(); ItIndices = indicesValidation[label].begin())
			{
				indicesTrain[label].push_back(*ItIndices);
				indicesValidation[label].remove(*ItIndices);
			}
			indicesTrain[label].sort();
		}
		std::cout << ".";
		if (pScreenLogFile) *pScreenLogFile << ".";
	}
	std::cout << std::endl;
	if (pScreenLogFile) *pScreenLogFile << std::endl;
	
	/// Test classifier with the unseen test set.
	if (numberObjectsTest.begin()->second > 0)
	{
		// train classifier with whole train data
		//TrainGlobal(pClassifierType, pClass, NumberSamplesCorrect-NumberSamplesCorrectTest+NumberSamplesIncorrect-NumberSamplesIncorrectTest, NumberFeatures, ItGlobalFeaturesClass, &IndicesTrainCorrect, &IndicesTrainIncorrect, NegativeSamplesMatrix);

		// test classifier
		//ValidateGlobal(pClassifierType, pClass, NumberFeatures, ItGlobalFeaturesClass, &IndicesTestCorrect, &IndicesTestIncorrect, NegativeSamplesMatrix, pTestSetOutput);
	}
	// variable importance
	//if (*pVariableImportance != NULL) GetVariableImportance(mData.mGlobalClassifierMap.find(pClass)->second, pClassifierType, pVariableImportance);


/*	std::cout << std::endl << "Multiclass statistics:" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
	double avgRecall=0.0, avgPrecision=0.0;
	for (std::map<std::string, std::map<std::string, int> >::iterator ItStatistics = multiclassStatistics.begin(); ItStatistics != multiclassStatistics.end(); ItStatistics++)
	{
		double recall = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fn"]);
		double precision = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fp"]);
		avgRecall += recall;
		avgPrecision += precision;
		std::cout << ItStatistics->first << (ItStatistics->first.length()<8 ? "\t" : "") << "\t" << ItStatistics->second["tp"] << "\t" << ItStatistics->second["fn"] << "\t" << ItStatistics->second["fp"] << "\t" << recall << "\t" << precision << std::endl;
	}
	std::cout << "\t\t\t\t\t" << avgRecall/(double)multiclassStatistics.size() << "\t" << avgPrecision/(double)multiclassStatistics.size() << std::endl;
*/

	if (pClassifierType == CLASSIFIER_KNN || pClassifierType == CLASSIFIER_SVM)
	{
		// results from each fold
		std::cout << std::endl << "Multiclass statistics from multiclass classifiers (for each of " << pFold << " folds):" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl << "Multiclass statistics from multiclass classifiers (for each of " << pFold << " folds):" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		for (int fold=0; fold<pFold; fold++)
		{
			for (std::map<std::string, std::map<std::string, int> >::iterator ItStatistics = singleFoldMulticlassStatistics[fold].begin(); ItStatistics != singleFoldMulticlassStatistics[fold].end(); ItStatistics++)
			{
				double recall = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fn"]);
				double precision = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fp"]);
				std::cout << ItStatistics->first << (ItStatistics->first.length()<8 ? "\t" : "") << "\t" << ItStatistics->second["tp"] << "\t" << ItStatistics->second["fn"] << "\t" << ItStatistics->second["fp"] << "\t" << recall << "\t" << precision << std::endl;
				if (pScreenLogFile) *pScreenLogFile << ItStatistics->first << (ItStatistics->first.length()<8 ? "" : "") << " " << ItStatistics->second["tp"] << " " << ItStatistics->second["fn"] << " " << ItStatistics->second["fp"] << " " << recall << " " << precision << std::endl;
			}
		}

		// average from all folds
		std::cout << std::endl << "Multiclass statistics from multiclass classifiers:" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl << "Multiclass statistics from multiclass classifiers:" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		double avgRecall=0.0, avgPrecision=0.0;
		for (std::map<std::string, std::map<std::string, int> >::iterator ItStatistics = multiclassStatistics.begin(); ItStatistics != multiclassStatistics.end(); ItStatistics++)
		{
			double recall = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fn"]);
			double precision = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fp"]);
			avgRecall += recall;
			avgPrecision += precision;
			std::cout << ItStatistics->first << (ItStatistics->first.length()<8 ? "\t" : "") << "\t" << ItStatistics->second["tp"] << "\t" << ItStatistics->second["fn"] << "\t" << ItStatistics->second["fp"] << "\t" << recall << "\t" << precision << std::endl;
			if (pScreenLogFile) *pScreenLogFile << ItStatistics->first << (ItStatistics->first.length()<8 ? "" : "") << " " << ItStatistics->second["tp"] << " " << ItStatistics->second["fn"] << " " << ItStatistics->second["fp"] << " " << recall << " " << precision << std::endl;
		}
		std::cout << "\t\t\t\t\t" << avgRecall/(double)multiclassStatistics.size() << "\t" << avgPrecision/(double)multiclassStatistics.size() << std::endl;
		if (pScreenLogFile) *pScreenLogFile << " " << avgRecall/(double)multiclassStatistics.size() << " " << avgPrecision/(double)multiclassStatistics.size() << std::endl;
	}
	else
	{
		// results from each fold
		std::cout << std::endl << "Multiclass statistics from binary classifiers (for each of " << pFold << " folds):" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl << "Multiclass statistics from binary classifiers (for each of " << pFold << " folds):" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		for (int fold=0; fold<pFold; fold++)
		{
			for (std::map<std::string, std::map<std::string, int> >::iterator ItStatistics = singleFoldMulticlassStatisticsBinary[fold].begin(); ItStatistics != singleFoldMulticlassStatisticsBinary[fold].end(); ItStatistics++)
			{
				double recall = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fn"]);
				double precision = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fp"]);
				std::cout << ItStatistics->first << (ItStatistics->first.length()<8 ? "\t" : "") << "\t" << ItStatistics->second["tp"] << "\t" << ItStatistics->second["fn"] << "\t" << ItStatistics->second["fp"] << "\t" << recall << "\t" << precision << std::endl;
				if (pScreenLogFile) *pScreenLogFile << ItStatistics->first << (ItStatistics->first.length()<8 ? "" : "") << " " << ItStatistics->second["tp"] << " " << ItStatistics->second["fn"] << " " << ItStatistics->second["fp"] << " " << recall << " " << precision << std::endl;
			}
		}

		// average from all folds
		std::cout << std::endl << "Multiclass statistics from binary classifiers:" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl << "Multiclass statistics from binary classifiers:" << std::endl << "\t\ttp\tfn\tfp\trecall\tprecision" << std::endl;
		double avgRecall=0.0, avgPrecision=0.0;
		for (std::map<std::string, std::map<std::string, int> >::iterator ItStatistics = multiclassStatisticsBinary.begin(); ItStatistics != multiclassStatisticsBinary.end(); ItStatistics++)
		{
			double recall = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fn"]);
			double precision = (double)ItStatistics->second["tp"]/((double)ItStatistics->second["tp"]+ItStatistics->second["fp"]);
			avgRecall += recall;
			avgPrecision += precision;
			std::cout << ItStatistics->first << (ItStatistics->first.length()<8 ? "\t" : "") << "\t" << ItStatistics->second["tp"] << "\t" << ItStatistics->second["fn"] << "\t" << ItStatistics->second["fp"] << "\t" << recall << "\t" << precision << std::endl;
			if (pScreenLogFile) *pScreenLogFile << ItStatistics->first << (ItStatistics->first.length()<8 ? "" : "") << " " << ItStatistics->second["tp"] << " " << ItStatistics->second["fn"] << " " << ItStatistics->second["fp"] << " " << recall << " " << precision << std::endl;
		}
		std::cout << "\t\t\t\t\t" << avgRecall/(double)multiclassStatisticsBinary.size() << "\t" << avgPrecision/(double)multiclassStatisticsBinary.size() << std::endl;
		if (pScreenLogFile) *pScreenLogFile << " " << avgRecall/(double)multiclassStatisticsBinary.size() << " " << avgPrecision/(double)multiclassStatisticsBinary.size() << std::endl;
	}

	std::cout << "\n\nClasswise errors per view:" << std::endl;
	if (pScreenLogFile) *pScreenLogFile << "\n\nClasswise errors per view:" << std::endl;
	std::string pathFile = pStatisticsPath + "individualResults.txt";
	std::ofstream fout(pathFile.c_str(), std::fstream::out);
	std::map<std::string, std::vector<int> > classwiseViewErrors;
	for (std::map<std::string, std::map< int, std::vector< std::string > > >::iterator itClasses=individualResults.begin(); itClasses!=individualResults.end(); itClasses++)
	{
		std::string label=itClasses->first;
		int numberViewsPerObject = 36;
		classwiseViewErrors[label].resize(numberViewsPerObject, 0);		// todo: this is database-dependent on the views per object
		fout << label;

		for (std::map< int, std::vector< std::string > >::iterator itObjects=itClasses->second.begin(); itObjects!=itClasses->second.end(); itObjects++)
		{
			for (int view = 0; view<(int)itObjects->second.size(); view++)
			{
				if (view%numberViewsPerObject == 0) fout << std::endl << itObjects->first << "\t";
				if (itObjects->second[view] != label)
				{
					classwiseViewErrors[label][view%numberViewsPerObject]++;
				}
				fout << itObjects->second[view] << "\t";
			}
		}
		fout << std::endl;

		std::cout << label << std::endl;
		if (pScreenLogFile) *pScreenLogFile << label << std::endl;
		for (int view=0; view<(int)classwiseViewErrors[label].size(); view++)
		{
			std::cout << ((view<10)? "  " : " ") << view << "  ";
			if (pScreenLogFile) *pScreenLogFile << ((view<10)? "  " : " ") << view << "  ";
		}
		std::cout << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl;
		for (int view=0; view<(int)classwiseViewErrors[label].size(); view++)
		{
			std::cout << ((classwiseViewErrors[label][view]<10)? "  " : " ") << classwiseViewErrors[label][view] << "  ";
			if (pScreenLogFile) *pScreenLogFile << ((classwiseViewErrors[label][view]<10)? "  " : " ") << classwiseViewErrors[label][view] << "  ";
		}
		std::cout << std::endl;
		if (pScreenLogFile) *pScreenLogFile << std::endl;
	}
	fout.close();

	return ipa_utils::RET_OK;
}

///////////////////////////////////////////////////////////////////////////// ----


void ObjectClassifier::StatisticsFromOutput(ClassifierOutputCollection pResponses, ClassifierPerformanceStruct& pPerformance, double pThreshold)
{
	// positive samples
	for (unsigned int i=0; i<pResponses.positiveSampleResponses.size(); i++)
	{
		if (pResponses.positiveSampleResponses[i] >= pThreshold) pPerformance.RightPositives++;
		else pPerformance.WrongNegatives++;
	}
	
	// negative samples
	for (unsigned int i=0; i<pResponses.negativeSampleResponses.size(); i++)
	{
		if (pResponses.negativeSampleResponses[i] >= pThreshold) pPerformance.WrongPositives++;
		else pPerformance.RightNegatives++;
	}
}


struct ClassifiedBlob{double DegreeOfMembership; BlobFeatureRiB Blob;};
bool CompareBlobs(ClassifiedBlob Blob1, ClassifiedBlob Blob2)
{
	if (Blob1.DegreeOfMembership <= Blob2.DegreeOfMembership) return false;
	return true;
}
bool CompareDistances(ClassifiedBlob Blob1, ClassifiedBlob Blob2)
{
	if (Blob1.DegreeOfMembership >= Blob2.DegreeOfMembership) return false;
	return true;
}

int ObjectClassifier::CategorizeObject(SharedImage* pSourceImage, std::map<std::string, double>& pResults, std::map<double, std::string>& pResultsOrdered, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams)
{
	/// create a pseudo blob
	BlobFeatureRiB Blob;
	BlobListRiB Blobs;
	ipa_utils::IntVector Keys;

	Blob.m_x = 0;
	Blob.m_y = 0;
	Blob.m_r = 2;
	Blob.m_Phi = 0;
	Blob.m_Res = 0;
	Blob.m_Id = 1;
	for (unsigned int j=0; j<64; j++) Blob.m_D.push_back(1);
	Blob.m_D.push_back(1);		// min/max curvature approximate
	Blob.m_D.push_back(1);
	for (int i=0; i<10; i++) Blobs.push_back(Blob);

	// Use Mask for global feature extraction
	CvMat** featureVector = new CvMat*;
	*featureVector = NULL;
	IplImage* mask = cvCreateImage(cvGetSize(pSourceImage->Shared()), pSourceImage->Shared()->depth, 1);
	cvCvtColor(pSourceImage->Shared(), mask, CV_RGB2GRAY);
	ExtractGlobalFeatures(&Blobs, featureVector, pClusterMode, pGlobalFeatureParams, INVALID, pSourceImage->Coord(), mask, NULL, false, "common/files/timing.txt");
	cvReleaseImage(&mask);
		
	Timer tim;
	tim.start();

	// classify descriptor
	// validate multi classifier and binary classifiers
	//compute marginals p(o_k) for output o_k, assuming p(c_i) uniformly distributed
	ClassifierAccuracy::iterator itClasses, itClasses2;
	std::map<std::string, double> p_ok;
	for (itClasses = mData.mGlobalClassifierAccuracy.begin(); itClasses != mData.mGlobalClassifierAccuracy.end(); itClasses++)
	{
		std::string outputLabel = itClasses->first;
		p_ok[outputLabel] = 0.0;
		for (itClasses2 = mData.mGlobalClassifierAccuracy.begin(); itClasses2 != mData.mGlobalClassifierAccuracy.end(); itClasses2++)
		{
			std::string groundTruthLabel = itClasses2->first;
			p_ok[outputLabel] += mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel];
		}
		p_ok[outputLabel] /= (double)mData.mGlobalClassifierAccuracy.size();

		//std::cout << "p_ok: " << p_ok[outputLabel] << std::endl;
	}
	// validate binary classifiers
	/*if (pClassifierTypeGlobal == CLASSIFIER_SVM)
	{
		// validate multi classifier
		float result = SVM->predict(*featureVector);
		std::cout << "SVM result: " << result << std::endl;
		//if (result == (float)objectClassNumberMapping[label])
	}
	else*/ if (pClassifierTypeGlobal == CLASSIFIER_RTC)
	{
		// validate binary classifiers
		std::map<std::string, double> classProbabilities;	// outputs p(o_k|x) of the different binary classifiers given the sample x
		double maxAPrioriProbability = -1.0;
		std::string maxAPrioriLabel = "";
		for (GlobalClassifierMap::iterator ItGlobalClassifierMap=mData.mGlobalClassifierMap.begin(); ItGlobalClassifierMap!=mData.mGlobalClassifierMap.end(); ItGlobalClassifierMap++)
		{
			double prediction = 0.0, th = mData.mGlobalClassifierThresholdMap[ItGlobalClassifierMap->first];
			PredictGlobal(pClassifierTypeGlobal, ItGlobalClassifierMap->first, *featureVector, prediction);
			double mappedPrediction = 0;
			if (prediction>=th) mappedPrediction = 2*(prediction-th)/(1.0-th);
			else mappedPrediction = 2*(prediction-th)/th;
			classProbabilities[ItGlobalClassifierMap->first] = exp(mappedPrediction)/(exp(mappedPrediction)+exp(-mappedPrediction));
						
			if (classProbabilities[ItGlobalClassifierMap->first] > maxAPrioriProbability)
			{
				maxAPrioriProbability = classProbabilities[ItGlobalClassifierMap->first];
				maxAPrioriLabel = ItGlobalClassifierMap->first;
			}
			//std::cout << "a priori: " << ItGlobalClassifierMap->first << "\t" << classProbabilities[ItGlobalClassifierMap->first] << std::endl;
		}
		// max a posteriori label
		std::map<std::string, double> p_ci_x;	// probability distribution for the actual object class given measurement x
		std::map<std::string, double>::iterator ItGroundTruthClass, ItOutputClass;
		double p_ci_x_sum = 0;
		for (ItGroundTruthClass = classProbabilities.begin(); ItGroundTruthClass != classProbabilities.end(); ItGroundTruthClass++)		// heuristic approach, light/fast probabilistic approach
		{
			std::string groundTruthLabel = ItGroundTruthClass->first;
			p_ci_x[groundTruthLabel] = 0.0;
			for (ItOutputClass = classProbabilities.begin(); ItOutputClass != classProbabilities.end(); ItOutputClass++)
			{
				std::string outputLabel = ItOutputClass->first;
				p_ci_x[groundTruthLabel] += mData.mGlobalClassifierAccuracy[outputLabel][groundTruthLabel]/(p_ok[outputLabel]*classProbabilities.size()) * classProbabilities[outputLabel];
			}
			p_ci_x_sum += p_ci_x[groundTruthLabel];
		}

		double maxAPosterioriProbability = -1.0;
		std::string maxAPosterioriLabel = "";
		for (ItGroundTruthClass = classProbabilities.begin(); ItGroundTruthClass != classProbabilities.end(); ItGroundTruthClass++)
		{
			std::string groundTruthLabel = ItGroundTruthClass->first;
			p_ci_x[groundTruthLabel] /= p_ci_x_sum;
			pResultsOrdered[p_ci_x[groundTruthLabel]] = groundTruthLabel;
			pResults[groundTruthLabel] = p_ci_x[groundTruthLabel];
			if (p_ci_x[groundTruthLabel] > maxAPosterioriProbability)
			{
				maxAPosterioriProbability = p_ci_x[groundTruthLabel];
				maxAPosterioriLabel = groundTruthLabel;
			}
			//std::cout << "a posteriori: " << groundTruthLabel << "\t" << p_ci_x[groundTruthLabel] << std::endl;
		}

		//for (std::map<double, std::string>::iterator it = pResultsOrdered.begin(); it != pResultsOrdered.end(); it++)
		//{
		//	std::cout << "a posteriori: " << it->second << "\t" << it->first << std::endl;
		//}

		std::cout << "\nmax a posteriori: " << maxAPosterioriLabel << "  (" << maxAPosterioriProbability << ")" << std::endl;

		//if (maxAPosterioriProbability > 0.3)
		//{
		//	std::stringstream text1a, text1b, text2a, text2b;
		//	std::map<double, std::string>::iterator it = pResultsOrdered.end();
		//	it--;
		//	text1a << it->second;
		//	text1b << setprecision(3) << 100*it->first << "%";
		//	it--;
		//	text2a << it->second;
		//	text2b << setprecision(3) << 100*it->first << "%";
		//	CvFont font;
		//	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1, 1);
		//	cvPutText(pSourceImage->Shared(), text1a.str().c_str(), cvPoint(10, 50), &font, CV_RGB(0, 255, 0));
		//	cvPutText(pSourceImage->Shared(), text1b.str().c_str(), cvPoint(320, 50), &font, CV_RGB(0, 255, 0));
		//	cvPutText(pSourceImage->Shared(), text2a.str().c_str(), cvPoint(10, 90), &font, CV_RGB(0, 255, 0));
		//	cvPutText(pSourceImage->Shared(), text2b.str().c_str(), cvPoint(320, 90), &font, CV_RGB(0, 255, 0));
		//}
	}

	//cvShowImage("color image", pSourceImage->Shared());
	//cvShowImage("range image", pSourceImage->Coord());
	//cv::waitKey(10);

	cvReleaseMat(featureVector);

	std::cout << "Classification time: " << tim.getElapsedTimeInMilliSec() << "ms.\n" << std::endl;
			
	return ipa_utils::RET_OK;
}

int ObjectClassifier::FindObject(SharedImage* pSourceImage, std::string pClass, ClusterMode pClusterMode, ClassifierType pClassifierTypeLocal, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams)
{
	/// Extract local features
	BlobListRiB Blobs;
	BlobListRiB::iterator ItBlobs;
	ExtractLocalFeatures(pSourceImage, Blobs, pClusterMode, MASK_NO);


	/// Classify local features into two classes: belongs to pClass and belongs not to pClass
	std::list<ClassifiedBlob> ClassifiedBlobs;
	std::list<ClassifiedBlob>::iterator ItClassifiedBlobs;
	// Use local classifiers to find the x best matching blobs
	for (ItBlobs = Blobs.begin(); ItBlobs != Blobs.end(); ItBlobs++)
	{	// build sorted list of classified blobs
		CvMat* LocalFeature = cvCreateMat(1, ItBlobs->m_D.size(), CV_32FC1);
		for (int j=0; j<(int)ItBlobs->m_D.size(); j++) cvSetReal1D(LocalFeature, j, ItBlobs->m_D[j]);
		ClassifiedBlob CBlob;
		PredictLocal(pClassifierTypeLocal, pClass, LocalFeature, CBlob.DegreeOfMembership);
		CBlob.Blob = *ItBlobs;		//geht kopieren so einfach?
		ClassifiedBlobs.push_back(CBlob);
		cvReleaseMat(&LocalFeature);
	}
	ClassifiedBlobs.sort(CompareBlobs);
	BlobListRiB InClassBlobs;
	int Counter = 0;
	int NumberBestBlobs = 20;
	for (ItClassifiedBlobs = ClassifiedBlobs.begin(); (ItClassifiedBlobs!=ClassifiedBlobs.end()) && (Counter<NumberBestBlobs); ItClassifiedBlobs++)
	{
		InClassBlobs.push_back(ItClassifiedBlobs->Blob);
		Counter++;
	}
	ClassifiedBlobs.clear();

	// draw best feature points in the picture
/*	IplImage* SaveImage = cvCloneImage((const IplImage*)pSourceImage->Shared());
	InClassBlobs.DrawListInIplImage(SaveImage, 0, true);
	cvSaveImage("BestLocalBlobs.png", SaveImage);
	cvReleaseImage(&SaveImage);
*/
	/// Use Blobs belonging to pClass in order to create a Mask for the most likely position(s) of the object(s)
	IplImage* Mask = cvCreateImage(cvGetSize(pSourceImage->Shared()), IPL_DEPTH_8U, 1);
	cvSetZero(Mask);
	// get an evaluation of the feature points with a measure directly proportional to the distance to other feature points
	// (most likely points have a small distance sum to all other points because they lie inside a likely cluster)
	BlobListRiB::iterator ItInClassBlobs, ItInClassBlobs2;
	for (ItInClassBlobs = InClassBlobs.begin(); ItInClassBlobs != InClassBlobs.end(); ItInClassBlobs++)
	{
		ClassifiedBlob ClassBlob;
		ClassBlob.Blob = *ItInClassBlobs;
		ClassBlob.DegreeOfMembership = 0.0;
		for (ItInClassBlobs2 = InClassBlobs.begin(); ItInClassBlobs2 != InClassBlobs.end(); ItInClassBlobs2++)
		{
			ipa_utils::Point3Dbl BlobPos1, BlobPos2;
			ItInClassBlobs->m_Frame.GetT(BlobPos1);
			ItInClassBlobs2->m_Frame.GetT(BlobPos2);
			ClassBlob.DegreeOfMembership += pow(BlobPos1.m_x-BlobPos2.m_x, 2) + pow(BlobPos1.m_y-BlobPos2.m_y, 2) + pow(BlobPos1.m_z-BlobPos2.m_z, 2);
		}
		ClassifiedBlobs.push_back(ClassBlob);
	}
	ClassifiedBlobs.sort(CompareDistances);
	InClassBlobs.clear();

	// find best fitting feature points in ClassifiedBlobs (those which distance sum to other feature points does not grow too large compared to the best distance)
	// and calculate their center of mass
	double BestDistance = ClassifiedBlobs.begin()->DegreeOfMembership;
	double CenterPosU=0, CenterPosV=0;
	Counter = 0;
	for (ItClassifiedBlobs = ClassifiedBlobs.begin(); ItClassifiedBlobs != ClassifiedBlobs.end(); ItClassifiedBlobs++)
	{
		//if (ItClassifiedBlobs->DegreeOfMembership < BestDistance*1.2) InClassBlobs.push_back(ItClassifiedBlobs->Blob);
		if (ItClassifiedBlobs->DegreeOfMembership < BestDistance*1.2)
		{
			CenterPosU += ItClassifiedBlobs->Blob.m_x;
			CenterPosV += ItClassifiedBlobs->Blob.m_y;
			Counter++;
			InClassBlobs.push_back(ItClassifiedBlobs->Blob);
		}
	}
	BlobListRiB RegionGrowingBlobs;
	BlobFeatureRiB AppendBlob;
	AppendBlob.m_x = cvRound(CenterPosU/(double)Counter);
	AppendBlob.m_y = cvRound(CenterPosV/(double)Counter);
	AppendBlob.m_r = 5;
	CvScalar CenterPos3D = cvGet2D(pSourceImage->Coord(), AppendBlob.m_y, AppendBlob.m_x);
	AppendBlob.m_Frame.SetTranslation(CenterPos3D.val[0], CenterPos3D.val[1], CenterPos3D.val[2]);
	RegionGrowingBlobs.push_back(AppendBlob);

	// draw best feature points in the picture
	IplImage* SaveImage = cvCloneImage((const IplImage*)pSourceImage->Shared());
	InClassBlobs.DrawListInIplImage(SaveImage, 0, true);
	cvCircle(SaveImage, cvPoint(ClassifiedBlobs.begin()->Blob.m_x, ClassifiedBlobs.begin()->Blob.m_y), 4, CV_RGB(0,255,0),3);
	cvSaveImage("BestLocalBlobs.png", SaveImage);
	cvReleaseImage(&SaveImage);

	// region growing from best feature points which captures all neighbors in near 3D distance or similar color
	double Max3DDistanceFromNeighbor = 10.0;
//	int MaxColorDistanceFromNeighbor = 2;
	Counter = 0;
	for (ItInClassBlobs = RegionGrowingBlobs.begin(); (ItInClassBlobs != RegionGrowingBlobs.end()) && (Counter<15000); ItInClassBlobs++, Counter++)
	{
		int u = ItInClassBlobs->m_x;
		int v = ItInClassBlobs->m_y;
		ipa_utils::Point3Dbl x, nx;
		ItInClassBlobs->m_Frame.GetT(x);		// central point coordinates
		cvSetReal2D(Mask, v, u, 255);
		for (int du=-1; du<2; du++)
		{
			if ((u+du)<0 || (u+du)>=Mask->width) continue;		// out of picture
			for (int dv=-1; dv<2; dv++)
			{
				if ((v+dv)<0 || (v+dv)>=Mask->height) continue;	// out of picture
				if (cvGetReal2D(Mask, v+dv, u+du) > 0) continue;	// already in mask
				// check whether new point should belong to Mask
				CvScalar nxCV = cvGet2D(pSourceImage->Coord(), v+dv, u+du);
				nx.m_x = nxCV.val[0];	nx.m_y = nxCV.val[1];	nx.m_z = nxCV.val[2];
				if (nx.GetDistance(x) < Max3DDistanceFromNeighbor)
				{	// near 3D distance -> take this neighbor for Mask
					cvSetReal2D(Mask, v+dv, u+du, 255);
					// use this point as another center point later
					AppendBlob.m_Frame.SetTranslation(nx);
					AppendBlob.m_x = u+du;
					AppendBlob.m_y = v+dv;
					RegionGrowingBlobs.push_back(AppendBlob);
				}
				else
				{
					if (false)
					{	// similar color -> take this neighbor for Mask
					}
				}
			}
		}
	}
	
	cvNamedWindow("Mask");
	while (cvWaitKey(10) != 'q') cvShowImage("Mask", Mask);
	cvDestroyWindow("Mask");


	/// Use Mask for global feature extraction
	CvMat** GlobalFeatures = new CvMat*;
	*GlobalFeatures = NULL;
	ExtractGlobalFeatures(&InClassBlobs, GlobalFeatures, pClusterMode, pGlobalFeatureParams, INVALID, pSourceImage->Coord(), Mask);


	/// Classify global features
	double PredictionResult = 0.0;
	PredictGlobal(pClassifierTypeGlobal, pClass, *GlobalFeatures, PredictionResult);
	if (mData.mGlobalClassifierThresholdMap.find(pClass) != mData.mGlobalClassifierThresholdMap.end())
	{
		/// Create return information about existence of object, position, etc.
		if (PredictionResult >= mData.mGlobalClassifierThresholdMap[pClass])
		{
			std::cout << "Object from class " << pClass << " is there.\n";
		}
		else
		{
			std::cout << "Object from class " << pClass << " was not found.\n";
		}
	}

	cvReleaseMat(GlobalFeatures);
	cvReleaseImage(&Mask);

	return ipa_utils::RET_OK;
}


int ObjectClassifier::GetVariableImportance(CvStatModel* pClassifier, int pClassifierType, const CvMat** pVariableImportance)
{
	switch (pClassifierType)
	{
		case CLASSIFIER_RTC:
			{
				CvRTrees* RTC = NULL;
				RTC = dynamic_cast<CvRTrees*>(pClassifier);
				*pVariableImportance = RTC->get_var_importance();
				break;
			}
		case CLASSIFIER_SVM:
			{
				std::cout << "ObjectClassifier::GetVariableImportance: This classifier does not support variable importance.\n";
				break;
			}
		case CLASSIFIER_BOOST:
			{
				std::cout << "ObjectClassifier::GetVariableImportance: This classifier does not support variable importance.\n";
				break;
			}
		case CLASSIFIER_KNN:
			{
				std::cout << "ObjectClassifier::GetVariableImportance: This classifier does not support variable importance.\n";
				break;
			}
		default:
			{
				std::cout << "ObjectClassifier::GetVariableImportance: Unknown classifier type.\n";
				return ipa_utils::RET_FAILED;
			}
	}
	return ipa_utils::RET_OK;
}


int ObjectClassifier::ExtractLocalFeatures(std::string pFileName, BlobListRiB& pBlobFeatures, ClusterMode pClusterMode, bool pVisualization)
{
	IplImage* SourceImage = NULL;

	if (!(SourceImage = cvLoadImage(pFileName.c_str(), 1))) return ipa_utils::RET_FAILED;

	ExtractLocalFeatures(SourceImage, pBlobFeatures, pClusterMode);

	// visualization
	if (pVisualization)
	{
		char c;
		cvNamedWindow(pFileName.c_str());
		while(1)
		{
			cvShowImage(pFileName.c_str(),SourceImage);
			c=cvWaitKey(10);
			if (c=='q')
			{
				cvDestroyAllWindows();
				break;
			}
		}
	}

	cvReleaseImage(&SourceImage);
	SourceImage = NULL;

	return ipa_utils::RET_OK;
}


int ObjectClassifier::ExtractLocalFeatures(IplImage* pSourceImage, BlobListRiB& pBlobFeatures, ClusterMode pClusterMode)
{
	/*SIFTDetector SiftDet;
	switch(pClusterMode)
	{
	case CLUSTER_8BIT:
		{
			SiftDet.Init(true, SIFT_INTVLS, SIFT_SIGMA, SIFT_CONTR_THR, SIFT_CURV_THR, SIFT_IMG_DBL, 2, 2, 1.0);
			break;
		}
	case CLUSTER_EM:
		{
			SiftDet.Init(true, SIFT_INTVLS, SIFT_SIGMA, SIFT_CONTR_THR, SIFT_CURV_THR, SIFT_IMG_DBL, 4, 8, 1.0);
			break;
		}
	default:
		std::cout << "ObjectClassifier::ExtractLocalFeatures: Invalid ClusterMode.\n";
		break;
	}

	SiftDet.DetectFeatures(pSourceImage, pBlobFeatures, AbstractBlobDetector::MODE_FAST);
	*/

	std::cout << "ObjectClassifier::ExtractLocalFeatures: WARNING: this function is obsolete and has no function anymore." << std::endl;

	return 0;
}


int ObjectClassifier::ExtractLocalFeatures(SharedImage* pSourceImage, BlobListRiB& pBlobFeatures, ClusterMode pClusterMode, MaskMode pMaskMode, std::string pMaskPath, Database pDatabase, std::string pClassName)
{
	int mode=1;	// 0=RGBI+channel of origin , 1=RGBI , 2=Intensity from color image , 3=HSVI+channel of origin, 4=HSVI

	std::vector<CvSURFPoint> SurfPoints;
	std::vector<CvSURFPoint>::iterator ItSurfPoints;
	ipa_utils::IntVector Ids;
	ipa_utils::DblMatrix Descriptors;
	DetectorCore SurfDet;
	IplImage** Mask = new IplImage*;
	*Mask = NULL;

	switch(pClusterMode)
	{
	case CLUSTER_8BIT:
		{
			std::cout << "ObjectClassifier::ExtractLocalFeatures: CLUSTER_8BIT not implemented for SURF.\n";
			return ipa_utils::RET_FAILED;
			break;
		}
	case CLUSTER_EM:
		{
			std::vector< std::vector<int> > supportPoints;
			supportPoints.resize(3, std::vector<int>(2));
			if (pDatabase == CIN)
			{
				supportPoints[0][0] = 90;	// x
				supportPoints[0][1] = 400;	// y
				supportPoints[1][0] = 440;
				supportPoints[1][1] = 350;
				supportPoints[2][0] = 122;	// should have a z-coordinate behind the object
				supportPoints[2][1] = 315;
			}
			else if (pDatabase == CIN2)
			{
				if (pClassName != "screens")
				{
					supportPoints[0][0] = 555;	// normal: 555; screen: 520	// x
					supportPoints[0][1] = 400;	// normal: 400; screen:	480 // y
					supportPoints[1][0] = 300;
					supportPoints[1][1] = 570;
					supportPoints[2][0] = 65;	// normal: 65, screen: 70	// should have a z-coordinate behind the object
					supportPoints[2][1] = 390;	// normal: 390, screen: 454
				}
				else
				{
					supportPoints[0][0] = 520;	// normal: 555; screen: 520	// x
					supportPoints[0][1] = 480;	// normal: 400; screen:	480 // y
					supportPoints[1][0] = 300;
					supportPoints[1][1] = 570;
					supportPoints[2][0] = 70;	// normal: 65, screen: 70	// should have a z-coordinate behind the object
					supportPoints[2][1] = 454;	// normal: 390, screen: 454
				}
			}
			else if (pDatabase == WASHINGTON)
			{
				supportPoints.clear();
			}
			else
			{
				std::cout << "Error: ExtractLocalFeatures: No valid database selected." << std::endl;
			}

			double hessianThreshold = 0.;
			if (pDatabase == CIN)
				hessianThreshold = 1250;
			else if (pDatabase == CIN2)
				hessianThreshold = 100;
			else if (pDatabase == WASHINGTON)
				hessianThreshold = 1250;

			// Get Feature points
			if (pMaskMode == MASK_NO)
				SurfDet.GetFeaturePoints(*pSourceImage, SurfPoints, Ids, &Descriptors, pMaskMode, 0, mode, supportPoints, "", hessianThreshold);
			else
			{
				if (pMaskMode == MASK_LOAD)
				{
					if (pDatabase == WASHINGTON)
					{
						*Mask = cvCreateImage(cvGetSize(pSourceImage->Shared()), IPL_DEPTH_8U, 1);
						IplImage* r = cvCreateImage(cvGetSize(pSourceImage->Shared()), IPL_DEPTH_8U, 1);
						IplImage* g = cvCreateImage(cvGetSize(pSourceImage->Shared()), IPL_DEPTH_8U, 1);
						IplImage* b = cvCreateImage(cvGetSize(pSourceImage->Shared()), IPL_DEPTH_8U, 1);
						cvSplit(pSourceImage->Shared(), r, g, b, 0);
						cvAdd(r,g,g);
						cvAdd(g,b,*Mask);
						cvReleaseImage(&r);
						cvReleaseImage(&g);
						cvReleaseImage(&b);
						cvThreshold(*Mask, *Mask, 0.1, 255, CV_THRESH_BINARY);
						//cvNamedWindow("mask");
						//cvShowImage("mask", *Mask);
						//cvWaitKey(10);
						//cvWaitKey();
					}
					else
					{
						*Mask = cvLoadImage((pMaskPath + "_Mask.png").c_str(), 0);
					}
					SurfDet.GetFeaturePoints(*pSourceImage, SurfPoints, Ids, &Descriptors, pMaskMode, Mask, mode, supportPoints, "", hessianThreshold);
					cvReleaseImage(Mask);
				}
				else
				{
					SurfDet.GetFeaturePoints(*pSourceImage, SurfPoints, Ids, &Descriptors, pMaskMode, Mask, mode, supportPoints, pClassName, hessianThreshold);
//					std::cout << "WARNING: NO mask images saved." << std::endl;
					cvSaveImage((pMaskPath + "_Mask.png").c_str(), *Mask);
					cvReleaseImage(Mask);
				}
			}
			break;
		}
	default:
		std::cout << "ObjectClassifier::ExtractLocalFeatures: Invalid ClusterMode.\n";
		break;
	}

	// convert to BlobList format
	int PointCounter=0;
	IplImage* Coordinates = cvCloneImage(pSourceImage->Coord());
	//cvSmooth(Coordinates, Coordinates);
	for (ItSurfPoints = SurfPoints.begin(); ItSurfPoints != SurfPoints.end(); ItSurfPoints++, PointCounter++)
	{
		BlobFeatureRiB Blob;
		ipa_utils::IntVector Keys;

		Blob.m_x = (int)ItSurfPoints->pt.x;
		Blob.m_y = (int)ItSurfPoints->pt.y;
		Blob.m_r = cvRound(ItSurfPoints->size/2);
		Blob.m_Phi = ItSurfPoints->dir/180 * CV_PI;
		Blob.m_Res = ItSurfPoints->hessian;
		Blob.m_Id = Ids[PointCounter];
		for (unsigned int j=0; j<Descriptors[PointCounter].size(); j++) Blob.m_D.push_back((const float)((Descriptors[PointCounter])[j]));
		
		// Get Frame for blob feature, if not succesful do not use this blob
		if (SurfDet.GetFeaturePointData(*pSourceImage, *ItSurfPoints, Ids[PointCounter], Blob.m_Frame, Keys, mode))
		{
			// calculate min and max deviation from tangential plane
			double MaxDeviation = -1.0;
			double MinDeviation = 1.0;
			ipa_utils::Point3Dbl CenterPoint;
			ipa_utils::Point3Dbl NormalDirection;
			Blob.m_Frame.GetT(CenterPoint);
			Blob.m_Frame.eZ(NormalDirection);
			int Radius = cvRound(Blob.m_r/2);
			if (Radius < 4) Radius = 4;
			for (int du=-Radius; du<=Radius; du++)
			{
				for (int dv=-Radius; dv<=Radius; dv++)
				{
					if (abs(du)<Radius && abs(dv)<Radius) continue;
					int u = Blob.m_x + du;
					int v = Blob.m_y + dv;
					if ((u<0) || (u>=pSourceImage->Coord()->width) || (v<0) || (v>=pSourceImage->Coord()->height)) continue;
					CvScalar OuterPointCV = cvGet2D(Coordinates, v, u);
					ipa_utils::Point3Dbl OuterPoint = ipa_utils::Point3Dbl(OuterPointCV.val[0], OuterPointCV.val[1], OuterPointCV.val[2]);
					ipa_utils::Point3Dbl Diff;
					Diff.SubVec(OuterPoint, CenterPoint);
					Diff.Normalize();

					// nun nur noch den cos(Schnittwinkel) bestimmen
					double Deviation = Diff.ScalarProd(NormalDirection);
					if (Deviation < MinDeviation) MinDeviation = Deviation;
					if (Deviation > MaxDeviation) MaxDeviation = Deviation;
				}
			}

			Blob.m_D.push_back((float)MinDeviation);		// min/max curvature approximate
			Blob.m_D.push_back((float)MaxDeviation);
			if (mode==0 || mode==3)
			{
				for (int i=0; i<Blob.m_Id; i++) Blob.m_D.push_back(0.f);
				Blob.m_D.push_back(1.f);
				for (int i=Blob.m_Id+1; i<4; i++) Blob.m_D.push_back(0.f);

			}
			pBlobFeatures.push_back(Blob);
		}
	}
	cvReleaseImage(&Coordinates);
	Descriptors.clear();

	return ipa_utils::RET_OK;
}


int ObjectClassifier::ExtractLocalRSDorFPFHFeatures(SharedImage* pSourceImage, BlobListRiB& pBlobFeatures, LocalFeatureParams& pLocalFeatureParams, MaskMode pMaskMode, std::string pMaskPath, Database pDatabase)
{
	IplImage** Mask = new IplImage*;
	*Mask = NULL;

	if (pMaskMode == MASK_NO)
	{
		std::cout << "ObjectClassifier::ExtractLocalRSDFeatures: Error: Cannot use no mask if local feature is rsd." << std::endl;
		return ipa_utils::RET_FAILED;
	}	
	else if (pMaskMode == MASK_LOAD)
	{
		if (pDatabase == WASHINGTON)
		{
			*Mask = cvCreateImage(cvGetSize(pSourceImage->Shared()), IPL_DEPTH_8U, 1);
			IplImage* r = cvCreateImage(cvGetSize(pSourceImage->Shared()), IPL_DEPTH_8U, 1);
			IplImage* g = cvCreateImage(cvGetSize(pSourceImage->Shared()), IPL_DEPTH_8U, 1);
			IplImage* b = cvCreateImage(cvGetSize(pSourceImage->Shared()), IPL_DEPTH_8U, 1);
			cvSplit(pSourceImage->Shared(), r, g, b, 0);
			cvAdd(r,g,g);
			cvAdd(g,b,*Mask);
			cvReleaseImage(&r);
			cvReleaseImage(&g);
			cvReleaseImage(&b);
			cvThreshold(*Mask, *Mask, 0.1, 255, CV_THRESH_BINARY);
			//cvNamedWindow("mask");
			//cvShowImage("mask", *Mask);
			//cvWaitKey(10);
			//cvWaitKey();
		}
		else
		{
			*Mask = cvLoadImage((pMaskPath + "_Mask.png").c_str(), 0);
		}

		
		// compute rsd or fpfh
		if ((pSourceImage->Coord()->width != (*Mask)->width) || (pSourceImage->Coord()->height != (*Mask)->height))
		{
			std::cout << "ObjectClassifier::ExtractLocalRSDFeatures: CoordinateImage and Mask do not have the same size." << std::endl;
			return ipa_utils::RET_FAILED;
		}
		//cvNamedWindow("mask before");
		//cvShowImage("mask before", pMask);
		//cvWaitKey(10);
		if (pDatabase == CIN)
			cvErode(*Mask, *Mask, 0, 8);	// necessary to avoid false depth pixels at object borders
		//cvNamedWindow("mask after");
		//cvShowImage("mask after", pMask);
		//cvWaitKey();
		//cvDestroyAllWindows();

		IplImage* CoordinateImage = cvCloneImage(pSourceImage->Coord()); //cvCreateImageHeader(cvGetSize(pCoordinateImage), pCoordinateImage->depth, pCoordinateImage->nChannels);
		cvSmooth(CoordinateImage, CoordinateImage, CV_GAUSSIAN, 5);

		// get 3D coordinates of points inside pMask and calculate object center (center of mass of pMask)
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclPoints (new pcl::PointCloud<pcl::PointXYZ>);
		double cx=0, cy=0, cz=0;
		for (int i=0; i<(*Mask)->height; i++)
		{
			for (int j=0; j<(*Mask)->width; j++)
			{
				if (cvGetReal2D(*Mask, i, j) != 0)
				{
					CvScalar pointScalar = cvGet2D(CoordinateImage, i, j);
					pcl::PointXYZ point;
					point.x = pointScalar.val[0];
					point.y = pointScalar.val[1];
					point.z = pointScalar.val[2];
					cx+=point.x;  cy+=point.y;  cz+=point.z;
					pclPoints->push_back(point);
				}
			}
		}
		cx /= (double)(pclPoints->size());
		cy /= (double)(pclPoints->size());
		cz /= (double)(pclPoints->size());

		//Timer tim;
		//tim.start();

		// normalize viewpoint
		double metricFactor = 1.0;
		if (pDatabase == CIN) metricFactor = 0.001;
		float maxX=0, maxY=0;
		for (int i=0; i<(int)pclPoints->size(); i++)
		{
			pclPoints->at(i).x = pclPoints->at(i).x*metricFactor - cx;
			pclPoints->at(i).y = pclPoints->at(i).y*metricFactor - cy;
			pclPoints->at(i).z = pclPoints->at(i).z*metricFactor - cz + 1.0;
			if (fabs(pclPoints->at(i).x) > maxX)
				maxX = fabs(pclPoints->at(i).x);
			if (fabs(pclPoints->at(i).y) > maxY)
				maxY = fabs(pclPoints->at(i).y);
		}

		// params
		double leafSize = 0.015f;
		double searchRadiusNormals = 0.03;
		double searchRadiusFeatures = 0.05;

		// Create the filtering object
		pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointsVoxelized(new pcl::PointCloud<pcl::PointXYZ>());
		pcl::VoxelGrid<pcl::PointXYZ> voxg;
		voxg.setInputCloud(pclPoints);
		if (maxX > 0.15 && maxY > 0.15)
		{
			leafSize = 0.025f;
			searchRadiusNormals = 0.05;
			searchRadiusFeatures = 0.08;
			std::cout << "<";
		}
		voxg.setLeafSize(leafSize, leafSize, leafSize);
		voxg.filter(*pclPointsVoxelized);

		// keep number of voxels small
		if (pclPointsVoxelized->size() > 1600)
		{
			leafSize = 0.035;
			searchRadiusNormals = 0.07;
			searchRadiusFeatures = 0.11;
			pclPointsVoxelized->clear();
			voxg.setLeafSize(leafSize, leafSize, leafSize);
			voxg.filter(*pclPointsVoxelized);
			std::cout << "!";
		}

		if (pclPointsVoxelized->size() > 1600)
		{
			std::cout << "\nObjectClassifier::ExtractLocalRSDorFPFHFeatures:Warning: pclPointsVoxelized is large with " << pclPointsVoxelized->size() << " points." << std::endl;
		}


		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(pclPointsVoxelized);

		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl_search<pcl::PointXYZ>::Ptr tree (new pcl_search<pcl::PointXYZ> ());
		ne.setSearchMethod(tree);

		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch(searchRadiusNormals);	//0.03

		// Compute the normals
		ne.compute(*normals);


		if (pLocalFeatureParams.useFeature.compare("rsd") == 0)
		{
			// Create the RSD estimation class, and pass the input dataset+normals to it
			pcl::RSDEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
			rsd.setInputCloud(pclPointsVoxelized);
			rsd.setInputNormals(normals);
			// alternatively, if cloud is of tpe PointNormal, do rsd.setInputNormals (cloud);

			// Create an empty kdtree representation, and pass it to the FPFH estimation object.
			// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
			pcl_search<pcl::PointXYZ>::Ptr rsdTree (new pcl_search<pcl::PointXYZ>());
			rsd.setSearchMethod(rsdTree);

			// Output datasets
			pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsds (new pcl::PointCloud<pcl::PrincipalRadiiRSD> ());

			// Use all neighbors in a sphere of radius 5cm
			// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
			rsd.setRadiusSearch(searchRadiusFeatures);

			// Compute the features
			rsd.compute(*rsds);


			// write descriptor into the descriptor vector
			for (int voxel=0; voxel<(int)rsds->size(); voxel++)
			{
				BlobFeature blob;
				blob.m_Frame.SetTranslation(pclPointsVoxelized->at(voxel).x, pclPointsVoxelized->at(voxel).y, pclPointsVoxelized->at(voxel).z);
				blob.m_Frame.SetRotation(normals->at(voxel).normal_x, normals->at(voxel).normal_y, normals->at(voxel).normal_z);
				blob.m_D.push_back(rsds->at(voxel).r_min);
				blob.m_D.push_back(rsds->at(voxel).r_max);
				pBlobFeatures.push_back(blob);
			}
		}
		else if (pLocalFeatureParams.useFeature.compare("fpfh") == 0)
		{
			// Create the RSD estimation class, and pass the input dataset+normals to it
			pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
			fpfh.setInputCloud(pclPointsVoxelized);
			fpfh.setInputNormals(normals);
			// alternatively, if cloud is of tpe PointNormal, do rsd.setInputNormals (cloud);

			// Create an empty kdtree representation, and pass it to the FPFH estimation object.
			// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
			pcl_search<pcl::PointXYZ>::Ptr fpfhTree (new pcl_search<pcl::PointXYZ>());
			fpfh.setSearchMethod(fpfhTree);

			// Output datasets
			pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

			// Use all neighbors in a sphere of radius 5cm
			// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
			fpfh.setRadiusSearch(searchRadiusFeatures);

			// Compute the features
			fpfh.compute(*fpfhs);


			// write descriptor into the descriptor vector
			for (int voxel=0; voxel<(int)fpfhs->size(); voxel++)
			{
				BlobFeature blob;
				blob.m_Frame.SetTranslation(pclPointsVoxelized->at(voxel).x, pclPointsVoxelized->at(voxel).y, pclPointsVoxelized->at(voxel).z);
				blob.m_Frame.SetRotation(normals->at(voxel).normal_x, normals->at(voxel).normal_y, normals->at(voxel).normal_z);
				for (int j=0; j<33; j++)
					blob.m_D.push_back(fpfhs->at(voxel).histogram[j]);
				pBlobFeatures.push_back(blob);
			}
		}
		else
		{
			std::cout << "ObjectClassifier::ExtractLocalRSDorFPFHFeatures: Error: Unknown descriptor type " << pLocalFeatureParams.useFeature << "." << std::endl;
			return ipa_utils::RET_FAILED;
		}

		cvReleaseImage(&CoordinateImage);
		cvReleaseImage(Mask);
	}
	else
	{
		std::cout << "ObjectClassifier::ExtractLocalRSDFeatures: Error: Cannot create masks if local feature is rsd." << std::endl;
		return ipa_utils::RET_FAILED;
	}

	if (pBlobFeatures.size() == 0)
	{
		std::cout << "ObjectClassifier::ExtractLocalRSDFeatures: Error: No rsd features were computed." << std::endl;
		return ipa_utils::RET_FAILED;
	}

	return ipa_utils::RET_OK;
}


struct Point2Dbl{double s; double z; Point2Dbl(double ps, double pz){s=ps; z=pz;}; };
int ObjectClassifier::ExtractGlobalFeatures(BlobListRiB* pBlobFeatures, CvMat** pGlobalFeatures, ClusterMode pClusterMode, GlobalFeatureParams& pGlobalFeatureParams, Database pDatabase, const IplImage* pCoordinateImage,
											IplImage* pMask, IplImage* pOutputImage, bool pFileOutput, std::string pTimingLogFileName, std::ofstream* pScreenLogFile)
{
	int NumberSamples = pBlobFeatures->size();
	if (NumberSamples == 0)
	{
		std::cout << "ObjectClassifier::ExtractGlobalFeatures: There are any blob features.\n";
		if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::ExtractGlobalFeatures: There are any blob features.\n";
		//return ipa_utils::RET_FAILED;
	}

	// Clear old data
	if (*pGlobalFeatures != NULL) cvReleaseMat(pGlobalFeatures);
	
	// make histogram with respective cluster method
	switch(pClusterMode)
	{
	case CLUSTER_8BIT:
		{
			int NumberFeatures = pBlobFeatures->begin()->m_D.size();
			*pGlobalFeatures = cvCreateMat(1,int(pow(2.0,(double)NumberFeatures)),CV_32FC1);
			cvSetZero(*pGlobalFeatures);

			BlobListRiB::iterator ItBlobFeatures;
			for (ItBlobFeatures = pBlobFeatures->begin(); ItBlobFeatures != pBlobFeatures->end(); ItBlobFeatures++)
			{
				int Bin = BinaryToInt(ItBlobFeatures->m_D);
				cvmSet(*pGlobalFeatures, 0, Bin, cvmGet(*pGlobalFeatures, 0, Bin)+1);
			}
			cvConvertScale(*pGlobalFeatures, *pGlobalFeatures, 1/(double)NumberSamples, 0);
			break;
		}
	case CLUSTER_EM:
		{
			/// Feature number control variables, features are not extracted if 0.
			const std::vector<int> polynomOrder = pGlobalFeatureParams.polynomOrder;	//2;
			const std::vector<int> numberLinesX = pGlobalFeatureParams.numberLinesX;	//6;		// number lines parallel to the x-axis
			const std::vector<int> numberLinesY = pGlobalFeatureParams.numberLinesY;	//6;		// number lines parallel to the y-axis
			const int pointDataExcess =	pGlobalFeatureParams.pointDataExcess;	//int(3.01*(polynomOrder+1));	// polynomial fitting will not happen with less than PolynomOrder+1+pointDataExcess points
			//const int Rotations = 6;		// old terminology, means that 6 curves are fitted
			const int NumberFramesStatisticsFeatures = 6;		// 6 bin histogram for statistics about frame alignment of the feature points with respect to the largest PCA eigenvector
			const unsigned int minNumber3DPixels = pGlobalFeatureParams.minNumber3DPixels;	//50;

			std::map<std::string, bool> useFeature;
			if (pGlobalFeatureParams.useFeature.find("bow") != pGlobalFeatureParams.useFeature.end())
				useFeature["bow"] =	pGlobalFeatureParams.useFeature["bow"];	//false;
			else
			{
				std::cout << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['bow'] not set." << std::endl;
				if (pScreenLogFile) *pScreenLogFile << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['bow'] not set." << std::endl;
				return ipa_utils::RET_FAILED;
			}
			if (pGlobalFeatureParams.useFeature.find("sap") != pGlobalFeatureParams.useFeature.end())
				useFeature["sap"] =	pGlobalFeatureParams.useFeature["sap"];	//true;
			else
			{
				std::cout << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['sap'] not set." << std::endl;
				if (pScreenLogFile) *pScreenLogFile << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['sap'] not set." << std::endl;
				return ipa_utils::RET_FAILED;
			}
			if (pGlobalFeatureParams.useFeature.find("sap2") != pGlobalFeatureParams.useFeature.end())
				useFeature["sap2"] = pGlobalFeatureParams.useFeature["sap2"];	//true;
			else
			{
				std::cout << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['sap2'] not set." << std::endl;
				if (pScreenLogFile) *pScreenLogFile << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['sap2'] not set." << std::endl;
				return ipa_utils::RET_FAILED;
			}
			if (pGlobalFeatureParams.useFeature.find("pointdistribution") != pGlobalFeatureParams.useFeature.end())
				useFeature["pointdistribution"] =	pGlobalFeatureParams.useFeature["pointdistribution"];	//true;
			else
			{
				std::cout << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['pointdistribution'] not set." << std::endl;
				if (pScreenLogFile) *pScreenLogFile << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['pointdistribution'] not set." << std::endl;
				return ipa_utils::RET_FAILED;
			}
			if (pGlobalFeatureParams.useFeature.find("normalstatistics") != pGlobalFeatureParams.useFeature.end())
				useFeature["normalstatistics"] = pGlobalFeatureParams.useFeature["normalstatistics"];	//false;
			else
			{
				std::cout << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['normalstatistics'] not set." << std::endl;
				if (pScreenLogFile) *pScreenLogFile << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['normalstatistics'] not set." << std::endl;
				return ipa_utils::RET_FAILED;
			}
			if (pGlobalFeatureParams.useFeature.find("vfh") != pGlobalFeatureParams.useFeature.end())
				useFeature["vfh"] =	pGlobalFeatureParams.useFeature["vfh"];	//true;
			else
			{
				std::cout << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['vfh'] not set." << std::endl;
				if (pScreenLogFile) *pScreenLogFile << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['vfh'] not set." << std::endl;
				return ipa_utils::RET_FAILED;
			}
			if (pGlobalFeatureParams.useFeature.find("grsd") != pGlobalFeatureParams.useFeature.end())
				useFeature["grsd"] =	pGlobalFeatureParams.useFeature["grsd"];	//true;
			else
			{
				std::cout << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['grsd'] not set." << std::endl;
				if (pScreenLogFile) *pScreenLogFile << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['grsd'] not set." << std::endl;
				return ipa_utils::RET_FAILED;
			}
			if (pGlobalFeatureParams.useFeature.find("gfpfh") != pGlobalFeatureParams.useFeature.end())
				useFeature["gfpfh"] = pGlobalFeatureParams.useFeature["gfpfh"];	//true;
			else
			{
				std::cout << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['gfpfh'] not set." << std::endl;
				if (pScreenLogFile) *pScreenLogFile << "Error: ObjectClassifier::ExtractGlobalFeatures: Parameter useFeature['gfpfh'] not set." << std::endl;
				return ipa_utils::RET_FAILED;
			}
			if (pGlobalFeatureParams.useFeature["grsd"] == true && pGlobalFeatureParams.useFeature["gfpfh"] == true)
			{
				std::cout << "Error: ObjectClassifier::ExtractGlobalFeatures: useFeature['grsd'] and useFeature['gfpfh'] cannot be used together." << std::endl;
				if (pScreenLogFile) *pScreenLogFile << "Error: ObjectClassifier::ExtractGlobalFeatures: useFeature['grsd'] and useFeature['gfpfh'] cannot be used together." << std::endl;
				return ipa_utils::RET_FAILED;
			}

			bool useFullPCAPoseNormalization = pGlobalFeatureParams.useFullPCAPoseNormalization;	// true;
			bool useRollPoseNormalization = pGlobalFeatureParams.useRollPoseNormalization;	// true;

			int descriptorSize = 0;
#if (CV_MAJOR_VERSION<=2 && CV_MINOR_VERSION<=3)
			if (useFeature["bow"]) descriptorSize += mData.mLocalFeatureClusterer->get_nclusters();
#else
			if (useFeature["bow"]) descriptorSize += mData.mLocalFeatureClusterer->get<int>("nclusters");
#endif
			if (useFeature["sap"]) descriptorSize += 3+(numberLinesX[0]+numberLinesY[0])*(polynomOrder[0]+1);
			if (useFeature["sap2"]) descriptorSize += (numberLinesX[1]+numberLinesY[1])*(polynomOrder[1]+1);
			if (useFeature["pointdistribution"]) descriptorSize += pGlobalFeatureParams.cellCount[0] * pGlobalFeatureParams.cellCount[1];
			if (useFeature["normalstatistics"]) descriptorSize += NumberFramesStatisticsFeatures;
			if (useFeature["vfh"]) descriptorSize += 308;
			if (useFeature["grsd"]) descriptorSize += 16;
			if (useFeature["gfpfh"]) descriptorSize += 16;
			
			*pGlobalFeatures = cvCreateMat(1, descriptorSize, CV_32FC1);
			cvSetZero(*pGlobalFeatures);
			int GlobalFeatureVectorPosition = 0;		// data is inserted into pGlobalFeatures at this position
			
			// if required, the object is tilted by a given angle and a second descriptor is computed
			int numberOfTiltAngles = 1 + pGlobalFeatureParams.additionalArtificialTiltedViewAngle.size();
			std::vector< std::vector<float> > descriptorBuffer(numberOfTiltAngles);
			for (int descriptorComputationPass = 0; descriptorComputationPass < numberOfTiltAngles; descriptorComputationPass++)
			{
				GlobalFeatureVectorPosition = 0;

				IplImage* mask = cvCloneImage(pMask);

				CvMat* BlobFPCoordinates = 0;
				if (pBlobFeatures->size() > 0)
					BlobFPCoordinates = cvCreateMat(pBlobFeatures->size(), 3, CV_32FC1);
			
				// make histogram
				//----------------
				if (useFeature["bow"])
				{
					BlobListRiB::iterator ItBlobFeatures;
					int FeatureCounter = 0;
					for (ItBlobFeatures = pBlobFeatures->begin(); ItBlobFeatures != pBlobFeatures->end(); ItBlobFeatures++, FeatureCounter++)
					{
						CvMat* LocalFeatureVector = cvCreateMat(1, ItBlobFeatures->m_D.size(), CV_32FC1);
						for (unsigned int j=0; j<ItBlobFeatures->m_D.size(); j++) cvSetReal1D(LocalFeatureVector, j, ItBlobFeatures->m_D[j]);
#if (CV_MAJOR_VERSION<=2 && CV_MINOR_VERSION<=3)
						int Bin = cvRound(mData.mLocalFeatureClusterer->predict((const CvMat*)LocalFeatureVector, NULL));		// speedup: replace round by (int)
#else
						cv::Mat localFeatureVectorMat(LocalFeatureVector);
						int Bin = cvRound((mData.mLocalFeatureClusterer->predict(localFeatureVectorMat))[1]);		// speedup: replace round by (int)
#endif
						//std::cout << Bin << "\n";
						cvSetReal1D(*pGlobalFeatures, Bin, cvGetReal1D(*pGlobalFeatures, Bin)+1.0);
						cvReleaseMat(&LocalFeatureVector);

						// make coordinate list (if 3D data available)
						if (ItBlobFeatures->m_Frame.size() == 6)
						{
							ipa_utils::Point3Dbl Point;
							ItBlobFeatures->m_Frame.GetT(Point);
							cvmSet(BlobFPCoordinates, FeatureCounter, 0, Point.m_x);
							cvmSet(BlobFPCoordinates, FeatureCounter, 1, Point.m_y);
							cvmSet(BlobFPCoordinates, FeatureCounter, 2, Point.m_z);
						}
					}
					if (NumberSamples > 0)
						cvConvertScale(*pGlobalFeatures, *pGlobalFeatures, 1/(double)NumberSamples, 0);
#if (CV_MAJOR_VERSION<=2 && CV_MINOR_VERSION<=3)
					GlobalFeatureVectorPosition += mData.mLocalFeatureClusterer->get_nclusters();		// data is inserted into pGlobalFeatures at this position
#else
					GlobalFeatureVectorPosition += mData.mLocalFeatureClusterer->get<int>("nclusters");		// data is inserted into pGlobalFeatures at this position
#endif
				}

				/*std::cout << "pGlobalFeatures:\n";
				for (int i=0; i<(*pGlobalFeatures)->height; i++)
				{
					for(int j=0; j<(*pGlobalFeatures)->width; j++) std::cout << cvGetReal2D(*pGlobalFeatures, i, j) << "\t";
					std::cout << "\n";
				}*/
			
				std::ofstream timeFout;
				if (pTimingLogFileName.compare("") != 0)
					timeFout.open(pTimingLogFileName.c_str(), std::ios::app);
				unsigned int numberOfPoints = 0;
				Timer tim, tim1;
				double elapsedTime = 0.0;
				double elapsedTime1[10];

				tim.start();
				tim1.start();

				/// Further features using 3D data
				if (pBlobFeatures->size() == 0 || pBlobFeatures->begin()->m_Frame.size() == 6)
				{
					//CvPoint ObjectCenter2D = cvPoint(0,0);
					CvPoint3D32f ObjectCenter3D = cvPoint3D32f(0.f, 0.f, 0.f);

					/// Perform PCA
					CvMat* Coordinates = NULL;			// Matrix of 3D coordinates of points used for the PCA
					pcl::PointCloud<pcl::PointXYZ>::Ptr pclPoints (new pcl::PointCloud<pcl::PointXYZ>);
					IplImage* CoordinateImage = NULL;

					if ((pCoordinateImage!=NULL) && (mask!=NULL))
					{	// PCA using all points inside mask
						if ((pCoordinateImage->width != mask->width) || (pCoordinateImage->height != mask->height))
						{
							std::cout << "ObjectClassifier::ExtractGlobalFeatures: pCoordinateImage and pMask do not have the same size." << std::endl;
							if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::ExtractGlobalFeatures: pCoordinateImage and pMask do not have the same size." << std::endl;
							return ipa_utils::RET_FAILED;
						}

						//cvNamedWindow("mask before");
						//cvShowImage("mask before", mask);
						//cvWaitKey(10);
						//IplConvKernel* kernel = cvCreateStructuringElementEx(1,3,0,1, CV_SHAPE_RECT);
						if (pDatabase == CIN)
							cvErode((IplImage*)mask, (IplImage*)mask, 0, 8);	// necessary to avoid false depth pixels at object borders
						//cvErode((IplImage*)mask, (IplImage*)mask, 0, 3);	// necessary to avoid false depth pixels at object borders
						//cvErode((IplImage*)mask, (IplImage*)mask, kernel, 10);
						//cvReleaseStructuringElement(&kernel);

						//cvNamedWindow("mask after");
						//cvShowImage("mask after", mask);
						//cvWaitKey();
						//cvDestroyAllWindows();


						CoordinateImage = cvCloneImage(pCoordinateImage); //cvCreateImageHeader(cvGetSize(pCoordinateImage), pCoordinateImage->depth, pCoordinateImage->nChannels);
						cvSmooth(CoordinateImage, CoordinateImage, CV_GAUSSIAN, 5);


						////////////////////////////////////////
						// tilt point cloud if in second pass
						if (descriptorComputationPass >= 1 && pGlobalFeatureParams.additionalArtificialTiltedViewAngle[descriptorComputationPass-1]!=0)
						{
							double tiltAngle = (double)pGlobalFeatureParams.additionalArtificialTiltedViewAngle[descriptorComputationPass-1] / 180. * M_PI;

							// compute 3d center
							double cx=0., cy=0., cz=0.;
							int numberPoints = 0;
							for (int v=0; v<mask->height; v++)
							{
								for (int u=0; u<mask->width; u++)
								{
									if (cvGetReal2D(mask, v, u) != 0)
									{
										CvScalar point = cvGet2D(CoordinateImage, v, u);
										cx += point.val[0];
										cy += point.val[1];
										cz += point.val[2];
										numberPoints++;
									}
								}
							}
							cx /= (double)numberPoints;
							cy /= (double)numberPoints;
							cz /= (double)numberPoints;

							double cosTilt = cos(tiltAngle);
							double sinTilt = sin(tiltAngle);

							// rotate point cloud by tiltAngle around its centroid
							for (int v=0; v<mask->height; v++)
							{
								// only keep cos(alpha) % of the lines, i.e. set the remainder of the data (and mask!) to zero
								bool keepThisLine = (rand() <= cosTilt*RAND_MAX);
								if (keepThisLine == false)
									for (int u=0; u<mask->width; u++) cvSetReal2D(mask, v, u, 0);
								else
								{
									for (int u=0; u<mask->width; u++)
									{
										if (cvGetReal2D(mask, v, u) != 0)
										{
											CvScalar point = cvGet2D(CoordinateImage, v, u);
											//point.val[0] -= cx;	does not rotate
											point.val[1] -= cy;
											point.val[2] -= cz;

											double y = cosTilt * point.val[1] - sinTilt * point.val[2];
											double z = sinTilt * point.val[1] + cosTilt * point.val[2];

											//point.val[0] += cx;
											point.val[1] = y + cy;
											point.val[2] = z + cz;
											cvSet2D(CoordinateImage, v, u, point);
										}
									}	
								}
							}
						}


						elapsedTime1[0] = tim1.getElapsedTimeInMicroSec();
						tim1.start();

						//////////////////////// START: new, 2d rotation
						if (useRollPoseNormalization == true)
						{
							// rotate 3d coordinates by rotating around z-axis so that mask image has a normalized orientation
							double cx=0, cy=0;
							unsigned int numberPoints = 0;
							std::vector<cv::Point2f> maskPointList;
							CvMat mask_buffer;
							CvMat* mask_mat = cvGetMat(mask, &mask_buffer);
							int type = CV_MAT_TYPE(mask_mat->type);
							assert(type == CV_8UC1);
							for (int y=0; y<mask->height; y++)
							{
								for (int x=0; x<mask->width; x++)
								{
									if (((uchar*)(mask_mat->data.ptr + (size_t)mask_mat->step*y))[x] != 0)  //cvmGet((CvMat*)mask, y, x) != 0)
									{
										maskPointList.push_back(cv::Point2i(x, y));
										CvScalar point = cvGet2D(CoordinateImage, y, x);
										cx += point.val[0];
										cy += point.val[1];
										numberPoints++;
									}
								}
							}
							cx /= (double)numberPoints;
							cy /= (double)numberPoints;

							elapsedTime1[1] = tim1.getElapsedTimeInMicroSec();
							tim1.start();

							if (maskPointList.size() > minNumber3DPixels)
							{
								cv::Mat maskPointMat(maskPointList.size(), 2, CV_32FC1);
								for (int i=0; i<(int)maskPointList.size(); i++)
								{
									maskPointMat.at<float>(i, 0) = maskPointList[i].x;
									maskPointMat.at<float>(i, 1) = maskPointList[i].y;
								}

								elapsedTime1[2] = tim1.getElapsedTimeInMicroSec();
								tim1.start();

								// find prominent direction in 2d image
								cv::PCA pca(maskPointMat, cv::noArray(), CV_PCA_DATA_AS_ROW);
								// find repeatable direction (e.g. side of the centroid with more points is always positive)
								int positiveDirection = 0, negativeDirection = 0;
								// is eigenvector of type float? yes, CV_32F
								// std::cout << "eigenvectors: " << pca.eigenvectors.depth() << "   mean: " << pca.mean.depth() << "   eigenvalues: " << pca.eigenvalues.depth() << std::endl;
								float e11 = pca.eigenvectors.at<float>(0, 0);
								float e12 = pca.eigenvectors.at<float>(0, 1);
								float m1 = pca.mean.at<float>(0, 0);
								float m2 = pca.mean.at<float>(0, 1);

								elapsedTime1[3] = tim1.getElapsedTimeInMicroSec();
								tim1.start();

								for (int i=0; i<(int)maskPointList.size(); i++)
								{
								//for (int y=0; y<mask->height; y++)		// todo: use maskPointList
								//{
								//	for (int x=0; x<mask->width; x++)
								//	{
										//if (cvGetReal2D(mask, y, x) != 0)
										//{
									float x = maskPointList[i].x;
									float y = maskPointList[i].y;
									if ((((float)x-m1)*e11 + ((float)y-m2)*e12) >= 0.f)
										positiveDirection++;
									else
										negativeDirection++;
//										}
//									}
								}
								if (positiveDirection < negativeDirection)
								{
									e11 *= -1;
									e12 *= -1;
									std::cout << "Have to turn 2d direction\n";
									if (pScreenLogFile) *pScreenLogFile << "Have to turn 2d direction\n";
								}

								elapsedTime1[4] = tim1.getElapsedTimeInMicroSec();
								tim1.start();

								// compute rotation around z-axis, i.e. the angle between old x-axis (1, 0, 0) and new x-axis (e11, e12, 0)
								double cosAlpha = e11/sqrt(e11*e11+e12*e12);
								double sinAlpha = sin(acos(cosAlpha));

								std::cout << "alpha=" << acos(cosAlpha)/M_PI * 180 << "\n";
								if (pScreenLogFile) *pScreenLogFile << "alpha=" << acos(cosAlpha)/M_PI * 180 << "\n";

								// rotate 3d coordinates around z-axis by alpha
								for (int i=0; i<(int)maskPointList.size(); i++)
								{
								//for (int v=0; v<mask->height; v++)		// todo: use maskPointList
								//{
								//	for (int u=0; u<mask->width; u++)
								//	{
								//		if (cvGetReal2D(mask, v, u) != 0)
								//		{
									int u = maskPointList[i].x;
									int v = maskPointList[i].y;
									CvScalar point = cvGet2D(CoordinateImage, v, u);
									double x = point.val[0] - cx;
									double y = point.val[1] - cy;
									point.val[0] = cosAlpha * x - sinAlpha * y + cx;
									point.val[1] = sinAlpha * x + cosAlpha * y + cy;
									cvSet2D(CoordinateImage, v, u, point);
									//	}
									//}
								}

								elapsedTime1[5] = tim1.getElapsedTimeInMicroSec();
								tim1.start();
							}
							else
							{
								std::cout << "ObjectClassifier::ExtractGlobalFeatures: Not enough 3D points available for roll pose normalization." << std::endl;
								if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::ExtractGlobalFeatures: Not enough 3D points available for roll pose normalization." << std::endl;
							}
						}
						//////////////////////// END: new, 2d rotation

						std::vector<CvScalar> CoordinateList;
						// get 3D coordinates of points inside mask and calculate object center (center of mass of mask)
						double Cu=0;
						double Cv=0;
						int MaskPoints=0;
						CvMat mask_buffer;
						CvMat* mask_mat = cvGetMat(mask, &mask_buffer);
						int type = CV_MAT_TYPE(mask_mat->type);
						assert(type == CV_8UC1);
						for (int i=0; i<mask->height; i++)
						{
							for (int j=0; j<mask->width; j++)
							{
								if (((uchar*)(mask_mat->data.ptr + (size_t)mask_mat->step*i))[j] != 0)  //(cvGetReal2D(mask, i, j) != 0)
								{
									Cu += j;
									Cv += i;
									MaskPoints++;
									if (pGlobalFeatureParams.thinningFactor >= 1.0)
										CoordinateList.push_back(cvGet2D(CoordinateImage, i, j));
									else
									{
										if ((descriptorComputationPass == 0) || rand() < pGlobalFeatureParams.thinningFactor * RAND_MAX)		// thinning of data to emulate scale change
											CoordinateList.push_back(cvGet2D(CoordinateImage, i, j));
									}
								}
							}
						}
						//ObjectCenter2D.x = cvRound(Cu/(double)MaskPoints);
						//ObjectCenter2D.y = cvRound(Cv/(double)MaskPoints);
					
						// can only process data if at least some 3d data of the object is available
						if (CoordinateList.size() > minNumber3DPixels)
						{
							Coordinates = cvCreateMat(CoordinateList.size(), 3, CV_32FC1);
							for (int i=0; i<(int)CoordinateList.size(); i++)
								for (int j=0; j<Coordinates->width; j++)
									cvmSet(Coordinates, i, j, CoordinateList[i].val[j]);

							if (useFeature["vfh"] == true || useFeature["grsd"] == true || useFeature["gfpfh"] == true)
							{
								for (int i=0; i<(int)CoordinateList.size(); i++)
								{
									pcl::PointXYZ point;
									point.x = CoordinateList[i].val[0];
									point.y = CoordinateList[i].val[1];
									point.z = CoordinateList[i].val[2];
									pclPoints->push_back(point);
								}
							}
						}
						else
						{	// PCA using feature points only
							Coordinates = BlobFPCoordinates;
							std::cout << "ObjectClassifier::ExtractGlobalFeatures: Not enough 3D points available, switching to BlobFPCoordinates." << std::endl;
							if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::ExtractGlobalFeatures: Not enough 3D points available, switching to BlobFPCoordinates." << std::endl;
						}
					}
					else
					{	// PCA using feature points only
						Coordinates = BlobFPCoordinates;
						std::cout << "ObjectClassifier::ExtractGlobalFeatures: No 3D points available, switching to BlobFPCoordinates." << std::endl;
						if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::ExtractGlobalFeatures: No 3D points available, switching to BlobFPCoordinates." << std::endl;
					}

					// stop data processing if too few 3d data of the object is available
					if (Coordinates == 0 || Coordinates->rows <= (int)minNumber3DPixels)
					{
						//for (int i=mData.mLocalFeatureClusterer->get_nclusters(); i<(*pGlobalFeatures)->cols; i++) cvSetReal1D(*pGlobalFeatures, i, 0.0);
						// already done with setZero()
						//timeFout << "0\t";
						numberOfPoints = 0;
						std::cout << "Not enough 3d points available. Skipping." << std::endl;
						if (pScreenLogFile) *pScreenLogFile << "Not enough 3d points available. Skipping." << std::endl;
					}
					else
					{
						numberOfPoints = Coordinates->rows;
						//timeFout << Coordinates->rows << "\t";

						// PCA
						CvMat* Avgs = NULL;
						CvMat* Eigenvalues = NULL;
						//cv::Mat* Eigenvalues = new cv::Mat(1, 3, CV_32FC1);
						CvMat* Eigenvectors = NULL;			// Eigenvectors are stored one in each row -> cvGetReal2D(Eigenvectors, Eigenvector_index, Component_index)
															// and are normalized to L_2 norm of each eigenvector is 1

						if (Coordinates->height > 2)
						{
							Avgs = cvCreateMat(1, 3, CV_32FC1);
							Eigenvalues = cvCreateMat(1, 3, CV_32FC1);
							Eigenvectors = cvCreateMat(3, 3, CV_32FC1);

							cvCalcPCA(Coordinates, Avgs, Eigenvalues, Eigenvectors, CV_PCA_DATA_AS_ROW);

							// test: are the PCA eigenvectors orthogonal?
							//double tempval = 0.0;
							//for (int i=0; i<3; i++) tempval += cvGetReal2D(Eigenvectors, 0, i)*cvGetReal2D(Eigenvectors, 1, i);
							//std::cout << "PCA: EV1*EV2 = " << tempval << "\n";
							//tempval=0.0;
							//for (int i=0; i<3; i++) tempval += cvGetReal2D(Eigenvectors, 0, i)*cvGetReal2D(Eigenvectors, 2, i);
							//std::cout << "PCA: EV1*EV3 = " << tempval << "\n";
							//tempval=0.0;
							//for (int i=0; i<3; i++) tempval += cvGetReal2D(Eigenvectors, 1, i)*cvGetReal2D(Eigenvectors, 2, i);
							//std::cout << "PCA: EV2*EV3 = " << tempval << "\n";

							//std::cout << "Coordinates:\n";
							//std::ofstream fout("common/files/coordinates.txt");
							//for (int i=0; i<Coordinates->height; i++)
							//{
							//	for(int j=0; j<Coordinates->width; j++) fout << cvGetReal2D(Coordinates, i, j) << " \t";
							//	fout << "\n";
							//}
							//fout.close();

							//std::cout << "Avgs:\n";
							//for (int i=0; i<3; i++)
							//{
							//	std::cout << cvGetReal1D(Avgs, i) << "\n";
							//}
							//std::cout << "Eigenvectors:\n";
							//for (int i=0; i<3; i++)
							//{
							//	double sum = 0.0;
							//	for(int j=0; j<3; j++)
							//	{
							//		double a = cvGetReal2D(Eigenvectors, i, j);
							//		sum += a*a;
							//		std::cout << a << "\t";
							//	}
							//	std::cout << "\t" << sum << "\n";
							//}
							//std::cout << "Eigenvalues:\n";
							//for (int i=0; i<3; i++)
							//{
							//	std::cout << cvGetReal1D(Eigenvalues, i) << "\n";
							//}

							/// save all 3 PCA Eigenvalues as they are
							//for (int s=0; s<3; s++, GlobalFeatureVectorPosition++) cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, cvGetReal1D(Eigenvalues, s)/10000.0);

							// save largest PCA Eigenvalue as it is and the both others relative to it
							if (useFeature["sap"])
							{
								if (pDatabase == CIN)
									cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, cvGetReal1D(Eigenvalues, 0)/10000.0);	// CIN database measures 3d coordinates in mm
								else
									cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, cvGetReal1D(Eigenvalues, 0));	// for 3d coordinates measured in m
								GlobalFeatureVectorPosition++;
								for(int s=1; s<3; s++, GlobalFeatureVectorPosition++) cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, cvGetReal1D(Eigenvalues, s)/cvGetReal1D(Eigenvalues, 0));
							}
	
							/// copy 3d data central point
							ObjectCenter3D.x = (float)cvGetReal1D(Avgs, 0);
							ObjectCenter3D.y = (float)cvGetReal1D(Avgs, 1);
							ObjectCenter3D.z = (float)cvGetReal1D(Avgs, 2);
						}

						elapsedTime1[6] = tim1.getElapsedTimeInMicroSec();
						tim1.start();

						if (useFullPCAPoseNormalization == false)
							elapsedTime += tim.getElapsedTimeInMicroSec();


						//std::cout << "pGlobalFeatures:\n";
						//for (int i=0; i<(*pGlobalFeatures)->height; i++)
						//{
						//	for(int j=0; j<(*pGlobalFeatures)->width; j++) std::cout << cvGetReal2D(*pGlobalFeatures, i, j) << "\t";
						//	std::cout << "\n";
						//}

						cvReleaseImage(&CoordinateImage);
						cvReleaseMat(&Avgs);


						// polynomial fitting
						if ((useFeature["sap"] || useFeature["pointdistribution"]) && Eigenvectors != NULL && mask != NULL)
						{
							// align coordinate system
							if (useFullPCAPoseNormalization == true)
							{
								// check if eigenvalues form a right hand system ((XxY)*Z > 0)
								std::vector<double> tempVec;
								tempVec.resize(3);
								tempVec[0] = (cvmGet(Eigenvectors, 0, 1)*cvmGet(Eigenvectors, 1, 2)-cvmGet(Eigenvectors, 0, 2)*cvmGet(Eigenvectors, 1, 1));
								tempVec[1] = (cvmGet(Eigenvectors, 0, 2)*cvmGet(Eigenvectors, 1, 0)-cvmGet(Eigenvectors, 0, 0)*cvmGet(Eigenvectors, 1, 2));
								tempVec[2] = (cvmGet(Eigenvectors, 0, 0)*cvmGet(Eigenvectors, 1, 1)-cvmGet(Eigenvectors, 0, 1)*cvmGet(Eigenvectors, 1, 0));
								if (tempVec[0]*cvmGet(Eigenvectors, 2, 0) + tempVec[1]*cvmGet(Eigenvectors, 2, 1) + tempVec[2]*cvmGet(Eigenvectors, 2, 2) < 0)
								{
									// left hand system --> invert z-axis
									std::cout << "Left hand system. Have to turn z." << std::endl;
									if (pScreenLogFile) *pScreenLogFile << "Left hand system. Have to turn z." << std::endl;
									for (int j=0; j<3; j++)
										cvmSet(Eigenvectors, 2, j, -cvmGet(Eigenvectors, 2, j));
								}


								// keep the direction of the eigenvectors repeatable
								// 1. rule: the new z'-axis must point towards the camera, which is z*z' < 0 or z'_3 < 0 since z = (0,0,1)   --> can be done before
								// 2. rule: positive x'-direction on that side of the x'=0 plane where fewer points are located   --> will be decided after first point coordinate tranformation (50% chance that the outcome is already well-aligned)
								// 3. rule: choose y' to yield a right-hand system   --> adapted after steps 1 and 2

								if (cvmGet(Eigenvectors, 2, 2) > 0)
								{
									// 1. rule not fulfilled
									// so keep x'-axis coordinates and invert y' and z' to enforce rule 1 and 3
									for (int i=1; i<3; i++)
										for (int j=0; j<3; j++)
											cvmSet(Eigenvectors, i, j, -cvmGet(Eigenvectors, i, j));
								}


								// translate origin to center of mass of the point cloud and
								// rotate frame so that the eigenvectors are the coordinate axes (1,0,0), (0,1,0) and (0,0,1)
								// rotation matrix = scalar products of the old base vectors with the new base vectors (see DMS script eq. (1.3))
								// in this case the Eigenvector matrix is the rotation matrix when the eigenvectors are stored row-wise
								int pointMajoritySide = 0;	// counts +1 if a transformed point has positive x' coordinates and -1 for negative x' coordinates
								for (int i=0; i<Coordinates->height; i++)
								{
									// translate
									double x = cvmGet((CvMat*)Coordinates, i, 0) - ObjectCenter3D.x;
									double y = cvmGet((CvMat*)Coordinates, i, 1) - ObjectCenter3D.y;
									double z = cvmGet((CvMat*)Coordinates, i, 2) - ObjectCenter3D.z;

									// rotate
									double coordinateValue = 0.;
									for (int j=2; j>=0; j--)
									{
										coordinateValue = cvmGet(Eigenvectors, j, 0)*x + cvmGet(Eigenvectors, j, 1)*y + cvmGet(Eigenvectors, j, 2)*z;		// todo: speedup possible
										cvmSet(Coordinates, i, j, coordinateValue);  //cvSetReal2D(Coordinates, i, j, coordinateValue);
									}
									pointMajoritySide += (int)sign(coordinateValue);	// checks the x'-coordinate for rule 2
								}

								if (pointMajoritySide > 0)
								{
									std::cout << "Turning x' and y' coordinates necessary (rule 2)." << std::endl;
									if (pScreenLogFile) *pScreenLogFile << "Turning x' and y' coordinates necessary (rule 2)." << std::endl;
									// 2. rule not fulfilled -> invert x' and y' coordinates to enforce rule 2 and 3
									for (int i=0; i<Coordinates->height; i++)
									{
										for (int j=0; j<2; j++)
											cvmSet(Coordinates, i, j, -cvmGet(Coordinates, i, j));
									}

									// change the coordinate system as well
									for (int i=0; i<2; i++)
										for (int j=0; j<3; j++)
											cvmSet(Eigenvectors, i, j, -cvmGet(Eigenvectors, i, j));
								}
							}

							// approximate a polynomial along lines parallel to the x axis and the y axis in the new coordinate system of the principal components
							double normX, normY;
							if (useFullPCAPoseNormalization == true)
							{
								normX = 1.0/(2.0*sqrt(cvGetReal1D(Eigenvalues, 0))); // normalize the coordinates by the magnitude of the respective eigenvalue to the eigenvector (new coordinate system's axis)
								normY = 1.0/(2.0*sqrt(cvGetReal1D(Eigenvalues, 1))); // this provides scale invariance
							}

							// without pose normalization
							if (useFullPCAPoseNormalization == false)
							{
								tim.start();
								double maxX=0, maxY=0;
								for (int i=0; i<Coordinates->height; i++)
								{
									// translate
									double x = cvmGet(Coordinates, i, 0) - ObjectCenter3D.x;
									cvmSet(Coordinates, i, 0, x);
									double y = cvmGet(Coordinates, i, 1) - ObjectCenter3D.y;
									cvmSet(Coordinates, i, 1, y);
									cvmSet(Coordinates, i, 2, cvmGet(Coordinates, i, 2) - ObjectCenter3D.z);
									if (fabs(x) > maxX)
										maxX = fabs(x);
									if (fabs(y) > maxY)
										maxY = fabs(y);
								}
								normX = 1.0/maxX;
								normY = 1.0/maxY;
							}

							elapsedTime1[7] = tim1.getElapsedTimeInMicroSec();
							tim1.start();

							//if (normX > normY) normY = normX;	// this is wrong because it does not scale the largest dimension to 1
							if (normX < normY) normY = normX;	// scale the largest dimension to 1 and use the same factor for the remaining dimensions
							else normX = normY;
							double normZ = normX;	//1.0/(2.0*sqrt(cvGetReal1D(Eigenvalues, 2)));
							std::vector< std::vector<double> > linesX(numberLinesX.size(), std::vector<double>());	// y coordinates of the polynomials parallel to the x-axis (the outer vector enumerates the sap levels - sap, sap2, ...)
							std::vector< std::vector<double> > linesY(numberLinesX.size(), std::vector<double>());	// x coordinates of the polynomials parallel to the y-axis (the outer vector enumerates the sap levels - sap, sap2, ...)
							for (int i=0; i<(int)numberLinesX.size(); i++)
							{
								double step = 2.0/(double)(numberLinesX[i]+1.0);
								for (double y=-1.0+step; y<0.998; y+=step) linesX[i].push_back(y);
								step = 2.0/(double)(numberLinesY[i]+1.0);
								for (double x=-1.0+step; x<0.998; x+=step) linesY[i].push_back(x);
							}
							std::vector< std::vector< std::vector<Point2Dbl> > > RegressionPointList(numberLinesX.size(), std::vector< std::vector<Point2Dbl> >());	// first index=sap level index (sap, sap2, ...) ; second index=list index (0..numberLinesX-1 -> x lines, numberLinesX..numberLinesY -> y lines), third index=point index
							for (int i=0; i<(int)RegressionPointList.size(); i++) RegressionPointList[i].resize(numberLinesX[i]+numberLinesY[i], std::vector<Point2Dbl>());	
							double distanceThreshold = 2.0/sqrt((double)Coordinates->height);	// sampling invariance - should it be dependent on number of curves? maybe not, since it is a sampling parameter
						

							//// output the point cloud to file
							//std::cout << "Coordinates transformed:\n";
							//std::ofstream fout("common/files/coordinatestf.txt");
							//for (int i=0; i<Coordinates->height; i++)
							//{
							//	fout << cvGetReal2D(Coordinates, i, 0)*normX << " \t" << cvGetReal2D(Coordinates, i, 1)*normY << " \t" << cvGetReal2D(Coordinates, i, 2)*normZ << std::endl;
							//}
							//fout.close();
							//std::cout << "press any key\n";
							//getchar();

							//double cellCount[2] = {3, 3};	// x/y-coordinate limits of intersections of the camera plane into segments in which the point percentages are counted
							//double cellSize[2] = {0.8, 0.8};
							std::map< double, std::map<double, int> > pointCount;	// matrix of point counts in the respective cells of the point distribution grid

							// set number of SAP computations at different polynomial degrees
							int numLevels = 1;
							for (int i=1; i<(int)numberLinesX.size(); i++)
							{
								std::stringstream ss;
								ss << "sap" << i+1;
								//if (i>0) ss << i+1;
								if ((useFeature.find(ss.str()) != useFeature.end()) && (useFeature[ss.str()]==true))
									numLevels++;
							}

							// fill point lists for the polynomials
							for (int p=0; p<Coordinates->height; p++)
							{
								// normalize 3d point coordinates
								double x = cvmGet(Coordinates, p, 0)*normX;
								double y = cvmGet(Coordinates, p, 1)*normY;
								double z = cvmGet(Coordinates, p, 2)*normZ;
							
								// check whether this point contributes to any line
	//#if (RUNTIME_TEST_SAP!=1)
	//							for (int i=0; i<(int)numberLinesX.size(); i++)
	//							{
	//								std::stringstream ss;
	//								ss << "sap";
	//								if (i>0) ss << i+1;
	//								if ((useFeature.find(ss.str()) != useFeature.end()) && (useFeature[ss.str()]==true))
	//								{
	//#else
									for (int i=0; i<numLevels; i++)
									{
										//int i = 0;
	//#endif
										for (int l=0; l<(int)linesX[i].size(); l++)
											if (fabs(y-linesX[i][l]) < distanceThreshold) RegressionPointList[i][l].push_back(Point2Dbl(x,z));
										for (int l=0; l<(int)linesY[i].size(); l++)
											if (fabs(x-linesY[i][l]) < distanceThreshold) RegressionPointList[i][linesX[i].size()+l].push_back(Point2Dbl(y,z));
									}
	//#if (RUNTIME_TEST_SAP!=1)
	//							}
	//#endif
							
							
	#if (RUNTIME_TEST_SAP!=1)
								// compute distribution of 3d points in the current camera plane (which is either the original view or normalized to the plane spanned by the two largest eigenvectors of the point cloud)
								if (useFeature["pointdistribution"] == true)
								{
									double cell[2] = {floor(x/pGlobalFeatureParams.cellSize[0] + 0.5)*pGlobalFeatureParams.cellSize[0], floor(y/pGlobalFeatureParams.cellSize[1] + 0.5)*pGlobalFeatureParams.cellSize[1]};
									if ((pointCount.find(cell[0]) != pointCount.end()) && (pointCount[cell[0]].find(cell[1]) != pointCount[cell[0]].end()))
										pointCount[cell[0]][cell[1]]++;
									else
										pointCount[cell[0]][cell[1]] = 1;
								}
	#endif
							}

							// fit the polynomials into the data
	//#if (RUNTIME_TEST_SAP!=1)
	//						for (int level=0; level<(int)RegressionPointList.size(); level++)
	//						{
	//							std::stringstream ss;
	//							ss << "sap";
	//							if (level > 0) ss << level+1;
	//							for (int l=0; l<(int)RegressionPointList[level].size() && useFeature.find(ss.str())!=useFeature.end() && useFeature[ss.str()]==true; l++)
	//							{
	//#else
							for (int level=0; level<numLevels; level++)
							{
								//int level = 0;
								for (int l=0; l<(int)RegressionPointList[level].size(); l++)
								{
	//#endif
									// check availability of enough points for polynomial fitting
									if (RegressionPointList[level][l].size()<=(polynomOrder[level]+1+pointDataExcess))
									{
										std::cout << "ObjectClassifier::ExtractGlobalFeatures: Too few points in polynomial " << l << ".\n";
										if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::ExtractGlobalFeatures: Too few points in polynomial " << l << ".\n";
										//save zeros in pGlobalFeatures
										for (int s=0; s<=polynomOrder[level]; s++, GlobalFeatureVectorPosition++) cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, 0);
										continue;
									}

									std::stringstream DataFileName, ParamsFileName;
									std::ofstream DataFile, ParamsFile;
									if (pFileOutput)
									{
										DataFileName << "GlobalFP_CurveFitting(" << level << "-" << l << ")_SensorData.txt";
										ParamsFileName << "GlobalFP_CurveFitting(" << level << "-" << l << ")_PolyParams.txt";
										DataFile.open((DataFileName.str()).c_str(), std::fstream::out);
										ParamsFile.open(ParamsFileName.str().c_str(), std::fstream::out);
									}

									//create regression problem matrices
									CvMat* A = cvCreateMat(RegressionPointList[level][l].size(), polynomOrder[level]+1, CV_32FC1);
									CvMat* B = cvCreateMat(RegressionPointList[level][l].size(), 1, CV_32FC1);
									CvMat* X = cvCreateMat(polynomOrder[level]+1, 1, CV_32FC1);

									for (int i=0; i<A->height; i++)
									{
										//for (int j=0; j<A->width; j++) cvSetReal2D(A, i, j, pow(RegressionPointList[l][i].s, j));		// speedup: replace pow
										double value = 1.0;
										for (int j=0; j<A->width; j++)
										{
											cvmSet(A, i, j, value);
											value *= RegressionPointList[level][l][i].s;
										}
										cvSetReal1D(B, i, RegressionPointList[level][l][i].z);
										//std::cout << RegressionPointList[l][i].s/DeltaS << "\t" << RegressionPointList[l][i].z/DeltaS << "\n";
										if (pFileOutput) DataFile << RegressionPointList[level][l][i].s << "\t" << RegressionPointList[level][l][i].z << "\n";
									}
									cvSolve(A, B, X, CV_SVD);

									if (pFileOutput)
									{
										//std::cout << "Regression parameters: \n";
										for (int i=0; i<X->height; i++)
										{
											//for (int j=0; j<X->width; j++) std::cout << cvGetReal2D(X, i, j) << "\t";
											for (int j=0; j<X->width; j++) ParamsFile << cvmGet(X, i, j) << "\t";
											//std::cout << "\n";
											ParamsFile << "\n";
										}
										DataFile.close();
										ParamsFile.close();
									}

									//save in pGlobalFeatures
									//int d = mData.mLocalFeatureClusterer->get_nclusters()+3+polynomOrder*Rotation;		//start position in pGlobalFeatures
									//bool exceedsLimits = false;
									//for (int s=0; s<=polynomOrder; s++)
									//	if (cvGetReal1D(X, s) > 15.)
									//		exceedsLimits = true;
									//if (exceedsLimits == false)
									for (int s=0; s<=polynomOrder[level]; s++, GlobalFeatureVectorPosition++) cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, cvGetReal1D(X, s));
									//else
									//	for (int s=0; s<=polynomOrder; s++, GlobalFeatureVectorPosition++) cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, 0.);

									cvReleaseMat(&A);
									cvReleaseMat(&B);
									cvReleaseMat(&X);
								}
							}

							if (useFeature["pointdistribution"] == true)
							{
								double cellLimits[2] = { pGlobalFeatureParams.cellSize[0]/2*(pGlobalFeatureParams.cellCount[0]-1), pGlobalFeatureParams.cellSize[1]/2*(pGlobalFeatureParams.cellCount[1]-1) };
								for (double cellX = -cellLimits[0]; cellX < cellLimits[0]+1e-3; cellX += pGlobalFeatureParams.cellSize[0])
								{
									for (double cellY = -cellLimits[1]; cellY < cellLimits[1]+1e-3; cellY += pGlobalFeatureParams.cellSize[1])
									{
										if ((pointCount.find(cellX) != pointCount.end()) && (pointCount[cellX].find(cellY) != pointCount[cellX].end()))
											cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, (double)pointCount[cellX][cellY]/(double)Coordinates->height);
										else
											cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, 0);
										GlobalFeatureVectorPosition++;
									}
								}
							}
						}
						if (pOutputImage)
						{
							cvSaveImage("CurveFittingImage.png", pOutputImage);
							cvReleaseImage(&pOutputImage);
						}





		/*				// --old-- /// Curve fitting along the strongest Eigenvector and 3 further directions rotating the strongest eigenvector by 45deg steps counter-clockwise
						/// Curve fitting along the Eigenvector projections.
						if (Eigenvectors != NULL && pMask != NULL)
						{
							// determine object's center point z-value (approximate it by the mean of the neighborhood if the central value is not valid)
							double CenterZ = 0.0;
							for (int d=0; ; d++)
							{
								int ZCounter = 0;
								for (int du=-d; du<=d; du++)
								{
									for (int dv=-d; dv<=d; dv++)
									{
										if (cvGetReal2D(pMask, ObjectCenter2D.y+dv, ObjectCenter2D.x+du) != 0)
										{	// point is part of pMask
											CvScalar Center = cvGet2D(pCoordinateImage, ObjectCenter2D.y+dv, ObjectCenter2D.x+du);
											CenterZ += Center.val[2];
											ZCounter++;
										}
									}
								}
								if (CenterZ!=0.0)
								{
									CenterZ /= (double)ZCounter;
									break;
								}
							}

							double dx=0.0, dy=0.0;
							for (int Rotation = 0; Rotation<Rotations; Rotation++)
							{
								// --old-- calculate direction, use second eigenvector if first shows directly into z-direction
								// for (int EigenvectorIndex=0; (dx==0.0 && dy==0.0); EigenvectorIndex++)
								// {
								//	dx=cvGetReal2D(Eigenvectors, EigenvectorIndex, 0);
								//	dy=cvGetReal2D(Eigenvectors, EigenvectorIndex, 1);
								// }

								dx=cvGetReal2D(Eigenvectors, Rotation, 0);
								dy=cvGetReal2D(Eigenvectors, Rotation, 1);
								if (dy==0.0 && dx==0.0) dy=1.0;

								/// curve fitting
								// normalize direction
								if (fabs(dx) > fabs(dy))
								{
									dy = dy/dx;
									dx = 1.0;
								}
								else
								{
									dx = dx/dy;
									dy = 1.0;
								}

								//if (fabs(dx) > fabs(dy))	// change start
								//{
								//	dy = dy/dx*sign(dx);
								//	dx = sign(dx);
								//}
								//else
								//{
								//	dx = dx/dy*sign(dy);
								//	dy = sign(dy);
								//}							// change end 06.03.2011
								// is this needed for anything but normalization?
									// better rotation invariant decision:
										//double SumNeg = 0.0;
										//for (int Multiplicator = -1; ; Multiplicator--)
										//{
										//	int u = ObjectCenter2D.x + cvRound(dx * Multiplicator);
										//	int v = ObjectCenter2D.y + cvRound(dy * Multiplicator);

										//	if (u<0 || u>=pMask->width || v<0 || v>=pMask->height) break;

										//	if (cvGetReal2D(pMask, v, u) != 0)
										//	{
										//		CvScalar Point = cvGet2D(pCoordinateImage, v, u);
										//		SumNeg = Point.val[2] - CenterZ;
										//	}
										//}

										//double SumPos = 0.0;
										//for (int Multiplicator = 1; ; Multiplicator++)
										//{
										//	int u = ObjectCenter2D.x + cvRound(dx * Multiplicator);
										//	int v = ObjectCenter2D.y + cvRound(dy * Multiplicator);

										//	if (u<0 || u>=pMask->width || v<0 || v>=pMask->height) break;

										//	if (cvGetReal2D(pMask, v, u) != 0)
										//	{
										//		CvScalar Point = cvGet2D(pCoordinateImage, v, u);
										//		SumPos = Point.val[2] - CenterZ;
										//	}
										//}

										////let (dx,dy) always point into the direction with positive Sum (larger z-values than other direction)
										//if (SumNeg > SumPos)
										//{
										//	dx *= -1;
										//	dy *= -1;
										//	// correct eigenvector in eigenvector matrix, too
										//	cvSetReal2D(Eigenvectors, Rotation, 0, -1*cvGetReal2D(Eigenvectors, Rotation, 0));
										//	cvSetReal2D(Eigenvectors, Rotation, 1, -1*cvGetReal2D(Eigenvectors, Rotation, 1));
										//	cvSetReal2D(Eigenvectors, Rotation, 2, -1*cvGetReal2D(Eigenvectors, Rotation, 2));
										//}
									//
								//if (dx <= 0.0)			// change start
								//{
								//	if (dx==0.0 && dy<0.0) dy *= -1;
								//	else
								//	{
								//		dx *= -1;
								//		dy *= -1;
								//	}
								//}							// chnage end 06.03.2011
								//create list with curve points along direction given by dx, dy
								int u = 0;
								int v = 0;
								CvScalar Point;
								struct Point2Dbl{double s; double z;} Point2D;
								double SMin=0.0, SMax=0.0;
								double UnitLength = sqrt(dx*dx + dy*dy);
								std::vector<Point2Dbl> RegressionPointList;
								for (int Step=-1; Step<2; Step+=2)	// go once into the negative and once into the positive direction along the eigenvector starting at the center point
								{
									for (int Multiplicator = 0; ; Multiplicator+=Step)
									{
										u = ObjectCenter2D.x + cvRound(dx * Multiplicator);
										v = ObjectCenter2D.y + cvRound(dy * Multiplicator);

										if (u<0 || u>=pMask->width || v<0 || v>=pMask->height)
										{
											if (RegressionPointList.size()!=0)
											{
												if (Step==-1) SMin = RegressionPointList.back().s;
												else SMax = RegressionPointList.back().s;
											}
											break;
										}

										// OutputImage
										if (pOutputImage)
											cvSet2D(pOutputImage, v, u, CV_RGB(255.0*(double)Rotation/(Rotations-1.0),255.0-255.0*(double)Rotation/(Rotations-1.0), 0));
									
										if (cvGetReal2D(pMask, v, u) != 0)
										{	// append point for regression
								// use real 3D surface alignment to eigenvector
											Point2D.s = sign(Multiplicator)*UnitLength * Multiplicator*Multiplicator;
											Point = cvGet2D(pCoordinateImage, v, u);
											Point2D.z = Point.val[2] - CenterZ;
											RegressionPointList.push_back(Point2D);
										}
									}
								}
								if ((int)RegressionPointList.size()>polynomOrder)
								{
									std::stringstream DataFileName;
									DataFileName << "GlobalFP_CurveFitting(" << Rotation << ")_SensorData.txt";
									std::ofstream DataFile((DataFileName.str()).c_str(), std::fstream::out);
									std::stringstream ParamsFileName;
									ParamsFileName << "GlobalFP_CurveFitting(" << Rotation << ")_PolyParams.txt";
									std::ofstream ParamsFile(ParamsFileName.str().c_str(), std::fstream::out);

									//create regression problem matrices
									double DeltaS = SMax-SMin;		// normalize RegressionPointList
									CvMat* A = cvCreateMat(RegressionPointList.size(), polynomOrder, CV_32FC1);
									CvMat* B = cvCreateMat(RegressionPointList.size(), 1, CV_32FC1);
									CvMat* X = cvCreateMat(polynomOrder, 1, CV_32FC1);

									for (int i=0; i<A->height; i++)
									{
										for (int j=0; j<A->width; j++)
											cvSetReal2D(A, i, j, pow(RegressionPointList[i].s/DeltaS, (j+1)));
										cvSetReal1D(B, i, RegressionPointList[i].z/DeltaS);
										//std::cout << RegressionPointList[i].s/DeltaS << "\t" << RegressionPointList[i].z/DeltaS << "\n";
										DataFile << RegressionPointList[i].s/DeltaS << "\t" << RegressionPointList[i].z/DeltaS << "\n";
									}

									cvSolve(A, B, X, CV_SVD);

									//std::cout << "Regression parameters: \n";
									for (int i=0; i<X->height; i++)
									{
										//for (int j=0; j<X->width; j++) std::cout << cvGetReal2D(X, i, j) << "\t";
										for (int j=0; j<X->width; j++) ParamsFile << cvGetReal2D(X, i, j) << "\t";
										//std::cout << "\n";
										ParamsFile << "\n";
									}
									DataFile.close();
									ParamsFile.close();

									//save in pGlobalFeatures
									//int d = mData.mLocalFeatureClusterer->get_nclusters()+3+polynomOrder*Rotation;		//start position in pGlobalFeatures
									for (int s=0; s<polynomOrder; s++, GlobalFeatureVectorPosition++) cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, cvGetReal1D(X, s));

									cvReleaseMat(&A);
									cvReleaseMat(&B);
									cvReleaseMat(&X);
								}
								else
								{
									std::cout << "ObjectClassifier::ExtractGlobalFeatures: No data for curve fitting available.\n";
								}

								// --old-- rotate direction
								// double Temp = dx;
								// dx = dx * cos(CV_PI/(double)Rotations) + dy * sin(CV_PI/(double)Rotations);
								// dy = -Temp * sin(CV_PI/(double)Rotations) + dy * cos(CV_PI/(double)Rotations);
							}

							if (pOutputImage)
							{
								cvSaveImage("CurveFittingImage.png", pOutputImage);
								cvReleaseImage(&pOutputImage);
							}
						}*/


						/// Statistics about feature point frame directions compared to the largest principal component (eigenvector).
						if (useFeature["normalstatistics"] && Eigenvectors!=NULL && NumberFramesStatisticsFeatures > 0)
						{
							BlobListRiB::iterator ItBlobFeatures;

							// find invariant direction (largest PCA direction)
							ipa_utils::Point3Dbl PCAMainDirection = ipa_utils::Point3Dbl(cvmGet(Eigenvectors, 0, 0), cvmGet(Eigenvectors, 0, 1), cvmGet(Eigenvectors, 0, 2));
			// --improvement needed: direction is not chosen by chance but still not invariant with respect to the object
			// simply use directed eigenvectors from above
							if (PCAMainDirection.m_x < 0.0) PCAMainDirection.Negative();
							else
							{
								if (PCAMainDirection.m_x==0.0)
								{
									if (PCAMainDirection.m_y < 0.0) PCAMainDirection.Negative();
									else
									{
										if (PCAMainDirection.m_y==0.0)
										{
											if (PCAMainDirection.m_z < 0.0) PCAMainDirection.Negative();
										}
									}
								}
							}
							PCAMainDirection.Normalize();
			// improvement needed --
							double Bins[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
							for (ItBlobFeatures = pBlobFeatures->begin(); ItBlobFeatures != pBlobFeatures->end(); ItBlobFeatures++)
							{
								ipa_utils::Point3Dbl FPDirection;
								ItBlobFeatures->m_Frame.eX(FPDirection);
								if (FPDirection.ScalarProd(PCAMainDirection) > 0) Bins[0]++;
								else Bins[1]++;

								ItBlobFeatures->m_Frame.eY(FPDirection);
								if (FPDirection.ScalarProd(PCAMainDirection) > 0) Bins[2]++;
								else Bins[3]++;

								ItBlobFeatures->m_Frame.eZ(FPDirection);
								if (FPDirection.ScalarProd(PCAMainDirection) > 0) Bins[4]++;
								else Bins[5]++;
							}
							double Sum = 0;
							for (int i=0; i<5; i+=2, GlobalFeatureVectorPosition+=2)
							{
								Sum = Bins[i]+Bins[i+1];
								cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, Bins[i]/Sum);
								cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition+1, Bins[i+1]/Sum);
							}
						}

					
						// compute vfh feature
						if (useFeature["vfh"]==true)
						{
							elapsedTime = 0.0;
							tim.start();

							// normalize viewpoint
							double metricFactor = 1.0;
							if (pDatabase == CIN) metricFactor = 0.001;
							for (int i=0; i<(int)pclPoints->size(); i++)
							{
								pclPoints->at(i).x = pclPoints->at(i).x*metricFactor - ObjectCenter3D.x;
								pclPoints->at(i).y = pclPoints->at(i).y*metricFactor - ObjectCenter3D.y;
								pclPoints->at(i).z = pclPoints->at(i).z*metricFactor - ObjectCenter3D.z + 1.0;
							}

							// Create the filtering object
							pcl::PointCloud<pcl::PointXYZ>::Ptr vfhPointsVoxelized(new pcl::PointCloud<pcl::PointXYZ>());
							pcl::VoxelGrid<pcl::PointXYZ> voxg;
							voxg.setInputCloud(pclPoints);
							voxg.setLeafSize(0.005f, 0.005f, 0.005f);
							voxg.filter(*vfhPointsVoxelized);


							// Create the normal estimation class, and pass the input dataset to it
							pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
							ne.setInputCloud(vfhPointsVoxelized);

							// Create an empty kdtree representation, and pass it to the normal estimation object.
							// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
							pcl_search<pcl::PointXYZ>::Ptr tree (new pcl_search<pcl::PointXYZ> ());
							ne.setSearchMethod(tree);

							// Output datasets
							pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());

							// Use all neighbors in a sphere of radius 3cm
							ne.setRadiusSearch(0.03);	//0.03

							// Compute the normals
							ne.compute(*normals);


							// Create the VFH estimation class, and pass the input dataset+normals to it
							pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
							vfh.setInputCloud(vfhPointsVoxelized);
							vfh.setInputNormals(normals);
							// alternatively, if cloud is of tpe PointNormal, do vfh.setInputNormals (cloud);

							// Create an empty kdtree representation, and pass it to the FPFH estimation object.
							// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
							pcl_search<pcl::PointXYZ>::Ptr vfhTree (new pcl_search<pcl::PointXYZ>());
							vfh.setSearchMethod(vfhTree);
						
							// Output datasets
							pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

							// Compute the features
							vfh.compute(*vfhs);


							// write descriptor into the descriptor vector
							for (int i=0; i<308; i++, GlobalFeatureVectorPosition++)
								cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, vfhs->at(0).histogram[i]);
						}

						if (useFeature["grsd"] == true || useFeature["gfpfh"] == true)
						{
#ifndef __LINUX__
							elapsedTime = 0.0;
							tim.start();

							// normalize viewpoint
							//double metricFactor = 1.0;
							//if (pDatabase == CIN) metricFactor = 0.001;
							//for (int i=0; i<pclPoints->size(); i++)
							//{
							//	pclPoints->at(i).x = pclPoints->at(i).x*metricFactor - ObjectCenter3D.x;
							//	pclPoints->at(i).y = pclPoints->at(i).y*metricFactor - ObjectCenter3D.y;
							//	pclPoints->at(i).z = pclPoints->at(i).z*metricFactor - ObjectCenter3D.z + 1.0;
							//}

							// Create the filtering object
							pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointsVoxelized(new pcl::PointCloud<pcl::PointXYZ>());
							//pcl::VoxelGrid<pcl::PointXYZ> voxg;
							//voxg.setInputCloud(pclPoints);
							//voxg.setLeafSize(0.015f, 0.015f, 0.015f);
							//voxg.filter(*pclPointsVoxelized);


							// Create the normal estimation class, and pass the input dataset to it
							//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
							//ne.setInputCloud(pclPointsVoxelized);

							//// Create an empty kdtree representation, and pass it to the normal estimation object.
							//// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
							//pcl_search<pcl::PointXYZ>::Ptr tree (new pcl_search<pcl::PointXYZ> ());
							//ne.setSearchMethod(tree);

							//// Output datasets
							//pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>());

							//// Use all neighbors in a sphere of radius 3cm
							//ne.setRadiusSearch(0.03);	//0.03

							//// Compute the normals
							//ne.compute(*normals);

							// labels
							//pcl::getSimpleType();
							pcl::PointCloud<pcl::PointXYZL>::Ptr labels (new pcl::PointCloud<pcl::PointXYZL>());
							for (BlobListRiB::iterator ItBlobFeatures = pBlobFeatures->begin(); ItBlobFeatures != pBlobFeatures->end(); ItBlobFeatures++)
							{
								pcl::PointXYZL pointl;
								ipa_utils::Point3Dbl ipaPoint;
								ItBlobFeatures->m_Frame.GetT(ipaPoint);
								pointl.x = ipaPoint.m_x;
								pointl.y = ipaPoint.m_y;
								pointl.z = ipaPoint.m_z;
								if (useFeature["grsd"] == true)
									pointl.label = pcl::getSimpleType(ItBlobFeatures->m_D[0], ItBlobFeatures->m_D[1]);
								else if (useFeature["gfpfh"] == true)
								{
									CvMat* LocalFeatureVector = cvCreateMat(1, ItBlobFeatures->m_D.size(), CV_32FC1);
									for (unsigned int j=0; j<ItBlobFeatures->m_D.size(); j++) cvSetReal1D(LocalFeatureVector, j, ItBlobFeatures->m_D[j]);
#if (CV_MAJOR_VERSION<=2 && CV_MINOR_VERSION<=3)
									pointl.label = (unsigned char)cvRound(mData.mLocalFeatureClusterer->predict((const CvMat*)LocalFeatureVector, NULL));		// speedup: replace round by (int)
#else
									cv::Mat localFeatureVector(LocalFeatureVector);
									pointl.label = (unsigned char)cvRound((mData.mLocalFeatureClusterer->predict(localFeatureVector))[1]);		// speedup: replace round by (int)
#endif
									cvReleaseMat(&LocalFeatureVector);
								}
								labels->push_back(pointl);

								pcl::PointXYZ point;
								point.x = pointl.x;
								point.y = pointl.y;
								point.z = pointl.z;
								pclPointsVoxelized->push_back(point);
							}

							// Output datasets
							pcl::PointCloud<pcl::GFPFHSignature16>::Ptr gfpfhs (new pcl::PointCloud<pcl::GFPFHSignature16> ());
							pcl::GFPFHEstimation<pcl::PointXYZ, pcl::PointXYZL, pcl::GFPFHSignature16> gfpfh;
							gfpfh.setInputCloud(pclPointsVoxelized);
							gfpfh.setInputLabels(labels);

							// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
							pcl_search<pcl::PointXYZ>::Ptr gfpfhTree (new pcl_search<pcl::PointXYZ>());
							gfpfh.setSearchMethod(gfpfhTree);

							gfpfh.compute(*gfpfhs);

							// write descriptor into the descriptor vector
							for (int i=0; i<gfpfhs->at(0).descriptorSize(); i++, GlobalFeatureVectorPosition++)
								cvSetReal1D(*pGlobalFeatures, GlobalFeatureVectorPosition, gfpfhs->at(0).histogram[i]);
#endif
						}


						if (Eigenvalues) cvReleaseMat(&Eigenvalues);
						if (Coordinates) cvReleaseMat(&Coordinates);
						if (Eigenvectors) cvReleaseMat(&Eigenvectors);
					}
				}
				elapsedTime += tim.getElapsedTimeInMicroSec();

				if (pTimingLogFileName.compare("") != 0)
				{
					timeFout << numberOfPoints << "\t";
					timeFout << elapsedTime;
					for (int i=0; i<7; i++)
						timeFout << "\t" << elapsedTime1[i];
					timeFout << std::endl;
					timeFout.close();
				}

				cvReleaseMat(&BlobFPCoordinates);

				// save descriptor in temporary buffer in case a second is computed which overwrites the first
				if (numberOfTiltAngles > 1)
				{
					for (int j=0; j<(*pGlobalFeatures)->cols; j++)
						descriptorBuffer[descriptorComputationPass].push_back(cvGetReal1D(*pGlobalFeatures, j));
				}

				cvReleaseImage(&mask);
			}

			// return multiple descriptor vectors if a second/third/... was required from a tilted view
			if (numberOfTiltAngles > 1)
			{
				cvReleaseMat(pGlobalFeatures);
				*pGlobalFeatures = cvCreateMat(numberOfTiltAngles, descriptorSize, CV_32FC1);
				for (int i=0; i<(int)descriptorBuffer.size(); i++)
					for (int j=0; j<(int)descriptorBuffer[i].size(); j++)
						cvmSet(*pGlobalFeatures, i, j, descriptorBuffer[i][j]);
			}

			break;
		}
	default:
		std::cout << "ObjectClassifier::ExtractGlobalFeatures: Invalid ClusterMode.\n";
		if (pScreenLogFile) *pScreenLogFile << "ObjectClassifier::ExtractGlobalFeatures: Invalid ClusterMode.\n";
		break;
	}

	// Output
	//for (int i=0; i<(*pGlobalFeatures)->width; i++) std::cout << cvGetReal2D(*pGlobalFeatures, 0, i) << "\t";
	//std::cout << "\n";

	return ipa_utils::RET_OK;
}

int ObjectClassifier::BinaryToInt(ipa_utils::IpaVector<float> pBinary)
{
	int Int=0;
	int i=0;
	for (ipa_utils::IpaVector<float>::iterator ItBinary = pBinary.begin(); ItBinary != pBinary.end(); ItBinary++, i++)
	{
		Int+=int(*ItBinary*cvRound(pow((double)2.0,(double)i)));
	}
	return Int;
}


int ObjectClassifier::CrossValidationStatisticsOutput(std::string pClass, ClassifierPerformanceStruct& pValidationSetPerformance, ClassifierPerformanceStruct& pTestSetPerformance,
													  std::ofstream& pOutputFileStream, float pFactorIncorrect, double& pROCDistanceToTopLeftCorner, float pThreshold, bool pPrintHeader, std::ofstream* pScreenLogFile)
{
	std::cout.setf(ios::fixed,ios::floatfield);
	if (pScreenLogFile) pScreenLogFile->setf(ios::fixed,ios::floatfield);
	std::cout.precision(6);
	if (pScreenLogFile) pScreenLogFile->precision(6);

	/// Cross-validation statistics
	if (pPrintHeader) std::cout << "\nSet\tClass\t\tThresh\tRightPos\tWrongPos\tWrongNeg\tRightNeg\tSN%\t\tnSN%\t\tSP%\t\tnSP%\t\tDist\n";
	if (pPrintHeader && pScreenLogFile) *pScreenLogFile << "\nSet\tClass\t\tThresh\tRightPos\tWrongPos\tWrongNeg\tRightNeg\tSN%\t\tnSN%\t\tSP%\t\tnSP%\t\tDist\n";
	std::cout << "Valid\t";
	if (pScreenLogFile) *pScreenLogFile << "Valid\t";


	double SN=(double)pValidationSetPerformance.RightPositives/(double)(pValidationSetPerformance.RightPositives+pValidationSetPerformance.WrongNegatives);	// right positives, sensitivity
	double nSN=(double)pValidationSetPerformance.RightNegatives/(double)(pValidationSetPerformance.RightNegatives+pValidationSetPerformance.WrongPositives);	// right negatives, negative sensitivity
	double SP=(double)pValidationSetPerformance.RightPositives/(double)(pValidationSetPerformance.RightPositives+(double)pValidationSetPerformance.WrongPositives/pFactorIncorrect);	// specificity
	double nSP=(double)pValidationSetPerformance.RightNegatives/(double)(pValidationSetPerformance.RightNegatives+(double)pValidationSetPerformance.WrongNegatives/pFactorIncorrect);	// negative specificity

	/// Cross-validation performance: Distance to top left corner (0,1) in ROC curve.
	pROCDistanceToTopLeftCorner = sqrt((1-SN)*(1-SN) + (1-nSN)*(1-nSN));

	std::cout << pClass << "\t\t" << pThreshold << "\t" << /*IndicesTrainCorrect.size() << "\t\t" <<*/
			pValidationSetPerformance.RightPositives << "\t\t" <<
			pValidationSetPerformance.WrongPositives << "\t\t" <<
			pValidationSetPerformance.WrongNegatives << "\t\t" <<
			pValidationSetPerformance.RightNegatives << "\t\t" << SN << "\t" << nSN << "\t" << SP << "\t" << nSP << "\t" << pROCDistanceToTopLeftCorner << "\n";
	if (pScreenLogFile) *pScreenLogFile << pClass << "\t\t" << pThreshold << "\t" << /*IndicesTrainCorrect.size() << "\t\t" <<*/
		pValidationSetPerformance.RightPositives << "\t\t" <<
		pValidationSetPerformance.WrongPositives << "\t\t" <<
		pValidationSetPerformance.WrongNegatives << "\t\t" <<
		pValidationSetPerformance.RightNegatives << "\t\t" << SN << "\t" << nSN << "\t" << SP << "\t" << nSP << "\t" << pROCDistanceToTopLeftCorner << "\n";

	/// Test set statistics
	double SNT = 0;
	double nSNT = 0;
	double SPT = 0;
	double nSPT = 0;
	if ((pTestSetPerformance.RightPositives + pTestSetPerformance.WrongNegatives) != 0)
	{
		SNT = (double)pTestSetPerformance.RightPositives/(double)(pTestSetPerformance.RightPositives+pTestSetPerformance.WrongNegatives);		// right positives, sensitivity
		nSNT = (double)pTestSetPerformance.RightNegatives/(double)(pTestSetPerformance.RightNegatives+pTestSetPerformance.WrongPositives);		// right negatives, negative sensitivity
		SPT = (double)pTestSetPerformance.RightPositives/(double)(pTestSetPerformance.RightPositives+(double)pTestSetPerformance.WrongPositives/pFactorIncorrect);		// specificity
		nSPT = (double)pTestSetPerformance.RightNegatives/(double)(pTestSetPerformance.RightNegatives+(double)pTestSetPerformance.WrongNegatives/pFactorIncorrect);		// negative specificity
		std::cout << "Test\t" << pClass << "\t\t" << pThreshold << "\t" << /*IndicesTestCorrect.size() << "\t\t" <<*/
				pTestSetPerformance.RightPositives << "\t\t" <<
				pTestSetPerformance.WrongPositives << "\t\t" <<
				pTestSetPerformance.WrongNegatives << "\t\t" <<
				pTestSetPerformance.RightNegatives << "\t\t" << SNT << "\t" << nSNT << "\t" << SPT << "\t" << nSPT << "\n";
		if (pScreenLogFile) *pScreenLogFile << "Test\t" << pClass << "\t\t" << pThreshold << "\t" << /*IndicesTestCorrect.size() << "\t\t" <<*/
			pTestSetPerformance.RightPositives << "\t\t" <<
			pTestSetPerformance.WrongPositives << "\t\t" <<
			pTestSetPerformance.WrongNegatives << "\t\t" <<
			pTestSetPerformance.RightNegatives << "\t\t" << SNT << "\t" << nSNT << "\t" << SPT << "\t" << nSPT << "\n";
	}

	pOutputFileStream << pThreshold << "\t" << SN << "\t" << nSN << "\t" << SP << "\t" << nSP << "\t" << SNT << "\t" << nSNT << "\t" << SPT << "\t" << nSPT;
	pOutputFileStream << "\n";

	return ipa_utils::RET_OK;
}



int ObjectClassifier::LoadParameters(std::string pFilename)
{
	// standard parameters
	mTargetScreenResolution.width = 1920;
	mTargetScreenResolution.height = 1200;
	mTargetSegmentationImageResolution.width = 640;
	mTargetSegmentationImageResolution.height = 480;
	mConsideredVolume.x = 0.2;
	mConsideredVolume.y = 100.0;
	mConsideredVolume.z = 1.2;
	mVoxelFilterLeafSize.x = 0.01f;
	mVoxelFilterLeafSize.y = 0.01f;
	mVoxelFilterLeafSize.z = 0.01f;
	mPlaneSearchMaxIterations = 100;
	mPlaneSearchDistanceThreshold = 0.02;
	mPlaneSearchAbortRemainingPointsFraction = 0.3;
	mPlaneSearchAbortMinimumPlaneSize = 1000;
	mClusterSearchToleranceValue = 0.05;
	mClusterSearchMinClusterSize = 50;
	mClusterSearchMaxClusterSize = 25000;
	mConsideredClusterCenterVolume.x = 0.3;
	mConsideredClusterCenterVolume.y = 0.3;
	mConsideredClusterCenterVolume.z = 1.0;
	mTrackingInfluenceFactorOldData = 0.6;

	std::fstream paramFile(pFilename.c_str(), std::ios::in);
	if (paramFile.is_open() == false)
	{
		std::cout << "ObjectClassifier::LoadParameters: Warning: Could not open file " << pFilename << ". Using default parameters." << std::endl;
		return ipa_utils::RET_FAILED;
	}

	while (paramFile.eof() == false)
	{
		std::string tag;
		
		paramFile >> tag;
		if (tag.compare("TargetScreenResolutionWidth:")==0)
			paramFile >> mTargetScreenResolution.width;
		else if (tag.compare("TargetScreenResolutionHeight:")==0)
			paramFile >> mTargetScreenResolution.height;
		else if (tag.compare("TargetSegmentationImageResolutionWidth:")==0)
			paramFile >> mTargetSegmentationImageResolution.width;
		else if (tag.compare("TargetSegmentationImageResolutionHeight:")==0)
			paramFile >> mTargetSegmentationImageResolution.height;
		else if (tag.compare("ConsideredVolumeX:")==0)
			paramFile >> mConsideredVolume.x;
		else if (tag.compare("ConsideredVolumeY:")==0)
			paramFile >> mConsideredVolume.y;
		else if (tag.compare("ConsideredVolumeZ:")==0)
			paramFile >> mConsideredVolume.z;
		else if (tag.compare("VoxelFilterLeafSizeX:")==0)
			paramFile >> mVoxelFilterLeafSize.x;
		else if (tag.compare("VoxelFilterLeafSizeY:")==0)
			paramFile >> mVoxelFilterLeafSize.y;
		else if (tag.compare("VoxelFilterLeafSizeZ:")==0)
			paramFile >> mVoxelFilterLeafSize.z;
		else if (tag.compare("PlaneSearchMaxIterations:")==0)
			paramFile >> mPlaneSearchMaxIterations;
		else if (tag.compare("PlaneSearchDistanceThreshold:")==0)
			paramFile >> mPlaneSearchDistanceThreshold;
		else if (tag.compare("PlaneSearchAbortRemainingPointsFraction:")==0)
			paramFile >> mPlaneSearchAbortRemainingPointsFraction;
		else if (tag.compare("PlaneSearchAbortMinimumPlaneSize:")==0)
			paramFile >> mPlaneSearchAbortMinimumPlaneSize;
		else if (tag.compare("ClusterSearchToleranceValue:")==0)
			paramFile >> mClusterSearchToleranceValue;
		else if (tag.compare("ClusterSearchMinClusterSize:")==0)
			paramFile >> mClusterSearchMinClusterSize;
		else if (tag.compare("ClusterSearchMaxClusterSize:")==0)
			paramFile >> mClusterSearchMaxClusterSize;
		else if (tag.compare("ConsideredClusterCenterVolumeX:")==0)
			paramFile >> mConsideredClusterCenterVolume.x;
		else if (tag.compare("ConsideredClusterCenterVolumeY:")==0)
			paramFile >> mConsideredClusterCenterVolume.y;
		else if (tag.compare("ConsideredClusterCenterVolumeZ:")==0)
			paramFile >> mConsideredClusterCenterVolume.z;
		else if (tag.compare("DisplayFontColorB:")==0)
			paramFile >> mDisplayFontColorBGR.val[0];
		else if (tag.compare("DisplayFontColorG:")==0)
			paramFile >> mDisplayFontColorBGR.val[1];
		else if (tag.compare("DisplayFontColorR:")==0)
			paramFile >> mDisplayFontColorBGR.val[2];
		else if (tag.compare("TrackingInfluenceFactorOldData:")==0)
			paramFile >> mTrackingInfluenceFactorOldData;
	}

	paramFile.close();

	return ipa_utils::RET_OK;
}


int ObjectClassifier::CategorizeContinuously(ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams)
{
	mImageNumber = 0;

#ifndef __LINUX__
	pcl::Grabber* kinectGrabber = new pcl::OpenNIGrabber();
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind(&ObjectClassifier::PointcloudCallback, this, _1, pClusterMode, pClassifierTypeGlobal, pGlobalFeatureParams);
	kinectGrabber->registerCallback(f);
	kinectGrabber->start();

	std::cout << "Categorization started.\n" << std::endl;
	
	mDisplayImageOriginal = cv::Mat::zeros(480,640,CV_8UC3);
	mDisplayImageSegmentation = cv::Mat::zeros(48,64,CV_8UC3);
	cv::imshow("original image", mDisplayImageOriginal);
	cv::imshow("cluster image", mDisplayImageSegmentation);
	cvMoveWindow("original image", 0, 0);
	cvMoveWindow("cluster image", 0, 30);
	cv::waitKey(10);

	char key = 0;
	std::cout << "Hit 'q' to quit.\n\n";
	while (key != 'q')
	{
		{
			//key = getchar();
			boost::mutex::scoped_lock lock(mDisplayImageMutex);
			cv::imshow("original image", mDisplayImageOriginal);
			cv::imshow("cluster image", mDisplayImageSegmentation);
			cv::waitKey(20);
		}
		key = cvWaitKey(30);
	}

	kinectGrabber->stop();

#endif

	return ipa_utils::RET_OK;
}


#define isnan(x) ((x) != (x))
void ObjectClassifier::PointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pInputCloud, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams)
{
	try
	{
		//boost::mutex::scoped_lock lock(callbackMutex);

		if (pInputCloud->isOrganized() == false)
		{
			std::cout << "ObjectClassifier::PointcloudCallback: Warning: Point cloud is not organized. Skipping.\n" << std::endl;
			return;
		}

		// segment incoming point cloud
		std::cout << "\nSegmenting data..." << std::endl;

		// only keep points inside a defined volume
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudVoI(new pcl::PointCloud<pcl::PointXYZRGB>);
		for (unsigned int v=0; v<pInputCloud->height; v++)
			for (unsigned int u=0; u<pInputCloud->width; u++)
				if (fabs(pInputCloud->at(u,v).x)<mConsideredVolume.x && fabs(pInputCloud->at(u,v).y)<mConsideredVolume.y && pInputCloud->at(u,v).z<mConsideredVolume.z)
					inputCloudVoI->push_back(pInputCloud->at(u,v));
	
		// Create the filtering object: downsample the dataset using a leaf size of 1cm
		pcl::VoxelGrid<pcl::PointXYZRGB> vg;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
		vg.setInputCloud(inputCloudVoI);
		vg.setLeafSize(mVoxelFilterLeafSize.x, mVoxelFilterLeafSize.y, mVoxelFilterLeafSize.z);
	//	vg.setLeafSize(0.02f, 0.02f, 0.02f);
		vg.filter(*cloud_filtered);
		std::cout << "PointCloud after filtering has: " << cloud_filtered->size()  << " data points left from " << pInputCloud->size() << "." << std::endl;
	
		if (cloud_filtered->points.size() == 0)
			return;

		// Create the segmentation object for the planar model and set all the parameters
		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(mPlaneSearchMaxIterations);
		seg.setDistanceThreshold(mPlaneSearchDistanceThreshold);

		int nr_points = (int) cloud_filtered->points.size();
		while (cloud_filtered->points.size () > mPlaneSearchAbortRemainingPointsFraction * nr_points)
		{
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud(cloud_filtered);
			seg.segment (*inliers, *coefficients);
			if (inliers->indices.size() == 0)
			{
				std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
				break;
			}
			if (inliers->indices.size() < mPlaneSearchAbortMinimumPlaneSize)
				break;

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud(cloud_filtered);
			extract.setIndices(inliers);

			// Write the planar inliers to disk
			//extract.setNegative(false);
			//extract.filter(*cloud_plane);
			//std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

			// Remove the planar inliers, extract the rest
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGB>);
			extract.setNegative(true);
			extract.filter(*temp);
			cloud_filtered = temp;
		}

		// Creating the KdTree object for the search method of the extraction
		pcl_search<pcl::PointXYZRGB>::Ptr ktree(new pcl_search<pcl::PointXYZRGB>);
		ktree->setInputCloud(cloud_filtered);

		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	//	ec.setClusterTolerance(0.05); //0.05 //0.10// 2cm
		ec.setClusterTolerance(mClusterSearchToleranceValue); //0.05 //0.10// 2cm
		ec.setMinClusterSize(mClusterSearchMinClusterSize);
		ec.setMaxClusterSize(mClusterSearchMaxClusterSize);
		ec.setSearchMethod(ktree);
		ec.setInputCloud(cloud_filtered);
		ec.extract(cluster_indices);

		int height = pInputCloud->height;
		int width = pInputCloud->width;
		cv::Mat originalImage(cvSize(width, height), CV_8UC3);
		int u=0, v=0;
		for (unsigned int i=0; i<pInputCloud->size(); i++)
		{
			pcl::PointXYZRGB point = pInputCloud->at(i);
			originalImage.at< cv::Point3_<uchar> >(v, u) = cv::Point3_<uchar>(point.b, point.g, point.r);
			u++;
			if (u>=width)
			{
				u=0;
				v++;
			}
		}
		//std::stringstream ss;
		//ss << "common/files/live/img" << mImageNumber << ".png";
		//cv::imwrite(ss.str().c_str(), originalImage);
		cv::Mat clusterImage = cv::Mat::zeros(cvSize(width, height), CV_8UC3);

		mLastDetections = mCurrentDetections;
		mCurrentDetections.clear();

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
		//int j = 0;
		int clusterIndex = 0;
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
		{
			cv::Point3d avgPoint;
			avgPoint.x = 0; avgPoint.y = 0; avgPoint.z = 0;
			for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			{
				cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
				avgPoint.x += cloud_filtered->points[*pit].x;
				avgPoint.y += cloud_filtered->points[*pit].y;
				avgPoint.z += cloud_filtered->points[*pit].z;
			}

			std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

			//if ((fabs(avgPoint.x) < cloud_cluster->points.size()*0.15) && (fabs(avgPoint.y) < 0.30*cloud_cluster->points.size()))
			if ((fabs(avgPoint.x) < mConsideredClusterCenterVolume.x*cloud_cluster->points.size()) && (fabs(avgPoint.y) < mConsideredClusterCenterVolume.y*cloud_cluster->points.size()) && (fabs(avgPoint.z) < mConsideredClusterCenterVolume.z*cloud_cluster->points.size()))
			{
				std::cout << "found a cluster in the center" << std::endl;

				// write the pixels within the cluster back to an image
				pcl_search<pcl::PointXYZRGB>::Ptr selectionTree (new pcl_search<pcl::PointXYZRGB>);
				selectionTree->setInputCloud(cloud_cluster);

				IplImage* coordinateImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
				cvSetZero(coordinateImage);
				IplImage* colorImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
				cvSetZero(colorImage);

				int u=0, v=0;
				int umin=100000, vmin=100000;
				cv::Point3_<uchar> randomColor(255.*rand()/(double)RAND_MAX, 255.*rand()/(double)RAND_MAX, 255.*rand()/(double)RAND_MAX);
				for (unsigned int i=0; i<pInputCloud->size(); i++)
				{
					pcl::PointXYZRGB point = pInputCloud->at(i);

					if (isnan(point.z) == false)
					{
						std::vector<int> indices;
						std::vector<float> sqr_distances;
						selectionTree->nearestKSearch(point, 1, indices, sqr_distances);

						// if there is a voxel of the cluster close to the query point, save this point in the image
						if (sqr_distances[0] < 0.01*0.01)
						{
							clusterImage.at< cv::Point3_<uchar> >(v, u) = randomColor;
							cvSet2D(colorImage, v, u, CV_RGB(point.r, point.g, point.b));
							cvSet2D(coordinateImage, v, u, cvScalar(point.x, point.y, point.z));
							if (u<umin) umin=u;
							if (v<vmin) vmin=v;
						}
					}

					u++;
					if (u>=width)
					{
						u=0;
						v++;
					}
				}

				std::map<double, std::string> resultsOrdered;
				std::map<std::string, double> results;
				SharedImage si;
				si.setCoord(coordinateImage);
				si.setShared(colorImage);
				CategorizeObject(&si, results, resultsOrdered, pClusterMode, pClassifierTypeGlobal, pGlobalFeatureParams);
				si.Release();

				ObjectLocalizationIdentification oli;
				oli.objectCenter = avgPoint;
				oli.textPosition = cv::Point2i(umin, vmin);
				oli.identificationPDF = results;
				mCurrentDetections.push_back(oli);

				clusterIndex++;
			}
			cloud_cluster->clear();

			//std::stringstream ss;
			//ss << "cloud_cluster_" << j << ".pcd";
			//writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
			//j++;
		}

		// display results
		for (unsigned int i=0; i<mCurrentDetections.size(); i++)
		{
			// track detections over the frames
			int indexNearest = -1;
			double distanceSquaredNearest = 1e10;
			for (unsigned int j=0; j<mLastDetections.size(); j++)
			{
				double distSquared = (mCurrentDetections[i].objectCenter.x-mLastDetections[j].objectCenter.x)*(mCurrentDetections[i].objectCenter.x-mLastDetections[j].objectCenter.x)
					+ (mCurrentDetections[i].objectCenter.y-mLastDetections[j].objectCenter.y)*(mCurrentDetections[i].objectCenter.y-mLastDetections[j].objectCenter.y)
					+ (mCurrentDetections[i].objectCenter.z-mLastDetections[j].objectCenter.z)*(mCurrentDetections[i].objectCenter.z-mLastDetections[j].objectCenter.z);
				if (distSquared < distanceSquaredNearest)
				{
					indexNearest = j;
					distanceSquaredNearest = distSquared;
				}
			}

			// if there is a good fit with a previous detection, smooth the current result and remove the last result from the list
			if (distanceSquaredNearest < 0.2*0.2)
			{
				for (std::map<string, double>::iterator itPDF=mCurrentDetections[i].identificationPDF.begin(); itPDF!=mCurrentDetections[i].identificationPDF.end(); itPDF++)
					itPDF->second = mTrackingInfluenceFactorOldData*mLastDetections[indexNearest].identificationPDF[itPDF->first] + (1.0-mTrackingInfluenceFactorOldData)*itPDF->second;
				mLastDetections.erase(mLastDetections.begin()+indexNearest);
			}

			// find best result
			std::string predictedClass = "";
			double predictedConfidence = 0.0;
			for (std::map<string, double>::iterator itPDF=mCurrentDetections[i].identificationPDF.begin(); itPDF!=mCurrentDetections[i].identificationPDF.end(); itPDF++)
			{
				if (itPDF->second > predictedConfidence)
				{
					predictedClass = itPDF->first;
					predictedConfidence = itPDF->second;
				}
			}


			std::stringstream text1a;
			if (predictedClass.compare("coffee_mug") == 0)
				text1a << "cup";
			else if (predictedClass.compare("water_bottle") == 0)
				text1a << "bottle";
			else if (predictedClass.compare("shampoo") == 0)
				text1a << "bottle";
			else if (predictedClass.compare("food_can") == 0)
				text1a << "can";
			else if (predictedClass.compare("soda_can") == 0)
				text1a << "can";
			else if (predictedClass.compare("shampoo") == 0)
				text1a << "bottle";
			else if (predictedClass.compare("cereal_box") == 0)
				text1a << "box";
			else
				//if (predictedClass.second.compare("dishliquids") == 0)
				//	text1a << "bottle";
				//else
				text1a << predictedClass;
			text1a << " (" << setprecision(3) << 100*predictedConfidence << "%)";
			cv::putText(originalImage, text1a.str().c_str(), cvPoint(mCurrentDetections[i].textPosition.x, std::max(0,mCurrentDetections[i].textPosition.y-20)), cv::FONT_HERSHEY_SIMPLEX, 1.0, mDisplayFontColorBGR);
		}

		
		//if (cloud_cluster->size() == 0)
		//{
		//	std::cout << "Could not find any object in the center." << std::endl;
		//	return;
		//}
		std::cout << "\n-----------------------------------------------------------------------------------------------\n\n\n" << std::endl;
		//std::stringstream ss2,ss3;
		//ss2 << "common/files/live/img_ann" << mImageNumber << ".png";
		//cv::imwrite(ss2.str().c_str(), originalImage);
		//ss3 << "common/files/live/img_cluster" << mImageNumber << ".png";
		//cv::imwrite(ss3.str().c_str(), clusterImage);
		//mImageNumber++;

		{
			boost::mutex::scoped_lock lock(mDisplayImageMutex);
			int cutHeight = (int)(originalImage.cols/(double)mTargetScreenResolution.width * mTargetScreenResolution.height);
			cv::Mat originalImageROI = originalImage(cv::Rect(0, (originalImage.rows-cutHeight)/2, originalImage.cols, cutHeight));
			cv::Mat originalImageResized;
			cv::resize(originalImageROI, originalImageResized, mTargetScreenResolution);
			cv::Mat clusterImageResized;
			cv::resize(clusterImage, clusterImageResized, mTargetSegmentationImageResolution);
			mDisplayImageOriginal = originalImageResized;
			mDisplayImageSegmentation = clusterImageResized;
		}
		//cv::imshow("original image", originalImageResized);
		//cv::imshow("cluster image", clusterImage);
		//cv::waitKey(100);
		//cvMoveWindow("original image", 0, 0);
		//cvMoveWindow("cluster image", 1920-640, 0);
		std::cout << std::endl;

		//// write the pixels within the cluster back to an image
		//pcl_search<pcl::PointXYZRGB>::Ptr selectionTree (new pcl_search<pcl::PointXYZRGB>);
		//selectionTree->setInputCloud(cloud_cluster);
		//
		//int height = pInputCloud->height;
		//int width = pInputCloud->width;
		//IplImage* coordinateImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
		//cvSetZero(coordinateImage);
		//IplImage* colorImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
		//cvSetZero(colorImage);
		//IplImage* originalImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);

		//int u=0, v=0;
		//for (unsigned int i=0; i<(int)pInputCloud->size(); i++)
		//{
		//	pcl::PointXYZRGB point = pInputCloud->at(i);

		//	cvSet2D(originalImage, v, u, CV_RGB(point.r, point.g, point.b));

		//	if (isnan(point.z) == false)
		//	{
		//		std::vector<int> indices;
		//		std::vector<float> sqr_distances;
		//		selectionTree->nearestKSearch(point, 1, indices, sqr_distances);

		//		// if there is a voxel of the cluster close to the query point, save this point in the image
		//		if (sqr_distances[0] < 0.01*0.01)
		//		{
		//			cvSet2D(colorImage, v, u, CV_RGB(point.r, point.g, point.b));
		//			cvSet2D(coordinateImage, v, u, cvScalar(point.x, point.y, point.z));
		//		}
		//	}

		//	u++;
		//	if (u>=640)
		//	{
		//		u=0;
		//		v++;
		//	}
		//}

		//std::map<double, std::string> resultsOrdered;
		//SharedImage si;
		//si.setCoord(coordinateImage);
		//si.setShared(colorImage);
		//CategorizeObject(&si, resultsOrdered, pClusterMode, pClassifierTypeGlobal, pGlobalFeatureParams);
		//si.Release();

		//std::stringstream text1a, text1b, text2a, text2b;
		//std::map<double, std::string>::iterator it = resultsOrdered.end();
		//it--;
		//text1a << it->second;
		//text1b << setprecision(3) << 100*it->first << "%";
		//it--;
		//text2a << it->second;
		//text2b << setprecision(3) << 100*it->first << "%";
		//CvFont font;
		//cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1, 1);
		//cvPutText(originalImage, text1a.str().c_str(), cvPoint(10, 50), &font, CV_RGB(0, 255, 0));
		//cvPutText(originalImage, text1b.str().c_str(), cvPoint(320, 50), &font, CV_RGB(0, 255, 0));
		//cvPutText(originalImage, text2a.str().c_str(), cvPoint(10, 90), &font, CV_RGB(0, 255, 0));
		//cvPutText(originalImage, text2b.str().c_str(), cvPoint(320, 90), &font, CV_RGB(0, 255, 0));

		//cvShowImage("original image", originalImage);
		//cv::waitKey(10);

		//cvReleaseImage(&originalImage);
	}
	catch (exception& e)
	{
		std::cout << e.what() << std::endl;
		getchar();
	}
}







// capture code

int ObjectClassifier::CaptureSegmentedPCD(ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams)
{
#ifndef __LINUX__
	std::cout << "Input object name: ";
	std::cin >> mFilePrefix;

	mFinishCapture = false;
	mPanAngle = 0;
	mTiltAngle = 0;
	mFileCounter = 0;

	std::cout << "Begin file counter with number? ";
	std::cin >> mFileCounter;

	pcl::Grabber* kinectGrabber = new pcl::OpenNIGrabber();
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind(&ObjectClassifier::CapturePointcloudCallback, this, _1, pClusterMode, pClassifierTypeGlobal, pGlobalFeatureParams);
	kinectGrabber->registerCallback(f);
	kinectGrabber->start();

	std::cout << "Capture started.\n" << std::endl;

	while(!mFinishCapture)
	{
		//std::cout << "--------------------------------------------------------------" << std::endl;
		cvWaitKey(50);
	}

	kinectGrabber->stop();
	
#endif

	return ipa_utils::RET_OK;
}

void ObjectClassifier::CapturePointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pInputCloud, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams)
{
	if (pInputCloud->isOrganized() == false)
	{
		std::cout << "ObjectClassifier::PointcloudCallback: Warning: Point cloud is not organized. Skipping.\n" << std::endl;
		return;
	}

	if (mFinishCapture == true)
		return;

	// display image
	int width = pInputCloud->width;
	int height = pInputCloud->height;
	double scaleFactor = 255./5.;
	cv::Mat colorImage(height, width, CV_8UC3);
	cv::Mat depthImage(height, width, CV_8UC1);
	for (int v=0; v<height; v++)
	{
		for (int u=0; u<width; u++)
		{
			pcl::PointXYZRGB point = pInputCloud->points[v*width + u];
			cv::Vec3b rgb(point.b, point.g, point.r);
			colorImage.at<cv::Vec3b>(v,u) = rgb;
			depthImage.at<uchar>(v,u) = 255-(uchar)(point.z*scaleFactor);
		}
	}
	cv::imshow("depth image", depthImage);
	cv::imshow("color image", colorImage);
	int key = cv::waitKey(10);

	// check key inputs
	if (key=='q')
	{
		mFinishCapture = true;
		return;
	}
	else if (key=='c')
	{
		// capture an image, i.e. execute the remainder of the code
		cvDestroyWindow("cluster image");
	}
	else
	{
		// if no action initiated, then just return after the display
		return;
	}

	// segment incoming point cloud
	std::cout << "\nSegmenting data..." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	vg.setInputCloud(pInputCloud);
	vg.setLeafSize(0.0075f, 0.0075f, 0.0075f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->size()  << " data points left from " << pInputCloud->size() << "." << std::endl;

	if (cloud_filtered->points.size() == 0)
		return;

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	int nr_points = (int) cloud_filtered->points.size();
	while (cloud_filtered->points.size () > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Write the planar inliers to disk
		//extract.filter (*cloud_plane);
		//std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
		extract.setNegative (true);
		extract.filter(*temp);
		cloud_filtered = temp;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl_search<pcl::PointXYZRGB>::Ptr ktree (new pcl_search<pcl::PointXYZRGB>);
	ktree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.15); //0.05 //0.10// 2cm
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(ktree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);


	//int height = pInputCloud->height;
	//int width = pInputCloud->width;
	//IplImage* originalImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	//int u=0, v=0;
	//for (unsigned int i=0; i<(int)pInputCloud->size(); i++)
	//{
	//	pcl::PointXYZRGB point = pInputCloud->at(i);
	//	cvSet2D(originalImage, v, u, CV_RGB(point.r, point.g, point.b));
	//	u++;
	//	if (u>=width)
	//	{
	//		u=0;
	//		v++;
	//	}
	//}
	IplImage* clusterImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	cvSetZero(clusterImage);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<PointXYZRGBIM>::Ptr center_cluster (new pcl::PointCloud<PointXYZRGBIM>);
	int clusterIndex = 0;
	int numberGoodClusters = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointXYZ avgPoint;
		avgPoint.x = 0; avgPoint.y = 0; avgPoint.z = 0;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
		{
			PointXYZRGBIM point;
			point.x = cloud_filtered->points[*pit].x;
			point.y = cloud_filtered->points[*pit].y;
			point.z = cloud_filtered->points[*pit].z;
			uint32_t rgb = (((uint32_t)cloud_filtered->points[*pit].r<<16) + ((uint32_t)cloud_filtered->points[*pit].g<<8) + ((uint32_t)cloud_filtered->points[*pit].b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			center_cluster->points.push_back(point);
			cloud_cluster->push_back(cloud_filtered->points[*pit]);
			avgPoint.x += cloud_filtered->points[*pit].x;
			avgPoint.y += cloud_filtered->points[*pit].y;
			avgPoint.z += cloud_filtered->points[*pit].z;
		}
		avgPoint.x /= cloud_cluster->points.size();
		avgPoint.y /= cloud_cluster->points.size();
		avgPoint.z /= cloud_cluster->points.size();


		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		//if ((fabs(avgPoint.x) < cloud_cluster->points.size()*0.15) && (fabs(avgPoint.y) < 0.30*cloud_cluster->points.size()))
		if ((fabs(avgPoint.x) < 0.15) && (fabs(avgPoint.y) < 0.15) && (fabs(avgPoint.z) < 1.25))
		{
			std::cout << "found a cluster in the center" << std::endl;
			numberGoodClusters++;

			// write the pixels within the cluster back to an image
			pcl_search<pcl::PointXYZRGB>::Ptr selectionTree (new pcl_search<pcl::PointXYZRGB>);
			selectionTree->setInputCloud(cloud_cluster);

			int u=0, v=0;
			CvScalar randomColor = CV_RGB(255.*rand()/(double)RAND_MAX, 255.*rand()/(double)RAND_MAX, 255.*rand()/(double)RAND_MAX);
			for (unsigned int i=0; i<pInputCloud->size(); i++)
			{
				pcl::PointXYZRGB point = pInputCloud->at(i);

				if (isnan(point.z) == false)
				{
					std::vector<int> indices;
					std::vector<float> sqr_distances;
					selectionTree->nearestKSearch(point, 1, indices, sqr_distances);

					// if there is a voxel of the cluster close to the query point, save this point in the image
					if (sqr_distances[0] < 0.01*0.01)
					{
						cvSet2D(clusterImage, v, u, randomColor);
						center_cluster->at(indices[0]).imX = u;
						center_cluster->at(indices[0]).imY = v;
					}
				}

				u++;
				if (u>=width)
				{
					u=0;
					v++;
				}
			}

			clusterIndex++;
		}

		cloud_cluster->clear();
	}

	std::cout << "\n-----------------------------------------------------------------------------------------------\n\n\n" << std::endl;
	//cvShowImage("original image", originalImage);
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1, 1);
	bool saveData = false;
	if (numberGoodClusters != 1)
	{
		cvPutText(clusterImage, "Image automatically discarded!", cvPoint(20, 30), &font, CV_RGB(255, 0, 0));
	}
	else
	{
		cvPutText(clusterImage, "Save picture (y/n)?", cvPoint(20, 30), &font, CV_RGB(0, 255, 0));
		cvShowImage("cluster image", clusterImage);
		key = cv::waitKey();
		if (key == 'y' || key=='c')
		{
			// save image
			cvPutText(clusterImage, "Saved!", cvPoint(20, 70), &font, CV_RGB(0, 255, 0));
			saveData = true;
		}
		else
		{
			cvPutText(clusterImage, "Discarded!", cvPoint(20, 70), &font, CV_RGB(255, 0, 0));
		}
	}
	cvShowImage("cluster image", clusterImage);
	cv::waitKey(20);
	//cvReleaseImage(&originalImage);
	cvReleaseImage(&clusterImage);
	std::cout << std::endl;

	if (saveData)
	{
		// save raw data
		std::stringstream filename;
		filename << "common/files/capture/" << mFilePrefix << "_" << mFileCounter << ".pcd";
		center_cluster->height = 1;
		center_cluster->width = center_cluster->size();
		pcl::io::savePCDFileASCII(filename.str(), *center_cluster);
		mFileCounter++;

		std::cout << "Saved " << center_cluster->points.size () << " data points to " << filename.str() << "." << std::endl;
	}
}




// -------------------- Hermes code start ---------------------------------------------------------------------------------------

// Training
// 1. Capture image (press c, set camera angles with keys)
// 2. Segment object
// 3. Save raw data (coloured point cloud) to file, save labels and file names in a separate file
// 4. Compute descriptor and save it with angular tags

// Runtime
// 1. Load known descriptor data and labels
// 2. Capture image
// 3. Segment object cluster
// 4. Compute descriptor
// 5. Nearest Neighbor search among descriptors


int ObjectClassifier::HermesCapture(ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams)
{
#ifndef __LINUX__
	std::cout << "Input object name: ";
	std::cin >> mFilePrefix;

	std::stringstream metaFileName;
	metaFileName << "common/files/hermes/" << mFilePrefix << "_labels.txt";
	mLabelFile.open(metaFileName.str().c_str(), std::ios::app);
	if (mLabelFile.is_open() == false)
	{
		std::cout << "ObjectClassifier::HermesCapture: Error: Could not open " << metaFileName.str() << "." << std::endl;
		return ipa_utils::RET_FAILED;
	}

	mFinishCapture = false;
	mPanAngle = 0;
	mTiltAngle = 0;
	mFileCounter = 0;

	std::cout << "Begin file counter with number? ";
	std::cin >> mFileCounter;

	pcl::Grabber* kinectGrabber = new pcl::OpenNIGrabber();
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind(&ObjectClassifier::HermesPointcloudCallbackCapture, this, _1, pClusterMode, pClassifierTypeGlobal, pGlobalFeatureParams);
	kinectGrabber->registerCallback(f);
	kinectGrabber->start();

	std::cout << "Hermes capture started.\n" << std::endl;

	while (!mFinishCapture)
	{
		std::cout << "--------------------------------------------------------------" << std::endl;
		std::cout << "pan=" << mPanAngle << "   tilt=" << mTiltAngle << "\n" << std::endl;
		cvWaitKey(50);
	}

	kinectGrabber->stop();

	mLabelFile.close();

#endif

	return ipa_utils::RET_OK;
}

void ObjectClassifier::HermesPointcloudCallbackCapture(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pInputCloud, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams)
{
	if (pInputCloud->isOrganized() == false)
	{
		std::cout << "ObjectClassifier::PointcloudCallback: Warning: Point cloud is not organized. Skipping.\n" << std::endl;
		return;
	}

	if (mFinishCapture == true)
		return;

	// display image
	int width = pInputCloud->width;
	int height = pInputCloud->height;
	double scaleFactor = 255./5.;
	cv::Mat colorImage(height, width, CV_8UC3);
	cv::Mat depthImage(height, width, CV_8UC1);
	for (int v=0; v<height; v++)
	{
		for (int u=0; u<width; u++)
		{
			pcl::PointXYZRGB point = pInputCloud->points[v*width + u];
			cv::Vec3b rgb(point.b, point.g, point.r);
			colorImage.at<cv::Vec3b>(v,u) = rgb;
			depthImage.at<uchar>(v,u) = 255-(uchar)(point.z*scaleFactor);
		}
	}
	cv::imshow("depth image", depthImage);
	cv::imshow("color image", colorImage);
	int key = cv::waitKey(10);

	// check key inputs
	if (key=='q')
	{
		mFinishCapture = true;
		return;
	}
	else if (key=='j')
	{
		mPanAngle -= 5;
		cvDestroyWindow("cluster image");
		return;
	}
	else if (key=='l')
	{
		mPanAngle += 5;
		cvDestroyWindow("cluster image");
		return;
	}
	else if (key=='k')
	{
		mTiltAngle -= 5;
		cvDestroyWindow("cluster image");
		return;
	}
	else if (key=='i')
	{
		mTiltAngle += 5;
		cvDestroyWindow("cluster image");
		return;
	}
	else if (key=='c')
	{
		// capture an image, i.e. execute the remainder of the code
		cvDestroyWindow("cluster image");
	}
	else
	{
		// if no action initiated, then just return after the display
		return;
	}

	// segment incoming point cloud
	std::cout << "\nSegmenting data..." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	vg.setInputCloud(pInputCloud);
	vg.setLeafSize(0.0075f, 0.0075f, 0.0075f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->size()  << " data points left from " << pInputCloud->size() << "." << std::endl;

	if (cloud_filtered->points.size() == 0)
		return;

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	int nr_points = (int) cloud_filtered->points.size();
	while (cloud_filtered->points.size () > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Write the planar inliers to disk
		//extract.filter (*cloud_plane);
		//std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
		extract.setNegative (true);
		extract.filter(*temp);
		cloud_filtered = temp;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl_search<pcl::PointXYZRGB>::Ptr ktree (new pcl_search<pcl::PointXYZRGB>);
	ktree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.15); //0.05 //0.10// 2cm
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(ktree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);


	//int height = pInputCloud->height;
	//int width = pInputCloud->width;
	//IplImage* originalImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	//int u=0, v=0;
	//for (unsigned int i=0; i<(int)pInputCloud->size(); i++)
	//{
	//	pcl::PointXYZRGB point = pInputCloud->at(i);
	//	cvSet2D(originalImage, v, u, CV_RGB(point.r, point.g, point.b));
	//	u++;
	//	if (u>=width)
	//	{
	//		u=0;
	//		v++;
	//	}
	//}
	IplImage* clusterImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	cvSetZero(clusterImage);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr center_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	SharedImage si;
	cv::Mat histogram;
	//int j = 0;
	int clusterIndex = 0;
	int numberGoodClusters = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointXYZ avgPoint;
		avgPoint.x = 0; avgPoint.y = 0; avgPoint.z = 0;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
		{
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
			avgPoint.x += cloud_filtered->points[*pit].x;
			avgPoint.y += cloud_filtered->points[*pit].y;
			avgPoint.z += cloud_filtered->points[*pit].z;
		}
		avgPoint.x /= cloud_cluster->points.size();
		avgPoint.y /= cloud_cluster->points.size();
		avgPoint.z /= cloud_cluster->points.size();


		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		//if ((fabs(avgPoint.x) < cloud_cluster->points.size()*0.15) && (fabs(avgPoint.y) < 0.30*cloud_cluster->points.size()))
		if ((fabs(avgPoint.x) < 0.15) && (fabs(avgPoint.y) < 0.15) && (fabs(avgPoint.z) < 1.25))
		{
			std::cout << "found a cluster in the center" << std::endl;
			*center_cluster = *cloud_cluster;
			numberGoodClusters++;

			// write the pixels within the cluster back to an image
			pcl_search<pcl::PointXYZRGB>::Ptr selectionTree (new pcl_search<pcl::PointXYZRGB>);
			selectionTree->setInputCloud(cloud_cluster);

			IplImage* coordinateImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
			cvSetZero(coordinateImage);
			IplImage* colorImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
			cvSetZero(colorImage);

			int u=0, v=0;
			int umin=100000, vmin=100000;
			CvScalar randomColor = CV_RGB(255.*rand()/(double)RAND_MAX, 255.*rand()/(double)RAND_MAX, 255.*rand()/(double)RAND_MAX);
			for (unsigned int i=0; i<pInputCloud->size(); i++)
			{
				pcl::PointXYZRGB point = pInputCloud->at(i);

				if (isnan(point.z) == false)
				{
					std::vector<int> indices;
					std::vector<float> sqr_distances;
					selectionTree->nearestKSearch(point, 1, indices, sqr_distances);

					// if there is a voxel of the cluster close to the query point, save this point in the image
					if (sqr_distances[0] < 0.01*0.01)
					{
						cvSet2D(clusterImage, v, u, randomColor);
						cvSet2D(colorImage, v, u, CV_RGB(point.r, point.g, point.b));
						cvSet2D(coordinateImage, v, u, cvScalar(point.x, point.y, point.z));
						if (u<umin) umin=u;
						if (v<vmin) vmin=v;
					}
				}

				u++;
				if (u>=width)
				{
					u=0;
					v++;
				}
			}

			si.Release();
			si.setCoord(coordinateImage);
			si.setShared(colorImage);

			HermesComputeRollHistogram(center_cluster, avgPoint, histogram, true, true);

			clusterIndex++;
		}

		cloud_cluster->clear();

	}

	std::cout << "\n-----------------------------------------------------------------------------------------------\n\n\n" << std::endl;
	//cvShowImage("original image", originalImage);
	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1, 1);
	bool saveData = false;
	if (numberGoodClusters != 1)
	{
		cvPutText(clusterImage, "Image automatically discarded!", cvPoint(20, 30), &font, CV_RGB(255, 0, 0));
	}
	else
	{
		cvPutText(clusterImage, "Save picture (y/n)?", cvPoint(20, 30), &font, CV_RGB(0, 255, 0));
		cvShowImage("cluster image", clusterImage);
		key = cv::waitKey();
		if (key == 'y')
		{
			// save image
			cvPutText(clusterImage, "Saved!", cvPoint(20, 70), &font, CV_RGB(0, 255, 0));
			saveData = true;
		}
		else
		{
			cvPutText(clusterImage, "Discarded!", cvPoint(20, 70), &font, CV_RGB(255, 0, 0));
		}
	}
	cvShowImage("cluster image", clusterImage);
	cv::waitKey(20);
	//cvReleaseImage(&originalImage);
	cvReleaseImage(&clusterImage);
	std::cout << std::endl;

	if (saveData)
	{
		// save raw data
		std::stringstream filename;
		filename << "common/files/hermes/" << mFilePrefix << "_" << mFileCounter << ".pcd";
		center_cluster->height = 1;
		center_cluster->width = center_cluster->size();
		pcl::io::savePCDFileASCII(filename.str(), *center_cluster);
		mFileCounter++;
		mLabelFile << filename.str() << "\t" << mPanAngle << "\t" << mTiltAngle << std::endl;


		/////////////////////
		// create a pseudo blob
		BlobFeatureRiB Blob;
		BlobListRiB Blobs;
		ipa_utils::IntVector Keys;

		Blob.m_x = 0;
		Blob.m_y = 0;
		Blob.m_r = 2;
		Blob.m_Phi = 0;
		Blob.m_Res = 0;
		Blob.m_Id = 1;
		for (unsigned int j=0; j<64; j++) Blob.m_D.push_back(1);
		Blob.m_D.push_back(1);		// min/max curvature approximate
		Blob.m_D.push_back(1);
		for (int i=0; i<10; i++) Blobs.push_back(Blob);

		// Use Mask for global feature extraction
		CvMat** featureVector = new CvMat*;
		*featureVector = NULL;
		IplImage* mask = cvCreateImage(cvGetSize(si.Shared()), si.Shared()->depth, 1);
		cvCvtColor(si.Shared(), mask, CV_RGB2GRAY);

		// SAP feature
		pGlobalFeatureParams.useFeature["sap"] = true;
		pGlobalFeatureParams.useFeature["vfh"] = false;
		ExtractGlobalFeatures(&Blobs, featureVector, pClusterMode, pGlobalFeatureParams, INVALID, si.Coord(), mask, NULL, false, "common/files/timing.txt");
		mLabelFile << "sap-" << pGlobalFeatureParams.numberLinesX[0] << "-" << pGlobalFeatureParams.numberLinesY[0] << "-" << pGlobalFeatureParams.polynomOrder[0] << "\t" << (*featureVector)->cols << "\t";
		for (int i=0; i<(*featureVector)->cols; i++)
			mLabelFile << cvGetReal1D(*featureVector, i) << "\t";
		mLabelFile << std::endl;

		// VFH
		pGlobalFeatureParams.useFeature["sap"] = false;
		pGlobalFeatureParams.useFeature["vfh"] = true;
		ExtractGlobalFeatures(&Blobs, featureVector, pClusterMode, pGlobalFeatureParams, INVALID, si.Coord(), mask, NULL, false, "common/files/timing.txt");
		mLabelFile << "vfh\t" << (*featureVector)->cols << "\t";
		for (int i=0; i<(*featureVector)->cols; i++)
			mLabelFile << cvGetReal1D(*featureVector, i) << "\t";
		mLabelFile << std::endl;

		mLabelFile << "rollhistogram\t" << histogram.cols << "\t";
		for (int i=0; i<histogram.cols; i++)
			mLabelFile << histogram.at<float>(i) << "\t";
		mLabelFile << std::endl;

		// free memory
		cvReleaseImage(&mask);
		si.Release();
		cvReleaseMat(featureVector);

		/////////////////////


		std::cout << "Saved " << center_cluster->points.size () << " data points to " << filename.str() << "." << std::endl;
	}
}


int ObjectClassifier::HermesBuildDetectionModelFromRecordedData(const std::string& object_name, const cv::Mat& projection_matrix, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams)
{
	// create subfolders for data storage
	std::string data_storage_path = std::string(getenv("HOME")) + "/.ros/";

	fs::path top_level_directory(data_storage_path);
	if (fs::is_directory(top_level_directory) == false)
	{
		std::cerr << "ERROR - ObjectClassifier::HermesBuildDetectionModelFromRecordedData:" << std::endl;
		std::cerr << "\t ... Path '" << top_level_directory.string() << "' is not a directory." << std::endl;
		return false;
	}

	fs::path object_recording_data_storage_directory = top_level_directory / fs::path("cob_object_recording/");
	if (fs::is_directory(object_recording_data_storage_directory) == false)
	{
		std::cerr << "ERROR - ObjectClassifier::HermesBuildDetectionModelFromRecordedData:" << std::endl;
		std::cerr << "\t ... Could not create path '" << object_recording_data_storage_directory.string() << "'." << std::endl;
		return false;
	}

	fs::path object_recording_data_instance_storage_directory = object_recording_data_storage_directory / fs::path(object_name);
	if (fs::is_directory(object_recording_data_instance_storage_directory) == false)
	{
		std::cerr << "ERROR - ObjectClassifier::HermesBuildDetectionModelFromRecordedData:" << std::endl;
		std::cerr << "\t ... Could not create path '" << object_recording_data_instance_storage_directory.string() << "'." << std::endl;
		return false;
	}

	fs::path object_model_data_storage_directory_package = top_level_directory / fs::path("cob_object_categorization/");
	if (fs::is_directory(object_model_data_storage_directory_package) == false)
	{
		// create subfolder
		if (fs::create_directory(object_model_data_storage_directory_package) == false)
		{
			std::cerr << "ERROR - ObjectClassifier::HermesBuildDetectionModelFromRecordedData:" << std::endl;
			std::cerr << "\t ... Could not create path '" << object_model_data_storage_directory_package.string() << "'." << std::endl;
			return false;
		}
	}

	fs::path object_model_data_storage_directory_hermes = object_model_data_storage_directory_package / fs::path("hermes/");
	if (fs::is_directory(object_model_data_storage_directory_hermes) == false)
	{
		// create subfolder
		if (fs::create_directory(object_model_data_storage_directory_hermes) == false)
		{
			std::cerr << "ERROR - ObjectClassifier::HermesBuildDetectionModelFromRecordedData:" << std::endl;
			std::cerr << "\t ... Could not create path '" << object_model_data_storage_directory_hermes.string() << "'." << std::endl;
			return false;
		}
	}

	std::string metaFileName = object_name + "_labels.txt";
	fs::path metaFilePath = object_model_data_storage_directory_hermes / fs::path(metaFileName);
	mLabelFile.open(metaFilePath.string().c_str(), std::ios::out);
	if (mLabelFile.is_open() == false)
	{
		std::cout << "ObjectClassifier::HermesCapture: Error: Could not open " << metaFilePath.string().c_str() << "." << std::endl;
		return ipa_utils::RET_FAILED;
	}

	for (fs::directory_iterator classDirIter(object_recording_data_instance_storage_directory.string()); classDirIter!=fs::directory_iterator(); ++classDirIter)
	{
		std::string path = classDirIter->path().string();
		std::string fileName = fs::basename(path);
		std::string extension = fs::extension(path);

		//std::cout << "path=" << path << "    className=" << fileName << "     extension=" << extension << std::endl;

		if (extension.compare(".pcd") != 0)
			continue;

		int image_width = 640;
		int image_height = 480;

		// load pointcloud
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::io::loadPCDFile(path, *pointcloud);
		std::string pcd_path = object_model_data_storage_directory_hermes.string() + object_name + "_" + fileName + extension;
		pcl::io::savePCDFile(pcd_path, *pointcloud, true);

		// compute pan and tilt
		double pan = atan2(pointcloud->sensor_origin_[1], pointcloud->sensor_origin_[0]);
		double tilt = asin(pointcloud->sensor_origin_[2]/sqrt(pointcloud->sensor_origin_[0]*pointcloud->sensor_origin_[0] + pointcloud->sensor_origin_[1]*pointcloud->sensor_origin_[1] + pointcloud->sensor_origin_[2]*pointcloud->sensor_origin_[2]));

		mLabelFile << pcd_path << "\t" << pan << "\t" << tilt << std::endl;
		std::cout << pcd_path << "\t" << pan << "\t" << tilt << std::endl;

		// center pointcloud and convert to shared image
		IplImage* color_image = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_8U, 3);
		cvSetZero(color_image);
		IplImage* coordinate_image = cvCreateImage(cvSize(image_width, image_height), IPL_DEPTH_32F, 3);
		cvSetZero(coordinate_image);
		pcl::PointXYZ avgPoint(0., 0., 0.);
		unsigned int numberValidPoints = 0;
		for (unsigned int i=0; i<pointcloud->size(); i++)
		{
			if ((*pointcloud)[i].x==0 && (*pointcloud)[i].y==0 && (*pointcloud)[i].z==0)
				continue;
			++numberValidPoints;
			avgPoint.x += (*pointcloud)[i].x;
			avgPoint.y += (*pointcloud)[i].y;
			avgPoint.z += (*pointcloud)[i].z;
			cv::Mat X = (cv::Mat_<double>(4, 1) << (*pointcloud)[i].x, (*pointcloud)[i].y, (*pointcloud)[i].z, 1.0);
			cv::Mat x = projection_matrix * X;
			int v = x.at<double>(1)/x.at<double>(2), u = x.at<double>(0)/x.at<double>(2);
			cvSet2D(color_image, v, u, CV_RGB((*pointcloud)[i].r, (*pointcloud)[i].g, (*pointcloud)[i].b));
			cvSet2D(coordinate_image, v, u, cvScalar((*pointcloud)[i].x, (*pointcloud)[i].y, (*pointcloud)[i].z));
		}
		avgPoint.x /= (double)numberValidPoints;
		avgPoint.y /= (double)numberValidPoints;
		avgPoint.z /= (double)numberValidPoints;

		SharedImage si;
		si.setCoord(coordinate_image);
		si.setShared(color_image);

		/////////////////////
		// create a pseudo blob
		BlobFeatureRiB Blob;
		BlobListRiB Blobs;
		ipa_utils::IntVector Keys;

		Blob.m_x = 0;
		Blob.m_y = 0;
		Blob.m_r = 2;
		Blob.m_Phi = 0;
		Blob.m_Res = 0;
		Blob.m_Id = 1;
		for (unsigned int j=0; j<64; j++) Blob.m_D.push_back(1);
		Blob.m_D.push_back(1);		// min/max curvature approximate
		Blob.m_D.push_back(1);
		for (int i=0; i<10; i++) Blobs.push_back(Blob);

		// Use Mask for global feature extraction
		CvMat** featureVector = new CvMat*;
		*featureVector = NULL;
		IplImage* mask = cvCreateImage(cvGetSize(si.Shared()), si.Shared()->depth, 1);
		cvCvtColor(si.Shared(), mask, CV_RGB2GRAY);

		// SAP feature
		pGlobalFeatureParams.useFeature["sap"] = true;
		pGlobalFeatureParams.useFeature["vfh"] = false;
		ExtractGlobalFeatures(&Blobs, featureVector, pClusterMode, pGlobalFeatureParams, INVALID, si.Coord(), mask, NULL, false, "");
		mLabelFile << "sap-" << pGlobalFeatureParams.numberLinesX[0] << "-" << pGlobalFeatureParams.numberLinesY[0] << "-" << pGlobalFeatureParams.polynomOrder[0] << "\t" << (*featureVector)->cols << "\t";
		for (int i=0; i<(*featureVector)->cols; i++)
			mLabelFile << cvGetReal1D(*featureVector, i) << "\t";
		mLabelFile << std::endl;

		// VFH
		pGlobalFeatureParams.useFeature["sap"] = false;
		pGlobalFeatureParams.useFeature["vfh"] = true;
		ExtractGlobalFeatures(&Blobs, featureVector, pClusterMode, pGlobalFeatureParams, INVALID, si.Coord(), mask, NULL, false, "");
		mLabelFile << "vfh\t" << (*featureVector)->cols << "\t";
		for (int i=0; i<(*featureVector)->cols; i++)
			mLabelFile << cvGetReal1D(*featureVector, i) << "\t";
		mLabelFile << std::endl;

		// roll histogram
		cv::Mat histogram;
		HermesComputeRollHistogram(pointcloud, avgPoint, histogram, true, true);
		mLabelFile << "rollhistogram\t" << histogram.cols << "\t";
		for (int i=0; i<histogram.cols; i++)
			mLabelFile << histogram.at<float>(i) << "\t";
		mLabelFile << std::endl;

		// free memory
		cvReleaseImage(&mask);
		si.Release();
		cvReleaseMat(featureVector);

		/////////////////////
	}

	mLabelFile.close();

	return ipa_utils::RET_OK;
}

int ObjectClassifier::HermesLoadCameraCalibration(const std::string& object_name, cv::Mat& projection_matrix)
{
	std::string data_storage_path = std::string(getenv("HOME")) + "/.ros/";

	fs::path top_level_directory(data_storage_path);
	if (fs::is_directory(top_level_directory) == false)
	{
		std::cerr << "ERROR - ObjectClassifier::HermesLoadCameraCalibration:" << std::endl;
		std::cerr << "\t ... Path '" << top_level_directory.string() << "' is not a directory." << std::endl;
		return false;
	}

	fs::path object_recording_data_storage_directory = top_level_directory / fs::path("cob_object_recording/");
	if (fs::is_directory(object_recording_data_storage_directory) == false)
	{
		std::cerr << "ERROR - ObjectClassifier::HermesLoadCameraCalibration:" << std::endl;
		std::cerr << "\t ... Could not create path '" << object_recording_data_storage_directory.string() << "'." << std::endl;
		return false;
	}

	fs::path object_recording_data_instance_storage_directory = object_recording_data_storage_directory / fs::path(object_name);
	if (fs::is_directory(object_recording_data_instance_storage_directory) == false)
	{
		std::cerr << "ERROR - ObjectClassifier::HermesLoadCameraCalibration:" << std::endl;
		std::cerr << "\t ... Could not create path '" << object_recording_data_instance_storage_directory.string() << "'." << std::endl;
		return false;
	}

	std::string calibration_filename = object_recording_data_instance_storage_directory.string() + "/camera_calibration.txt";
	std::ifstream calibration_file(calibration_filename.c_str(), std::ios::in);
	if (calibration_file.is_open() == false)
	{
		std::cerr << "ERROR - ObjectClassifier::HermesLoadCameraCalibration:" << std::endl;
		std::cerr << "\t ... Could not create file '" << calibration_filename << "'." << std::endl;
		return false;
	}
	projection_matrix.create(3, 4, CV_64FC1);
	for (int v=0; v<3; ++v)
		for (int u=0; u<4; ++u)
			calibration_file >> projection_matrix.at<double>(v,u);
	calibration_file.close();

	return ipa_utils::RET_OK;
}


int ObjectClassifier::HermesDetectInit(ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams)
{
	std::cout << "Input object name: ";
	std::cin >> mFilePrefix;

	std::string metaFileName = std::string(getenv("HOME")) + "/.ros/cob_object_categorization/hermes/" + mFilePrefix + "_labels.txt";

	mLabelFile.open(metaFileName.c_str(), std::ios::in);
	if (mLabelFile.is_open() == false)
	{
		std::cout << "ObjectClassifier::HermesCapture: Error: Could not open " << metaFileName << "." << std::endl;
		return ipa_utils::RET_FAILED;
	}

	// read in data
	while (mLabelFile.eof() == false)
	{
		std::string tempstr;
		double pan, tilt;
		int descSize;
		mLabelFile >> tempstr;  // filename
		mLabelFile >> pan;
		mLabelFile >> tilt;
		if (mReferenceFilenames.find(pan) == mReferenceFilenames.end() || mReferenceFilenames[pan].find(tilt) == mReferenceFilenames[pan].end())
			mReferenceFilenames[pan][tilt] = tempstr;
		mLabelFile >> tempstr;
		mLabelFile >> descSize;
		std::vector<float> sapDesc(descSize);
		for (int i=0; i<descSize; i++)
			mLabelFile >> sapDesc[i];
		mSapData[pan][tilt].push_back(sapDesc);
		mLabelFile >> tempstr;
		mLabelFile >> descSize;
		std::vector<float> vfhDesc(descSize);
		for (int i=0; i<descSize; i++)
			mLabelFile >> vfhDesc[i];
		mVfhData[pan][tilt].push_back(vfhDesc);
		mLabelFile >> tempstr;
		mLabelFile >> descSize;
		std::vector<float> histogram(descSize);
		for (int i=0; i<descSize; i++)
			mLabelFile >> histogram[i];
		mRollHistogram[pan][tilt].push_back(histogram);
	}
	mLabelFile.close();

	// compute mean descriptor for each angle
	std::map<double, std::map<double, std::vector<std::vector<float> > > >::iterator itOuter;
	std::map<double, std::vector<std::vector<float> > >::iterator itInner;
	for (itOuter = mSapData.begin(); itOuter != mSapData.end(); itOuter++)
	{
		for (itInner = itOuter->second.begin(); itInner != itOuter->second.end(); itInner++)
		{
			int samples = itInner->second.size();
			std::vector<float> avgSapDesc(itInner->second[0].size(), 0);
			for (int i=0; i<samples; i++)
			{
				for (int j=0; j<(int)avgSapDesc.size(); j++)
					avgSapDesc[j] += itInner->second[i][j];
			}
			for (int j=0; j<(int)avgSapDesc.size(); j++)
				avgSapDesc[j] /= samples;
			itInner->second.clear();
			itInner->second.push_back(avgSapDesc);
		}
	}
	
	for (itOuter = mVfhData.begin(); itOuter != mVfhData.end(); itOuter++)
	{
		for (itInner = itOuter->second.begin(); itInner != itOuter->second.end(); itInner++)
		{
			int samples = itInner->second.size();
			std::vector<float> avgVfhDesc(itInner->second[0].size(), 0);
			for (int i=0; i<samples; i++)
			{
				for (int j=0; j<(int)avgVfhDesc.size(); j++)
					avgVfhDesc[j] += itInner->second[i][j];
			}
			for (int j=0; j<(int)avgVfhDesc.size(); j++)
				avgVfhDesc[j] /= samples;
			itInner->second.clear();
			itInner->second.push_back(avgVfhDesc);
		}
	}

	// mean roll histogram
	for (itOuter = mRollHistogram.begin(); itOuter != mRollHistogram.end(); itOuter++)
	{
		for (itInner = itOuter->second.begin(); itInner != itOuter->second.end(); itInner++)
		{
			int samples = itInner->second.size();
			std::vector<float> avgRollHistogram(itInner->second[0].size(), 0);
			for (int i=0; i<samples; i++)
			{
				for (int j=0; j<(int)avgRollHistogram.size(); j++)
					avgRollHistogram[j] += itInner->second[i][j];
			}
			for (int j=0; j<(int)avgRollHistogram.size(); j++)
				avgRollHistogram[j] /= samples;
			itInner->second.clear();
			itInner->second.push_back(avgRollHistogram);
		}
	}

	std::cout << "Data for object " << mFilePrefix << " loaded." << std::endl;

#ifndef __LINUX__

	pcl::Grabber* kinectGrabber = new pcl::OpenNIGrabber();
	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f = boost::bind(&ObjectClassifier::HermesPointcloudCallbackDetect, this, _1, pClusterMode, pClassifierTypeGlobal, pGlobalFeatureParams);
	kinectGrabber->registerCallback(f);
	kinectGrabber->start();

	std::cout << "Hermes capture started.\n" << std::endl;

	mFinishCapture = false;
	while (!mFinishCapture)
	{
		cvWaitKey(50);
	}

	kinectGrabber->stop();

#endif

	return ipa_utils::RET_OK;
}


void ObjectClassifier::HermesPointcloudCallbackDetect(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pInputCloud, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams)
{
	if (pInputCloud->isOrganized() == false)
	{
		std::cout << "ObjectClassifier::PointcloudCallback: Warning: Point cloud is not organized. Skipping.\n" << std::endl;
		return;
	}

	if (mFinishCapture == true)
		return;

	// display image
	int width = pInputCloud->width;
	int height = pInputCloud->height;
	double scaleFactor = 255./5.;
	cv::Mat colorImage(height, width, CV_8UC3);
	cv::Mat depthImage(height, width, CV_8UC1);
	for (int v=0; v<height; v++)
	{
		for (int u=0; u<width; u++)
		{
			pcl::PointXYZRGB point = pInputCloud->points[v*width + u];
			cv::Vec3b rgb(point.b, point.g, point.r);
			colorImage.at<cv::Vec3b>(v,u) = rgb;
			depthImage.at<uchar>(v,u) = 255-(uchar)(point.z*scaleFactor);
		}
	}
	cv::imshow("depth image", depthImage);
	cvMoveWindow("depth image", 640, 0);
	cv::imshow("color image", colorImage);
	cvMoveWindow("color image", 0, 0);
	int key = cv::waitKey(10);

	// check key inputs
	if (key=='q')
	{
		mFinishCapture = true;
		return;
	}

	// segment incoming point cloud
	std::cout << "\nSegmenting data..." << std::endl;

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::VoxelGrid<pcl::PointXYZRGB> vg;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	vg.setInputCloud(pInputCloud);
	vg.setLeafSize(0.0075f, 0.0075f, 0.0075f);
	//vg.setLeafSize(0.015f, 0.015f, 0.015f);
	vg.filter(*cloud_filtered);
	std::cout << "PointCloud after filtering has: " << cloud_filtered->size()  << " data points left from " << pInputCloud->size() << "." << std::endl;

	if (cloud_filtered->points.size() == 0)
		return;

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	int nr_points = (int) cloud_filtered->points.size();
	while (cloud_filtered->points.size () > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

		// Write the planar inliers to disk
		//extract.filter (*cloud_plane);
		//std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
		extract.setNegative (true);
		extract.filter(*temp);
		cloud_filtered = temp;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl_search<pcl::PointXYZRGB>::Ptr ktree (new pcl_search<pcl::PointXYZRGB>);
	ktree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
	ec.setClusterTolerance(0.15); //0.05 //0.10// 2cm
	ec.setMinClusterSize(50);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(ktree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);


	IplImage* clusterImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	cvSetZero(clusterImage);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
	//int j = 0;
	int clusterIndex = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointXYZ avgPoint;
		avgPoint.x = 0; avgPoint.y = 0; avgPoint.z = 0;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
		{
			cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
			avgPoint.x += cloud_filtered->points[*pit].x;
			avgPoint.y += cloud_filtered->points[*pit].y;
			avgPoint.z += cloud_filtered->points[*pit].z;
		}
		avgPoint.x /= cloud_cluster->points.size();
		avgPoint.y /= cloud_cluster->points.size();
		avgPoint.z /= cloud_cluster->points.size();

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

		//if ((fabs(avgPoint.x) < cloud_cluster->points.size()*0.15) && (fabs(avgPoint.y) < 0.30*cloud_cluster->points.size()))
		if ((fabs(avgPoint.x) < 0.15) && (fabs(avgPoint.y) < 0.15) && (fabs(avgPoint.z) < 1.0))
		{
			std::cout << "found a cluster in the center" << std::endl;

			// write the pixels within the cluster back to an image
			pcl_search<pcl::PointXYZRGB>::Ptr selectionTree (new pcl_search<pcl::PointXYZRGB>);
			selectionTree->setInputCloud(cloud_cluster);

			IplImage* coordinateImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 3);
			cvSetZero(coordinateImage);
			IplImage* colorImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
			cvSetZero(colorImage);

			int u=0, v=0;
			int umin=100000, vmin=100000;
			CvScalar randomColor = CV_RGB(255.*rand()/(double)RAND_MAX, 255.*rand()/(double)RAND_MAX, 255.*rand()/(double)RAND_MAX);
			for (unsigned int i=0; i<pInputCloud->size(); i++)
			{
				pcl::PointXYZRGB point = pInputCloud->at(i);

				if (isnan(point.z) == false)
				{
					std::vector<int> indices;
					std::vector<float> sqr_distances;
					selectionTree->nearestKSearch(point, 1, indices, sqr_distances);

					// if there is a voxel of the cluster close to the query point, save this point in the image
					if (sqr_distances[0] < 0.01*0.01)
					{
						cvSet2D(clusterImage, v, u, randomColor);
						cvSet2D(colorImage, v, u, CV_RGB(point.r, point.g, point.b));
						cvSet2D(coordinateImage, v, u, cvScalar(point.x, point.y, point.z));
						if (u<umin) umin=u;
						if (v<vmin) vmin=v;
					}
				}

				u++;
				if (u>=width)
				{
					u=0;
					v++;
				}
			}

			SharedImage si;
			si.setCoord(coordinateImage);
			si.setShared(colorImage);

			double pan=0, tilt=0, roll=0;
			Eigen::Matrix4f finalTransform;
			HermesCategorizeObject(cloud_cluster, avgPoint, &si, pClusterMode, pClassifierTypeGlobal, pGlobalFeatureParams, pan, tilt, roll, finalTransform);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//			// create a pseudo blob
//			BlobFeatureRiB Blob;
//			BlobListRiB Blobs;
//			ipa_utils::IntVector Keys;
//
//			Blob.m_x = 0;
//			Blob.m_y = 0;
//			Blob.m_r = 2;
//			Blob.m_Phi = 0;
//			Blob.m_Res = 0;
//			Blob.m_Id = 1;
//			for (unsigned int j=0; j<64; j++) Blob.m_D.push_back(1);
//			Blob.m_D.push_back(1);		// min/max curvature approximate
//			Blob.m_D.push_back(1);
//			for (int i=0; i<10; i++) Blobs.push_back(Blob);
//
//			// Use Mask for global feature extraction
//			CvMat** featureVector = new CvMat*;
//			*featureVector = NULL;
//			IplImage* mask = cvCreateImage(cvGetSize(si.Shared()), si.Shared()->depth, 1);
//			cvCvtColor(si.Shared(), mask, CV_RGB2GRAY);
//
//			std::map<double, std::map<double, std::vector<std::vector<float> > > >::iterator itOuter;
//			std::map<double, std::vector<std::vector<float> > >::iterator itInner;
//			std::multimap<double, std::pair<double, double> > sapOrderedList;	// [difference score](pan, tilt)
//			std::multimap<double, std::pair<double, double> > vfhOrderedList;
//			std::multimap<double, std::pair<double, double> >::iterator itOrderedList;
//			CvFont font;
//			cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1, 1);
//			std::stringstream displayText;
//
//			// SAP feature
///*			std::cout << "SAP response:" << std::endl;
//			pGlobalFeatureParams.useFeature["sap"] = true;
//			pGlobalFeatureParams.useFeature["vfh"] = false;
//			ExtractGlobalFeatures(&Blobs, featureVector, pClusterMode, pGlobalFeatureParams, INVALID, si.Coord(), mask, false, false, "common/files/timing.txt");
//			for (itOuter = mSapData.begin(); itOuter != mSapData.end(); itOuter++)
//			{
//				for (itInner = itOuter->second.begin(); itInner != itOuter->second.end(); itInner++)
//				{
//					double diff = 0.;
//					for (int i=0; i<3; i++)
//					{
//						double val = cvGetReal1D(*featureVector, i) - itInner->second[0][i];
//						diff += val*val/itInner->second[0][i];
//					}
//					for (int i=3;i<(*featureVector)->cols;i+=3)
//					{
//						double a = cvGetReal1D(*featureVector, i);
//						double a_ = a;
//						if (a_==0.) a_ = 0.01;
//						double b = cvGetReal1D(*featureVector, i+1);
//						double c = cvGetReal1D(*featureVector, i+2);
//						double p = b/(2*a_);
//						double q = c - p*p*a_;
//						double ap = itInner->second[0][i];
//						double ap_ = ap;
//						if (ap_==0.) { ap = a+0.1; ap_ = 1; }
//						double bp = itInner->second[0][i+1];
//						double bp_ = bp;
//						if (bp_==0.) { bp = b+0.1; bp_ = 1; }
//						double cp = itInner->second[0][i+2];
//						double cp_ = cp;
//						if (cp_==0.) { cp = c+0.1; cp_ = 1; }
//						double pp = bp/(2*ap_);
//						double pp_ = pp;
//						if (pp_==0.) pp_ = 1;
//						double qp = cp - pp*pp*ap_;
//						double qp_ = qp;
//						if (qp_==0.) qp_ = 1;
//						diff += (a-ap)*(a-ap)/(ap_*ap_) + (b-bp)*(b-bp)/(bp_*bp_) + (c-cp)*(c-cp)/(cp_*cp_);         //(p-pp)*(p-pp)/(pp_*pp_) + (q-qp)*(q-qp)/(qp_*qp_);
//					}
//					sapOrderedList.insert(std::pair<double, std::pair<double, double> >(diff, std::pair<double, double>(itOuter->first, itInner->first)));
//				}
//			}
//			std::cout << "pan\ttilt\tdiff" << std::endl;
//			for (itOrderedList = sapOrderedList.begin(); itOrderedList != sapOrderedList.end(); itOrderedList++)
//				std::cout <<  itOrderedList->second.first << "\t" << itOrderedList->second.second << "\t" << itOrderedList->first << std::endl;
//			displayText << "p:" << sapOrderedList.begin()->second.first << "  t:" << sapOrderedList.begin()->second.second;
//			cvPutText(clusterImage, displayText.str().c_str(), cvPoint(umin, max(0,vmin-20)), &font, CV_RGB(0, 255, 0));
//			*/
//
//			// VFH
//			std::cout << "VFH response:" << std::endl;
//			pGlobalFeatureParams.useFeature["sap"] = false;
//			pGlobalFeatureParams.useFeature["vfh"] = true;
//			ExtractGlobalFeatures(&Blobs, featureVector, pClusterMode, pGlobalFeatureParams, INVALID, si.Coord(), mask, NULL, false, "common/files/timing.txt");
//			for (itOuter = mVfhData.begin(); itOuter != mVfhData.end(); itOuter++)
//			{
//				for (itInner = itOuter->second.begin(); itInner != itOuter->second.end(); itInner++)
//				{
//					double diff = 0.;
//					for (int i=0; i<(*featureVector)->cols; i++)
//					{
//						double val = cvGetReal1D(*featureVector, i) - itInner->second[0][i];
//						diff += val*val;
//					}
//					vfhOrderedList.insert(std::pair<double, std::pair<double, double> >(diff, std::pair<double, double>(itOuter->first, itInner->first)));
//				}
//			}
//			std::cout << "pan\ttilt\tdiff" << std::endl;
//			for (itOrderedList = vfhOrderedList.begin(); itOrderedList != vfhOrderedList.end(); itOrderedList++)
//				std::cout <<  itOrderedList->second.first << "\t" << itOrderedList->second.second << "\t" << itOrderedList->first << std::endl;
//			double pan = vfhOrderedList.begin()->second.first;
//			double tilt = vfhOrderedList.begin()->second.second;
//			cv::Mat histogram;
//			HermesComputeRollHistogram(cloud_cluster, avgPoint, histogram, true, false);
//			int roll = 0;
//			double matchScore = 0;
//			HermesMatchRollHistogram(mRollHistogram[pan][tilt][0], histogram, 10, roll, matchScore);
//
//			displayText.str("");
//			displayText.clear();
//			displayText << "p:" << pan << "  t:" << tilt << "  r:" << roll;
//			cvPutText(clusterImage, displayText.str().c_str(), cvPoint(umin, max(0,vmin-20)), &font, CV_RGB(0, 255, 0));
//
//			// match full point clouds (ICP)
//			HermesMatchPointClouds(cloud_cluster, avgPoint, pan, tilt, roll);
//
//			// free memory
//			cvReleaseImage(&mask);
//			cvReleaseMat(featureVector);
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			CvFont font;
			cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 1, 1);
			std::stringstream displayText;
			displayText << "p:" << pan << "  t:" << tilt << "  r:" << roll;
			cvPutText(clusterImage, displayText.str().c_str(), cvPoint(umin, max(0,vmin-20)), &font, CV_RGB(0, 255, 0));

			si.Release();
			clusterIndex++;
		}

		cloud_cluster->clear();
	}

	cvShowImage("cluster image", clusterImage);
	cvMoveWindow("cluster image", 1280, 0);
	cv::waitKey(1000);
	//cv::waitKey();
	cvReleaseImage(&clusterImage);
}


int ObjectClassifier::HermesCategorizeObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud, pcl::PointXYZ pAvgPoint, SharedImage* pSourceImage, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams, double& pan, double& tilt, double& roll, Eigen::Matrix4f& pFinalTransform)
{
	// create a pseudo blob
	BlobFeatureRiB Blob;
	BlobListRiB Blobs;
	ipa_utils::IntVector Keys;

	Blob.m_x = 0;
	Blob.m_y = 0;
	Blob.m_r = 2;
	Blob.m_Phi = 0;
	Blob.m_Res = 0;
	Blob.m_Id = 1;
	for (unsigned int j=0; j<64; j++) Blob.m_D.push_back(1);
	Blob.m_D.push_back(1);		// min/max curvature approximate
	Blob.m_D.push_back(1);
	for (int i=0; i<10; i++) Blobs.push_back(Blob);

	// Use Mask for global feature extraction
	CvMat** featureVector = new CvMat*;
	*featureVector = NULL;
	IplImage* mask = cvCreateImage(cvGetSize(pSourceImage->Shared()), pSourceImage->Shared()->depth, 1);
	cvCvtColor(pSourceImage->Shared(), mask, CV_RGB2GRAY);

	std::map<double, std::map<double, std::vector<std::vector<float> > > >::iterator itOuter;
	std::map<double, std::vector<std::vector<float> > >::iterator itInner;
	std::multimap<double, std::pair<double, double> > sapOrderedList;	// [difference score](pan, tilt)
	std::multimap<double, std::pair<double, double> > vfhOrderedList;
	std::multimap<double, std::pair<double, double> >::iterator itOrderedList;

	// SAP feature
/*			std::cout << "SAP response:" << std::endl;
	pGlobalFeatureParams.useFeature["sap"] = true;
	pGlobalFeatureParams.useFeature["vfh"] = false;
	ExtractGlobalFeatures(&Blobs, featureVector, pClusterMode, pGlobalFeatureParams, INVALID, pSourceImage.Coord(), mask, false, false, "common/files/timing.txt");
	for (itOuter = mSapData.begin(); itOuter != mSapData.end(); itOuter++)
	{
		for (itInner = itOuter->second.begin(); itInner != itOuter->second.end(); itInner++)
		{
			double diff = 0.;
			for (int i=0; i<3; i++)
			{
				double val = cvGetReal1D(*featureVector, i) - itInner->second[0][i];
				diff += val*val/itInner->second[0][i];
			}
			for (int i=3;i<(*featureVector)->cols;i+=3)
			{
				double a = cvGetReal1D(*featureVector, i);
				double a_ = a;
				if (a_==0.) a_ = 0.01;
				double b = cvGetReal1D(*featureVector, i+1);
				double c = cvGetReal1D(*featureVector, i+2);
				double p = b/(2*a_);
				double q = c - p*p*a_;
				double ap = itInner->second[0][i];
				double ap_ = ap;
				if (ap_==0.) { ap = a+0.1; ap_ = 1; }
				double bp = itInner->second[0][i+1];
				double bp_ = bp;
				if (bp_==0.) { bp = b+0.1; bp_ = 1; }
				double cp = itInner->second[0][i+2];
				double cp_ = cp;
				if (cp_==0.) { cp = c+0.1; cp_ = 1; }
				double pp = bp/(2*ap_);
				double pp_ = pp;
				if (pp_==0.) pp_ = 1;
				double qp = cp - pp*pp*ap_;
				double qp_ = qp;
				if (qp_==0.) qp_ = 1;
				diff += (a-ap)*(a-ap)/(ap_*ap_) + (b-bp)*(b-bp)/(bp_*bp_) + (c-cp)*(c-cp)/(cp_*cp_);         //(p-pp)*(p-pp)/(pp_*pp_) + (q-qp)*(q-qp)/(qp_*qp_);
			}
			sapOrderedList.insert(std::pair<double, std::pair<double, double> >(diff, std::pair<double, double>(itOuter->first, itInner->first)));
		}
	}
	std::cout << "pan\ttilt\tdiff" << std::endl;
	for (itOrderedList = sapOrderedList.begin(); itOrderedList != sapOrderedList.end(); itOrderedList++)
		std::cout <<  itOrderedList->second.first << "\t" << itOrderedList->second.second << "\t" << itOrderedList->first << std::endl;
	displayText << "p:" << sapOrderedList.begin()->second.first << "  t:" << sapOrderedList.begin()->second.second;
	cvPutText(clusterImage, displayText.str().c_str(), cvPoint(umin, max(0,vmin-20)), &font, CV_RGB(0, 255, 0));
	*/

	// VFH
	std::cout << "VFH response:" << std::endl;
	pGlobalFeatureParams.useFeature["sap"] = false;
	pGlobalFeatureParams.useFeature["vfh"] = true;
	ExtractGlobalFeatures(&Blobs, featureVector, pClusterMode, pGlobalFeatureParams, INVALID, pSourceImage->Coord(), mask, NULL, false, "");
	for (itOuter = mVfhData.begin(); itOuter != mVfhData.end(); itOuter++)
	{
		for (itInner = itOuter->second.begin(); itInner != itOuter->second.end(); itInner++)
		{
			double diff = 0.;
			for (int i=0; i<(*featureVector)->cols; i++)
			{
				double val = cvGetReal1D(*featureVector, i) - itInner->second[0][i];
				diff += val*val;
			}
			vfhOrderedList.insert(std::pair<double, std::pair<double, double> >(diff, std::pair<double, double>(itOuter->first, itInner->first)));
		}
	}
	std::cout << "pan\ttilt\tdiff" << std::endl;
	for (itOrderedList = vfhOrderedList.begin(); itOrderedList != vfhOrderedList.end(); itOrderedList++)
		std::cout <<  itOrderedList->second.first << "\t" << itOrderedList->second.second << "\t" << itOrderedList->first << std::endl;
	pan = vfhOrderedList.begin()->second.first;
	tilt = vfhOrderedList.begin()->second.second;
	cv::Mat histogram;
	HermesComputeRollHistogram(pPointCloud, pAvgPoint, histogram, true, false);
	int roll_i = 0;
	double matchScore = 0;
	HermesMatchRollHistogram(mRollHistogram[pan][tilt][0], histogram, 10, roll_i, matchScore);
	roll = roll_i;
	roll = 0; // hack
	std::cout << "best matching roll angle: " << roll << std::endl;

	// match full point clouds (ICP)
	HermesMatchPointClouds(pPointCloud, pAvgPoint, pan, tilt, roll, pFinalTransform);

	// free memory
	cvReleaseImage(&mask);
	cvReleaseMat(featureVector);

	return ipa_utils::RET_OK;
}


int ObjectClassifier::HermesComputeRollHistogram(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud, pcl::PointXYZ pAvgPoint, cv::Mat& pHistogram, bool pSmooth, bool pDisplay)
{
	// get roll orientation
	cv::Mat histogram = cv::Mat::zeros(1, 360, CV_32FC1);
	for (int p=0; p<(int)pPointCloud->size(); p++)
	{
		if ((*pPointCloud)[p].x==0 && (*pPointCloud)[p].y==0 && (*pPointCloud)[p].z==0)
			continue;
		double alpha = 180 + 180/M_PI*atan2(pPointCloud->points[p].y-pAvgPoint.y, pPointCloud->points[p].x-pAvgPoint.x);
		histogram.at<float>((int)alpha) += 1.;
	}

	if (pSmooth)
	{
		cv::Mat histogramSmoothed(1, histogram.cols, CV_32FC1);
		histogramSmoothed.at<float>(0) = 0.25*histogram.at<float>(histogram.cols-1) + 0.5*histogram.at<float>(0) + 0.25*histogram.at<float>(1);
		histogramSmoothed.at<float>(histogram.cols-1) = 0.25*histogram.at<float>(histogram.cols-2) + 0.5*histogram.at<float>(histogram.cols-1) + 0.25*histogram.at<float>(0);
		for (int i=1; i<histogramSmoothed.cols-1; i++)
			histogramSmoothed.at<float>(i) = 0.25*histogram.at<float>(i-1) + 0.5*histogram.at<float>(i) + 0.25*histogram.at<float>(i+1);
		pHistogram = histogramSmoothed;
	}
	else
		pHistogram = histogram;

	if (pDisplay)
	{
		cv::Mat dispHist(300, 360, CV_8UC1);
		dispHist.setTo(255);
		for (int i=0; i<pHistogram.cols; i++)
			cv::line(dispHist, cv::Point(i, 0), cv::Point(i, (int)pHistogram.at<float>(i)), CV_RGB(0,0,0));
		cv::imshow("histogram", dispHist);
		cv::waitKey();
	}

	return ipa_utils::RET_OK;
}


int ObjectClassifier::HermesMatchRollHistogram(std::vector<float>& pReferenceHistogram, cv::Mat& pMatchHistogram, int pCoarseStep, int& pOffset, double& pMatchScore)
{
	pMatchScore = 0;
	for (int offset=0; offset<pMatchHistogram.cols; offset+=pCoarseStep)
	{
		double score = HermesHistogramIntersectionKernel(pReferenceHistogram, pMatchHistogram, offset);
		if (score > pMatchScore)
		{
			pMatchScore = score;
			pOffset = offset;
		}
	}

	// refinement
	for (int offset=-pCoarseStep/2-1; offset<pCoarseStep/2+1; offset++)
	{
		double score = HermesHistogramIntersectionKernel(pReferenceHistogram, pMatchHistogram, pOffset+offset);
		if (score > pMatchScore)
		{
			pMatchScore = score;
			pOffset = pOffset+offset;
		}
	}

	return ipa_utils::RET_OK;
}


double ObjectClassifier::HermesHistogramIntersectionKernel(std::vector<float>& pReferenceHistogram, cv::Mat& pMatchHistogram, int pOffset)
{
	double score = 0;

	if ((int)pReferenceHistogram.size() != pMatchHistogram.cols)
		std::cout << "ObjectClassifier::HermesHistogramIntersectionKernel: Error: Array sizes do not match." << std::endl;

	int size = pMatchHistogram.cols;

	for (int i=0; i<size; i++)
		score += std::min<float>(pReferenceHistogram[i], pMatchHistogram.at<float>((i+pOffset)%size));
	return score;
}


int ObjectClassifier::HermesMatchPointClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCapturedCloud, pcl::PointXYZ pAvgPoint, double pan, double tilt, double roll, Eigen::Matrix4f& pFinalTransform)
{
	// look up suitable file for reference point cloud
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr referenceCloudOriginal(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr referenceCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedReferenceCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "ObjectClassifier::HermesMatchPointClouds: Loading pcd file... " << std::flush;
	pcl::io::loadPCDFile(mReferenceFilenames[pan][tilt], *referenceCloudOriginal);
	std::cout << "finished." << std::endl;
	pcl::PointXYZ avgRefrencePoint(0,0,0);
	for (int p=0; p<(int)referenceCloudOriginal->points.size(); p++)
	{
		pcl::PointXYZRGB& point = referenceCloudOriginal->points[p];
		if (point.x == 0 && point.y == 0 && point.z == 0)
			continue;
		avgRefrencePoint.x += point.x;
		avgRefrencePoint.y += point.y;
		avgRefrencePoint.z += point.z;
//		point.r = 255;
//		point.g = 0;
//		point.b = 0;
		referenceCloud->push_back(point);
	}
	avgRefrencePoint.x /= referenceCloud->points.size();
	avgRefrencePoint.y /= referenceCloud->points.size();
	avgRefrencePoint.z /= referenceCloud->points.size();
	referenceCloud->header = referenceCloudOriginal->header;
	referenceCloud->height = referenceCloudOriginal->height;
	referenceCloud->width = referenceCloudOriginal->width;
	referenceCloud->is_dense = referenceCloudOriginal->is_dense;
	referenceCloud->sensor_orientation_ = referenceCloudOriginal->sensor_orientation_;
	referenceCloud->sensor_origin_ = referenceCloudOriginal->sensor_origin_;

	Eigen::Matrix4f transform;
	transform.setIdentity();
	transform(0,3) = -avgRefrencePoint.x;
	transform(1,3) = -avgRefrencePoint.y;
	transform(2,3) = -avgRefrencePoint.z;
	pcl::transformPointCloud(*referenceCloud, *alignedReferenceCloud, transform);

	// align roll of captured point cloud (reference roll is 0)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedCapturedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	Eigen::Matrix4f shift;
	Eigen::Matrix4f rotation;
	shift.setIdentity();
	shift(0, 3) = -pAvgPoint.x;
	shift(1, 3) = -pAvgPoint.y;
	shift(2, 3) = -pAvgPoint.z;
	rotation.setIdentity();
	double co = cos(-roll/180*M_PI) , si = sin(-roll/180*M_PI);
	rotation(0,0) = co;
	rotation(0,1) = -si;
	rotation(1,0) = si;
	rotation(1,1) = co;
	transform = rotation * shift;
	pcl::transformPointCloud(*pCapturedCloud, *alignedCapturedCloud, transform);

	for (int p=0; p<(int)alignedCapturedCloud->points.size(); p++)
	{
		alignedCapturedCloud->points[p].r = 0;
		alignedCapturedCloud->points[p].g = 255;
		alignedCapturedCloud->points[p].b = 0;
	}

	// icp
	std::cout << "Starting ICP" << std::endl;
	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputCloud(alignedCapturedCloud);
	icp.setInputTarget(alignedReferenceCloud);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr fusedCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	icp.align(*fusedCloud);
	std::cout << "ICP has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	pFinalTransform = icp.getFinalTransformation();

	// display
	// C:\Users\rmb\Documents\Studienarbeit\Software\object_categorization\common\files\hermes>"C:\Program Files\PCL 1.4.0\bin\pcd_viewer.exe" -bc 255,255,255 -ps 3 -ax 0.01 output.pcd
	// "C:\Program Files\PCL 1.4.0\bin\pcd_viewer.exe" -bc 255,255,255 -ps 3 -ax 0.01 C:\Users\rmb\Documents\Studienarbeit\Software\object_categorization\common\files\hermes\output.pcd

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (255, 255, 255);
	*fusedCloud += *alignedReferenceCloud;
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(fusedCloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(fusedCloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce(100);
	}

	// output to file to check result
//	*fusedCloud += *alignedReferenceCloud;
//	std::stringstream ss;
//	ss << "common/files/hermes/output.pcd";
//	pcl::io::savePCDFileASCII(ss.str().c_str(), *fusedCloud);

	return ipa_utils::RET_OK;
}


// Hermes code end



ClassificationData::ClassificationData()
{
	mSqrtInverseCovarianceMatrix = NULL;
#if (CV_MAJOR_VERSION<=2 && CV_MINOR_VERSION<=3)
	mLocalFeatureClusterer = new CvEM();
#else
	mLocalFeatureClusterer = new cv::EM;
#endif
}

ClassificationData::~ClassificationData()
{
	cvReleaseMat(&mSqrtInverseCovarianceMatrix);
}


int ClassificationData::GetLocalFeatureMatrix(std::string pClass, CvMat** pFeatureMatrix, float pFactorCorrect, float pFactorIncorrect, int& pNumberCorrectSamples)
{
	if ((mItLocalFeaturesMap=mLocalFeaturesMap.find(pClass)) == mLocalFeaturesMap.end())
	{
		*pFeatureMatrix = NULL;
		std::cout << "ClassificaionData::GetLocalFeatureMatrix: No class found with name" << pClass;
		return ipa_utils::RET_FAILED;
	}

	if ((pFactorCorrect<=0.0) || (pFactorCorrect > 1.0))
	{
		std::cout << "ClassificaionData::GetLocalFeatureMatrix: pFactorCorrect not in (0..1]";
		return ipa_utils::RET_FAILED;
	}

	if (pFactorIncorrect<0.0)
	{
		std::cout << "ClassificaionData::GetLocalFeatureMatrix: pFactorIncorrect not in [0..inf]";
		return ipa_utils::RET_FAILED;
	}
	LocalFeaturesMap::value_type::second_type::value_type::second_type::iterator ItBlobListStructs;
	int NumberSamples = GetNumberLocalFeaturePoints(pClass);
	int NumberFeatures = GetNumberLocalFeatures();

	// create matrix
	int NumberCorrectSamples = cvRound((double)NumberSamples * pFactorCorrect);
	pNumberCorrectSamples = NumberCorrectSamples;
	int NumberIncorrectSamples = cvRound((double)NumberCorrectSamples * pFactorIncorrect);

	*pFeatureMatrix = cvCreateMat(NumberCorrectSamples+NumberIncorrectSamples, NumberFeatures, CV_32FC1);
	CvMat* TempFeatureMatrix = cvCreateMat(NumberSamples, NumberFeatures, CV_32FC1);

	// fill temp matrix with all existing correct samples
	int SampleCounter = 0;
	mItLocalFeaturesMap=mLocalFeaturesMap.find(pClass);
	for (mItObjectMap = mItLocalFeaturesMap->second.begin(); mItObjectMap != mItLocalFeaturesMap->second.end(); mItObjectMap++)
	{
		for (ItBlobListStructs = mItObjectMap->second.begin(); ItBlobListStructs != mItObjectMap->second.end(); ItBlobListStructs++)
		{
			BlobListRiB::iterator ItBlobList;
			for (ItBlobList = ItBlobListStructs->BlobFPs.begin(); ItBlobList != ItBlobListStructs->BlobFPs.end(); ItBlobList++, SampleCounter++)
			{
				for (int i=0; i<NumberFeatures; i++)
				{
					cvmSet(TempFeatureMatrix, SampleCounter, i, ItBlobList->m_D[i]);
				}
			}
		}
	}

	// fill first part of *pFeatureMatrix with chosen correct samples
	std::list<int> Indices;
	std::list<int>::iterator ItIndices;
	for (int i=0; i<NumberSamples; i++) Indices.push_back(i);
	for (int i=0; i<NumberCorrectSamples; i++)
	{
		int Index = int(Indices.size()*((double)rand()/((double)RAND_MAX+1.0)));
		ItIndices = Indices.begin();
		for (int k=0; k<Index; k++, ItIndices++);
		for (int j=0; j<NumberFeatures; j++)
		{
			cvmSet(*pFeatureMatrix, i, j, cvmGet(TempFeatureMatrix, *ItIndices, j));
		}
		Indices.remove(*ItIndices);
	}
	Indices.clear();
	cvReleaseMat(&TempFeatureMatrix);

	// fill rest of *pFeatureMatrix with incorrect samples
	GetNegativeSamplesMatrixLocal(pClass, *pFeatureMatrix, NumberCorrectSamples, (NumberCorrectSamples+NumberIncorrectSamples));
	
/*	// Output
	for (int i=0; i<(*pFeatureMatrix)->height; i++)
	{
		for (int j=0; j<(*pFeatureMatrix)->width; j++)
			std::cout << cvGetReal2D(*pFeatureMatrix, i, j) << "\t";
		std::cout << "\n";
	}
*/
	return ipa_utils::RET_OK;
}


int ClassificationData::GetNegativeSamplesMatrixLocal(std::string pClass, CvMat* pNegativeSamplesMatrix, int pLowerBound, int pUpperBound)
{
	ObjectMap::value_type::second_type::iterator ItBlobListStructs;
	int NumberFeatures = GetNumberLocalFeatures();

	for (int i=pLowerBound; i<pUpperBound; i++)
	{
		// find class randomly
		while(1)
		{
			int ClassIndex = int(mLocalFeaturesMap.size()*((double)rand()/((double)RAND_MAX+1.0)));
			mItLocalFeaturesMap = mLocalFeaturesMap.begin();
			for (int k=0; k<ClassIndex; k++, mItLocalFeaturesMap++);
			if (mItLocalFeaturesMap->first != pClass) break;
		}

		// find object randomly
		int ObjectIndex = int(mItLocalFeaturesMap->second.size()*((double)rand()/((double)RAND_MAX+1.0)));
		mItObjectMap = mItLocalFeaturesMap->second.begin();
		for (int k=0; k<ObjectIndex; k++, mItObjectMap++);

		while(1)
		{
			// find view randomly (which has feature points)
			int ViewIndex = int(mItObjectMap->second.size()*((double)rand()/((double)RAND_MAX+1.0)));
			ItBlobListStructs = mItObjectMap->second.begin();
			for (int k=0; k<ViewIndex; k++, ItBlobListStructs++);
			if (ItBlobListStructs->BlobFPs.size() > 0) break;
		}
		// find sample randomly
		int SampleIndex = int(ItBlobListStructs->BlobFPs.size()*((double)rand()/((double)RAND_MAX+1.0)));
		BlobListRiB::iterator ItBlobFPs = ItBlobListStructs->BlobFPs.begin();
		for (int k=0; k<SampleIndex; k++, ItBlobFPs++);
		
		// write sample into matrix of negative samples
		for (int j=0; j<NumberFeatures; j++)
		{
			cvmSet(pNegativeSamplesMatrix, i, j, ItBlobFPs->m_D[j]);
		}
	}

	return ipa_utils::RET_OK;
}

CvMat* ClassificationData::CreateAllLocalFeatureMatrix(int pNumberSamples, int pNumberFeatures)
{
	if (pNumberSamples == -1) pNumberSamples=GetNumberLocalFeaturePoints();
	if (pNumberFeatures == -1) pNumberFeatures=GetNumberLocalFeatures();

	std::vector<BlobListStruct>::iterator ItBlobListStructs;
	BlobListRiB::iterator ItBlobList;

	std::cout << "Create Matrix of all local features (" << pNumberSamples << " Samples, " << pNumberFeatures << " Features).\n";
	/// Create matrix of all local features
	CvMat* AllLocalFeatures = cvCreateMat(pNumberSamples, pNumberFeatures, CV_32FC1);
	int SampleCounter = 0;
	for (mItLocalFeaturesMap = mLocalFeaturesMap.begin(); mItLocalFeaturesMap != mLocalFeaturesMap.end(); mItLocalFeaturesMap++)
	{
		for (mItObjectMap = mItLocalFeaturesMap->second.begin(); mItObjectMap != mItLocalFeaturesMap->second.end(); mItObjectMap++)
		{
			for (ItBlobListStructs = mItObjectMap->second.begin(); ItBlobListStructs != mItObjectMap->second.end(); ItBlobListStructs++)
			{
				for (ItBlobList = ItBlobListStructs->BlobFPs.begin(); ItBlobList != ItBlobListStructs->BlobFPs.end(); SampleCounter++)
				{
					if (SampleCounter >= pNumberSamples) break;
					for (int j=0; j<pNumberFeatures; j++)
					{
						cvmSet(AllLocalFeatures, SampleCounter, j, ItBlobList->m_D[j]);
					}
					ItBlobList++;
				}
				if (SampleCounter >= pNumberSamples) break;
			}
			if (SampleCounter >= pNumberSamples) break;
		}
		if (SampleCounter >= pNumberSamples) break;
	}

	return AllLocalFeatures;
}


int ClassificationData::GetGlobalFeatureMatrix(std::string pClass, CvMat** pFeatureMatrix, float pFactorCorrect, float pFactorIncorrect, int& pNumberCorrectSamples)
{
	if ((mItGlobalFeaturesMap=mGlobalFeaturesMap.find(pClass)) == mGlobalFeaturesMap.end())
	{
		*pFeatureMatrix = NULL;
		std::cout << "ClassificaionData::GetGlobalFeatureMatrix: No class found with name" << pClass;
		return ipa_utils::RET_FAILED;
	}

	if ((pFactorCorrect<=0.0) || (pFactorCorrect > 1.0))
	{
		std::cout << "ClassificaionData::GetGlobalFeatureMatrix: pFactorCorrect not in (0..1]";
		return ipa_utils::RET_FAILED;
	}

	if (pFactorIncorrect<0.0)
	{
		std::cout << "ClassificaionData::GetGlobalFeatureMatrix: pFactorIncorrect not in [0..inf]";
		return ipa_utils::RET_FAILED;
	}

	int NumberSamples=0;
	int NumberFeatures=0;
	GlobalFeaturesMap::value_type::second_type::iterator ItObjectMap;
	for (ItObjectMap = mItGlobalFeaturesMap->second.begin(); ItObjectMap != mItGlobalFeaturesMap->second.end(); ItObjectMap++)
	{
		NumberSamples += ItObjectMap->second->rows; 
		NumberFeatures = ItObjectMap->second->width;
	}
	

	// create matrix
	int NumberCorrectSamples = cvRound((double)NumberSamples * pFactorCorrect);
	pNumberCorrectSamples = NumberCorrectSamples;
	int NumberIncorrectSamples = cvRound((double)NumberCorrectSamples * pFactorIncorrect);

	*pFeatureMatrix = cvCreateMat(NumberCorrectSamples+NumberIncorrectSamples, NumberFeatures, CV_32FC1);
	CvMat* TempFeatureMatrix = cvCreateMat(NumberSamples, NumberFeatures, CV_32FC1);

	// fill temp matrix with all existing correct samples
	int RowNumber=0;
	for (ItObjectMap = mItGlobalFeaturesMap->second.begin(); ItObjectMap != mItGlobalFeaturesMap->second.end(); ItObjectMap++)
	{
		for (int i=0; i<ItObjectMap->second->rows; i++, RowNumber++)
			for (int j=0; j<ItObjectMap->second->cols; j++)
				cvmSet(TempFeatureMatrix, RowNumber, j, cvmGet(ItObjectMap->second, i, j));
	}

	// fill first part of *pFeatureMatrix with chosen correct samples
	std::list<int> Indices;
	std::list<int>::iterator ItIndices;
	for (int i=0; i<NumberSamples; i++) Indices.push_back(i);
	for (int i=0; i<NumberCorrectSamples; i++)
	{
		int Index = int(Indices.size()*((double)rand()/((double)RAND_MAX+1.0)));
		ItIndices = Indices.begin();
		for (int k=0; k<Index; k++, ItIndices++);
		for (int j=0; j<NumberFeatures; j++)
		{
			cvmSet(*pFeatureMatrix, i, j, cvmGet(TempFeatureMatrix, *ItIndices, j));
		}
		Indices.remove(*ItIndices);
	}

	Indices.clear();
	cvReleaseMat(&TempFeatureMatrix);

	// fill rest of *pFeatureMatrix with incorrect samples
	std::vector<std::string> negativeSamplesLabels;
	GetNegativeSamplesMatrixGlobal(pClass, *pFeatureMatrix, negativeSamplesLabels, NumberCorrectSamples, (NumberCorrectSamples+NumberIncorrectSamples));

/*	// Output
	for (int i=0; i<(*pFeatureMatrix)->height; i++)
	{
		for (int j=0; j<(*pFeatureMatrix)->width; j++)
			std::cout << cvGetReal2D(*pFeatureMatrix, i, j) << "\t";
		std::cout << "\n";
	}
*/
	return ipa_utils::RET_OK;
}


int ClassificationData::GetNegativeSamplesMatrixGlobal(std::string pClass, CvMat* pNegativeSamplesMatrix, std::vector<std::string>& pLabels, int pLowerBound, int pUpperBound, int pViewsPerObject)
{
	GlobalFeaturesMap::value_type::second_type::iterator ItObjectMap;

	for (int i=pLowerBound; i<pUpperBound; i++)
	{
		// find class randomly
		while(1)
		{
			int ClassIndex = int(mGlobalFeaturesMap.size()*((double)rand()/((double)RAND_MAX+1.0)));//(double)rand()/(double)(RAND_MAX+1)));
			mItGlobalFeaturesMap = mGlobalFeaturesMap.begin();
			for (int k=0; k<ClassIndex; k++, mItGlobalFeaturesMap++);
			if (mItGlobalFeaturesMap->first != pClass) break;
		}

		// find object randomly
		int ObjectIndex = int(mItGlobalFeaturesMap->second.size()*((double)rand()/((double)RAND_MAX+1.0)));
		ItObjectMap = mItGlobalFeaturesMap->second.begin();
		for (int k=0; k<ObjectIndex; k++, ItObjectMap++);

		// find sample randomly
		int sampleIndex = 0;
		if (pViewsPerObject == -1)
			sampleIndex = int(ItObjectMap->second->rows*((double)rand()/((double)RAND_MAX+1.0)));
		else
		{
			double step=(double)ItObjectMap->second->rows/(double)pViewsPerObject;
			do
			{
				sampleIndex = int(ItObjectMap->second->rows*((double)rand()/((double)RAND_MAX+1.0)));
			} while (sampleIndex != int(step * int((double)sampleIndex/step)));
		}
		for (int j=0; j<ItObjectMap->second->width; j++)
			cvmSet(pNegativeSamplesMatrix, i, j, cvmGet(ItObjectMap->second, sampleIndex, j));
		pLabels[i] = mItGlobalFeaturesMap->first;
	}

	return ipa_utils::RET_OK;
}


int ClassificationData::GetNegativeSamplesMatrixGlobalSampleRange(std::string pClass, CvMat* pNegativeSamplesMatrix, std::vector<std::string>& pLabels, int pLowerBound, int pUpperBound, double rangeStartFactor, double rangeEndFactor)
{
	GlobalFeaturesMap::value_type::second_type::iterator ItObjectMap;

	for (int i=pLowerBound; i<pUpperBound; i++)
	{
		// find class randomly
		while(1)
		{
			int ClassIndex = int(mGlobalFeaturesMap.size()*((double)rand()/((double)RAND_MAX+1.0)));//(double)rand()/(double)(RAND_MAX+1)));
			mItGlobalFeaturesMap = mGlobalFeaturesMap.begin();
			for (int k=0; k<ClassIndex; k++, mItGlobalFeaturesMap++);
			if (mItGlobalFeaturesMap->first != pClass) break;
		}

		// find object randomly
		int ObjectIndex = int(mItGlobalFeaturesMap->second.size()*((double)rand()/((double)RAND_MAX+1.0)));
		ItObjectMap = mItGlobalFeaturesMap->second.begin();
		for (int k=0; k<ObjectIndex; k++, ItObjectMap++);

		// find sample randomly
		int sampleIndex = int((ItObjectMap->second->rows*rangeStartFactor) + (ItObjectMap->second->rows*(rangeEndFactor-rangeStartFactor))*((double)rand()/((double)RAND_MAX+1.0)));
		for (int j=0; j<ItObjectMap->second->width; j++)
			cvmSet(pNegativeSamplesMatrix, i, j, cvmGet(ItObjectMap->second, sampleIndex, j));
		pLabels[i] = mItGlobalFeaturesMap->first;
	}

	return ipa_utils::RET_OK;
}


int ClassificationData::GetNumberLocalFeatures()
{
	std::vector<BlobListStruct>::iterator ItBlobListStructs;
	int NumberLocalFeatures = 0;

	for (mItLocalFeaturesMap = mLocalFeaturesMap.begin(); mItLocalFeaturesMap != mLocalFeaturesMap.end(); mItLocalFeaturesMap++)
	{
		for (mItObjectMap = mItLocalFeaturesMap->second.begin(); mItObjectMap != mItLocalFeaturesMap->second.end(); mItObjectMap++)
		{
			for (ItBlobListStructs = mItObjectMap->second.begin(); ItBlobListStructs != mItObjectMap->second.end(); ItBlobListStructs++)
			{
				if (ItBlobListStructs->BlobFPs.size()!=0)
				{
					NumberLocalFeatures = ItBlobListStructs->BlobFPs.begin()->m_D.size();
					break;
				}
			}
			if (NumberLocalFeatures>0) break;
		}
		if (NumberLocalFeatures>0) break;
	}

	return NumberLocalFeatures;
}


int ClassificationData::GetNumberLocalFeaturePoints()
{
	std::vector<BlobListStruct>::iterator ItBlobListStructs;
	int NumberSamples = 0;

	for (mItLocalFeaturesMap = mLocalFeaturesMap.begin(); mItLocalFeaturesMap != mLocalFeaturesMap.end(); mItLocalFeaturesMap++)
	{
		for (mItObjectMap = mItLocalFeaturesMap->second.begin(); mItObjectMap != mItLocalFeaturesMap->second.end(); mItObjectMap++)
		{
			for (ItBlobListStructs = mItObjectMap->second.begin(); ItBlobListStructs != mItObjectMap->second.end(); ItBlobListStructs++)
			{
				NumberSamples += ItBlobListStructs->BlobFPs.size();
			}
		}
	}

	return NumberSamples;
}


int ClassificationData::GetNumberLocalFeaturePoints(std::string pClass)
{
	std::vector<BlobListStruct>::iterator ItBlobListStructs;
	int NumberSamples = 0;

	if ((mItLocalFeaturesMap = mLocalFeaturesMap.find(pClass)) != mLocalFeaturesMap.end())
	{
		for (mItObjectMap = mItLocalFeaturesMap->second.begin(); mItObjectMap != mItLocalFeaturesMap->second.end(); mItObjectMap++)
		{
			for (ItBlobListStructs = mItObjectMap->second.begin(); ItBlobListStructs != mItObjectMap->second.end(); ItBlobListStructs++)
			{
				NumberSamples += ItBlobListStructs->BlobFPs.size();
			}
		}
	}

	return NumberSamples;
}


int ClassificationData::SaveLocalFeatures(std::string pFileName)
{
	std::ofstream f(pFileName.c_str(), std::fstream::out);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::SaveLocalFeatures: Could not open '" << pFileName << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	int LocalFeaturesMapSize = mLocalFeaturesMap.size();
	if(LocalFeaturesMapSize==0)
	{
		std::cout << "ClassificationData::SaveLocalFeatures: No classes to be saved for '" << pFileName << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}

	f << LocalFeaturesMapSize << "\n";			/// Number of Categories
	for (mItLocalFeaturesMap=mLocalFeaturesMap.begin(); mItLocalFeaturesMap!=mLocalFeaturesMap.end(); mItLocalFeaturesMap++)
	{
		int ClassSize = mItLocalFeaturesMap->second.size();
		f << mItLocalFeaturesMap->first << "\n";		/// Class name
		f << ClassSize << "\n";					/// Class size (number of objects in this category)

		for (mItObjectMap = mItLocalFeaturesMap->second.begin(); mItObjectMap != mItLocalFeaturesMap->second.end(); mItObjectMap++)
		{
			std::vector<BlobListStruct>::iterator ItBlobListStructs;
			int ObjectSize = mItObjectMap->second.size();
			int BlobListStructCounter = 0;
			
			f << ObjectSize << "\n";					/// Object size (number of pictures for this object = number of BlobListStructs)

			for (ItBlobListStructs = mItObjectMap->second.begin(); ItBlobListStructs != mItObjectMap->second.end(); ItBlobListStructs++, BlobListStructCounter++)
			{
				f << ItBlobListStructs->FileName << "\n";		/// BlobList origin file name

				/// Save every BlobList in separate file
				//std::string FileNameWithoutExtension = pFileName.substr(0, pFileName.length()-4);
				//std::stringstream BlobListFileName;
				//BlobListFileName << FileNameWithoutExtension << "_" << mItLocalFeaturesMap->first << BlobListStructCounter << ".txt";
				ItBlobListStructs->BlobFPs.Save(&f);

				/*if(ItBlobListStructs->BlobFPs.size()==0)
				{
					f << ItBlobListStructs->BlobFPs.size() << " " << 0 << " " << 0 << std::endl;
					std::cout << "ClassificationData::SaveLocalFeatures: No features to be saved for class " << mItLocalFeaturesMap->first << " object no. " << mItObjectMap->first << "." << std::endl;
				}
				else
				{
					int dim = (int)(ItBlobListStructs->BlobFPs.begin()->m_D.size());
					int frameDim = (ItBlobListStructs->BlobFPs.begin())->m_Frame.size();
					f << ItBlobListStructs->BlobFPs.size() << " " << dim << " " << frameDim << std::endl;

					BlobList::iterator it;
					for(it=ItBlobListStructs->BlobFPs.begin(); it!=ItBlobListStructs->BlobFPs.end(); it++)
					{	
						//f << It->m_y << " " << It->m_x << " " << It->m_r << " " << It->m_Phi << " " << It->m_Label << " " << It->m_Id << "\n";
						f << it->m_y << " " << it->m_x << " " << it->m_r << " " << it->m_Phi << " " << it->m_Id << "\n";
						for(int j=0; j<dim; j++)
							f << " " << it->m_D[j];
						f << "\n";
						for(int j=0; j<frameDim; j++) f << " " << it->m_Frame[j];
						f << "\n";
					}
				}*/
			}
		}
	}

	f.close();

	std::cout << "FP data saved.\n";

	return ipa_utils::RET_OK;
}


int ClassificationData::LoadLocalFeatures(std::string pFileName)
{
	mLocalFeaturesMap.clear();

	std::ifstream f(pFileName.c_str(), std::fstream::in);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::LoadLocalFeatures: Could not load '" << pFileName << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	int LocalFeaturesMapSize=0;

	f >> LocalFeaturesMapSize;			/// Number of Categories
	for (int i=0; i<LocalFeaturesMapSize; i++)
	{
		std::string ClassName = "";
		int ClassSize = 0;
		
		f >> ClassName;		/// Class name
		f >> ClassSize;		/// Class size (number of objects in this category)

		for (int ObjectCounter = 0; ObjectCounter < ClassSize; ObjectCounter++)
		{
			int ObjectSize = 0;
			f >> ObjectSize;	/// Object size (number of pictures for this object = number of BlobListStructs)

			for (int BlobListStructCounter = 0; BlobListStructCounter < ObjectSize; BlobListStructCounter++)
			{
				BlobListStruct TempBlobListStruct;
				f >> TempBlobListStruct.FileName;		/// BlobList origin file name

				/// Load BlobList from separate file
				//std::string FileNameWithoutExtension = pFileName.substr(0, pFileName.length()-4);
				//std::stringstream BlobListFileName;
				//BlobListFileName << FileNameWithoutExtension << "_" << ClassName << BlobListStructCounter << ".txt";
				TempBlobListStruct.BlobFPs.Load(&f, false);

				(mLocalFeaturesMap[ClassName])[ObjectCounter].push_back(TempBlobListStruct);
			}
		}
	}

	f.close();

	std::cout << "FP data loaded.\n";

	return ipa_utils::RET_OK;
}


int ClassificationData::SaveGlobalFeatures(std::string pFileName)
{
	std::ofstream f(pFileName.c_str(), std::fstream::out);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::SaveGlobalFeatures: Could not open '" << pFileName << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	int GlobalFeaturesMapSize = mGlobalFeaturesMap.size();
	if(GlobalFeaturesMapSize==0)
	{
		std::cout << "ClassificationData::SaveGlobalFeatures: No classes to be saved for '" << pFileName << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	GlobalFeaturesMap::value_type::second_type::iterator ItObjectMap;

	f << GlobalFeaturesMapSize << "\n";			/// Number of Categories
	for (mItGlobalFeaturesMap=mGlobalFeaturesMap.begin(); mItGlobalFeaturesMap!=mGlobalFeaturesMap.end(); mItGlobalFeaturesMap++)
	{
		f << mItGlobalFeaturesMap->first << "\n";				/// Class name
		f << mItGlobalFeaturesMap->second.size() << "\n";		/// Number of objects in each class
		for (ItObjectMap = mItGlobalFeaturesMap->second.begin(); ItObjectMap != mItGlobalFeaturesMap->second.end(); ItObjectMap++)
		{
			int NumberSamples = ItObjectMap->second->height;
			int NumberFeatures = ItObjectMap->second->width;
			
			f << NumberSamples << " " << NumberFeatures << "\n";		/// Class size

			for (int i=0; i<NumberSamples; i++)
			{
				for (int j=0; j<NumberFeatures; j++)
				{
					f << cvmGet(ItObjectMap->second, i, j) << "\t";		/// Global feature matrix
				}
				f << "\n";
			}
		}
	}

	f.close();

	std::cout << "Global features data saved.\n";

	return ipa_utils::RET_OK;
}


int ClassificationData::LoadGlobalFeatures(std::string pFileName)
{
	GlobalFeaturesMap::value_type::second_type::iterator ItObjectMap;

	/// Clear from old data
	for (mItGlobalFeaturesMap = mGlobalFeaturesMap.begin(); mItGlobalFeaturesMap != mGlobalFeaturesMap.end(); mItGlobalFeaturesMap++)
	{
		for (ItObjectMap = mItGlobalFeaturesMap->second.begin(); ItObjectMap != mItGlobalFeaturesMap->second.end(); ItObjectMap++)
		{ cvReleaseMat(&(ItObjectMap->second)); }
	}
	mGlobalFeaturesMap.clear();

	std::ifstream f(pFileName.c_str(), std::fstream::in);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::LoadGlobalFeatures: Could not load '" << pFileName << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	int GlobalFeaturesMapSize=0;

	f >> GlobalFeaturesMapSize;			/// Number of Categories
	for (int ClassNumber=0; ClassNumber<GlobalFeaturesMapSize; ClassNumber++)
	{
		std::string ClassName = "";
		int NumberObjects = 0;
		f >> ClassName;				/// Class name
		f >> NumberObjects;			/// Number of objects in each class

		for (int ObjectNumber=0; ObjectNumber<NumberObjects; ObjectNumber++)
		{
			int NumberSamples = 0;
			int NumberFeatures = 0;
			double Number = 0.0;
			f >> NumberSamples >> NumberFeatures;		/// Class size

			(mGlobalFeaturesMap[ClassName])[ObjectNumber] = cvCreateMat(NumberSamples, NumberFeatures, CV_32FC1);
			
//			double sum = 0;
			for (int i=0; i<NumberSamples; i++)
			{
				for (int j=0; j<NumberFeatures; j++)
				{
					f >> Number;
//					sum+=Number;
					cvmSet((mGlobalFeaturesMap[ClassName])[ObjectNumber], i, j, Number);		/// Global feature matrix
				}
			}

			//for (int i=0; i<NumberSamples; i++)
			//	for (int j=0; j<NumberFeatures; j++)
			//		cvSetReal2D((mGlobalFeaturesMap[ClassName])[ObjectNumber], i, j, cvGetReal2D(mGlobalFeaturesMap[ClassName][ObjectNumber], i, j)/sum);		/// Global feature matrix

			/*	/// Output
			for (mItGlobalFeaturesMap = mGlobalFeaturesMap.begin(); mItGlobalFeaturesMap != mGlobalFeaturesMap.end(); mItGlobalFeaturesMap++)
			{
				std::cout << mItGlobalFeaturesMap->first << "\n";
				for (int i=0; i<(mGlobalFeaturesMap[mItGlobalFeaturesMap->first])[ObjectNumber]->height; i++)
				{
					for (int j=0; j<(mGlobalFeaturesMap[mItGlobalFeaturesMap->first])[ObjectNumber]->width; j++) std::cout << cvGetReal2D((mGlobalFeaturesMap[mItGlobalFeaturesMap->first])[ObjectNumber], i, j) << "\t";
					std::cout << "\n";
				}
			}
			*/
		}
	}

	f.close();

	std::cout << "Global features data loaded.\n";

	return ipa_utils::RET_OK;
}


int ClassificationData::SaveGlobalClassifiers(std::string pPath, ClassifierType pClassifierType)
{
	std::stringstream FileName;
	FileName << pPath << "ClassifierInfo_" << ClassifierLabel(pClassifierType) << "_glob.txt";
	std::ofstream f((FileName.str()).c_str(), std::fstream::out);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::SaveGlobalClassifiers: Could not open '" << FileName.str() << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	int GlobalClassifierMapSize = mGlobalClassifierMap.size();
	if(GlobalClassifierMapSize==0)
	{
		std::cout << "ClassificationData::SaveGlobalClassifiers: No classes to be saved for '" << FileName.str() << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	f << GlobalClassifierMapSize << "\n";	// save number of classes
	
	GlobalClassifierMap::iterator ItGlobalClassifierMap, ItGlobalClassifierMap2;
	for (ItGlobalClassifierMap = mGlobalClassifierMap.begin(); ItGlobalClassifierMap != mGlobalClassifierMap.end(); ItGlobalClassifierMap++)
	{
		f << ItGlobalClassifierMap->first << "\n";		// class name

		// best classifier threshold
		ClassifierThresholdMap::iterator ItGlobalClassifierThresholdMap;
		if ((ItGlobalClassifierThresholdMap = mGlobalClassifierThresholdMap.find(ItGlobalClassifierMap->first)) == mGlobalClassifierThresholdMap.end())
		{
			mGlobalClassifierThresholdMap[ItGlobalClassifierMap->first] = 0.5;
			std::cout << "WARNING: ClassificationData::SaveGlobalClassifiers: No threshold for class " << ItGlobalClassifierMap->first << " available. Threshold set to standard (0.5).\n";
		}
		f << mGlobalClassifierThresholdMap[ItGlobalClassifierMap->first] << "\n";	// best classifier threshold

		// classifier accuracies
		for (ItGlobalClassifierMap2 = mGlobalClassifierMap.begin(); ItGlobalClassifierMap2 != mGlobalClassifierMap.end(); ItGlobalClassifierMap2++)
			f << ItGlobalClassifierMap2->first << "\t" << mGlobalClassifierAccuracy[ItGlobalClassifierMap->first][ItGlobalClassifierMap2->first] << "\t";
		f << "\n";
		
		// save classifier
		switch(pClassifierType)
		{
		case CLASSIFIER_RTC:
			{
				CvRTrees* RTC = dynamic_cast<CvRTrees*>(mGlobalClassifierMap[ItGlobalClassifierMap->first]);
				std::stringstream ss;
				ss << pPath << ItGlobalClassifierMap->first << "_" << "RTC" << "_glob.txt";
				RTC->save((ss.str()).c_str());
				std::cout << ss.str() << " saved.\n";
				break;
			}
		case CLASSIFIER_SVM:
			{
				CvSVM* SVM = dynamic_cast<CvSVM*>(mGlobalClassifierMap[ItGlobalClassifierMap->first]);
				std::stringstream ss;
				ss << pPath << ItGlobalClassifierMap->first << "_" << "SVM" << "_glob.txt";
				SVM->save((ss.str()).c_str());
				std::cout << ss.str() << " saved.\n";
				break;
			}
		case CLASSIFIER_BOOST:
			{
				CvBoost* Boost = dynamic_cast<CvBoost*>(mGlobalClassifierMap[ItGlobalClassifierMap->first]);
				std::stringstream ss;
				ss << pPath << ItGlobalClassifierMap->first << "_" << "Boost" << "_glob.txt";
				Boost->save((ss.str()).c_str());
				std::cout << ss.str() << " saved.\n";
				break;
			}
		case CLASSIFIER_KNN:
			{
				CvKNearest* KNN = dynamic_cast<CvKNearest*>(mGlobalClassifierMap[ItGlobalClassifierMap->first]);
				std::stringstream ss;
				ss << pPath << ItGlobalClassifierMap->first << "_" << "KNN" << "_glob.txt";
				KNN->save((ss.str()).c_str());
				std::cout << ss.str() << " saved.\n";
				break;
			}
		default:
			{
				std::cout << "ClassificationData::SaveGlobalClassifiers: Wrong classifier type. No save function for this classifier implemented." << std::endl;
				f.close();
				return ipa_utils::RET_FAILED;
				break;
			}
		}
	}
	f.close();

	return 0;
}


int ClassificationData::LoadGlobalClassifiers(std::string pPath, ClassifierType pClassifierType)
{
	GlobalClassifierMap::iterator ItGlobalClassifierMap;
	for (ItGlobalClassifierMap = mGlobalClassifierMap.begin(); ItGlobalClassifierMap != mGlobalClassifierMap.end(); ItGlobalClassifierMap++)
	{
		ItGlobalClassifierMap->second->clear();
	}
	mGlobalClassifierMap.clear();

	std::stringstream FileName;
	FileName << pPath << "ClassifierInfo_" << ClassifierLabel(pClassifierType) << "_glob.txt";
	std::ifstream f((FileName.str()).c_str(), std::fstream::in);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::LoadGlobalClassifiers: Could not open '" << FileName.str() << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	int GlobalClassifierMapSize = 0;
	f >> GlobalClassifierMapSize;	// load number of classes
	
	for (int i=0; i<GlobalClassifierMapSize; i++)
	{
		std::string ClassName;
		f >> ClassName;			// class name
		f >> mGlobalClassifierThresholdMap[ClassName];		// classifier threshold

		// classifier accuracies
		for (int j=0; j<GlobalClassifierMapSize; j++)
		{
			std::string groundTruthClass;
			f >> groundTruthClass;
			f >> mGlobalClassifierAccuracy[ClassName][groundTruthClass];
			//std::cout << mGlobalClassifierAccuracy[ClassName][groundTruthClass] << "\t";
		}
		std::cout << std::endl;
		
		// load classifier
		switch(pClassifierType)
		{
		case CLASSIFIER_RTC:
			{
				mGlobalClassifierMap[ClassName] = new CvRTrees;
				CvRTrees* RTC = dynamic_cast<CvRTrees*>(mGlobalClassifierMap[ClassName]);
				std::stringstream ss;
				ss << pPath << ClassName << "_" << "RTC" << "_glob.txt";
				RTC->load((ss.str()).c_str());
				std::cout << ss.str() << " loaded.\n";
				break;
			}
		case CLASSIFIER_SVM:
			{
				mGlobalClassifierMap[ClassName] = new CvSVM;
				CvSVM* SVM = dynamic_cast<CvSVM*>(mGlobalClassifierMap[ClassName]);
				std::stringstream ss;
				ss << pPath << ClassName << "_" << "SVM" << "_glob.txt";
				SVM->load((ss.str()).c_str());
				std::cout << ss.str() << " loaded.\n";
				break;
			}
		case CLASSIFIER_BOOST:
			{
				mGlobalClassifierMap[ClassName] = new CvBoost;
				CvBoost* Boost = dynamic_cast<CvBoost*>(mGlobalClassifierMap[ClassName]);
				std::stringstream ss;
				ss << pPath << ClassName << "_" << "Boost" << "_glob.txt";
				Boost->load((ss.str()).c_str());
				std::cout << ss.str() << " loaded.\n";
				break;
			}
		case CLASSIFIER_KNN:
			{
				mGlobalClassifierMap[ClassName] = new CvKNearest;
				CvKNearest* KNN = dynamic_cast<CvKNearest*>(mGlobalClassifierMap[ClassName]);
				std::stringstream ss;
				ss << pPath << ClassName << "_" << "KNN" << "_glob.txt";
				KNN->load((ss.str()).c_str());
				std::cout << ss.str() << " loaded.\n";
				break;
			}
		default:
			{
				std::cout << "ClassificationData::LoadGlobalClassifiers: Wrong classifier type. No load function for this classifier implemented." << std::endl;
				f.close();
				return ipa_utils::RET_FAILED;
				break;
			}
		}
	}
	f.close();

	return 0;
}


int ClassificationData::SaveLocalClassifiers(std::string pPath, ClassifierType pClassifierType)
{
	std::stringstream FileName;
	FileName << pPath << "ClassifierInfo_" << ClassifierLabel(pClassifierType) << "_loc.txt";
	std::ofstream f((FileName.str()).c_str(), std::fstream::out);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::SaveLocalClassifiers: Could not open '" << FileName.str() << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	int LocalClassifierMapSize = mLocalClassifierMap.size();
	if(LocalClassifierMapSize==0)
	{
		std::cout << "ClassificationData::SaveLocalClassifiers: No classes to be saved for '" << FileName.str() << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	f << LocalClassifierMapSize << "\n";	// save number of classes
	
	LocalClassifierMap::iterator ItLocalClassifierMap;
	for (ItLocalClassifierMap = mLocalClassifierMap.begin(); ItLocalClassifierMap != mLocalClassifierMap.end(); ItLocalClassifierMap++)
	{
		f << ItLocalClassifierMap->first << "\n";		// class name
		
		// save classifier
		switch(pClassifierType)
		{
		case CLASSIFIER_RTC:
			{
				CvRTrees* RTC = NULL;
				RTC = dynamic_cast<CvRTrees*>(mLocalClassifierMap[ItLocalClassifierMap->first]);
				std::stringstream ss;
				ss << pPath << ItLocalClassifierMap->first << "_" << "RTC" << "_loc.txt";
				RTC->save((ss.str()).c_str());
				std::cout << ss.str() << " saved.\n";
				break;
			}
		case CLASSIFIER_SVM:
			{
				CvSVM* SVM = dynamic_cast<CvSVM*>(mLocalClassifierMap[ItLocalClassifierMap->first]);
				std::stringstream ss;
				ss << pPath << ItLocalClassifierMap->first << "_" << "SVM" << "_loc.txt";
				SVM->save((ss.str()).c_str());
				std::cout << ss.str() << " saved.\n";
				break;
			}
		case CLASSIFIER_BOOST:
			{
				CvBoost* Boost = dynamic_cast<CvBoost*>(mLocalClassifierMap[ItLocalClassifierMap->first]);
				std::stringstream ss;
				ss << pPath << ItLocalClassifierMap->first << "_" << "Boost" << "_loc.txt";
				Boost->save((ss.str()).c_str());
				std::cout << ss.str() << " saved.\n";
				break;
			}
		case CLASSIFIER_KNN:
			{
				CvKNearest* KNN = NULL;
				KNN = dynamic_cast<CvKNearest*>(mLocalClassifierMap[ItLocalClassifierMap->first]);
				std::stringstream ss;
				ss << pPath << ItLocalClassifierMap->first << "_" << "KNN" << "_loc.txt";
				KNN->save((ss.str()).c_str());
				std::cout << ss.str() << " saved.\n";
				break;
			}
		default:
			{
				std::cout << "ClassificationData::SaveLocalClassifiers: Wrong classifier type. No save function for this classifier implemented." << std::endl;
				f.close();
				return ipa_utils::RET_FAILED;
				break;
			}
		}
	}
	f.close();

	return 0;
}


int ClassificationData::LoadLocalClassifiers(std::string pPath, ClassifierType pClassifierType)
{
	/// at first, clean from old classifiers
	LocalClassifierMap::iterator ItLocalClassifierMap;
	for (ItLocalClassifierMap = mLocalClassifierMap.begin(); ItLocalClassifierMap != mLocalClassifierMap.end(); ItLocalClassifierMap++)
	{
		ItLocalClassifierMap->second->clear();
	}
	mLocalClassifierMap.clear();

	/// load classifiers
	std::stringstream FileName;
	FileName << pPath << "ClassifierInfo_" << ClassifierLabel(pClassifierType) << "_loc.txt";
	std::ifstream f((FileName.str()).c_str(), std::fstream::in);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::LoadLocalClassifiers: Could not open '" << FileName.str() << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	int LocalClassifierMapSize = 0;
	f >> LocalClassifierMapSize;	// load number of classes
	
	for (int i=0; i<LocalClassifierMapSize; i++)
	{
		std::string ClassName;
		f >> ClassName;			// class name
		
		// load classifier
		switch(pClassifierType)
		{
		case CLASSIFIER_RTC:
			{
				mLocalClassifierMap[ClassName] = new CvRTrees;
				CvRTrees* RTC = NULL;
				RTC = dynamic_cast<CvRTrees*>(mLocalClassifierMap[ClassName]);
				std::stringstream ss;
				ss << pPath << ClassName << "_" << "RTC" << "_loc.txt";
				RTC->load((ss.str()).c_str());
				std::cout << ss.str() << " loaded.\n";
				break;
			}
		case CLASSIFIER_SVM:
			{
				mLocalClassifierMap[ClassName] = new CvSVM;
				CvSVM* SVM = dynamic_cast<CvSVM*>(mLocalClassifierMap[ClassName]);
				std::stringstream ss;
				ss << pPath << ClassName << "_" << "SVM" << "_loc.txt";
				SVM->load((ss.str()).c_str());
				std::cout << ss.str() << " loaded.\n";
				break;
			}
		case CLASSIFIER_BOOST:
			{
				mLocalClassifierMap[ClassName] = new CvBoost;
				CvBoost* Boost = dynamic_cast<CvBoost*>(mLocalClassifierMap[ClassName]);
				std::stringstream ss;
				ss << pPath << ClassName << "_" << "Boost" << "_loc.txt";
				Boost->load((ss.str()).c_str());
				std::cout << ss.str() << " loaded.\n";
				break;
			}
		case CLASSIFIER_KNN:
			{
				mLocalClassifierMap[ClassName] = new CvKNearest;
				CvKNearest* KNN = dynamic_cast<CvKNearest*>(mLocalClassifierMap[ClassName]);
				std::stringstream ss;
				ss << pPath << ClassName << "_" << "KNN" << "_loc.txt";
				KNN->load((ss.str()).c_str());
				std::cout << ss.str() << " loaded.\n";
				break;
			}
		default:
			{
				std::cout << "ClassificationData::LoadLocalClassifiers: Wrong classifier type. No load function for this classifier implemented." << std::endl;
				f.close();
				return ipa_utils::RET_FAILED;
				break;
			}
		}
	}
	f.close();

	return 0;
}


int ClassificationData::SaveSqrtInverseCovarianceMatrix(std::string pFileName)
{
	std::ofstream f(pFileName.c_str(), std::fstream::out);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::SaveSqrtInverseCovarianceMatrix: Could not open '" << pFileName << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	int CovarMatrixSize = mSqrtInverseCovarianceMatrix->cols;
	if(CovarMatrixSize==0 || CovarMatrixSize!=mSqrtInverseCovarianceMatrix->rows)
	{
		std::cout << "ClassificationData::SaveSqrtInverseCovarianceMatrix: Covariance matrix corrupted." << std::endl;
		return ipa_utils::RET_FAILED;
	}

	f << CovarMatrixSize << "\n";		//size

	for (int i=0; i<CovarMatrixSize; i++)
	{
		for (int j=0; j<CovarMatrixSize; j++)
			f << cvmGet(mSqrtInverseCovarianceMatrix, i, j) << "\t";
		f << "\n";
	}

	f.close();

	std::cout << "Covariance matrix (sqrt(inv(Cov))) saved.\n";

	return ipa_utils::RET_OK;
}


int ClassificationData::LoadSqrtInverseCovarianceMatrix(std::string pFileName)
{
	std::ifstream f(pFileName.c_str(), std::fstream::in);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::LoadSqrtInverseCovarianceMatrix: Could not open '" << pFileName << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}
	int CovarMatrixSize = 0;

	f >> CovarMatrixSize;	//size

	if (CovarMatrixSize < 1)
	{
		std::cout << "ClassificationData::LoadSqrtInverseCovarianceMatrix: Covariance matrix corrupted." << std::endl;
		return ipa_utils::RET_FAILED;
	}

	cvReleaseMat(&mSqrtInverseCovarianceMatrix);
	mSqrtInverseCovarianceMatrix = cvCreateMat(CovarMatrixSize, CovarMatrixSize, CV_32FC1);

	double Data=0;
	for (int i=0; i<CovarMatrixSize; i++)
	{
		for (int j=0; j<CovarMatrixSize; j++)
		{
			f >> Data;
			cvmSet(mSqrtInverseCovarianceMatrix, i, j, Data);
		}
	}

	f.close();

	std::cout << "Covariance matrix (sqrt(inv(Cov))) loaded.\n";

	return ipa_utils::RET_OK;
}


int ClassificationData::SaveLocalFeatureClusterer(std::string pFileName)
{
	//mLocalFeatureClusterer->save(pFileName.c_str());

	std::ofstream f(pFileName.c_str(), std::fstream::out);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::SaveLocalFeatureClusterer: Could not open '" << pFileName << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}

#if (CV_MAJOR_VERSION<=2 && CV_MINOR_VERSION<=3)
	int numberClusters = mLocalFeatureClusterer->get_nclusters();
	CvMat* Means = (CvMat*)mLocalFeatureClusterer->get_means();
	CvMat* Weights = (CvMat*)mLocalFeatureClusterer->get_weights();
#else
	int numberClusters = mLocalFeatureClusterer->get<int>("nclusters");
	cv::Mat meansMat = mLocalFeatureClusterer->get<cv::Mat>("means");
	CvMat Means_ = (CvMat)meansMat;
	CvMat* Means = &Means_;
	cv::Mat weightsMat = mLocalFeatureClusterer->get<cv::Mat>("weights");
	CvMat Weights_ = (CvMat)weightsMat;
	CvMat* Weights = &Weights_;
#endif
	f << numberClusters << "\n";			// number of clusters

	f << Means->rows << "\t" << Means->cols << "\n";				// size of means matrix
	f << MatToString(Means);										// means matrix

	f << Weights->rows << "\t" << Weights->cols << "\n";			// size of weights matrix
	f << MatToString(Weights);										// weights matrix

#if (CV_MAJOR_VERSION<=2 && CV_MINOR_VERSION<=3)
	CvMat** Covs = (CvMat**)mLocalFeatureClusterer->get_covs();
	for (int i=0; i<numberClusters; i++)
	{
		f << (Covs[i])->rows << "\t" << (Covs[i])->cols << "\n";	// size of covariance matrix
		f << MatToString(Covs[i]);									// covariance matrix
	}
#else
	std::vector<cv::Mat> covsVector = mLocalFeatureClusterer->get<std::vector<cv::Mat> >("covs");
	for (int i=0; i<numberClusters; i++)
	{
		f << covsVector[i].rows << "\t" << covsVector[i].cols << "\n";	// size of covariance matrix
		CvMat covsI = (CvMat)covsVector[i];
		f << MatToString(&covsI);									// covariance matrix
	}
#endif
	
/*	CvMat* Probs = (CvMat*)mLocalFeatureClusterer->get_probs();
	f << Probs->rows << "\t" << Probs->cols	<< "\n";				// size of probabilities matrix
	f << MatToString(Probs);										// probabilities matrix
*/
	f.close();

	std::cout << "Local feature clusterer (EM) saved.\n";

	return ipa_utils::RET_OK;
}


int ClassificationData::LoadLocalFeatureClusterer(std::string pFileName)
{
	//mLocalFeatureClusterer->load(pFileName.c_str());

	std::ifstream f(pFileName.c_str(), std::fstream::in);
	if(!f.is_open())
	{
		std::cout << "ClassificationData::LoadLocalFeatureClusterer: Could not open '" << pFileName << "'" << std::endl;
		return ipa_utils::RET_FAILED;
	}

	int NumberClusters=0;
	f >> NumberClusters;											// number of clusters

	int Height=0, Width=0;
	f >> Height >> Width;											// size of means matrix
	CvMat* Means = cvCreateMat(Height, Width, CV_32FC1);
	IfstreamToMat(f, Means);										// means matrix

	f >> Height >> Width;											// size of weights matrix
	CvMat* Weights = cvCreateMat(Height, Width, CV_32FC1);
	IfstreamToMat(f, Weights);										// weights matrix

	CvMat** Covs = new CvMat*[NumberClusters];
	for (int i=0; i<NumberClusters; i++)
	{
		f >> Height >> Width;										// size of covariance matrix
		Covs[i] = cvCreateMat(Height, Width, CV_32FC1);
		IfstreamToMat(f, Covs[i]);									// covariance matrix
	}
	
/*	f >> Height >> Width;											// size of probabilities matrix
	CvMat* Probs = cvCreateMat(Height, Width, CV_32FC1);
	IfstreamToMat(f, Probs);										// probabilities matrix
*/
	f.close();

	int NumberSamples = GetNumberLocalFeaturePoints();
	if (NumberSamples < 1)
	{
		std::cout << "ClassificationData::LoadLocalFeatureClusterer: Could not determine NumberSamples (" << NumberSamples << ") correctly." << std::endl;
		return ipa_utils::RET_FAILED;
	}
	int NumberFeatures = GetNumberLocalFeatures();
	if (NumberFeatures < 1)
	{
		std::cout << "ClassificationData::LoadLocalFeatureClusterer: Could not determine NumberFeatures (" << NumberFeatures << ") correctly." << std::endl;
		return ipa_utils::RET_FAILED;
	}
	CvMat* AllLocalFeatures = CreateAllLocalFeatureMatrix(NumberSamples, NumberFeatures);
#if (CV_MAJOR_VERSION<=2 && CV_MINOR_VERSION<=3)
	CvEMParams EMParams = CvEMParams(NumberClusters, CvEM::COV_MAT_DIAGONAL, CvEM::START_E_STEP, cvTermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 100, FLT_EPSILON), /*(const CvMat*)Probs*/NULL, (const CvMat*)Weights, (const CvMat*)Means, (const CvMat**)Covs);
	mLocalFeatureClusterer->train(AllLocalFeatures, NULL, EMParams);
#else
	mLocalFeatureClusterer->set("nclusters", NumberClusters);
	mLocalFeatureClusterer->set("covMatType", cv::EM::COV_MAT_DIAGONAL);
	mLocalFeatureClusterer->set("maxIters", 100);
	mLocalFeatureClusterer->set("epsilon", FLT_EPSILON);
	cv::Mat weightsMat(Weights);
	cv::Mat meansMat(Means);
	std::vector<cv::Mat> covsVector(NumberClusters);
	for (int i=0; i<NumberClusters; i++)
		covsVector[i] = cv::Mat(Covs[i]);
	cv::Mat allLocalFeaturesMat(AllLocalFeatures);
	mLocalFeatureClusterer->trainE(allLocalFeaturesMat, meansMat, covsVector, weightsMat);
#endif
	cvReleaseMat(&AllLocalFeatures);

	std::cout << "Local feature clusterer (EM) loaded.\n";

	return ipa_utils::RET_OK;
}


int ClassificationData::analyzeGlobalFeatureRepeatability(int pImageOffset, double& pIntraclassMeanDistance, double& pTransclassMeanDistance, double& pExtraclassMeanDistance)
{
	double outlierRejection = 50; //50.0;

	double intraDistanceAccumulator = 0.0;
	int intraDescriptorCounter = 0;
	double transDistanceAccumulator = 0.0;
	int transDescriptorCounter = 0;
	double extraDistanceAccumulator = 0.0;
	int extraDescriptorCounter = 0;

	// iterate over all classes
	GlobalFeaturesMap::iterator itGlobalFeaturesMap;
	for (itGlobalFeaturesMap = mGlobalFeaturesMap.begin(); itGlobalFeaturesMap != mGlobalFeaturesMap.end(); itGlobalFeaturesMap++)
	{
		// iterate over all objects of each class
		ObjectNrFeatureMap::iterator itObject, itObject2;
		for (itObject = itGlobalFeaturesMap->second.begin(); itObject != itGlobalFeaturesMap->second.end(); itObject++)
		{
			std::string currentClass = itGlobalFeaturesMap->first;
			CvMat* data = itObject->second;
			int numberSamples = data->rows;
			for (int sample=0; sample<numberSamples; sample++)
			{
				CvMat* baseDescriptor = cvCreateMatHeader(1, data->cols, data->type);
				CvMat* offsetDescriptor = cvCreateMatHeader(1, data->cols, data->type);
				cvGetRow(data, baseDescriptor, sample);
				int nextSampleIndex = (sample+pImageOffset)%numberSamples;
				cvGetRow(data, offsetDescriptor, nextSampleIndex);

				//for (int j=0; j<baseDescriptor->cols; j++) std::cout << cvGetReal1D(baseDescriptor, j) << "\t";
				//std::cout << "\n";
				//for (int j=0; j<offsetDescriptor->cols; j++) std::cout << cvGetReal1D(offsetDescriptor, j) << "\t";
				//std::cout << "\n";

				// measure distance between both descriptors
				double norm = cvNorm(baseDescriptor, offsetDescriptor, CV_L2);
				if (norm<outlierRejection)
				{
					intraDistanceAccumulator += norm;
					intraDescriptorCounter++;
				}

				// measure distance to descriptor of same rotation offset of other instances from this class
				for (itObject2 = itGlobalFeaturesMap->second.begin(); itObject2 != itGlobalFeaturesMap->second.end(); itObject2++)
				{
					if (itObject2->first == itObject->first) continue;

					CvMat* dataInclass = itObject2->second;
					CvMat* inclassDescriptor = cvCreateMatHeader(1, dataInclass->cols, dataInclass->type);
					int nextSampleIndex2 = (sample+pImageOffset)%itObject2->second->rows;
					cvGetRow(dataInclass, inclassDescriptor, nextSampleIndex2);

					//for (int j=0; j<baseDescriptor->cols; j++) std::cout << cvGetReal1D(baseDescriptor, j) << "\t";
					//std::cout << "\n";
					//for (int j=0; j<inclassDescriptor->cols; j++) std::cout << cvGetReal1D(inclassDescriptor, j) << "\t";
					//std::cout << "\n";

					// measure distance between both descriptors
					double norm = cvNorm(baseDescriptor, inclassDescriptor, CV_L2);
					if (norm<outlierRejection)
					{
						transDistanceAccumulator += norm;
						transDescriptorCounter++;
					}

					cvReleaseMatHeader(&inclassDescriptor);
				}

				// measure distance to descriptors from other classes
				for (int i=0; i<100; i++)
				{
					// find class randomly
					while(1)
					{
						int ClassIndex = int(mGlobalFeaturesMap.size()*((double)rand()/((double)RAND_MAX+1.0)));
						mItGlobalFeaturesMap = mGlobalFeaturesMap.begin();
						for (int k=0; k<ClassIndex; k++, mItGlobalFeaturesMap++);
						if (mItGlobalFeaturesMap->first != currentClass) break;
					}

					// find object randomly
					int ObjectIndex = int(mItGlobalFeaturesMap->second.size()*((double)rand()/((double)RAND_MAX+1.0)));
					GlobalFeaturesMap::value_type::second_type::iterator ItObjectMap = mItGlobalFeaturesMap->second.begin();
					for (int k=0; k<ObjectIndex; k++, ItObjectMap++);

					// find sample randomly
					int otherSampleIndex = int(ItObjectMap->second->height*((double)rand()/((double)RAND_MAX+1.0)));
					CvMat* otherDescriptor = cvCreateMatHeader(1, data->cols, data->type);
					cvGetRow(ItObjectMap->second, otherDescriptor, otherSampleIndex);
					double norm = cvNorm(baseDescriptor, otherDescriptor, CV_L2);
					if(norm<outlierRejection)
					{
						extraDistanceAccumulator += norm;
						extraDescriptorCounter++;
					}
					cvReleaseMatHeader(&otherDescriptor);
				}

				cvReleaseMatHeader(&baseDescriptor);
				cvReleaseMatHeader(&offsetDescriptor);
			}
		}
	}

	pIntraclassMeanDistance = intraDistanceAccumulator/(double)intraDescriptorCounter;
	pTransclassMeanDistance = transDistanceAccumulator/(double)transDescriptorCounter;
	pExtraclassMeanDistance = extraDistanceAccumulator/(double)extraDescriptorCounter;

	return ipa_utils::RET_OK;
}


std::string ClassifierLabel(ClassifierType pClassifierType)
{
	switch (pClassifierType)
	{
		case CLASSIFIER_RTC: return "RTC"; break;
		case CLASSIFIER_SVM: return "SVM"; break;
		case CLASSIFIER_BOOST: return "Boost"; break;
		case CLASSIFIER_KNN: return "KNN"; break;
	}

	return "RET_FAILED";
}


std::string MatToString(CvMat* pMatrix)
{
	std::stringstream Str;

	for (int i=0; i<pMatrix->rows; i++)
	{
		for (int j=0; j<pMatrix->cols; j++) Str << cvmGet(pMatrix, i, j) << "\t";
		Str << "\n";
	}

	return Str.str();
}


int IfstreamToMat(std::ifstream& pFile, CvMat* pMatrix)
{
	double Value = 0;
	for (int i=0; i<pMatrix->rows; i++)
	{
		for (int j=0; j<pMatrix->cols; j++)
		{
			pFile >> Value;
			cvmSet(pMatrix, i, j, Value);
		}
	}

	return ipa_utils::RET_OK;
}


double sign(double pNumber)
{
	if (pNumber>0) return 1.0;
	if (pNumber<0) return -1.0;
	return 0.0;
}
