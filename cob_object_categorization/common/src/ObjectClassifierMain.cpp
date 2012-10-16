#include "object_categorization/ObjectClassifier.h"

#include <boost/filesystem.hpp>
//#include <boost/format.hpp>
namespace fs = boost::filesystem;

//Libs:
//opencv_core231.lib
//opencv_imgproc231.lib
//opencv_highgui231.lib
//opencv_ml231.lib
//opencv_features2d231.lib
//opencv_calib3d231.lib
//pcl_features.lib
//pcl_common.lib
//pcl_kdtree.lib
//pcl_search.lib
//pcl_filters.lib


int main()
{
	ObjectClassifier OC;

	//OC.GetDataPointer()->LoadLocalFeatures("IPA2Data/IPA2_Surf64Dev2_loc.txt");
	//OC.GetDataPointer()->SaveLocalFeatures("IPA2Data/IPA2_Surf64_loc.txt");
	//std::cout << "end...";
	//getchar();


	// parameters
	ObjectClassifier::LocalFeatureParams localFeatureParams;
	localFeatureParams.useFeature = "surf";		// "surf" //"rsd" //"fpfh"

	ObjectClassifier::GlobalFeatureParams globalFeatureParams;
	globalFeatureParams.minNumber3DPixels = 50;
	globalFeatureParams.numberLinesX.push_back(7);
//	globalFeatureParams.numberLinesX.push_back(2);
	globalFeatureParams.numberLinesY.push_back(7);
//	globalFeatureParams.numberLinesY.push_back(2);
	globalFeatureParams.polynomOrder.push_back(2);
//	globalFeatureParams.polynomOrder.push_back(4);
	globalFeatureParams.pointDataExcess = 0;	//int(3.01*(globalFeatureParams.polynomOrder+1));	// excess decreases the accuracy
	globalFeatureParams.cellCount[0] = 5;
	globalFeatureParams.cellCount[1] = 5;
	globalFeatureParams.cellSize[0] = 0.5;
	globalFeatureParams.cellSize[1] = 0.5;
	globalFeatureParams.vocabularySize = 250;
//	globalFeatureParams.additionalArtificialTiltedViewAngle.push_back(0.);
//	globalFeatureParams.additionalArtificialTiltedViewAngle.push_back(45.);
	double factorSamplesTrainData = 1.;//2./3.;		// for each object, this ratio of samples should go to the training set, the rest is for testing (i.e. there are training and test objects but only factorSamplesTrainData of the samples of each training object are used for training - and only 1.0-factorSamplesTrainData of the samples from test objects are used for testing, e.g. use this to put samples from different tilt angles into the list - first tilt=0, then tilt=beta and finally tilt=beta/2 -> test set only with beta/2)
	globalFeatureParams.thinningFactor = 1.0;
	globalFeatureParams.useFeature["bow"] = false;
	globalFeatureParams.useFeature["sap"] = true;
	globalFeatureParams.useFeature["sap2"] = false;
	globalFeatureParams.useFeature["pointdistribution"] = false;
	globalFeatureParams.useFeature["normalstatistics"] = false;
	globalFeatureParams.useFeature["vfh"] = false;
	globalFeatureParams.useFeature["grsd"] = false;
	globalFeatureParams.useFeature["gfpfh"] = false;
	globalFeatureParams.useFullPCAPoseNormalization = false;
	globalFeatureParams.useRollPoseNormalization = false;

	bool useSloppyMasks = true;

	int viewsPerObject = -1;

	const ClusterMode clusterMode = CLUSTER_EM;
	const ClassifierType classifierType = CLASSIFIER_RTC;
	const int crossValidationFold = 10;
	const float factorNegativeSet = 6.f;	// very old: 5.f
	const float percentTest = 0.f;
	const float percentValidation = 0.1f;

	// hermes capture and object finding with angle determination
	bool hermes = false;
	if (hermes == true)
	{
		ObjectClassifier objectClassifier;

		//////////////////////////////////////////////////////////////////////////
		//std::stringstream metaFileName;
		//metaFileName << "common/files/hermes/" << "shoe" << "_labels.txt";
		//std::fstream mLabelFile(metaFileName.str().c_str(), std::ios::in);
		//std::fstream newLabelFile("common/files/hermes/shoe_labels_new.txt", std::ios::out);
		//if (mLabelFile.is_open() == false || newLabelFile.is_open()==false)
		//{
		//	std::cout << "ObjectClassifier::HermesCapture: Error: Could not open " << metaFileName.str() << "." << std::endl;
		//	return ipa_utils::RET_FAILED;
		//}
		//for (int i=0; i<261; i++)
		//{
		//	std::stringstream filename;
		//	filename << "common/files/hermes/" << "shoe" << "_" << i << ".pcd";
		//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr center_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
		//	pcl::io::loadPCDFile(filename.str(), *center_cluster);
		//	pcl::PointXYZ avgPoint;
		//	cv::Mat histogram;
		//	objectClassifier.HermesComputeRollHistogram(center_cluster, avgPoint, histogram);
		//	
		//	std::string tempstr;
		//	double pan, tilt;
		//	int descSize;
		//	mLabelFile >> tempstr;
		//	newLabelFile << tempstr << "\t";
		//	mLabelFile >> pan;
		//	newLabelFile << pan << "\t";
		//	mLabelFile >> tilt;
		//	newLabelFile << tilt << std::endl;
		//	mLabelFile >> tempstr;
		//	newLabelFile << tempstr << "\t";
		//	mLabelFile >> descSize;
		//	newLabelFile << descSize << "\t";
		//	std::vector<float> sapDesc(descSize);
		//	for (int i=0; i<descSize; i++)
		//	{
		//		mLabelFile >> sapDesc[i];
		//		newLabelFile << sapDesc[i] << "\t";
		//	}
		//	newLabelFile << std::endl;
		//	mLabelFile >> tempstr;
		//	newLabelFile << tempstr << "\t";
		//	mLabelFile >> descSize;
		//	newLabelFile << descSize << "\t";
		//	std::vector<float> vfhDesc(descSize);
		//	for (int i=0; i<descSize; i++)
		//	{
		//		mLabelFile >> vfhDesc[i];
		//		newLabelFile << vfhDesc[i] << "\t";
		//	}
		//	newLabelFile << std::endl;
		//	newLabelFile << "rollhistogram\t" << histogram.cols << "\t";
		//	for (int i=0; i<histogram.cols; i++)
		//	{
		//		newLabelFile << histogram.at<float>(i) << "\t";
		//	}
		//	newLabelFile << std::endl;
		//}
		//mLabelFile.close();
		//newLabelFile.close();
		//return 0;
		//////////////////////////////////////////////////////////////////////////

		globalFeatureParams.useFullPCAPoseNormalization = false;
		globalFeatureParams.useRollPoseNormalization = true;
		std::cout << "Capture images (1) or run detection (2)? ";
		std::string input;
		std::cin >> input;
		if (input.compare("1")==0)
			objectClassifier.HermesCapture(clusterMode, classifierType, globalFeatureParams);
		else
			objectClassifier.HermesDetect(clusterMode, classifierType, globalFeatureParams);

		return 0;
	}
	

	// classify objects with attached kinect
	bool runtimeUse = false;
	if (runtimeUse)
	{
		//ObjectClassifier objectClassifier("common/files/WashingtonData/PCA3CF - roll pose normalization/PCA3CF7-7-2/Classifier/EMClusterer5.txt", "common/files/WashingtonData/PCA3CF - roll pose normalization/PCA3CF7-7-2/Classifier/");
		ObjectClassifier objectClassifier("common/files/IPA2Data/exp 21 - PCA3CF - roll pose normalization/PCA3CF7-7-2/Classifier/EMClusterer5.txt", "common/files/IPA2Data/exp 21 - PCA3CF - roll pose normalization/PCA3CF7-7-2/Classifier/");
		objectClassifier.CategorizeContinuously(clusterMode, classifierType, globalFeatureParams);
		return 0;
	}


	std::string comments = "Surf64 on RGBI image data.\n\n";

	bool experimentalSeries = false;
	if (experimentalSeries)
	{
		//for (globalFeatureParams.useFullPCAPoseNormalization = true; globalFeatureParams.useFullPCAPoseNormalization == true; globalFeatureParams.useFullPCAPoseNormalization = false)
		//{
			std::string baseFolder = "common/files/IPA2Data/";//"IPA2Data/";
			std::string experimentSubFolder = "exp 17 - vfh/"; //"exp 11 - double sap/";//"exp 1 - PCA3CF/"; //"exp 2 - PCA3CF no pose normalization/"; //"Exp15 - PCA3CF - roll pose normalization/";//"exp 21 - PCA3CF - roll pose normalization/"; //"exp 24 - tilt/";	//"exp 20 - thin point clouds/"; //"Exp15 - PCA3CF - roll pose normalization/";//"exp 4 - PCA3CF less views/";//"exp 6 - single descriptors/";//"exp 5 - PCA3CF no pose normalization - less views/";//"exp 6 - single descriptors/";//"exp 2 - PCA3CF no pose normalization/";//"Exp13 - PCA3CF-nonorm/";//"Exp7 - PCA3CF/";//"exp 1 - PCA3CF/";
			/*if (globalFeatureParams.useFullPCAPoseNormalization == true)
				experimentSubFolder = "exp 4 - PCA3CF less views/";
			else
				experimentSubFolder = "exp 5 - PCA3CF no pose normalization - less views/";*/
			std::string classifierFolder = "Classifier/";
			std::string statisticsFolder = "Statistics/";
			fs::path basePath(baseFolder);
			if (!fs::exists(basePath))
			{
				std::cout << "Error: main: The provided base folder '" << baseFolder << "' does not exist." << std::endl;
				return 0;
			}
			fs::path experimentSubPath(baseFolder + experimentSubFolder);
			if (!fs::exists(experimentSubPath))
			{
				bool res = fs::create_directory(experimentSubPath);
				if (res == false)
				{
					std::cout << "Error: main: The provided sub folder '" << experimentSubPath << "' could not be created." << std::endl;
					return 0;
				}
			}

			// descriptor parameters
			//for (globalFeatureParams.polynomOrder[0] = 2; globalFeatureParams.polynomOrder[0] <= 2; globalFeatureParams.polynomOrder[0] += 2)
			//{
				//for (globalFeatureParams.numberLinesX[0] = 7; globalFeatureParams.numberLinesX[0] <= 8; globalFeatureParams.numberLinesX[0]++)
				//{
				//	globalFeatureParams.numberLinesY[0] = globalFeatureParams.numberLinesX[0];
			//for (globalFeatureParams.polynomOrder[1] = 4; globalFeatureParams.polynomOrder[1] <= 4; globalFeatureParams.polynomOrder[1] += 2)
			//{
				//for (globalFeatureParams.numberLinesX[1] = 2; globalFeatureParams.numberLinesX[1] <= 7; globalFeatureParams.numberLinesX[1]++)
				//{
				//	globalFeatureParams.numberLinesY[1] = globalFeatureParams.numberLinesX[1];


			// thinning
			std::vector<double> thinningFactors;
			thinningFactors.push_back(0.04);
			//thinningFactors.push_back(0.0625);
			thinningFactors.push_back(0.1);
			thinningFactors.push_back(0.25);
			//thinningFactors.push_back(0.5);

			for (int thf=0; thf<(int)thinningFactors.size(); thf++)
			{
				globalFeatureParams.thinningFactor = thinningFactors[thf];

			//for (globalFeatureParams.vocabularySize = 150; globalFeatureParams.vocabularySize<=250; globalFeatureParams.vocabularySize+=50)
			//{

			// less views
			//std::vector<int> viewsPerObjectVec;
			//viewsPerObjectVec.push_back(4);
			//viewsPerObjectVec.push_back(6);
			//viewsPerObjectVec.push_back(8);
			//viewsPerObjectVec.push_back(12);
			//viewsPerObjectVec.push_back(16);
			//viewsPerObjectVec.push_back(18);
			//viewsPerObjectVec.push_back(24);
			//for (int viewsPerObjectIndex = 0; viewsPerObjectIndex < (int)viewsPerObjectVec.size(); viewsPerObjectIndex++)
			//{
			//	viewsPerObject = viewsPerObjectVec[viewsPerObjectIndex];

			// tilt angles
			//for (double tiltAngle=20.; tiltAngle<=40.0; tiltAngle+=10.)
			//{
			//	globalFeatureParams.additionalArtificialTiltedViewAngle[0] = 2*tiltAngle;
			//	globalFeatureParams.additionalArtificialTiltedViewAngle[1] = tiltAngle;

					std::cout << "\n\n\nProcessing PCA3CF" << globalFeatureParams.numberLinesX[0] << "-" << globalFeatureParams.numberLinesY[0] << "-" << globalFeatureParams.polynomOrder[0] << "+"
															<< (globalFeatureParams.numberLinesX.size()>1 ? globalFeatureParams.numberLinesX[1] : 0) << "-" << (globalFeatureParams.numberLinesY.size()>1 ? globalFeatureParams.numberLinesY[1] : 0) << "-" << (globalFeatureParams.polynomOrder.size()>1 ? globalFeatureParams.polynomOrder[1] : 0)
															<< " Pose normalization=" << globalFeatureParams.useFullPCAPoseNormalization << " viewsPerObject=" << viewsPerObject << "\n" << std::endl;

					std::stringstream parameters;
					parameters << "Parameters:\nminNumber3DPixels = " << globalFeatureParams.minNumber3DPixels <<
						"\nnumberLinesX = {" << globalFeatureParams.numberLinesX[0] << ", " << (globalFeatureParams.numberLinesX.size()>1 ? globalFeatureParams.numberLinesX[1] : 0) << "}" <<
						"\nnumberLinesY = {" << globalFeatureParams.numberLinesY[0] << ", " << (globalFeatureParams.numberLinesY.size()>1 ? globalFeatureParams.numberLinesY[1] : 0) << "}" <<
						"\npolynomOrder = {" << globalFeatureParams.polynomOrder[0] << ", " << (globalFeatureParams.polynomOrder.size()>1 ? globalFeatureParams.polynomOrder[1] : 0) << "}" <<
						"\npointDataExcess = " << globalFeatureParams.pointDataExcess << 
						"\ncellCount = (" << globalFeatureParams.cellCount[0] << ", " << globalFeatureParams.cellCount[1] << ")" <<
						"\ncellSize = (" << globalFeatureParams.cellSize[0] << ", " << globalFeatureParams.cellSize[1] << ")" <<
						"\nvocabularySize = " << globalFeatureParams.vocabularySize << 
						"\nadditionalArtificialTiltedViewAngle = " << globalFeatureParams.additionalArtificialTiltedViewAngle.size() << "times,  {" << (globalFeatureParams.additionalArtificialTiltedViewAngle.size()>0 ? globalFeatureParams.additionalArtificialTiltedViewAngle[0] : 0) << ", " << (globalFeatureParams.additionalArtificialTiltedViewAngle.size()>1 ? globalFeatureParams.additionalArtificialTiltedViewAngle[1] : 0) << "}" << 
						"\nfactorSamplesTrainData = " << factorSamplesTrainData <<
						"\nthinningFactor = " << globalFeatureParams.thinningFactor << 
						"\nuseFeature['bow'] = " << globalFeatureParams.useFeature["bow"] <<
						"\nuseFeature['sap'] = " << globalFeatureParams.useFeature["sap"] <<
						"\nuseFeature['sap2'] = " << globalFeatureParams.useFeature["sap2"] <<
						"\nuseFeature['pointdistribution'] = " << globalFeatureParams.useFeature["pointdistribution"] <<
						"\nuseFeature['normalstatistics'] = " << globalFeatureParams.useFeature["normalstatistics"] <<
						"\nuseFeature['vfh'] = " << globalFeatureParams.useFeature["vfh"] <<
						"\nuseFeature['grsd'] = " << globalFeatureParams.useFeature["grsd"] <<
						"\nuseFeature['gfpfh'] = " << globalFeatureParams.useFeature["gfpfh"] <<
						"\nuseFullPCAPoseNormalization = " << globalFeatureParams.useFullPCAPoseNormalization <<
						"\nuseRollPoseNormalization = " << globalFeatureParams.useRollPoseNormalization <<
						"\nviewsPerObject (-1 = all) = " << viewsPerObject <<
						"\nclusterMode = " << clusterMode <<
						"\nclassifierType = " << classifierType <<
						"\ncrossValidationFold = " << crossValidationFold <<
						"\nfactorNegativeSet = " << factorNegativeSet <<
						"\npercentTest = " << percentTest <<
						"\npercentValidation = " << percentValidation <<
						"\nuseSloppyMasks = " << useSloppyMasks << std::endl;

					// create paths
					std::stringstream ss;
					ss << "vfh"/*"PCA3CF" << globalFeatureParams.numberLinesX[0] << "-" << globalFeatureParams.numberLinesY[0] << "-" << globalFeatureParams.polynomOrder[0] << "+"
									<< globalFeatureParams.numberLinesX[1] << "-" << globalFeatureParams.numberLinesY[1] << "-" << globalFeatureParams.polynomOrder[1];*/  << " - " << globalFeatureParams.thinningFactor << "points" ; //<< "-tilt-0-" << globalFeatureParams.additionalArtificialTiltedViewAngle[1] << "-" << globalFeatureParams.additionalArtificialTiltedViewAngle[0]; //<< "-view" << viewsPerObject;
		//			ss << /*"EM" << globalFeatureParams.vocabularySize /*<<*/ "PCA3CF" << globalFeatureParams.numberLinesX[0] << "-" << globalFeatureParams.numberLinesY[0] << "-" << globalFeatureParams.polynomOrder[0]; // << "-view" << viewsPerObject;	//<< " - rollnorm - tilt 0-" << tiltAngle << "-" << 2*tiltAngle; //(globalFeatureParams.useFullPCAPoseNormalization == false ? " - nonorm - " : " - ") << globalFeatureParams.thinningFactor << "points";    // << "-view" << viewsPerObject;
					//if (globalFeatureParams.useFullPCAPoseNormalization == true) ss << "normalization - ";
					//else ss << "no normalization - ";
					//if (useSloppyMasks == true) ss << "oldmask";
					//else ss << "goodmask";
					std::string instanceFolder = ss.str() + "/";
					ss.str("");
					ss.clear();
					ss << "vfh";//"PCA3CF" << globalFeatureParams.numberLinesX[0] << "-" << globalFeatureParams.numberLinesY[0] << "-" << globalFeatureParams.polynomOrder[0] << "+"
								//	<< globalFeatureParams.numberLinesX[1] << "-" << globalFeatureParams.numberLinesY[1] << "-" << globalFeatureParams.polynomOrder[1];
		//			ss << /*"EM" << globalFeatureParams.vocabularySize /*<<*/ "PCA3CF" << globalFeatureParams.numberLinesX[0] << "-" << globalFeatureParams.numberLinesY[0] << "-" << globalFeatureParams.polynomOrder[0];
					fs::path instancePath(baseFolder + experimentSubFolder + instanceFolder);
					if (!fs::exists(instancePath))
					{
						bool res = fs::create_directory(instancePath);
						if (res == false)
						{
							std::cout << "Error: main: The provided sub folder '" << instancePath << "' could not be created." << std::endl;
							return 0;
						}
					}
					fs::path classifierPath(baseFolder + experimentSubFolder + instanceFolder + classifierFolder);
					if (!fs::exists(classifierPath))
					{
						bool res = fs::create_directory(classifierPath);
						if (res == false)
						{
							std::cout << "Error: main: The provided sub folder '" << classifierPath << "' could not be created." << std::endl;
							return 0;
						}
					}
					fs::path statisticsPath(baseFolder + experimentSubFolder + instanceFolder + classifierFolder + statisticsFolder);
					if (!fs::exists(statisticsPath))
					{
						bool res = fs::create_directory(statisticsPath);
						if (res == false)
						{
							std::cout << "Error: main: The provided sub folder '" << statisticsPath << "' could not be created." << std::endl;
							return 0;
						}
					}
				
					// load database or compute features
					std::string localFeatureFileName = baseFolder + experimentSubFolder + "IPA2_Surf64Dev2_loc.txt";//"IPA2_Surf64Dev2_loc.txt";
					std::string globalFeatureFileName = baseFolder + experimentSubFolder + instanceFolder + /*"IPA2_Surf64Dev2_"*/"IPA2_Surf64Dev2_"  + ss.str() + "_glob.txt";
					//std::string globalFeatureFileName = baseFolder + experimentSubFolder + "IPA2_Surf64Dev2_"  + ss.str() + "_glob.txt";
					std::string screenOutputLogFileName = baseFolder + experimentSubFolder + instanceFolder + /*"IPA2_Surf64Dev2_"*/"IPA2_Surf64Dev2_"  + ss.str() + "_screen_log.txt";
					//std::string screenOutputLogFileName = baseFolder + experimentSubFolder + /*"IPA2_Surf64Dev2_"*/"IPA2_Surf64Dev2_"  + ss.str() + "_screen_log_trash.txt";
					std::string timingLogFileName = baseFolder + experimentSubFolder + instanceFolder + /*"IPA2_Surf64Dev2_"*/"IPA2_Surf64Dev2_"  + ss.str() + "_timing.txt";
					std::string covarianceMatrixFileName = baseFolder + experimentSubFolder + "IPA2_Surf64Dev2_loc_covar";//"IPA2_Surf64Dev2_loc_covar";
					std::string localFeatureClustererPath = baseFolder + experimentSubFolder + instanceFolder + "Classifier";
					std::string databasePath;
					if (useSloppyMasks == true)
						databasePath = "F:/ObjectDataNew/TrainingData/";
					else
					{
						databasePath = "F:/ObjectDataNew_goodmasks/TrainingData/";
						localFeatureFileName = baseFolder + experimentSubFolder + "IPA2_Surf64Dev2_loc_goodmask.txt";
					}
					std::ofstream screenLogFile(screenOutputLogFileName, std::ios::app);
					if (!screenLogFile.is_open())
					{
						std::cout << "Error: main: The log file " << screenOutputLogFileName << " could not be opened." << std::endl;
						return 0;
					}
					screenLogFile << "\n\n\n\nCorrected multi-class computation, exp. factor 2, with same random seed for all experiments:\n\n" << comments << parameters.str() << std::endl;
					std::cout << comments << parameters.str() << std::endl;

					// reset random seed to have same randomized conditions for all experiments
					srand(1);

					//OC.LoadCINDatabase("common/files/object_lists/annotation_cin.txt", "../../Datenbanken/IPA/", 1, clusterMode, globalFeatureParams, localFeatureFileName, globalFeatureFileName, covarianceMatrixFileName,
					//	localFeatureClustererPath, timingLogFileName, MASK_LOAD);
					OC.LoadCIN2Database("common/files/object_lists/annotation_cin2.txt", databasePath, 2, clusterMode, globalFeatureParams, localFeatureParams, localFeatureFileName, globalFeatureFileName, covarianceMatrixFileName,
										localFeatureClustererPath, timingLogFileName, screenLogFile, MASK_LOAD);
					//OC.LoadCIN2Database("common/files/object_lists/annotation_cin2.txt", databasePath, 2, clusterMode, globalFeatureParams, localFeatureFileName, globalFeatureFileName, covarianceMatrixFileName,
					//	localFeatureClustererPath, timingLogFileName, screenLogFile, MASK_LOAD);


					// cross-validate
					std::string nameTag = /*"IPA2_Surf64Dev2_"*/"IPA2_Surf64Dev2_" + ss.str();
					//OC.CrossValidationGlobalSampleRange(statisticsPath.string(), nameTag, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile, factorSamplesTrainData);
					OC.CrossValidationGlobal(statisticsPath.string(), nameTag, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile);
					OC.GetDataPointer()->SaveGlobalClassifiers(classifierPath.string(), classifierType);
					//OC.GetDataPointer()->LoadGlobalClassifiers(classifierPath.string(), classifierType);
					//OC.CrossValidationGlobalMultiClassSampleRange(statisticsPath.string(), nameTag, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile, factorSamplesTrainData);
					OC.CrossValidationGlobalMultiClass(statisticsPath.string(), nameTag, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile);

					screenLogFile.close();
				}
			//}
		//}

		return 0;
	}
	else
	{
		std::stringstream parameters;
		parameters << "Parameters:\nminNumber3DPixels = " << globalFeatureParams.minNumber3DPixels <<
			"\nnumberLinesX = {" << globalFeatureParams.numberLinesX[0] << ", " << (globalFeatureParams.numberLinesX.size()>1 ? globalFeatureParams.numberLinesX[1] : 0) << "}" <<
			"\nnumberLinesY = {" << globalFeatureParams.numberLinesY[0] << ", " << (globalFeatureParams.numberLinesY.size()>1 ? globalFeatureParams.numberLinesY[1] : 0) << "}" <<
			"\npolynomOrder = {" << globalFeatureParams.polynomOrder[0] << ", " << (globalFeatureParams.polynomOrder.size()>1 ? globalFeatureParams.polynomOrder[1] : 0) << "}" <<
			"\npointDataExcess = " << globalFeatureParams.pointDataExcess << 
			"\ncellCount = (" << globalFeatureParams.cellCount[0] << ", " << globalFeatureParams.cellCount[1] << ")" <<
			"\ncellSize = (" << globalFeatureParams.cellSize[0] << ", " << globalFeatureParams.cellSize[1] << ")" <<
			"\nvocabularySize = " << globalFeatureParams.vocabularySize << 
			"\nadditionalArtificialTiltedViewAngle = " << globalFeatureParams.additionalArtificialTiltedViewAngle.size() << "times,  {" << (globalFeatureParams.additionalArtificialTiltedViewAngle.size()>0 ? globalFeatureParams.additionalArtificialTiltedViewAngle[0] : 0) << ", " << (globalFeatureParams.additionalArtificialTiltedViewAngle.size()>1 ? globalFeatureParams.additionalArtificialTiltedViewAngle[1] : 0) << "}" << 
			"\nfactorSamplesTrainData = " << factorSamplesTrainData <<
			"\nthinningFactor = " << globalFeatureParams.thinningFactor << 
			"\nuseFeature['bow'] = " << globalFeatureParams.useFeature["bow"] <<
			"\nuseFeature['sap'] = " << globalFeatureParams.useFeature["sap"] <<
			"\nuseFeature['sap2'] = " << globalFeatureParams.useFeature["sap2"] <<
			"\nuseFeature['pointdistribution'] = " << globalFeatureParams.useFeature["pointdistribution"] <<
			"\nuseFeature['normalstatistics'] = " << globalFeatureParams.useFeature["normalstatistics"] <<
			"\nuseFeature['vfh'] = " << globalFeatureParams.useFeature["vfh"] <<
			"\nuseFeature['grsd'] = " << globalFeatureParams.useFeature["grsd"] <<
			"\nuseFeature['gfpfh'] = " << globalFeatureParams.useFeature["gfpfh"] <<
			"\nuseFullPCAPoseNormalization = " << globalFeatureParams.useFullPCAPoseNormalization <<
			"\nuseRollPoseNormalization = " << globalFeatureParams.useRollPoseNormalization <<
			"\nviewsPerObject (-1 = all) = " << viewsPerObject <<
			"\nclusterMode = " << clusterMode <<
			"\nclassifierType = " << classifierType <<
			"\ncrossValidationFold = " << crossValidationFold <<
			"\nfactorNegativeSet = " << factorNegativeSet <<
			"\npercentTest = " << percentTest <<
			"\npercentValidation = " << percentValidation <<
			"\nuseSloppyMasks = " << useSloppyMasks << std::endl;

		std::cout << parameters.str() << std::endl;

		std::string baseFolder = "common/files/";
		std::string databasePath;
		std::string localFeatureFileName = baseFolder + "IPA2Data/IPA2_Surf64Dev2_loc.txt"; //"IPA2Data/IPA2_FPFH_loc.txt"; //"IPA2Data/IPA2_RSD_loc.txt"; //"WashingtonData/Wa_Surf64Dev2_loc.txt";//"IPA2Data/IPA2_Surf64Dev2_loc.txt";	//"IPAData/IPA_Surf64Dev2_loc.txt";
		if (useSloppyMasks == true)
			databasePath = "F:/ObjectDataNew/TrainingData/";
		else
		{
			databasePath = "F:/ObjectDataNew_goodmasks/TrainingData/";
			localFeatureFileName = baseFolder + "IPA2Data/IPA2_Surf64Dev2_loc_goodmask.txt";
		}
		//std::string screenOutputLogFileName = baseFolder + "WashingtonData/Wa_Surf64Dev2_PCA3CF7-7-2_screen_log.txt";
		std::string screenOutputLogFileName = baseFolder + "IPA2Data/IPA2_Surf64Dev2_PCA3CF7-7-2_screen_log.txt";
		//std::string screenOutputLogFileName = baseFolder + "IPA2Data/IPA2_RSD_GRSD_screen_log.txt";
		//std::string screenOutputLogFileName = baseFolder + "IPA2Data/IPA2_FPFH_GFPFH_screen_log.txt";
		//std::string screenOutputLogFileName = baseFolder + "IPAData/IPA_Surf64Dev2_PCA3CF7-7-2_screen_log.txt";
		std::ofstream screenLogFile(screenOutputLogFileName.c_str(), std::ios::app);
		if (!screenLogFile.is_open())
		{
			std::cout << "Error: main: The log file could not be opened." << std::endl;
			return 0;
		}
		// screenLogFile << "thin point cloud: 10% of original points used.\n";
		screenLogFile << comments << parameters.str() << std::endl;

		srand(1);

		//std::string annotationFileName = baseFolder + "object_lists/annotation_washington_test.txt";
		//std::string globalFeatureFileName = baseFolder + "WashingtonData/Wa_Surf64Dev2_PCA3CF7-7-2_glob.txt";
		//std::string covarianceMatrixFileName = baseFolder + "WashingtonData/Wa_Surf64Dev2_loc_covar";
		//std::string localFeatureClustererPath = baseFolder + "WashingtonData/Classifier";
		//std::string timingLogFileName = baseFolder + "WashingtonData/Wa_Surf64Dev2_PCA3CF7-7-2_timing.txt";
		//OC.LoadWashingtonDatabase(annotationFileName, "G:/Washington3dObjectsDataset/segmented/", 1, clusterMode, globalFeatureParams, localFeatureFileName, globalFeatureFileName,
		//					covarianceMatrixFileName, localFeatureClustererPath, timingLogFileName, screenLogFile, MASK_LOAD);

		std::string annotationFileName = baseFolder + "object_lists/annotation_cin2.txt";
		std::string globalFeatureFileName = baseFolder + "IPA2Data/IPA2_Surf64Dev2_PCA3CF7-7-2_glob.txt";
		std::string covarianceMatrixFileName = baseFolder + "IPA2Data/IPA2_Surf64Dev2_loc_covar";
		std::string localFeatureClustererPath = baseFolder + "IPA2Data/Classifier";
		std::string timingLogFileName = baseFolder + "IPA2Data/IPA2_Surf64Dev2_PCA3CF7-7-2_timing.txt";
		OC.LoadCIN2Database(annotationFileName, databasePath, 1, clusterMode, globalFeatureParams, localFeatureParams, localFeatureFileName, globalFeatureFileName,
							covarianceMatrixFileName, localFeatureClustererPath, timingLogFileName, screenLogFile, MASK_LOAD);
		//OC.LoadCIN2Database("common/files/object_lists/annotation_cin2.txt", databasePath, 2, clusterMode, globalFeatureParams, localFeatureParams, localFeatureFileName, "IPA2Data/IPA2_RSD_GRSD_glob.txt",
		//					"IPA2Data/IPA2_RSD_loc_covar", "IPA2Data/Classifier", "IPA2Data/IPA2_RSD_GRSD_timing.txt", screenLogFile, MASK_LOAD);
		//OC.LoadCIN2Database("common/files/object_lists/annotation_cin2.txt", databasePath, 2, clusterMode, globalFeatureParams, localFeatureParams, localFeatureFileName, "IPA2Data/IPA2_FPFH_GFPFH_glob.txt",
		//	"IPA2Data/IPA2_FPFH_loc_covar", "IPA2Data/Classifier", "IPA2Data/IPA2_FPFH_GFPFH_timing.txt", screenLogFile, MASK_LOAD);

		//std::string annotationFileName = baseFolder + "object_lists/annotation_cin.txt";
		//std::string globalFeatureFileName = baseFolder + "IPAData/IPA_Surf64Dev2_EM250_glob.txt"; //"IPAData/IPA_Surf64Dev2_PCA3CF7-7-2_glob.txt";
		//std::string covarianceMatrixFileName = baseFolder + "IPAData/IPA_Surf64Dev2_loc_covar";
		//std::string localFeatureClustererPath = baseFolder + "IPAData/Classifier";
		//std::string timingLogFileName = baseFolder + "IPAData/IPA_Surf64Dev2_EM250_timing.txt"; //"IPAData/IPA_Surf64Dev2_PCA3CF7-7-2_timing.txt";
		//OC.LoadCINDatabase(annotationFileName, "../../Datenbanken/IPA/", 2, clusterMode, globalFeatureParams, localFeatureFileName, globalFeatureFileName,
		//	covarianceMatrixFileName, localFeatureClustererPath, timingLogFileName, MASK_LOAD);

		//// descriptor repeatability
		//std::cout << "\n\noffset\tintra\ttrans\textra\n";
		//for (int i=0; i<19; i++)
		//{
		//	double intraDist = 0.0, transDist = 0.0, extraDist = 0.0;
		//	OC.GetDataPointer()->analyzeGlobalFeatureRepeatability(i, intraDist, transDist, extraDist);
		//	std::cout << i << "\t" << intraDist << "\t" << transDist << "\t" << extraDist << "\n";
		//	screenLogFile << i << "\t" << intraDist << "\t" << transDist << "\t" << extraDist << "\n";
		//}
		//return 0;

		srand(1);
		
		std::string statisticsPath = baseFolder + "IPA2Data/Classifier/Statistics/";	//"WashingtonData/Classifier/Statistics/";	//"IPA2Data/Classifier/Statistics/"         //"IPAData/Classifier/Statistics/"
		std::string configurationPrefix = "IPA2_Surf64Dev2_PCA3CF7-7-2"; //"IPA_Surf64Dev2_EM250";	//"IPA2_Surf64Dev2_PCA3CF7-7-2";	// "IPA2_FPFH_GFPFH" //"IPA2_RSD_GRSD"	//"IPA_Surf64Dev2_vfh"
		//OC.CrossValidationGlobalSampleRange(statisticsPath, configurationPrefix, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile, factorSamplesTrainData);
		OC.CrossValidationGlobal(statisticsPath, configurationPrefix, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile);
		std::string localClassifierSavePath = localFeatureClustererPath + "/";
		OC.GetDataPointer()->SaveGlobalClassifiers(localClassifierSavePath, classifierType);
		//OC.GetDataPointer()->LoadGlobalClassifiers(localClassifierSavePath, CLASSIFIER_RTC);
		//OC.CrossValidationGlobalMultiClassSampleRange(statisticsPath, configurationPrefix, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile, factorSamplesTrainData);
		OC.CrossValidationGlobalMultiClass(statisticsPath, configurationPrefix, classifierType, crossValidationFold, factorNegativeSet, percentTest, percentValidation, viewsPerObject, &screenLogFile);

		screenLogFile.close();

		return 0;
	}

	/// 1st step
	/// Extract CIN Database's feature point information, create feature point files and create masks
	/// This is the first step to take when starting with everything. The feature points are extracted and saved to file for later use.
	//OC.LoadCINDatabase("annotation_cin.txt", "../../Datenbanken/IPA/", 0, CLUSTER_EM, "IPAData/IPA_Surf64Dev2_loc.txt", "IPAData/IPA_Surf64Dev2_EM184PCA3CF12FS6_glob.txt", "IPAData/IPA_Surf64Dev2_loc_covar", "IPAData/Classifier", MASK_SAVE);
	//OC.LoadCINDatabase("annotation_cin.txt", "../../Datenbanken/IPA/", 1, CLUSTER_EM, "IPAData/IPA_Surf64Dev2_loc.txt", "IPAData/IPA_Surf64Dev2_EMxxxPCA3CF12FS6_glob.txt", "IPAData/IPA_Surf64Dev2_test_loc_covar", "IPAData/Classifier", MASK_LOAD);

	/// 2nd step
	/// Load CIN Database's feature data from file (when files already exist)
	/// When the feature point data is available from file, use this file to load the feature point data.
	//OC.LoadCINDatabase("annotation_cin.txt", "../../Datenbanken/IPA/", 1, CLUSTER_EM, "IPAData/IPA_Surf64Dev2_loc.txt", "IPAData/IPA_Surf64Dev2_EM046PCA3CF30FS6_glob.txt", "IPAData/IPA_Surf64Dev2_loc_covar", "IPAData/Classifier", MASK_LOAD);
//	OC.LoadCINDatabase("annotation_cin.txt", "../../Datenbanken/IPA/", 1, CLUSTER_EM, "IPAData/IPA_Surf64Dev2_loc.txt", "IPAData/IPA_Surf64Dev2_PCA3CF6-6-2_nonorm_glob.txt", "IPAData/IPA_Surf64Dev2_loc_covar", "IPAData/Classifier", MASK_LOAD);

	// descriptor repeatability
	//std::cout << "\n\noffset\tintra\ttrans\textra\n";
	//for (int i=0; i<60; i++)
	//{
	//	double intraDist = 0.0, transDist = 0.0, extraDist = 0.0;
	//	OC.GetDataPointer()->analyzeGlobalFeatureRepeatability(i, intraDist, transDist, extraDist);
	//	std::cout << i << "\t" << intraDist << "\t" << transDist << "\t" << extraDist << "\n";
	//}
	//return 0;

	/// Do the cross-validation statistics, the second parameter is part of the output file's name which can be displayed with a Matlab script.
	//OC.CrossValidationGlobal("IPAData/Classifier/Statistics/", "IPA_Surf64Dev2_EM046PCA3CF30FS6", CLASSIFIER_RTC, 10, 5.0f, 0.0f, 0.1f);
	OC.CrossValidationGlobal("IPAData/Classifier/Statistics/", "IPA_Surf64Dev2_PCA3CF6-6-2_nonorm", CLASSIFIER_RTC, 10, 5.0f, 0.0f, 0.1f);
	OC.GetDataPointer()->SaveGlobalClassifiers("IPAData/Classifier/", CLASSIFIER_RTC);
//	OC.GetDataPointer()->LoadGlobalClassifiers("IPAData/Classifier/", CLASSIFIER_RTC);
	//OC.CrossValidationGlobalMultiClass("IPAData/Classifier/Statistics/", "IPA_Surf64Dev2_EM046PCA3CF30FS6", CLASSIFIER_RTC, 10, 5.0f, 0.0f, 0.1f);
	OC.CrossValidationGlobalMultiClass("IPAData/Classifier/Statistics/", "IPA_Surf64Dev2_PCA3CF6-6-2_nonorm", CLASSIFIER_RTC, 10, 5.0f, 0.0f, 0.1f);

	/// ------------------ End: Functions for Jens ------------------


	/// Experimental
	//OC.LoadCINDatabase("annotation_cin.txt", 2, CLUSTER_EM, "IPAData/IPA_Surf64Dev2_loc.txt", "IPAData/IPA_Surf64Dev2_EM184PCA3CF12FS6_glob.txt", "IPAData/IPA_Surf64Dev2_loc_covar", "IPAData/Classifier");
	//OC.LoadCINDatabase("annotation_cin.txt", 1, CLUSTER_EM, "IPAData/IPA_Surf64Dev2_loc.txt", "IPAData/IPA_Surf64Dev2_Test_glob.txt", "IPAData/IPA_Surf64Dev2_loc_covar", "IPAData/Classifier");
	//OC.LoadALOIDatabase("aloi_annotation_min15occ.txt", 2, CLUSTER_EM, "ALOI4Data/ALOI4FP_min15occ_loc.txt", "ALOI4Data/ALOI4FP_min15occ_glob.txt", "ALOI4Data/ALOI4FP_min15occ_loc_covar.txt", "ALOI4Data/Classifier/");
	//OC.LoadALOIDatabase("aloi_annotation_short.txt", 1, CLUSTER_EM, "ALOI4Data/ALOI4FP_short_loc.txt", "ALOI4Data/ALOI4FP_short_glob.txt", "ALOI4Data/ALOI4FP_short_loc_covar.txt", "ALOI4Data/Classifier/");
	//OC.SaveFPDataLocal("ALOI4Data/ALOI4FP_loc.txt");
	//OC.SaveFPDataGlobal("ALOI4Data/ALOI4FP_glob.txt");
	//OC.LoadFPDataLocal("ALOI4Data/ALOI4FP_min15occ_loc.txt");
//	OC.LoadFPDataGlobal("ALOI4Data/ALOI4FP_min15occ_glob.txt");
	//int NumberCorrectSamples=0;
	//CvMat* FeatMat=NULL;
	//OC.GetDataPointer()->GetLocalFeatureMatrix("apricot", &FeatMat, 0.5, 0.5, NumberCorrectSamples);
	//OC.ExtractGlobalFeatures(&(OC.GetDataPointer()->mLocalFeaturesMap["bell"].begin()->BlobFPs), FeatMat);
	//OC.Train("ALOI4Data/Classifier/", 0.8, 2.0, CLASSIFIER_RTC);
/*	OC.LoadClassifiersGlobal("ALOI4Data/Classifier/", CLASSIFIER_RTC);
	srand(1);
	OC.GetDataPointer()->GetGlobalFeatureMatrix("shell", &FeatMat, (float)2.0/68.0, (float)1.0, NumberCorrectSamples);
	CvMat* SampleMat = cvCreateMat(1, FeatMat->cols, FeatMat->type);
	cvGetRow(FeatMat, SampleMat, 1);
	for (int j=0; j<SampleMat->cols; j++) std::cout << cvGetReal2D(SampleMat, 0, j) << "\t";
	std::cout << "\n";
	double PredictionResult = 0.0;
	OC.Predict("shell", SampleMat, PredictionResult, CLASSIFIER_RTC);
*/
	//double RP, RN;
	//OC.ClassifierStatistics("ALOI4Data/Classifier/", CLASSIFIER_RTC, 0.3, RP, RN);

	//OC.ROCCurve("ALOI4Data/Classifier/", CLASSIFIER_RTC);

	//OC.ClusterLocalFeatures("ALOI4Data/ALOI4FP_loc_covar.txt", "ALOI4Data/Classifier/", 15, 15, 15, 1);
	
	//OC.CrossValidation("ALOI4Data/Classifier/Statistics/", "128BitEM50", CLASSIFIER_RTC, 8, 2.0, 0.2, 0.1);

	// H -Histogram, P - PCA
//	OC.LoadFPDataGlobal("IPAData/IPA_Surf64Dev2_EM184PCA3CF12FS6_glob.txt");
//	OC.CrossValidationGlobal("IPAData/Classifier/Statistics/", "IPA_Surf64Dev2_EM184PCA3CF12FS6", CLASSIFIER_RTC, 8, 5.0, 0.0, 0.1);
//	OC.TrainGlobal("IPAData/Classifier/", 1.0, 5.0, CLASSIFIER_RTC);

//	OC.LoadFPDataLocal("IPAData/IPA_Surf64Dev2_loc.txt");
//	OC.CrossValidationLocal("IPAData/Classifier/Statistics/", "IPA_Surf64Dev2", CLASSIFIER_RTC, 8, 1.0, 0.2, 0.1);


	// Train mode
//	OC.LoadFPDataLocal("IPAData/IPA_Surf64Dev2_loc.txt");
//	OC.TrainLocal("IPAData/Classifier/", 1.0, 2.0, CLASSIFIER_RTC);

	// Run mode
/*	OC.LoadClassifiersLocal("IPAData/Classifier/", CLASSIFIER_RTC);
	OC.LoadClassifiersGlobal("IPAData/Classifier/", CLASSIFIER_RTC);
	
	OC.LoadFPDataLocal("IPAData/IPA_Surf64Dev2_loc.txt");
	std::cout << "Clusterer start\n";
	OC.LoadLocalFeatureClusterer("IPAData/Classifier/EMClusterer184.txt");
	std::cout << "Clusterer end\n";
	SharedImage* si = new SharedImage;

	char input;
	std::cout << "1. New object\nq. Quit\n";
	std::cin >> input;
	while(input != 'q')
	{
		std::string Path = "../../Datenbanken/IPA/", FileName;
		std::cout << "file: ";
		std::cin >> FileName;
		si->LoadSharedImage((Path+FileName).c_str());
		std::string Class;
		std::cout << "class: ";
		std::cin >> Class;
		OC.FindObject(si, Class.c_str(), CLUSTER_EM, CLASSIFIER_RTC, CLASSIFIER_RTC);

		std::cout << "1. New object\nq. Quit\n";
		std::cin >> input;
	}
*/
	//getchar();

	return 0;
}