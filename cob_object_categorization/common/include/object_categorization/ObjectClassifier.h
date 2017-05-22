/// @file ObjectClassifier.h
/// Framework for object categorization tasks.
/// @author Richard Bormann
/// @date August 2009.


#define RUNTIME_TEST_SAP 1		// if set to 1 then all unnecessary code, i.e. code not used for SAP, will be removed from the feature extraction


#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <sstream>
#include <map>
#include <vector>
#include <fstream>

#include "object_categorization/BlobList.h"
#include "object_categorization/DetectorCore.h"
#include "object_categorization/GlobalDefines.h"
#include "object_categorization/StopWatch.h"

#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>

typedef BlobList BlobListRiB;
typedef BlobFeature BlobFeatureRiB;

/// Type for selecting the classifier in operation mode.
/// CLASSIFIER_RTC means you like to use the random trees classifier.
typedef int ClassifierType;
enum {CLASSIFIER_SVM, CLASSIFIER_RTC, CLASSIFIER_BOOST, CLASSIFIER_KNN};

/// Type defining several cluster methods specifiers.
/// CLUSTER_8BIT is a very simple cluster mode which actually only makes a histogram out of SIFT features (with 8 bit descriptor) of a color image (without 3D information).
/// CLUSTER_EM is a variable concept which can cluster feature points from any descriptor with a variable cluster number via expectation maximization clustering. The optimal cluster number is determined by K-Means before.
typedef int ClusterMode;
enum {CLUSTER_8BIT, CLUSTER_EM};

/// Defines the possibilities of using a mask during local feature extraction.
/// MASK_NO means that no mask is used for local feature extraction (i.e. the whole image is searched for features).
/// MASK_LOAD uses a ready-to-use mask loaded from file for this image.
/// MASK_SAVE creates one mask per image which marks the object in the image. The mask is saved afterwards into a file.
typedef int MaskMode;
enum {MASK_NO, MASK_LOAD, MASK_SAVE};


enum Database {INVALID, CIN, CIN2, ALOI, WASHINGTON};

/// A simple function returning the correct shortened form for the used classifier.
/// Used for convenience when labelling files.
/// @param pClassifierType The type of the used classifier.
/// @return Returns an abbreviation for the used classifier.
std::string ClassifierLabel(ClassifierType pClassifierType);

/// Convenience function for simple row by row matrix output into a string.
/// @param pMatrix The matrix which shall be converted into a string.
/// @return The matrix written to a string.
std::string MatToString(CvMat* pMatrix);

/// Convenience funtion which converts the string data of an input stream into a matrix filled row by row.
/// @param pFile Input file stream with the matrix data. Format: Numbers one after another which are written row by row into the matrix.
/// @param pMatrix Destination matrix for the data.
/// @return Return code.
int IfstreamToMat(std::ifstream& pFile, CvMat* pMatrix);

/// The well-known signum function.
/// @param pNumber The number whose sign is searched.
/// @return The sign of pNumber (+1 or -1) or 0 if pNumber==0.
double sign(double pNumber);

/// Structure which concentrates important variables for the statistics of classifier performance.
struct ClassifierPerformanceStruct
{
	int RightPositives;			///< Positive sample, positive classification
	int WrongPositives;			///< Negative sample, positive classification
	int RightNegatives;			///< Negative sample, negative classification
	int WrongNegatives;			///< Positive sample, negative classification

	void clear()
	{
		this->RightPositives = 0;
		this->WrongPositives = 0;
		this->RightNegatives = 0;
		this->WrongNegatives = 0;
	}
};

/// Structure which stores a classifier's responses to multiple queries with data samples
struct ClassifierOutputCollection
{
	std::vector<double> positiveSampleResponses;	///< classifier output for positive class samples
	std::vector<double> negativeSampleResponses;	///< classifier output for negative class samples
	std::vector<std::string> negativeSampleCorrectLabel;	///< correct labels corresponding to the outputs in negativeSampleResponses
	void clear()
	{
		this->positiveSampleResponses.clear();
		this->negativeSampleResponses.clear();
		this->negativeSampleCorrectLabel.clear();
	}
};

/// Saves the relations between certain objects and their categories.
struct ObjectStruct
{
	int ObjectNum;						///< Object number in the database
//	std::string Name;
	std::string CategoryName;			///< Category of the object
	std::string GeneralCategoryName;	///< More general category of the object (perhaps for future use)
};

/// Expands the ordinary BlobList by the name of the file from which the blob features are drawn.
struct BlobListStruct
{
	std::string FileName;		///< Origin of feature point data
	BlobListRiB BlobFPs;		///< List of detected feature points in image "FileName"
};

/// Vector of BlobListStruct.
typedef std::vector<BlobListStruct> BlobListStructVector;
/// Map which saves a vector of the respective BlobListStructs (from different views) for each object (ObjectNumber, BlobListStructs).
typedef std::map<int, BlobListStructVector> ObjectMap;			//typedef std::map<int, std::vector<BlobListStruct>> ObjectMap;
/// Map for the assignment of local feature lists of objects to their class (ClassName, Objects with their BlobLists).
typedef std::map<std::string, ObjectMap> LocalFeaturesMap;
/// Map which saves a feature matrix for numbered objects.
typedef std::map<int, CvMat*> ObjectNrFeatureMap;
/// Map which stores the global feature matrices for each object of each class (ClassName, Map of Objects with their feature matrices).
typedef std::map<std::string, ObjectNrFeatureMap> GlobalFeaturesMap;			//typedef std::map<std::string, std::map<int, CvMat*>> GlobalFeaturesMap;
/// Map which stores a local feature classifier for each class (ClassName, local classifier).
typedef std::map<std::string, CvStatModel*> LocalClassifierMap;
/// Map which stores a global feature classifier for each class (ClassName, global classifier).
typedef std::map<std::string, CvStatModel*> GlobalClassifierMap;
/// Map for the (temporary) storage of the classifier performance statistics for each class' classifier (ClassName, (Threshold, Statistics)).
typedef std::map< std::string, std::map<float, ClassifierPerformanceStruct> > StatisticsMap;
/// Map for the storage of the classifier output for each class
typedef std::map<std::string, float> ClassifierThresholdMap;
/// Map that saves the reliability of binary classifiers, e.g. the probability ClassifierAccuracy[k][i] stands for the probability p(o_k|c_i) with o_k=classifier k outputs a hit, c_i=ground truth class of the data is i
typedef std::map<std::string, std::map<std::string, double> > ClassifierAccuracy;


/// Large data container which holds all relevant data for the categorization task.
class ClassificationData
{
public:
	ClassificationData();
	~ClassificationData();

//	---------- Load/save functions ----------

	/// Saves the local feature point data (<code>mLocalFeaturesMap</code>) to file.
	/// @param pFileName The file (and path) name for local feature data storage.
	/// @return Return code.
	int SaveLocalFeatures(std::string pFileName);
	/// Loads the local feature point data (<code>mLocalFeaturesMap</code>) from file.
	/// @param pFileName The file (and path) name for local feature data storage.
	/// @return Return code.
	int LoadLocalFeatures(std::string pFileName);
	/// Saves the global feature point data (<code>mGlobalFeaturesMap</code>) to file.
	/// @param pFileName The file (and path) name for global feature data storage.
	/// @return Return code.
	int SaveGlobalFeatures(std::string pFileName);
	/// Loads the global feature point data (<code>mGlobalFeaturesMap</code>) from file.
	/// @param pFileName The file (and path) name for global feature data storage.
	/// @return Return code.
	int LoadGlobalFeatures(std::string pFileName);
	/// Saves the global classifier models (<code>mGlobalClassifierMap</code>) to files.
	/// There is one general file (class names, thresholds) and furthermore one model file for each classifier.
	/// @param pPath The path where the files shall be stored.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int SaveGlobalClassifiers(std::string pPath, ClassifierType pClassifierType);
	/// Loads the global classifier models (<code>mGlobalClassifierMap</code>) from files.
	/// There is one general file (class names, thresholds) and furthermore one model file for each classifier.
	/// @param pPath The path in which the model files can be found.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int LoadGlobalClassifiers(std::string pPath, ClassifierType pClassifierType);
	/// Saves the local classifier models (<code>mLocalClassifierMap</code>) to files.
	/// There is one general file (class names, thresholds) and furthermore one model file for each classifier.
	/// @param pPath The path where the files shall be stored.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int SaveLocalClassifiers(std::string pPath, ClassifierType pClassifierType);
	/// Loads the local classifier models (<code>mLocalClassifierMap</code>) from files.
	/// There is one general file (class names, thresholds) and furthermore one model file for each classifier.
	/// @param pPath The path in which the model files can be found.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int LoadLocalClassifiers(std::string pPath, ClassifierType pClassifierType);
	/// Saves the squareroot of the inverse covariance matrix of the local feature data (<code>mSqrtInverseCovarianceMatrix</code>) to file.
	/// @param pFileName The file (and path) name for the covariance matrix storage.
	/// @return Return code.
	int SaveSqrtInverseCovarianceMatrix(std::string pFileName);
	/// Loads the squareroot of the inverse covariance matrix of the local feature data (<code>mSqrtInverseCovarianceMatrix</code>) from file.
	/// @param pFileName The file (and path) name for the covariance matrix storage.
	/// @return Return code.
	int LoadSqrtInverseCovarianceMatrix(std::string pFileName);
	/// Saves the EM cluster model of the local feature data (<code>mLocalFeatureClusterer</code>) to file.
	/// ATTENTION: The function <code>save()</code> for the CvEM cluster class was not implemented yet. This save function can only save all matrices of the CvEM model.
	/// @param pFileName The file (and path) name for the EM cluster model storage.
	/// @return Return code.
	int SaveLocalFeatureClusterer(std::string pFileName);
	/// Loads the EM cluster model of the local feature data (<code>mLocalFeatureClusterer</code>) from file.
	/// ATTENTION: The function <code>load()</code> for the CvEM cluster class was not implemented yet. Since there is no way to write back the matrices into the
	/// CvEM cluster model directly, this load function initializes CvEMParams with all matrices only and runs the normal train mode. Therefore, the <b>local feature
	/// data</b> needs to be <b>loaded before</b> loading the cluster model!
	/// @param pFileName The file (and path) name for the EM cluster model storage.
	/// @return Return code.
	int LoadLocalFeatureClusterer(std::string pFileName);


	/// Analyzes similarities of the descriptors
	/// @param pImageOffset Counts the number of images of offset between both descriptors.
	/// @param pIntraclassMeanDistance Returns the mean descriptor distance at the given offset within class samples in this parameter.
	/// @param pTransclassMeanDistance Returns the mean descriptor distance at the given offset within class samples from other object instances of the current class in this parameter.
	/// @param pExtraclassMeanDistance Returns the mean descriptor distance to samples from other classes in this parameter.
	/// @return Return code.
	int analyzeGlobalFeatureRepeatability(int pImageOffset, double& pIntraclassMeanDistance, double& pTransclassMeanDistance, double& pExtraclassMeanDistance);


//	---------- Local/global feature handling ----------

	LocalFeaturesMap mLocalFeaturesMap;		///< Map for the assignment of local feature lists of objects to their class (ClassLabel, ObjectsMaps with BlobLists with file info).
	
	GlobalFeaturesMap mGlobalFeaturesMap;	///< Map which stores the global feature matrices for each object of each class (ClassName, Map of Objects with their feature matrices (files x feature vector dimension)&gt).

	/// Get the matrix of local features for a certain class.
	/// This function composes a matrix with local feature samples of a certain class (in a given percentage of all class' local features) and
	/// further non-class local features (in a given multiple of the class' local features).
	/// @param pClass The class of interest from which the positive samples are drawn.
	/// @param pFeatureMatrix This matrix is filled with the desired composition of local features (first positive, following negative class' samples).
	/// @param pFactorCorrect Is the part (0..1] of the whole amout of the given class' local features which shall occur in <code>pFeatureMatrix</code>.
	/// @param pFactorIncorrect In <code>pFeatureMatrix</code> other class' local features will occur <code>pFactorIncorrect</code> times (0..inf) of the correct class.
	/// @param pNumberCorrectSamples Is used to return the exact number of positive class samples in <code>pFeatureMatrix</code>.
	/// @return Return code.
	int GetLocalFeatureMatrix(std::string pClass, CvMat** pFeatureMatrix, float pFactorCorrect, float pFactorIncorrect, int& pNumberCorrectSamples);

	/// Get a matrix with a given number of negative (non-class) local feature point samples drawn by chance from <code>mLocalFeaturesMap</code>.
	/// @param pClass The positive class from which no samples are drawn.
	/// @param pNegativeSamplesMatrix The destination matrix for the negative samples.
	/// @param pLowerBound Start index from which on negative samples are written into <code>pNegativeSamplesMatrix</code>.
	/// @param pUpperBound Negative samples are written into <code>pNegativeSamplesMatrix</code> until index (pUpperBound-1).
	/// @return Return code.
	int GetNegativeSamplesMatrixLocal(std::string pClass, CvMat* pNegativeSamplesMatrix, int pLowerBound, int pUpperBound);
	
	/// Create a matrix of all local feature samples which occur in <code>mLocalFeaturesMap</code>.
	/// @param pNumberSamples The total number of samples can be given to the function if already known.
	/// @param pNumberFeatures The number of features can be given to the function if already known.
	/// @return The matrix which holds all local feature samples which occur in <code>mLocalFeaturesMap</code>.
	CvMat* CreateAllLocalFeatureMatrix(int pNumberSamples=-1, int pNumberFeatures=-1);

	/// Get the matrix of global features for a certain class.
	/// This function composes a matrix with global feature samples of a certain class (in a given percentage of all class' global features) and
	/// further non-class global features (in a given multiple of the class' global features).
	/// @param pClass The class of interest from which the positive samples are drawn.
	/// @param pFeatureMatrix This matrix is filled with the desired composition of global features (first positive, following negative class' samples).
	/// @param pFactorCorrect Is the part (0..1] of the whole amout of the given class' global features which shall occur in <code>pFeatureMatrix</code>.
	/// @param pFactorIncorrect In <code>pFeatureMatrix</code> other class' global features will occur <code>pFactorIncorrect</code> times (0..inf) of the correct class.
	/// @param pNumberCorrectSamples Is used to return the exact number of positive class samples in <code>pFeatureMatrix</code>.
	/// @return Return code.
	int GetGlobalFeatureMatrix(std::string pClass, CvMat** pFeatureMatrix, float pFactorCorrect, float pFactorIncorrect, int& pNumberCorrectSamples);

	/// Get a matrix with a given number of negative (non-class) global feature point samples drawn by chance from <code>mGlobalFeaturesMap</code>.
	/// @param pClass The positive class from which no samples are drawn.
	/// @param pNegativeSamplesMatrix The destination matrix for the negative samples.
	/// @param pLabels The labels corresponding to each row of pNegativeSamplesMatrix.
	/// @param pLowerBound Start index from which on negative samples are written into <code>pNegativeSamplesMatrix</code>.
	/// @param pUpperBound Negative samples are written into <code>pNegativeSamplesMatrix</code> until index (pUpperBound-1).
	/// @param pViewsPerObject Can be used to reduce the number of views which are used for training of each object. By default (-1), all views are used for training.
	/// @return Return code.
	int GetNegativeSamplesMatrixGlobal(std::string pClass, CvMat* pNegativeSamplesMatrix, std::vector<std::string>& pLabels, int pLowerBound, int pUpperBound, int pViewsPerObject=-1);

	// this one just uses a certain range of samples from the global sample matrices 
	int GetNegativeSamplesMatrixGlobalSampleRange(std::string pClass, CvMat* pNegativeSamplesMatrix, std::vector<std::string>& pLabels, int pLowerBound, int pUpperBound, double rangeStartFactor, double rangeEndFactor);


	/// Get the number of local features (feature vector dimension).
	/// @return The number of local features.
	int GetNumberLocalFeatures();

	/// Counts total number of local feature points in <code>mLocalFeaturesMap</code>.
	/// @return The total number of local feature points in <code>mLocalFeaturesMap</code>.
	int GetNumberLocalFeaturePoints();

	/// Counts total number of local feature points of a certain class in <code>mLocalFeaturesMap</code>.
	/// @param pClass The class whose total number of local feature points shall be determined.
	/// @return The total number of local feature points of <code>pClass</code> in <code>mLocalFeaturesMap</code>.
	int GetNumberLocalFeaturePoints(std::string pClass);


//	---------- Local/global classifier handling ----------

	LocalClassifierMap mLocalClassifierMap;		///< Map which stores a local feature classifier for each class (ClassName, local classifier).

	GlobalClassifierMap mGlobalClassifierMap;	///< Map which stores a global feature classifier for each class (ClassName, global classifier).

	ClassifierThresholdMap mGlobalClassifierThresholdMap;	///< Map which stores the optimal classifier treshold, which is used to separate class from non-class prediction results (classifier runs in regression mode), for each class' classifier (ClassName, Threshold).
	ClassifierThresholdMap mLocalClassifierThresholdMap;	///< Map which stores the optimal classifier treshold, which is used to separate class from non-class prediction results (classifier runs in regression mode), for each class' classifier (ClassName, Threshold).

	ClassifierAccuracy mGlobalClassifierAccuracy;	///< /// Map that saves the reliability of global binary classifiers in their best configuration (i.e. with their corresponding optimal threshold from mGlobalClassifierThresholdMap), e.g. the probability ClassifierAccuracy[k][i] stands for the probability p(o_k|c_i) with o_k=classifier k outputs a hit, c_i=ground truth class of the data is i

	CvMat* mSqrtInverseCovarianceMatrix;		///< The squareroot of the inverse covariance matrix of the local feature point data.

#if (CV_MAJOR_VERSION<=2 && CV_MINOR_VERSION<=3)
	CvEM* mLocalFeatureClusterer;		///< Cluster model which performs local feature point clustering for global feature histograms.
#else
	cv::EM* mLocalFeatureClusterer;		///< Cluster model which performs local feature point clustering for global feature histograms.
#endif
	
	StatisticsMap mStatisticsMap;		///< Map for the (temporary) storage of the classifier performance statistics for each class' classifier (ClassName, ClassifierPerformanceStruct).

private:
	/// Some iterators for convenience.
	LocalFeaturesMap::iterator mItLocalFeaturesMap;
	ObjectMap::iterator mItObjectMap;
	GlobalFeaturesMap::iterator mItGlobalFeaturesMap;

};

struct ObjectLocalizationIdentification
{
	cv::Point3d objectCenter;
	cv::Point2i textPosition;
	std::map<std::string, double> identificationPDF;
};

/// Collection of functions und data structures for object categorization tasks. 
class ObjectClassifier
{
public:

	struct GlobalFeatureParams
	{
		std::vector<int> polynomOrder;		// sap: order of the approximating polynomial
		std::vector<int> numberLinesX;		// sap: number approximation cuts parallel to the x-axis  -  if sap is called at multiple levels of polynomOrder, we make use of the vector
		std::vector<int> numberLinesY;		// sap: number approximation cuts parallel to the y-axis
		int pointDataExcess;	// sap: polynomial fitting will not happen with less than PolynomOrder+1+pointDataExcess points
		double cellCount[2];	// pointdistribution: number of cells around the object's center in x and y dimension (of the normalized view if enabled)
		double cellSize[2];		// pointdistribution: size of the grid cells
		int vocabularySize;		// size of clusters for bag-of-words
		std::vector<double> additionalArtificialTiltedViewAngle;		// extracts another descriptor from the object tilted by this angle, if angle != 0, provide angle in degrees
		double thinningFactor;		// reduce the amount of points in the point cloud by randomly removing (1-thinningFactor)*100% of the original points
		unsigned int minNumber3DPixels;	// the 3d descriptor cannot be computed if less points are available
		std::map<std::string, bool> useFeature;	// enables/disables the use of features: useFeature["bow"] = false; 	useFeature["sap"] = true;	useFeature["pointdistribution"] = true;	useFeature["normalstatistics"] = false; useFeature["vfh"] = false;
		bool useFullPCAPoseNormalization;	// normalize the pose before the descriptor is computed
		bool useRollPoseNormalization;	// normalize the rotation around the camera axis before the descriptor is computed
	};

	struct LocalFeatureParams
	{
		std::string useFeature;	// enables/disables the use of features: useFeature["surf"] = false; 	useFeature["rsd"] = true;	useFeature["fpfh"] = true;
	};

	ObjectClassifier() {} ;
	ObjectClassifier(std::string pEMClusterFilename, std::string pGlobalClassifierPath);

	/// Load function for the CIN database.
	/// Loads the local and global features of all objects of the database for further use with cross-validation or classifier training tasks.
	/// That means, first of all, the class labels are loaded from <code>pAnnotationFileName</code>, then the local features are extracted from the images of the database (<code>ExtractLocalFeatures()</code>)
	/// or loaded from file, following the local features are clustered (<code>ClusterLocalFeatures()</code>) and finally, the global features are extracted (<code>ExtractGlobalFeatures()</code>) or loaded from file.
	/// If the local and global feature data shall only be loaded from file, the two functions <code>LoadFPDataLocal()</code> and <code>LoadFPDataGlobal()</code>
	/// are faster than <code>LoadCINDatabase</code> and they allow loading each file exclusively (if you only need one of them).
	/// @param pAnnotationFileName File name (and path if necessary) of the object annotation file which assigns every object a category and a generalized category (file format: object number [whitespace] category [whitespace] general category [newline]).
	/// @param pDatabasePath The path where the source images from the database can be found.
	/// @param pMode Defines whether the data shall be extracted and computed from the images or loaded from file (0 = calculate local and global features, 1 = load local from file, calculate global features, 2 = load local and global features from file, -2 = calculate local, load global features from file).
	/// @param pClusterMode Defines the cluster method to use (compare enum <code>ClusterMode</code>).
	/// @param pLocalFeatureFileName The file name (and path if necessary) where the local features shall be saved to or loaded from.
	/// @param pGlobalFeatureFileName The file name (and path if necessary) where the global features shall be saved to or loaded from.
	/// @param pCovarianceMatrixFileName The file name (and path if necessary) where the local features' covarince matrix shall be saved to (only applicable if the respective code is activated in <code>ClusterLocalFeatures()</code>).
	/// @param pLocalFeatureClustererPath The path where the clustering model shall be saved.
	/// @param pMaskMode Defines whether a mask is used and whether the used mask has to be created or loaded from file. View details at MaskMode.
	/// @return Return code.
	int LoadCINDatabase(std::string pAnnotationFileName, std::string pDatabasePath, int pMode, ClusterMode pClusterMode, GlobalFeatureParams& pGlobalFeatureParams, std::string pLocalFeatureFileName = "Data/IPAFP_loc.txt", std::string pGlobalFeatureFileName = "Data/IPAFP_glob.txt", std::string pCovarianceMatrixFileName = "Data/IPAFP_loc_covar.txt", std::string pLocalFeatureClustererPath = "Data/Classifier/", std::string pTimingLogFileName = "timing.txt", MaskMode pMaskMode = MASK_NO);

	int LoadCIN2Database(std::string pAnnotationFileName, std::string pDatabasePath, int pMode, ClusterMode pClusterMode, GlobalFeatureParams& pGlobalFeatureParams, LocalFeatureParams& pLocalFeatureParams, std::string pLocalFeatureFileName,
						 std::string pGlobalFeatureFileName, std::string pCovarianceMatrixFileName, std::string pLocalFeatureClustererPath, std::string pTimingLogFileName, std::ofstream& pScreenLogFile, MaskMode pMaskMode);

	int LoadWashingtonDatabase(std::string pAnnotationFileName, std::string pDatabasePath, int pMode, ClusterMode pClusterMode, GlobalFeatureParams& pGlobalFeatureParams, std::string pLocalFeatureFileName,
						std::string pGlobalFeatureFileName, std::string pCovarianceMatrixFileName, std::string pLocalFeatureClustererPath, std::string pTimingLogFileName, std::ofstream& pScreenLogFile, MaskMode pMaskMode, bool useIPA3Database=false);

	/// Load function for the ALOI database.
	/// Loads the local and global features of all objects of the database for further use with cross-validation or classifier training tasks.
	/// That means, first of all, the class labels are loaded from <code>pAnnotationFileName</code>, then the local features are extracted from the images of the database (<code>ExtractLocalFeatures()</code>)
	/// or loaded from file, following the local features are clustered (<code>ClusterLocalFeatures()</code>) and finally, the global features are extracted (<code>ExtractGlobalFeatures()</code>) or loaded from file.
	/// If the local and global feature data shall only be loaded from file, the two functions <code>LoadFPDataLocal()</code> and <code>LoadFPDataGlobal()</code>
	/// are faster than <code>LoadCINDatabase</code> and they allow loading each file exclusively (if you only need one of them).
	/// @param pAnnotationFileName File name (and path if necessary) of the object annotation file which assigns every object a category and a generalized category (file format: object number [whitespace] category [whitespace] general category [newline]).
	/// @param pMode Defines whether the data shall be extracted and computed from the images or loaded from file (0 = calculate local and global features, 1 = load local from file, calculate global features, 2 = load local and global features from file, -2 = calculate local, load global features from file).
	/// @param pClusterMode Defines the cluster method to use (compare enum <code>ClusterMode</code>).
	/// @param pLocalFeatureFileName The file name (and path if necessary) where the local features shall be saved to or loaded from.
	/// @param pGlobalFeatureFileName The file name (and path if necessary) where the global features shall be saved to or loaded from.
	/// @param pCovarianceMatrixFileName The file name (and path if necessary) where the local features' covarince matrix shall be saved to (only applicable if the respective code is activated in <code>ClusterLocalFeatures()</code>).
	/// @param pLocalFeatureClustererPath The path where the clustering model shall be saved.
	/// @return Return code.
	int LoadALOIDatabase(std::string pAnnotationFileName, int pMode, ClusterMode pClusterMode, GlobalFeatureParams& pGlobalFeatureParams, std::string pLocalFeatureFileName = "ALOI4Data/ALOI4FP_loc.txt", std::string pGlobalFeatureFileName = "ALOI4Data/ALOI4FP_glob.txt", std::string pCovarianceMatrixFileName = "ALOI4Data/ALOI4FP_loc_covar.txt", std::string pLocalFeatureClustererPath = "ALOI4Data/Classifier/");

	/// Cluster function for local feature clustering (needed as codebook for global bag-of-words features).
	/// This clustering method creates an appropriate number of clusters out of the local feature data which represent the words of a bag-of-words method.
	/// Later, histograms over these words are used for object categorization in <code>ExtractGlobalFeatures()</code>.
	/// At first, the number of necessary clusters is determined by running K-Means several times with increasing cluster numbers and watching the relative decrease of
	/// the mean distance of samples to their cluster centers compared to the run before. When this relative decrease gets below a threshold (<code>pMinRelativeDeviationChange</code>)
	/// then the optimal cluster number is found. Afterwards, an Expectation Maximization model is trainend using diagonal covanriance matrices starting
	/// with the K-Means result.
	/// This method needs the local features to be loaded into <code>mData.mLocalFeaturesMap</code> before.
	/// @param pCovarianceMatrixFileName The file name (and path if necessary) where the local features' covarince matrix shall be saved to (only applicable if the respective code is activated).
	/// @param pLocalFeatureClustererPath The path where the clustering model shall be saved.
	/// @param pNumberClustersMin The lower bound for the number of clusters the algorithm tries to find the optimal number of clusters.
	/// @param pNumberClustersMax The upper bound for the number of clusters the algorithm tries to find the optimal number of clusters.
	/// @param pNumberClustersStep During the search for the optimal number of clusters, the number of clusters (K) is increased by this step size every time.
	/// @param pNumberValidationRuns Parameter for number of K-Means runs at a certain K while searching for the optimal K (should be 1 since the OpenCV implementation does not support different (random) starting clusters at the moment yet).
	/// @param pMinRelativeDeviationChange Stop criterion when searching for optimal cluster number K (when the relative decrease of the mean distance of samples to their cluster centers drops below this threshold from one run with fixed K to the next with a larger K).
	/// @return Return code.
	int ClusterLocalFeatures(std::string pCovarianceMatrixFileName, std::string pLocalFeatureClustererPath, int pNumberClustersMin, int pNumberClustersMax, int pNumberClustersStep, int pNumberValidationRuns, double pMinRelativeDeviationChange=0.01, std::ofstream* pScreenLogFile=0);

	/// Training for all class' local classifiers (using <code>ClassificationData::GetLocalFeatureMatrix()</code> for data set preparation).
	/// This method iterates over all categories and trains one classifier for each class. The trained classifiers are stored in <code>mData.mLocalClassifierMap</code>.
	/// The training data set is set up with <code>ClassificationData::GetLocalFeatureMatrix()</code>. Finally, all classifiers are saved to file.
	/// This method needs the local features to be loaded into <code>mData.mLocalFeaturesMap</code> before.
	/// @param pClassifierSavePath The path where the classifier models shall be stored.
	/// @param pFactorCorrect Is the part (0..1] of the whole amout of the given class' local features which shall occur in the training data matrices (usually set to 1.0).
	/// @param pFactorIncorrect In the training data matrices other class' local features will occur <code>pFactorIncorrect</code> times (0..inf) of the correct class.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int TrainLocal(std::string pClassifierSavePath, float pFactorCorrect, float pFactorIncorrect, ClassifierType pClassifierType);

	/// Single class local classifier training method with previous training matrix construction.
	/// This method trains the local classifier for one category but builds the necessary training data matrix before. The built is done with a list of the 
	/// objects providing positive samples (object numbers in <code>pIndicesTrainCorrect</code> where the objects are found in <code>mData.mLocalFeaturesMap</code>)
	/// and a list of the negative sample's indices (<code>pIndicesTrainIncorrect</code>)
	/// where the negative samples can be found in the matrix <code>pNegativeSamplesMatrix</code>. This method is primarily used by <code>CrossValidationLocal()</code>
	/// and it is rather not commended to be used directly except one needs exactly this funtionality.
	/// This method needs the local features to be loaded into <code>mData.mLocalFeaturesMap</code> before.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClass The class whose classifier shall be trained.
	/// @param pNumberSamples The total number of positive and negative samples which determines the number of rows of the training data matrix.
	/// @param pNumberFeatures The number of local features which determines the number of columns of the training data matrix.
	/// @param pItLocalFeaturesClass Iterator over <code>LocalFeaturesMap</code> at the position of <code>pClass</code>.
	/// @param pIndicesTrainCorrect List containing all indices of the objects of <code>pClass</code> whose local feature samples are intended to be used for training.
	/// @param pIndicesTrainIncorrect List containing all indices of the non-class local feature samples which can be found in <code>pNegativeSamplesMatrix</code> and are intended to be used for training.
	/// @param pNegativeSamplesMatrix A large matrix filled with non-class local feature samples row by row.
	/// @return Return code.
	int TrainLocal(ClassifierType pClassifierType, std::string pClass, int pNumberSamples, int pNumberFeatures, LocalFeaturesMap::iterator pItLocalFeaturesClass, std::list<int>* pIndicesTrainCorrect, std::list<int>* pIndicesTrainIncorrect, CvMat* pNegativeSamplesMatrix);

	/// Single class local classifier training method.
	/// This method trains the local classifier for one category and needs the local features to be loaded into <code>mData.mLocalFeaturesMap</code> before.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClass The class whose classifier shall be trained.
	/// @param pTrainingFeatureMatrix A (number samples x number features) matrix with local feature samples aligned in rows used for training the classifier.
	/// @param pTrainingCorrectResponses A one-dimensional (number samples x 1) vector indicating the correct classifier responses for each local feature sample.
	/// @return Return code.
	int TrainLocal(ClassifierType pClassifierType, std::string pClass, CvMat* pTrainingFeatureMatrix, CvMat* pTrainingCorrectResponses);

	/// Training for all class' global classifiers (using <code>ClassificationData::GetGlobalFeatureMatrix()</code> for data set preparation).
	/// This method iterates over all categories and trains one classifier for each class. The trained classifiers are stored in <code>mData.mGlobalClassifierMap</code>.
	/// The training data set is set up with <code>ClassificationData::GetGlobalFeatureMatrix()</code>. Finally, all classifiers are saved.
	/// This method needs the global features to be loaded into <code>mData.mGlobalFeaturesMap</code> before.
	/// @param pPath The path where the classifier models shall be stored.
	/// @param pFactorCorrect Is the part (0..1] of the whole amout of the given class' global features which shall occur in the training data matrices (usually set to 1.0).
	/// @param pFactorIncorrect In the training data matrices other class' global features will occur <code>pFactorIncorrect</code> times (0..inf) of the correct class.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int TrainGlobal(std::string pPath, float pFactorCorrect, float pFactorIncorrect, ClassifierType pClassifierType);

	/// Single class global classifier training method with previous training matrix construction.
	/// This method trains the global classifier for one category but builds the necessary training data matrix before. The built is done with a list of the 
	/// objects providing positive samples (object numbers in <code>pIndicesTrainCorrect</code> where the objects are found in <code>mData.mGlobalFeaturesMap</code>)
	/// and a list of the negative sample's indices (<code>pIndicesTrainIncorrect</code>)
	/// where the negative samples can be found in the matrix <code>pNegativeSamplesMatrix</code>. This method is primarily used by <code>CrossValidationGlobal()</code>
	/// and it is rather not commended to be used directly except one needs exactly this funtionality.
	/// This method needs the global features to be loaded into <code>mData.mLocalFeaturesMap</code> before.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClass The class whose classifier shall be trained.
	/// @param pNumberSamples The total number of positive and negative samples which determines the number of rows of the training data matrix.
	/// @param pNumberFeatures The number of global features which determines the number of columns of the training data matrix.
	/// @param pItGlobalFeaturesClass Iterator over <code>GlobalFeaturesMap</code> at the position of <code>pClass</code>.
	/// @param pIndicesTrainCorrect List containing all indices of the objects of <code>pClass</code> whose global feature samples are intended to be used for training.
	/// @param pIndicesTrainIncorrect List containing all indices of the non-class global feature samples which can be found in <code>pNegativeSamplesMatrix</code> and are intended to be used for training.
	/// @param pNegativeSamplesMatrix A large matrix filled with non-class global feature samples row by row.
	/// @param pViewsPerObject Can be used to reduce the number of views which are used for training of each object. By default (-1), all views are used for training.
	/// @return Return code.
	int TrainGlobal(ClassifierType pClassifierType, std::string pClass, int pNumberSamples, int pNumberFeatures, GlobalFeaturesMap::iterator pItGlobalFeaturesClass, std::list<int>* pIndicesTrainCorrect, std::list<int>* pIndicesTrainIncorrect, CvMat* pNegativeSamplesMatrix, int pViewsPerObject=-1);
	
	// only uses samples between rangestart and range end
	int TrainGlobalSampleRange(ClassifierType pClassifierType, std::string pClass, int pNumberSamples, int pNumberFeatures, GlobalFeaturesMap::iterator pItGlobalFeaturesClass, std::list<int>* pIndicesTrainCorrect, std::list<int>* pIndicesTrainIncorrect, CvMat* pNegativeSamplesMatrix, double rangeStartFactor, double rangeEndFactor);
	
	/// Single class global classifier training method.
	/// This method trains the global classifier for one category and needs the global features to be loaded into <code>mData.mGlobalFeaturesMap</code> before.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClass The class whose classifier shall be trained.
	/// @param pTrainingFeatureMatrix A (number samples x number features) matrix with global feature samples aligned in rows used for training the classifier.
	/// @param pTrainingCorrectResponses A one-dimensional (number samples x 1) vector indicating the correct classifier responses for each global feature sample.
	/// @return Return code.
	int TrainGlobal(ClassifierType pClassifierType, std::string pClass, CvMat* pTrainingFeatureMatrix, CvMat* pTrainingCorrectResponses);

	/// Class membership prediction, uses loaded predictor from <code>mData.mLocalClassifierMap</code>.
	/// This method accepts one local feature sample (i.e. a feature point) and decides on the basis of a previously trained classifier whether this sample belongs to <code>pClass</code> or not.
	/// The local classifiers must be loaded before.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClass The category to which the samples are questioned to belong to.
	/// @param pFeatureData A one-dimensional matrix (1 x number local features) with the local feature vector.
	/// @param pPredictionResponse The prediction result is written into this variable.
	/// @return Return code.
	int PredictLocal(ClassifierType pClassifierType, std::string pClass, CvMat* pFeatureData, double& pPredictionResponse);

	/// Class membership prediction, uses loaded predictor from <code>mData.mGlobalClassifierMap</code>.
	/// This method accepts one global feature sample (i.e. a global feature vector) and decides on the basis of a previously trained classifier whether this sample belongs to <code>pClass</code> or not.
	/// The global classifiers must be loaded before.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClass The category to which the samples are questioned to belong to.
	/// @param pFeatureData A one-dimensional matrix (1 x number global features) with the global feature vector.
	/// @param pPredictionResponse The prediction result is written into this variable.
	/// @return Return code.
	int PredictGlobal(ClassifierType pClassifierType, std::string pClass, CvMat* pFeatureData, double& pPredictionResponse);

	/// Class membership prediction, loads the predictor from file.
	/// This method accepts one global feature sample (i.e. a global feature vector) and decides on the basis of a previously trained classifier whether this sample belongs to <code>pClass</code> or not. The predictor is loaded previously from file.
	/// Please note: Old code, should not be used.
	/// @param pPath The path where the classifier model file is stored.
	/// @param pClass The category to which the samples are questioned to belong to.
	/// @param pFeatureData A one-dimensional matrix (1 x number global features) with the global feature vector.
	/// @param pPredictionResponse The prediction result is written into this variable.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int PredictGlobal(std::string pPath, std::string pClass, CvMat* pFeatureData, double& pPredictionResponse, ClassifierType pClassifierType);

	/// Validation run during cross-validation.
	/// This function is used during cross-validation for the validation run. The tests are performed with samples drawn from the objects contained in the positive
	/// sample list (<code>pIndicesValidationCorrect</code>) as well as negative samples from <code>pNegativeSamplesMatrix</code> indicated by the negative samples list
	/// (<code>pIndicesValidationIncorrect</code>). The results (positive samples - how many correct or incorrect classified, neagtive samples - how many correct or incorrect
	/// classified) are written into <code>pStatistics</code>.
	/// Assumes, that the local features were loaded and the local classifiers were trained/loaded before.
	/// This function is rather not intended for individual use but for use in the cross-validation framework.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClass The positive samples belong to this category and should be classified as this class.
	/// @param pNumberFeatures The number of local features (i.e. length of the local feature vector).
	/// @param pItLocalFeaturesClass Iterator over <code>mData.mLocalFeaturesMap</code> at the position of <code>pClass</code>.
	/// @param pIndicesValidationCorrect A list of object numbers of <code>pClass</code> whose local feature points will be used for classifier testing.
	/// @param pIndicesValidationIncorrect A list of indices of non-class samples whose respective samples can be found in <code>pNegativeSamplesMatrix</code> which are used for classifier testing.
	/// @param pNegativeSamplesMatrix A large matrix filled with non-class local feature samples row by row.
	/// @param pOutputStorage A structure saving the classifier outputs for the positive and negative samples. Data is appended if the storage is not cleared.
	/// @return Return code.
	int ValidateLocal(ClassifierType pClassifierType, std::string pClass, int pNumberFeatures, LocalFeaturesMap::iterator pItLocalFeaturesClass,
					  std::list<int>* pIndicesValidationCorrect, std::list<int>* pIndicesValidationIncorrect, CvMat* pNegativeSamplesMatrix, ClassifierOutputCollection& pOutputStorage);

	/// Validation run during cross-validation.
	/// This function is used during cross-validation for the validation run. The tests are performed with samples drawn from the objects contained in the positive
	/// sample list (<code>pIndicesValidationCorrect</code>) as well as negative samples from <code>pNegativeSamplesMatrix</code> indicated by the negative samples list
	/// (<code>pIndicesValidationIncorrect</code>). The results (positive samples - how many correct or incorrect classified, neagtive samples - how many correct or incorrect
	/// classified) are written into <code>pStatistics</code>.
	/// Assumes, that the global features were loaded and the global classifiers were trained/loaded before.
	/// This function is rather not intended for individual use but for use in the cross-validation framework.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClass The positive samples belong to this category and should be classified as this class.
	/// @param pNumberFeatures The number of global features (i.e. length of the global feature vector).
	/// @param pItGlobalFeaturesClass Iterator over <code>mData.mGlobalFeaturesMap</code> at the position of <code>pClass</code>.
	/// @param pIndicesValidationCorrect A list of object numbers of <code>pClass</code> whose global features will be used for classifier testing.
	/// @param pIndicesValidationIncorrect A list of indices of non-class samples whose respective samples can be found in <code>pNegativeSamplesMatrix</code> which are used for classifier testing.
	/// @param pNegativeSamplesMatrix A large matrix filled with non-class global feature samples row by row.
	/// @param pNegativeSamplesLabels The corresponding labels to the negative samples in pNegativeSamplesLabels.
	/// @param pOutputStorage A structure saving the classifier outputs for the positive and negative samples. Data is appended if the storage is not cleared.
	/// @return Return code.
	int ValidateGlobal(ClassifierType pClassifierType, std::string pClass, int pNumberFeatures, GlobalFeaturesMap::iterator pItGlobalFeaturesClass,
		std::list<int>* pIndicesValidationCorrect, std::list<int>* pIndicesValidationIncorrect, CvMat* pNegativeSamplesMatrix, std::vector<std::string>& pNegativeSamplesLabels,
		ClassifierOutputCollection& pOutputStorage);
	
	// validate only a range from the test set(?)
	int ValidateGlobalSampleRange(ClassifierType pClassifierType, std::string pClass, int pNumberFeatures, GlobalFeaturesMap::iterator pItGlobalFeaturesClass,
		std::list<int>* pIndicesValidationCorrect, std::list<int>* pIndicesValidationIncorrect, CvMat* pNegativeSamplesMatrix, std::vector<std::string>& pNegativeSamplesLabels,
		ClassifierOutputCollection& pOutputStorage, double rangeStartFactor, double rangeEndFactor);

	/// Perform complete statistical test of the object classifier with all existing training samples (rather outdated code).
	/// This function iterates over all existing global features and determines the categorization result for each class. A complete statistic is built during this process.
	/// Remark: It is recommended to use a cross-validation function since this function performs statistics over unseen as well as trained samples. A good performance
	/// prediction can not be guaranteed by this method.
	/// @param pPath If the classifiers are not loaded already, they are loaded from this path.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClassificationThreshold If the classifier output is smaller than this threshold, then the feature does not belong to <code>pClass</code> otherwise it does.
	/// @param pRightPositives The total number of right positives over all classes is returned with this parameter.
	/// @param pRightNegatives The total number of right negatives over all classes is returned with this parameter.
	/// @param pLoadClassifiers Load the global classifiers from file before testing them if this parameter is true.
	/// @return Return code.
	//int ClassifierStatisticsGlobal(std::string pPath, ClassifierType pClassifierType, float pClassificationThreshold, double& pRightPositives, double& pRightNegatives, bool pLoadClassifiers=true);
	
	/// Creates a ROC curve using <code>ClassifierStatistics()</code> (rather outdated code).
	/// Tries several values for the classification threshold (belongs to class if larger or equal than threshold) and uses the statistics data for creating a ROC curve
	/// which is saved in <code>pPath/Statistics/ROCCurve_[ClassfierType].txt</code>.
	/// @param pPath The classifiers are loaded from this path and the ROCCurve is saved there.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int ROCCurve(std::string pPath, ClassifierType pClassifierType);

	/// Performs a cross-validation test of the local classifer peformance for a single class.
	/// This function devides the training data set of positive class samples into a set of objects used for testing, training and validating. The negative features
	/// set is drawn by chance and divided the same way. The test set is only used for testing the classifier performance on this unseen set of the data at the end, i.e.
	/// the classifier is trained with the remainder of the data and the test set performance helps to draw conclusions about the appropriateness of the size
	/// and distribution of the training data set. Before, the ordinary cross-validation cycles estimate the classifier performance on unseen data by training with the
	/// training data set and testing the classifier with the validation set. For each cycle, the validation set is built by taking some samples by chance from the 
	/// training set and pushing them back into the training set at the end of a cycle.
	/// The local features need to be loaded before.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClass The class whose classifier is intended to be trained.
	/// @param pNumberCycles The number of cross-validation runs (one run means the training and validation of the classifier).
	/// @param pFactorIncorrect In the training/validation/testing data matrices other class' local features will occur <code>pFactorIncorrect</code> times (0..inf) of the correct class.
	/// @param pValidationSetOutput A structure saving the classifier outputs for the positive and negative validation samples. Data is appended if the storage is not cleared.
	/// @param pTestSetOutput A structure saving the classifier outputs for the positive and negative test samples. Data is appended if the storage is not cleared.
	/// @param pPercentTest Determines how many percent of the class' objects should be hold back exclusively for testing (0 < pPercentTest < 1)).
	/// @param pPercentValidation Determines how many percent of the class' objects are used for validation (0 < pPercentValidation < 1).
	/// @param pVariableImportance If <code>pVariableImportance != NULL</code>, the variable importance is calculated based on the training with the training and validation set.
	/// @return Return code.
	int CrossValidationLocal(ClassifierType pClassifierType, std::string pClass, int pNumberCycles, float pFactorIncorrect,
							 ClassifierOutputCollection& pValidationSetOutput, ClassifierOutputCollection& pTestSetOutput,
							 float pPercentTest=0.2, float pPercentValidation=0.1, const CvMat** pVariableImportance=NULL);

	/// Performs a cross-validation test of the local classifer peformance for all classes and creates ROC curves for every class and calculates their variable importance.
	/// For every class, the cross-validation is done several times with increasing detection thresholds such that a ROC curve develops. The ROC data is saved to files
	/// beginning with "ROCCurve" for easy post processing e.g. with MATLAB. Furthermore, the variable importance (for the single classification variables = features)
	/// is calculated for each class and each threshold and saved to a file beginning with "VarImportance".
	/// The local features need to be loaded before.
	/// @param pStatisticsPath The path where all resulting data should be stored.
	/// @param pFileDescription Possibility to write a more detailed file description into this variable which will occur in each file name (so be short).
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pNumberCycles The number of cross-validation runs per class and threshold (one run means the training and validation of the classifier).
	/// @param pFactorIncorrect In the training/validation/testing data matrices other class' local features will occur <code>pFactorIncorrect</code> times (0..inf) of the correct class.
	/// @param pPercentTest Determines how many percent of the class' objects should be hold back exclusively for testing (0 < pPercentTest < 1)).
	/// @param pPercentValidation Determines how many percent of the class' objects are used for validation (0 < pPercentValidation < 1).
	/// @return Return code.
	int CrossValidationLocal(std::string pStatisticsPath, std::string pFileDescription, ClassifierType pClassifierType, int pNumberCycles, float pFactorIncorrect, float pPercentTest=0.2, float pPercentValidation=0.1);

	/// Performs a cross-validation test of the global classifer peformance for a single class.
	/// This function devides the training data set of positive class samples into a set of objects used for testing, training and validating. The negative features
	/// set is drawn by chance and divided the same way. The test set is only used for testing the classifier performance on this unseen set of the data at the end, i.e.
	/// the classifier is trained with the remainder of the data and the test set performance helps to draw conclusions about the appropriateness of the size
	/// and distribution of the training data set. Before, the ordinary cross-validation cycles estimate the classifier performance on unseen data by training with the
	/// training data set and testing the classifier with the validation set. For each cycle, the validation set is built by taking some samples by chance from the 
	/// training set and pushing them back into the training set at the end of a cycle.
	/// The global features need to be loaded before.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClass The class whose classifier is intended to be trained.
	/// @param pNumberCycles The number of cross-validation runs (one run means the training and validation of the classifier).
	/// @param pFactorIncorrect In the training/validation/testing data matrices other class' global features will occur <code>pFactorIncorrect</code> times (0..inf) of the correct class.
	/// @param pValidationSetOutput A structure saving the classifier outputs for the positive and negative validation samples. Data is appended if the storage is not cleared.
	/// @param pTestSetOutput A structure saving the classifier outputs for the positive and negative test samples. Data is appended if the storage is not cleared.
	/// @param pPercentTest Determines how many percent of the class' objects should be hold back exclusively for testing (0 < pPercentTest < 1)).
	/// @param pPercentValidation Determines how many percent of the class' objects are used for validation (0 < pPercentValidation < 1).
	/// @param pVariableImportance If <code>pVariableImportance != NULL</code>, the variable importance is calculated based on the training with the training and validation set and written into <code>pVariableImportance</code>.
	/// @param pViewsPerObject Can be used to reduce the number of views which are used for training of each object. By default (-1), all views are used for training.
	/// @return Return code.
	int CrossValidationGlobal(ClassifierType pClassifierType, std::string pClass, int pNumberCycles, float pFactorIncorrect,
							  ClassifierOutputCollection& pValidationSetOutput, ClassifierOutputCollection& pTestSetOutput,
							  float pPercentTest=0.2, float pPercentValidation=0.1, const CvMat** pVariableImportance=NULL, int pViewsPerObject=-1);

	int CrossValidationGlobalSampleRange(ClassifierType pClassifierType, std::string pClass, int pNumberCycles, float pFactorIncorrect,
		ClassifierOutputCollection& pValidationSetOutput, ClassifierOutputCollection& pTestSetOutput,
		float pPercentTest=0.2, float pPercentValidation=0.1, const CvMat** pVariableImportance=NULL, int pViewsPerObject=-1, double pFactorSamplesTrainData=0.5);

	/// Performs a cross-validation test of the global classifer peformance for all classes and creates ROC curves for every class and calculates their variable importance.
	/// For every class, the cross-validation is done several times with increasing detection thresholds such that a ROC curve develops. The ROC data is saved to files
	/// beginning with "ROCCurve" for easy post processing e.g. with MATLAB. Furthermore, the variable importance (for the single classification variables = features)
	/// is calculated for each class and each threshold and saved to a file beginning with "VarImportance".
	/// The global features need to be loaded before.
	/// @param pStatisticsPath The path where all resulting data should be stored.
	/// @param pFileDescription Possibility to write a more detailed file description into this variable which will occur in each file name (so be short).
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pNumberCycles The number of cross-validation runs per class and threshold (one run means the training and validation of the classifier).
	/// @param pFactorIncorrect In the training/validation/testing data matrices other class' local features will occur <code>pFactorIncorrect</code> times (0..inf) of the correct class.
	/// @param pPercentTest Determines how many percent of the class' objects should be hold back exclusively for testing (0 < pPercentTest < 1)).
	/// @param pPercentValidation Determines how many percent of the class' objects are used for validation (0 < pPercentValidation < 1).
	/// @param pViewsPerObject Can be used to reduce the number of views which are used for training of each object. By default (-1), all views are used for training.
	/// @return Return code.
	int CrossValidationGlobal(std::string pStatisticsPath, std::string pFileDescription, ClassifierType pClassifierType, int pNumberCycles, float pFactorIncorrect, float pPercentTest=0.2, float pPercentValidation=0.1,
								int pViewsPerObject=-1, std::ofstream* pScreenLogFile=0);

	int CrossValidationGlobalSampleRange(std::string pStatisticsPath, std::string pFileDescription, ClassifierType pClassifierType, int pNumberCycles, float pFactorIncorrect, float pPercentTest=0.2, float pPercentValidation=0.1,
		int pViewsPerObject=-1, std::ofstream* pScreenLogFile=0, double pFactorSamplesTrainData=0.5);

	/// Cross-validation with a multi-class classifier.
	/// @param pViewsPerObject Can be used to reduce the number of views which are used for training of each object. By default (-1), all views are used for training.
	int CrossValidationGlobalMultiClass(std::string pStatisticsPath, std::string pFileDescription, ClassifierType pClassifierType, int pFold, float pFactorIncorrect, float pPercentTest, float pPercentValidation,
								int pViewsPerObject=-1, std::ofstream* pScreenLogFile=0);

	int CrossValidationGlobalMultiClassSampleRange(std::string pStatisticsPath, std::string pFileDescription, ClassifierType pClassifierType, int pFold, float pFactorIncorrect, float pPercentTest, float pPercentValidation,
		int pViewsPerObject=-1, std::ofstream* pScreenLogFile=0, double pFactorSamplesTrainData=0.5);


	/// loads the parameters for runtime use
	int LoadParameters(std::string pFilename);

	/// Loop for runtime use
	int CategorizeContinuously(ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams);

	/// Callback for point cloud data from kinect
	void PointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pInputCloud, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams);
	int mImageNumber;

	/// Categorizes an object
	/// @param pResultsOrdered ordered list of results (percentage, class name)
	int CategorizeObject(SharedImage* pSourceImage, std::map<std::string, double>& pResults, std::map<double, std::string>& pResultsOrdered, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams);


	int CaptureSegmentedPCD(ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams);
	void CapturePointcloudCallback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pInputCloud, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams);

	// Hermes
	int HermesLoadCameraCalibration(const std::string& object_name, cv::Mat& projection_matrix);
	int HermesDetectInit(const ClusterMode pClusterMode, const ClassifierType pClassifierTypeGlobal, const GlobalFeatureParams& pGlobalFeatureParams, const std::string& pObjectName);
	void HermesPointcloudCallbackDetect(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pInputCloud, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams);
	int HermesCategorizeObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud, pcl::PointXYZ pAvgPoint, SharedImage* pSourceImage, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams, double& pan, double& tilt, double& roll, Eigen::Matrix4f& pFinalTransform, double& pMatchingScore);
	int HermesCapture(ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams);
	void HermesPointcloudCallbackCapture(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pInputCloud, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams);
	int HermesBuildDetectionModelFromRecordedData(const std::string& object_name, const cv::Mat& projection_matrix, ClusterMode pClusterMode, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams);
	int HermesComputeRollHistogram(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPointCloud, pcl::PointXYZ pAvgPoint, cv::Mat& pHistogram, bool pSmooth=true, bool pDisplay=false);
	int HermesMatchRollHistogram(std::vector<float>& pReferenceHistogram, cv::Mat& pMatchHistogram, int pCoarseStep, int& pOffset, double& pMatchScore);
	double HermesHistogramIntersectionKernel(std::vector<float>& pReferenceHistogram, cv::Mat& pMatchHistogram, int pOffset);
	int HermesMatchPointClouds(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCapturedCloud, pcl::PointXYZ pAvgPoint, double pan, double tilt, double roll, Eigen::Matrix4f& pFinalTransform, double& pMatchingScore);
	bool mFinishCapture;
	double mPanAngle;
	double mTiltAngle;
	unsigned int mFileCounter;
	std::string mFilePrefix;
	std::fstream mLabelFile;
	std::map<double, std::map<double, std::vector<std::vector<float> > > > mSapData;		// [pan][tilt][sample][descriptor index]
	std::map<double, std::map<double, std::vector<std::vector<float> > > > mVfhData;
	std::map<double, std::map<double, std::vector<std::vector<float> > > > mRollHistogram;
	std::map<double, std::map<double, std::string > > mReferenceFilenames;


	/// Decide whether an object of a certain class is visible or not (and where).
	/// Searches for local feature points and classifies them. A certain amount of the best fitting feature points is taken. From this set, one calculates the center of mass
	/// from those feature points with the shortest distance sum to all other feature points. The center of mass is then considered as object center.
	/// From the center of mass, a region growing algorithm should create a mask depicting the object in the picture. The implementation at the moment uses 3D distance
	/// criteria for this task but often creates a wrong mask. Finally, the global feature extraction (with help of local features and object mask) delivers data for the
	/// classification decision.
	/// Needs the local classifiers, the local feature clusterers and the global classifiers to be loaded before.
	/// @param pSourceImage The source image (<code>SharedImage</code>) from which image and 3D data originates.
	/// @param pClass The class whose availability in <code>pSourceImage</code> should be figured out.
	/// @param pClusterMode Defines the cluster method to use (compare enum <code>ClusterMode</code>).
	/// @param pClassifierTypeLocal The type of used local classifier (cf. enum <code>ClassifierType</code>).
	/// @param pClassifierTypeGlobal The type of used global classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int FindObject(SharedImage* pSourceImage, std::string pClass, ClusterMode pClusterMode, ClassifierType pClassifierTypeLocal, ClassifierType pClassifierTypeGlobal, GlobalFeatureParams& pGlobalFeatureParams);

	/// Calculates classifier variable importance.
	/// @param pClassifier The classifier model from which the variable importance should be determined.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @param pVariableImportance Vector with variable importance entries (the higher the more important). Each feature has an importance.
	/// @return Return code.
	int GetVariableImportance(CvStatModel* pClassifier, int pClassifierType, const CvMat** pVariableImportance);

	/// Extracts the local features (SIFT) for a single image which is previously loaded and may be displayed.
	/// Runs with cluster modes <code>CLUSTER_8BIT, CLUSTER_EM</code> since SIFT features have a scaleable descriptor.
	/// Rather out-dated.
	/// @param pFileName The name of the file from which the image is loaded.
	/// @param pBlobFeatures List of found SIFT features.
	/// @param pClusterMode Defines the cluster method to use (compare enum <code>ClusterMode</code>).
	/// @param pVisualization Result is displayed if true.
	/// @return Return code.
	int ExtractLocalFeatures(std::string pFileName, BlobListRiB& pBlobFeatures, ClusterMode pClusterMode, bool pVisualization=false);

	/// Extracts the local features (SIFT) for a single image.
	/// Runs with cluster modes <code>CLUSTER_8BIT, CLUSTER_EM</code> since SIFT features have a scaleable descriptor.
	/// Rather out-dated.
	/// @param pSourceImage The image in which SIFT features are searched.
	/// @param pBlobFeatures List of found SIFT features.
	/// @param pClusterMode Defines the cluster method to use (compare enum <code>ClusterMode</code>).
	/// @return Return code.
	int ExtractLocalFeatures(IplImage* pSourceImage, BlobListRiB& pBlobFeatures, ClusterMode pClusterMode);

	/// Routine for local SURF feature extraction.
	/// Extracts the local features with a SURF feature detector and writes them into <code>pBlobFeatures</code> with a 64bit SURF descriptor. As an additional feature,
	/// from the angles between the feature point's surface normal and the surface vectors starting in the feature point and directing in all directions
	/// of the object's surface in the neighborhood, the minimum and the maximum value are saved in the descriptor.
	/// In mask mode <code>MASK_NO</code> the whole image is searched for features. <code>MASK_LOAD</code> uses a ready-to-use mask loaded from file for this image.
	/// <code>MASK_SAVE</code> creates one mask per image which marks the object in the image. The mask is saved afterwards into a file.
	/// Runs only with cluster mode <code>CLUSTER_EM</code> since SURF descriptor has at least 64bit.
	/// @param pSourceImage The source image from which the features will be extracted.
	/// @param pBlobFeatures Returned list of found SURF features.
	/// @param pClusterMode Defines the cluster method to use (compare enum <code>ClusterMode</code>).
	/// @param pMaskMode Defines the mask mode (compare enum <code>MaskMode</code>).
	/// @param pMaskPath The path to which the mask image will be saved to or from which the mask image will be loaded if not in <code>MASK_NO</code>.
	/// @return Return code.
	int ExtractLocalFeatures(SharedImage* pSourceImage, BlobListRiB& pBlobFeatures, ClusterMode pClusterMode, MaskMode pMaskMode=MASK_NO, std::string pMaskPath="", Database pDatabase=INVALID, std::string pClassName="");

	int ExtractLocalRSDorFPFHFeatures(SharedImage* pSourceImage, BlobListRiB& pBlobFeatures, LocalFeatureParams& pLocalFeatureParams, MaskMode pMaskMode=MASK_NO, std::string pMaskPath="", Database pDatabase=INVALID);

	/// Extracts the global features for a single image.
	/// The global features composition depends on the cluster mode: In mode <code>CLUSTER_8BIT</code> only a 256 bin histogram is developed (= bag of words method).
	/// In <code>CLUSTER_EM</code> mode, first a variable bin (fixed size after training session) histogram is computed (= bag of words method) and various 3D features are
	/// added. These are: A PCA analysis where the largest PCA Eigenvalue is saved as feature as it is and the both other Eigenvalues relative to it (percentage),
	/// a 3D surface curve fitting along the Eigenvector projections to the 2D image and statistics about feature point frame directions compared to
	/// the largest principal component (eigenvector).
	/// The local features must be computed before because the list of local features of the image is needed. Furthermore, the local feature clusterer must be loaded before.
	/// @param pBlobFeatures List of local features (input to this function).
	/// @param pGlobalFeatures Returned vector containing the extracted global features for the object/image.
	/// @param pClusterMode Defines the cluster method to use (compare enum <code>ClusterMode</code>).
	/// @param pCoordinateImage The coordinate image of the shared image which contains the depth information. PCA can only be performed over the whole 3D surface inside the <code>pMask</code> region if this image is provided else only the 3D coordinates of the blob features can be used (if available).
	/// @param pMask A mask for the position of the object in the image. Some 3D features like full surface PCA and curve fitting need this mask.
	/// @param pOutputImage If not <code>NULL</code>, PCA Eigenvector directions are written into this image and it will be saved to file. The output image is not returned but deleted inside this function.
	/// @return Return code.
	int ExtractGlobalFeatures(BlobListRiB* pBlobFeatures, CvMat** pGlobalFeatures, ClusterMode pClusterMode, GlobalFeatureParams& pGlobalFeatureParams, Database pDatabase=INVALID, const IplImage* pCoordinateImage=NULL,
								IplImage* pMask=NULL, IplImage* pOutputImage=NULL, bool pFileOutput=false, std::string pTimingLogFileName="timing.txt", std::ofstream* pScreenLogFile=0);


	/// Saves the local feature point data (<code>mLocalFeaturesMap</code>) to file.
	/// @param pFileName The file (and path) name for local feature data storage.
	/// @return Return code.
	int SaveFPDataLocal(std::string pFileName) { return mData.SaveLocalFeatures(pFileName); };
	
	/// Loads the local feature point data (<code>mLocalFeaturesMap</code>) from file.
	/// @param pFileName The file (and path) name for local feature data storage.
	/// @return Return code.
	int LoadFPDataLocal(std::string pFileName) { return mData.LoadLocalFeatures(pFileName); };

	/// Saves the global feature point data (<code>mGlobalFeaturesMap</code>) to file.
	/// @param pFileName The file (and path) name for global feature data storage.
	/// @return Return code.
	int SaveFPDataGlobal(std::string pFileName) { return mData.SaveGlobalFeatures(pFileName); };

	/// Loads the global feature point data (<code>mGlobalFeaturesMap</code>) from file.
	/// @param pFileName The file (and path) name for global feature data storage.
	/// @return Return code.
	int LoadFPDataGlobal(std::string pFileName) { return mData.LoadGlobalFeatures(pFileName); };

	/// Saves the global classifier models (<code>mGlobalClassifierMap</code>) to files.
	/// There is one general file (class names, thresholds) and furthermore one model file for each classifier.
	/// @param pPath The path where the files shall be stored.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int SaveClassifiersGlobal(std::string pPath, ClassifierType pClassifierType) { return mData.SaveGlobalClassifiers(pPath, pClassifierType); };

	/// Loads the global classifier models (<code>mGlobalClassifierMap</code>) from files.
	/// There is one general file (class names, thresholds) and furthermore one model file for each classifier.
	/// @param pPath The path in which the model files can be found.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int LoadClassifiersGlobal(std::string pPath, ClassifierType pClassifierType) { return mData.LoadGlobalClassifiers(pPath, pClassifierType); };

	/// Saves the local classifier models (<code>mLocalClassifierMap</code>) to files.
	/// There is one general file (class names, thresholds) and furthermore one model file for each classifier.
	/// @param pPath The path where the files shall be stored.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int SaveClassifiersLocal(std::string pPath, ClassifierType pClassifierType) { return mData.SaveLocalClassifiers(pPath, pClassifierType); };

	/// Loads the local classifier models (<code>mLocalClassifierMap</code>) from files.
	/// There is one general file (class names, thresholds) and furthermore one model file for each classifier.
	/// @param pPath The path in which the model files can be found.
	/// @param pClassifierType The type of used classifier (cf. enum <code>ClassifierType</code>).
	/// @return Return code.
	int LoadClassifiersLocal(std::string pPath, ClassifierType pClassifierType) { return mData.LoadLocalClassifiers(pPath, pClassifierType); };

	/// Saves the squareroot of the inverse covariance matrix of the local feature data (<code>mSqrtInverseCovarianceMatrix</code>) to file.
	/// @param pFileName The file (and path) name for the covariance matrix storage.
	/// @return Return code.
	int SaveSqrtInverseCovarianceMatrix(std::string pFileName) { return mData.SaveSqrtInverseCovarianceMatrix(pFileName); };

	/// Loads the squareroot of the inverse covariance matrix of the local feature data (<code>mSqrtInverseCovarianceMatrix</code>) from file.
	/// @param pFileName The file (and path) name for the covariance matrix storage.
	/// @return Return code.
	int LoadSqrtInverseCovarianceMatrix(std::string pFileName) { return mData.LoadSqrtInverseCovarianceMatrix(pFileName); };

	/// Saves the EM cluster model of the local feature data (<code>mLocalFeatureClusterer</code>) to file.
	/// ATTENTION: The function <code>save()</code> for the CvEM cluster class was not implemented yet. This save function can only save all matrices of the CvEM model.
	/// @param pFileName The file (and path) name for the EM cluster model storage.
	/// @return Return code.
	int SaveLocalFeatureClusterer(std::string pFileName) { return mData.SaveLocalFeatureClusterer(pFileName); };

	/// Loads the EM cluster model of the local feature data (<code>mLocalFeatureClusterer</code>) from file.
	/// ATTENTION: The function <code>load()</code> for the CvEM cluster class was not implemented yet. Since there is no way to write back the matrices into the
	/// CvEM cluster model directly, this load function initializes CvEMParams with all matrices only and runs the normal train mode. Therefore, the <b>local feature
	/// data</b> needs to be <b>loaded before</b> loading the cluster model!
	/// @param pFileName The file (and path) name for the EM cluster model storage.
	/// @return Return code.
	int LoadLocalFeatureClusterer(std::string pFileName) { return mData.LoadLocalFeatureClusterer(pFileName); };

	/// Get a pointer to the data collection object.
	/// @return Data container.
	ClassificationData* GetDataPointer() { return &mData; };

private:
	/// Converts a binary number to an integer.
	/// @param pBinary Binary number, first entry = LSB, last entry = MSB.
	/// @return Integer value of the binary number.
	int BinaryToInt(ipa_utils::IpaVector<float> pBinary);

	/// Outputs the statistics.
	/// Creates an on-screen output of the statistics and writes SN, nSN, SP, nSP for cross-validation and test set into a file.
	/// @param pClass The class whose statistics is output.
	/// @param pTestSetPerformance Statistics of the test set.
	/// @param pOutputFileStream Output file stream for the data output to file.
	/// @param pFactorIncorrect Important for correct balanced SP and nSP calculation if not equal 1.
	/// @param pROCDistanceToTopLeftCorner Return variable for the distance to point (0,1) in ROC diagram.
	/// @param pThreshold A threshold for the ROC curve generation.
	/// @return Return code.
	int CrossValidationStatisticsOutput(std::string pClass, ClassifierPerformanceStruct& pValidationSetPerformance, ClassifierPerformanceStruct& pTestSetPerformance,
										std::ofstream& pOutputFileStream, float pFactorIncorrect, double& pROCDistanceToTopLeftCorner, float pThreshold, bool pPrintHeader=false, std::ofstream* pScreenLogFile=0);

	/// Generates a true positive, false positive, true negative, false negative statistics from a vector of classifier outputs.
	/// @param pResponses The classifier outputs
	/// @param pPerformance The container for the statistics.
	/// @param pThreshold The threshold above which a response is considered to be positive.
	void StatisticsFromOutput(ClassifierOutputCollection pResponses, ClassifierPerformanceStruct& pPerformance, double pThreshold);

	ClassificationData mData;		///< Data container for all classifier, feature and statistics data.

	boost::mutex mDisplayImageMutex;

	cv::Mat mDisplayImageOriginal, mDisplayImageSegmentation;

	std::vector<ObjectLocalizationIdentification> mCurrentDetections, mLastDetections;

	// parameters for runtime use
	cv::Size mTargetScreenResolution;	// in [pixels]
	cv::Size mTargetSegmentationImageResolution;	// in [pixels]
	cv::Point3d mConsideredVolume;	// in [m]
	cv::Point3f mVoxelFilterLeafSize;	// in [m]
	int mPlaneSearchMaxIterations;
	double mPlaneSearchDistanceThreshold;	// in [m]
	double mPlaneSearchAbortRemainingPointsFraction;	// within [0.0, 1.0]
	unsigned int mPlaneSearchAbortMinimumPlaneSize;	// int [#points]
	double mClusterSearchToleranceValue;	// in [m]
	int mClusterSearchMinClusterSize;
	int mClusterSearchMaxClusterSize;
	cv::Point3f mConsideredClusterCenterVolume;	// in [m]
	cv::Scalar mDisplayFontColorBGR;	// order: B, G, R
	double mTrackingInfluenceFactorOldData;	// within [0.0, 1.0]
};
