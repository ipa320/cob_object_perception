#ifndef __SHAREDIMAGEGLOBALCONSTANTS_H__
#define __SHAREDIMAGEGLOBALCONSTANTS_H__

#include <string>
#include <opencv/cv.h>

using namespace std;

#ifdef __JBK_OBEN__
static const string calibConfigurationFilesDirectory = "C:/Entwicklung/vision/Files/Vision/ObjectDetection/DemonstratorVision/JBKCalibration/";
#else
static const string calibConfigurationFilesDirectory = "E:/Software/vision/Files/Vision/ObjectDetection/DemonstratorVision/JBKCalibration/";
#endif // __JBK_OBEN__

static const string calibImagesPath = calibConfigurationFilesDirectory + "calibImages/";
static const string calibImagesName = "CalibImage";
static const string undistortedCalibImagesName = "CalibImageUndistSR";
static const string calibParameterPath = calibConfigurationFilesDirectory + "outputParams/";
static const string calibOutputImagesPath = calibConfigurationFilesDirectory + "outputImages/";

// parameter maps SR
static const string IntrinsicMatrixSRName = "IntrinsicMatrixSR.xml";
static const string UndistortXMapName = "UndistortionMapX.xml";;
static const string UndistortYMapName = "UndistortionMapY.xml";
static const string DistanceParamsName = "DistanceParams.xml";
// color cam
static const string IntrinsicMatrixCCName = "IntrinsicMatrixCC.xml";
static const string DistortionCoeffsCCName = "DistortionCoeffsCC.xml";

// extrinsic frame
static const string ExtrinsicTrafoTranslationVectorName = "ExtrinsicTranslationVector.xml";
static const string ExtrinsicTrafoRotationVectorName = "ExtrinsicRotationVector.xml";
static const string ExtrinsicTrafoRotationMatrixName = "ExtrinsicRotationMatrix.xml";

// sizes
static const CvSize RangerSize = cvSize(176, 144);
static const int SharedImageEnlargeFactor = 3;
static const CvSize SharedImageSize = cvSize(SharedImageEnlargeFactor*RangerSize.width, SharedImageEnlargeFactor*RangerSize.height);
static const CvSize OriginalColorImageSize = cvSize(1280, 960); // todo: take this from the configuration file
static const int SharedImageShrinkFactor = 2;
static const CvSize ColorImageSize = cvSize(OriginalColorImageSize.width/SharedImageShrinkFactor, OriginalColorImageSize.height/SharedImageShrinkFactor);
static const float Z_MAX = 100000;
static const float Z_OFFSET = 40;

// frame learning table
//static const string LearningTableFrameName = "LearningTable.txt";

#endif // __SHAREDIMAGEGLOBALCONSTANTS_H__