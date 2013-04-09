#ifndef _LITERATE_PR2_TEXT_DETECT_
#define _LITERATE_PR2_TEXT_DETECT_

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// OpenCV includes
#include "opencv2/core/core.hpp"
#include "cv.h"
#include "highgui.h"

// Different includes
#include <set>
#include <iostream>
#include <fstream>

class MeanShiftSegmentation1D
{
public:
	MeanShiftSegmentation1D(const std::vector<double>& data, std::vector<double>& convergencePoints, std::vector< std::vector<int> >& convergenceSets, double bandwidth, int maximumIterations=100)
	{
		// clear output structures
		convergencePoints.clear();
		convergenceSets.clear();

		//  prepare mean shift set
		std::vector<double> meanShiftSet = data, meanShiftSet2(data.size());

		//  mean shift iteration
		for (int iter=0; iter<maximumIterations; iter++)
		{
			for (int i=0; i<(int)meanShiftSet.size(); i++)
			{
				double nominator = 0., denominator = 0.;
				for (int j=0; j<(int)meanShiftSet.size(); j++)
				{
					double weight = exp(-bandwidth * (meanShiftSet[j]-meanShiftSet[i]) * (meanShiftSet[j]-meanShiftSet[i]));
					nominator += weight*meanShiftSet[j];
					denominator += weight;
				}
				meanShiftSet2[i] = nominator/denominator;
			}
			meanShiftSet = meanShiftSet2;
		}
//		for (int i=0; i<(int)meanShiftSet.size(); i++)
//			std::cout << "  meanshift[" << i << "]=" << meanShiftSet[i] << std::endl;

		//  cluster data according to convergence points
		convergenceSets.resize(1, std::vector<int>(1, 0));
		convergencePoints.resize(1, meanShiftSet[0]);
		for (int i=1; i<(int)meanShiftSet.size(); i++)
		{
			bool createNewSet = true;
			for (int j=0; j<(int)convergencePoints.size(); j++)
			{
				if (abs(meanShiftSet[i]-convergencePoints[j]) < bandwidth*0.01)
				{
					convergenceSets[j].push_back(i);
					convergencePoints[j] = (convergencePoints[j]*(convergenceSets[j].size()-1.) + meanShiftSet[i]) / (double)convergenceSets[j].size();	// update mean of convergence point
					createNewSet = false;
				}
			}
			if (createNewSet == true)
			{
				convergenceSets.push_back(std::vector<int>(1, i));
				convergencePoints.push_back(meanShiftSet[i]);
			}
		}
	};
};


class DetectText
{
public:
	DetectText();
	DetectText(bool eval, bool enableOCR);
	~DetectText();

	// API
	void detect(std::string filename);
	void detect(cv::Mat& image);

	// read correlation, dictionary and params.yaml
	void readLetterCorrelation(const char* filename);
	void readWordList(const char* filename);
	void setParams(ros::NodeHandle & nh);

	// getters
	cv::Mat& getDetection();
	std::vector<std::string>& getWords();
	std::vector<cv::RotatedRect>& getBoxes();

private:
	// internal structures
	enum Mode
	{
		IMAGE = 1, STREAM = 2
	};

	enum FontColor
	{
		BRIGHT = 1, DARK = 2
	};

	enum Purpose // for SWT
	{
		UPDATE = 1, REFINE = 2
	};

	struct Pair
	{
		Pair(int left, int right, double dx, double dy) :
			left(left), right(right), dx(dx), dy(dy)
		{
		}
		int left;
		int right;
		double dx;
		double dy;
	};

	struct Word
	{
		Word() :
			word(), score(1000)
		{
		}
		Word(std::string word, float score) :
			word(word), score(score)
		{
		}
		std::string word;
		float score;
	};

	struct bgr
	{
		uchar b; // blue channel value
		uchar g; // green channel value
		uchar r; // red channel value

		bgr()
		{
			b = 255;
			g = 255;
			r = 255;
		}

		bgr(uchar blue, uchar green, uchar red) :
			b(blue), g(green), r(red)
		{
		}
	};

	struct TreeNode
	{
		int parent;
		int element;
		int rank;
	};

	struct Letter
	{
		cv::Rect boundingBox;		// bounding box around letter region
		double diameter;			// the length of the diagonal of the bounding box
		cv::Point2d centerPoint;		// center of bounding box (stored explicitly because used very often directly --> saves computational effort)
		FontColor fontColor;		// brightness level of letter font: bright or dark
	};

	struct TextRegion
	{
		cv::Rect boundingBox;				// bounding box of the whole text region, including all bounding boxes of the assigned letters
		cv::RotatedRect lineEquation;		// equation for the fitted line through the letters (center is a point on the line, size is the normal, angle is the letter fitting score)
		double qualityScore;				// e.g. quality of line approximation
		std::vector<Letter> letters;		// the letters that are contained in the text region
	};


  // main method
	void detect();
	void detect_original_epshtein();
	void detect_bormann();

	void preprocess();

	void pipeline();

	void strokeWidthTransform(const cv::Mat &image, cv::Mat &swtmap, int searchDirection);

	cv::Mat computeEdgeMap(bool rgbCanny);

	void updateStrokeWidth(cv::Mat &swtmap, std::vector<cv::Point> &startPoints, std::vector<cv::Point> &strokePoints, int searchDirection, Purpose purpose);

	void closeOutline(cv::Mat& edgemap);

	int connectComponentAnalysis(const cv::Mat& swtmap, cv::Mat& ccmap);

	void identifyLetters(const cv::Mat& swtmap, const cv::Mat& ccmap);

	int countInnerLetterCandidates(std::vector<bool> & array);

	std::vector<float> getMeanIntensity(const cv::Mat& ccmap, const cv::Rect& rect, int element, bool background);

	void groupLetters(const cv::Mat& swtmap, const cv::Mat& ccmap);

	float getMedianStrokeWidth(const cv::Mat& ccmap, const cv::Mat& swtmap, const cv::Rect& rect, int element);

	// merges letterGroups_ to chains of letters, finds bounding box of these chains
	void chainPairs(std::vector<TextRegion>& textRegions);

	// finds bounding box of a chain
	void chainToBox(std::vector< std::vector<int> >& chain, /*std::vector<cv::Rect>& boundingBox,*/ std::vector<TextRegion>& textRegions);

	bool sameTextline(const TextRegion& a, const TextRegion& b);

	bool pairsInLine(const Pair& a, const Pair& b);

	void mergePairs(const std::vector<Pair>& groups, std::vector< std::vector<int> >& chains);

	bool mergePairs(const std::vector< std::vector<int> >& initialChains, std::vector< std::vector<int> >& chains);

	void merge(const std::vector<int>& token, std::vector<int>& chain);

	void combineNeighborBoxes(std::vector<cv::Rect> & boundingBoxes);

	// computes the signed distance of a point (point) to a line (represented by a line point and the line normal is Hessian Normal Form)
	double pointLineDistance2D(cv::Point2d linePoint, cv::Point2d lineNormal, cv::Point2d point);

	// this function splits up bounding boxes of potentially multiline text into boxes around single line text
	void breakLines(std::vector<TextRegion>& textRegions);

	void breakLinesIntoWords(std::vector<TextRegion>& textRegions);

//	int decideWhichBreaks(float negPosRatio, float max_Bin, float baselineStddev, unsigned int howManyNegative, unsigned int shift, int maxPeakDistance, int secondMaxPeakDistance,
//			int maxPeakNumbers, int secondMaxPeakNumbers, unsigned int boxWidth, unsigned int boxHeight, unsigned int numberBinsNotZero, std::vector<DetectText::Pair> wordBreaks,
//			cv::Rect box, bool textIsRotated, float relativeY, bool SteadyStructure, float sameArea);

	static bool spatialOrderX(cv::Rect a, cv::Rect b);

	void disposal();

	void deleteDoubleBrokenWords(std::vector<cv::Rect>& brokenWords_);

	void ocrPreprocess(std::vector<cv::Mat> &images);

	void rotate();

	cv::Mat colorBackgroundBinary(cv::Mat & m, FontColor f, cv::Mat cc);

	cv::Mat sharpenImage(cv::Mat input);

	void addBorder(cv::Mat & image, cv::Rect r, FontColor f);

	cv::Mat binarizeViaContrast(cv::Mat input);

	bgr findBorderColor(cv::Rect r, FontColor f);

	void ocrRead(std::vector<cv::Mat> textImages);

	float ocrRead(const cv::Mat& imagePatch, std::string& output);

	float spellCheck(std::string& str, std::string& output, int method);

	std::string& trim(std::string& str);

	void getTopkWords(const std::string& str, const int k, std::vector<Word>& words);

	static int editDistance(const std::string& s, const std::string& t);

	float editDistanceFont(const std::string& s, const std::string& t);

	int getCorrelationIndex(std::string letter);

	float insertToList(std::vector<Word>& words, Word& word);

	void showBoundingBoxes(std::vector<cv::RotatedRect>& boxes, std::vector<std::string>& text);

	void overlayText(std::vector<cv::RotatedRect>& box, std::vector<std::string>& text);

	void writeTxtsForEval();

	void ransacPipeline(std::vector<TextRegion>& textRegions);

	std::vector< std::pair< std::vector<cv::Point>, std::vector<cv::Point> > > ransac(std::vector<Letter> dataset);

	cv::Mat createBezierCurve(std::vector<cv::Point> & points, bool p);

	std::pair<float, float> getBezierDistance(cv::Mat curve, cv::Point q);

	float getBezierLength(cv::Mat curve, float mint, float maxt);

	void transformBezier(cv::Rect newR, cv::Mat curve, cv::Mat & transformedImage, float mint, float maxt);

	unsigned int calculateRealLetterHeight(cv::Point2f p, cv::Rect r, cv::Point2f alpha);

	void postRansacCriterions();

	void Cramer();

	// Methods not used at the moment:
	//------------------------------------------

	void filterBoundingBoxes(std::vector<cv::Rect>& boundingBoxes, cv::Mat& ccmap, int rejectRatio);

	static bool spatialOrder(cv::Rect a, cv::Rect b);

	static bool pairOrder(Pair i, Pair j);

	static bool pointYOrder(cv::Point a, cv::Point b);

	void overlapBoundingBoxes(std::vector<cv::Rect>& boundingBoxes);

	void DFT(cv::Mat& input);

	cv::Mat filterPatch(const cv::Mat& patch);

	// Methods for debugging only
	//------------------------------------------

	void showEdgeMap();
	void showCcmap(cv::Mat& ccmap);
	void showSwtmap(cv::Mat& swtmap);
	void showLetterDetection();
	void showLetterGroup();
	void testGetCorrelationIndex();
	void testEditDistance();
	void testInsertToList();
	void testMergePairs();
	void testEdgePoints(std::vector<cv::Point> &edgepoints);

	// Variables
	//------------------------------------------

	// I/O
	std::string filename_;
	std::string outputPrefix_;
	cv::Mat correlation_; // read from argv[1]
	std::vector<std::string> wordList_; // read from argv[2]

	// important images
	cv::Mat originalImage_;
	cv::Mat grayImage_;
	cv::Mat resultImage_;

	// general
	bool firstPass_; //  white font: true, black font: false

	// SWT
	int maxStrokeWidth_;
	float initialStrokeWidth_;
	cv::Mat edgemap_; // edges detected at gray image
	cv::Mat theta_; // gradient map, arctan(dy,dx)
	cv::Mat dx_;
	cv::Mat dy_;
	std::vector<cv::Point> edgepoints_; // all points where an edge is

	// Connect Component
	std::vector<cv::Rect> labeledRegions_; // all regions (with label) that could be a letter
	std::size_t nComponent_; // =labeledRegions_.size()
	cv::Mat ccmapBright_, ccmapDark_; // copy of whole cc map
	std::vector<TextRegion> textRegions_; // contains several region of letters that putatively belong to the same word or word group

	// Identify Letters
	std::vector<bool> isLetterRegion_; // which region is letter
	std::vector<double> medianStrokeWidth_; // median stroke width for each letter region
	std::vector< std::vector<float> > meanRGB_; // mean R,G,B and Gray value of foreground pixels of every region
	std::vector< std::vector<float> > meanBgRGB_; // same with background pixels
	unsigned int nLetter_; // how many regions are letters

	// Group Letters
	std::vector<Pair> letterGroups_; // 2 int values, correspond to indexes at isLetterComponects_, for letters that belong together

	// Chain to Box
	std::vector<cv::Rect> boundingBoxes_; // all boundingBoxes, black and white font combined
	std::vector<cv::Rect> brightBoxes_, darkBoxes_; // separate b/w font
	std::vector<FontColor> boxFontColor_; //remember which boundingBox had which font color

	// OCR Preprocess
	unsigned int fontColorIndex_;
	std::vector<cv::Mat> transformedImage_; // all boundingBoxes, rotated and transformed based on found bezier curve
	std::vector<cv::Mat> transformedFlippedImage_;
	std::vector<cv::Mat> notTransformedImage_;

	std::vector<cv::Rect> finalBoundingBoxes_;
	std::vector<cv::RotatedRect> finalRotatedBoundingBoxes_;
	std::vector<double> finalBoundingBoxesQualityScore_;

	double sigma_sharp, threshold_sharp, amount_sharp;

	// OCR
	bool enableOCR_;
	int result_;
	std::vector<cv::Mat> textImages_;
	std::vector<cv::RotatedRect> finalBoxes_;
	std::vector<std::string> finalTexts_;
	std::vector<float> finalScores_;

	// Debug etc.
	std::map<std::string, bool> debug;
	bool eval_; //true=evaluation (read_evaluation) false=standard

	// Not used but useful variables:
	//Mode mode_; // streaming from topic or reading image file

	// Parameters given by yaml
	// --- general ---
	enum ProcessingMethod
	{
		ORIGINAL_EPSHTEIN = 0, BORMANN
	};
	ProcessingMethod processing_method_; // defines the method for finding texts in the image

	// --- transform ---
	bool transformImages;

	// --- preprocess ---
	bool smoothImage; // default: false, smoothing leads to merging of letters within small texts (that is bad)
	int maxStrokeWidthParameter; // default: maxStrokeWidthParameter = 50, good for big text: <50, good for careobot/small texts: >50
	// --- strokeWidthTransform ---
	bool useColorEdge; // true = use rgb channels to compute edgeMap, false = only gray image is used
	// --- computeEdgeMap ---
	int cannyThreshold1; // default: 120
	int cannyThreshold2; // default: 50 , cannyThreshold1 > cannyThreshold2
	// --- updateStrokeWidth ---
	double compareGradientParameter_; // default: 3.14 / 2, in paper: 3.14 / 6 -> unrealistic
	// --- connectComponentAnalysis ---
	double swCompareParameter; // default: 3.0
	int colorCompareParameter; // default: 100, set to 255 to deactivate
	// --- identifyLetter ---
	int maxLetterHeight_; // default: 600
	int minLetterHeight_; //default: 10
	double varianceParameter; // default: 1.5 , also good: 3.5
	int diagonalParameter; // default 10 - but with maxStrokeWidth in code, medianStrokeWidth in paper
	int pixelCountParameter; // default: 5
	int innerLetterCandidatesParameter; // default: 5 , paper: 2
	double clrComponentParameter; // default: 20
	// ---  groupLetters ---
	double distanceRatioParameter; // default: 2.0
	double medianSwParameter; // default: 2.5
	double diagonalRatioParamter; // default 2.0
	double grayClrParameter; //default: 10.0
	double clrSingleParameter; // better 15 default: 35
	double areaParameter; // better: 1.5 ; default: 5
	double pixelParameter; // default 0.3
	// --- linear ransac (original implementation) ---
	double inlierDistanceThresholdFactor_;
	// --- ransac ---
	double p; // probability p, default: 0.99
	double maxE;
	double minE; // linear based on pca, default: [0.4,0.7]
	int bendParameter; // default: 30
	double distanceParameter; // maxDistance = distanceParameter * (Σ[i_n](sqrt(letterArea(i))) / n), default: 0.8
};

#endif
