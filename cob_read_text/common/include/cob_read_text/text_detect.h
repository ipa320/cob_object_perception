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

class DetectText
{
public:
  DetectText();
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
  std::vector<cv::Rect>& getBoxes();

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
    Pair(int left, int right) :
      left(left), right(right)
    {
    }
    int left;
    int right;
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
    uchar r; // red channel value
    uchar g; // green channel value
    uchar b; // blue channel value

    bgr()
    {
      r = 255;
      g = 255;
      b = 255;
    }

    bgr(uchar red, uchar green, uchar blue) :
      r(red), g(green), b(blue)
    {
    }
  };

  struct connectedComponent
  {
    cv::Rect r;
    cv::Point middlePoint;
    FontColor clr;
  };

  // main method
  void detect();

  void preprocess();

  void pipeline(int blackWhite);

  void strokeWidthTransform(const cv::Mat &image, cv::Mat &swtmap, int searchDirection);

  cv::Mat computeEdgeMap(bool rgbCanny);

  void updateStrokeWidth(cv::Mat &swtmap, std::vector<cv::Point> &startPoints, std::vector<cv::Point> &strokePoints,
                         int searchDirection, Purpose purpose);

  int connectComponentAnalysis(const cv::Mat& swtmap, cv::Mat& ccmap);

  void identifyLetters(const cv::Mat& swtmap, const cv::Mat& ccmap);

  int countInnerLetterCandidates(std::vector<bool> & array);

  std::vector<float> getMeanIntensity(const cv::Mat& ccmap, const cv::Rect& rect, int element, bool background);

  void groupLetters(const cv::Mat& swtmap, const cv::Mat& ccmap);

  float getMedianStrokeWidth(const cv::Mat& ccmap, const cv::Mat& swtmap, const cv::Rect& rect, int element);

  std::vector<cv::Rect> chainPairs();

  void chainToBox(std::vector<std::vector<int> >& chain, std::vector<cv::Rect>& boundingBox);

  void mergePairs(const std::vector<Pair>& groups, std::vector<std::vector<int> >& chains);

  bool mergePairs(const std::vector<std::vector<int> >& initialChains, std::vector<std::vector<int> >& chains);

  void merge(const std::vector<int>& token, std::vector<int>& chain);

  void combineNeighborBoxes(std::vector<cv::Rect> & boundingBoxes);

  void breakLines(std::vector<cv::Rect> & boundingBoxes);

  static bool spaticalOrderX(cv::Rect a, cv::Rect b);

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

  int getCorrelationIndex(char letter);

  float insertToList(std::vector<Word>& words, Word& word);

  void showBoundingBoxes(std::vector<cv::Rect>& boxes, std::vector<std::string>& text);

  void overlayText(std::vector<cv::Rect>& box, std::vector<std::string>& text);

  void writeTxtsForEval();

  void ransacPipeline(std::vector<cv::Rect> & boundingBoxes);

  std::vector<std::pair<std::vector<cv::Point>, std::vector<cv::Point> > >
  ransac(std::vector<connectedComponent> dataset);

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

  static bool spaticalOrder(cv::Rect a, cv::Rect b);

  static bool pairOrder(Pair i, Pair j);

  static bool pointYOrder(cv::Point a, cv::Point b);

  void overlapBoundingBoxes(std::vector<cv::Rect>& boundingBoxes);

  int decideWhichBreaks(float negPosRatio, float max_Bin, float heightVariance, unsigned int howManyNegative,
                        unsigned int shift, int maxPeakDistance, int secondMaxPeakDistance, int maxPeakNumbers,
                        int secondMaxPeakNumbers, unsigned int boxWidth, unsigned int boxHeight,
                        unsigned int numberBinsNotZero, std::vector<DetectText::Pair> wordBreaks, cv::Rect box,
                        bool textIsRotated, float relativeY, bool SteadyStructure, float sameArea);

  void DFT(cv::Mat& input);

  cv::Mat filterPatch(const cv::Mat& patch);

  void breakLinesIntoWords();

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
  std::vector<cv::Point> edgepoints_; // all points where an edge is

  // Connect Component
  std::vector<cv::Rect> labeledRegions; // all regions (with label) that could be a letter
  std::size_t nComponent_; // =labeledRegions.size()
  cv::Mat ccmapBright_, ccmapDark_; // copy of whole cc map
  std::vector<std::vector<connectedComponent> > connectedComponents_;

  // Identify Letters
  std::vector<bool> isLetterRegion_; // which region is letter
  std::vector<std::vector<float> > meanRGB_; // mean R,G,B and Gray value of foreground pixels of every region
  std::vector<std::vector<float> > meanBgRGB_; // same with background pixels
  unsigned int nLetter_; // how many regions are letters

  // Group Letters
  std::vector<Pair> letterGroups_; // 2 int values, correspond to indexes at isLetterComponects_, for letters that belong together

  // Chain to Box
  std::vector<cv::Rect> boundingBoxes_; // all boundingBoxes, black and white font combined
  std::vector<cv::Rect> brightBoxes_, darkBoxes_; // separate b/w font
  std::vector<cv::Rect> brightLetters_, darkLetters_; // b/w font letter boxes
  std::vector<FontColor> boxFontColor_; //remember which boundingBox had which font color

  // OCR Preprocess
  unsigned int fontColorIndex_;
  std::vector<cv::Mat> transformedBoundingBoxes_; // all boundingBoxes, rotated and transformed based on found bezier curve
  std::vector<cv::Mat> transformedFlippedBoundingBoxes_;
  std::vector<cv::Mat> notTransformedBoundingBoxes_;
  std::vector<cv::Rect> finalBoundingBoxes_;
  std::vector<cv::RotatedRect> finalRotatedBoundingBoxes_;
  double sigma_sharp, threshold_sharp, amount_sharp;

  // OCR
  int result_;
  std::vector<cv::Mat> textImages_;
  std::vector<cv::Rect> finalBoxes_;
  std::vector<std::string> finalTexts_;
  std::vector<float> finalScores_;

  // Debug etc.
  std::map<std::string, bool> debug;
  bool eval; //true=evaluation (read_evaluation) false=standard

  // Not used but useful variables:
  Mode mode_; // streaming from topic or reading image file

  // Parameters given by yaml
  // --- preprocess ---
  bool smoothImage; // default: false, smoothing leads to merging of letters within small texts (that is bad)
  int maxStrokeWidthParameter; // default: maxStrokeWidthParameter = 50, good for big text: <50, good for careobot/small texts: >50
  // --- strokeWidthTransform ---
  bool useColorEdge; // true = use rgb channels to compute edgeMap, false = only gray image is used
  // --- computeEdgeMap ---
  int cannyThreshold1; // default: 120
  int cannyThreshold2; // default: 50 , cannyThreshold1 > cannyThreshold2
  // --- updateStrokeWidth ---
  double compareGradientParameter; // default: 3.14 / 2, in paper: 3.14 / 6 -> unrealistic
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
  // --- ransac ---
  double p; // probability p, default: 0.99
  double maxE;
  double minE; // linear based on pca, default: [0.4,0.7]
  int bendParameter; // default: 30
  double distanceParameter; // maxDistance = distanceParameter * (Σ[i_n](sqrt(letterArea(i))) / n), default: 0.8
};

#endif
