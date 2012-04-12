#ifndef _LITERATE_PR2_TEXT_DETECT_
#define _LITERATE_PR2_TEXT_DETECT_

#include "opencv2/core/core.hpp"
#include "cv.h"
#include "highgui.h"

using namespace cv;
using namespace std;

class Rotated
{
public:
  Rotated(Mat m, Rect c)
  {
    rotated_img = m;
    coords = c;
  }
  ~Rotated()
  {
  }
  Mat rotated_img;
  Rect coords;
};

class DetectText
{
public:
  DetectText();
  ~DetectText();

  std::vector<Rotated> rotated;

  /* API */
  void detect(string filename);
  void detect(Mat& image);

  /* read useful files  */
  void readLetterCorrelation(const char* filename);

  void readWordList(const char* filename);

  /* getters */
  Mat& getDetection();

  vector<string>& getWords();

  vector<Rect>& getBoxesBothSides();

  /* tests */

  void testMergePairs();

  void testEditDistance();

  void testGetCorrelationIndex();

  void testInsertToList();

private:
  /* internal structures */
  enum Mode
  {
    IMAGE = 1, STREAM = 2
  };

  enum FontColor
  {
    BRIGHT = 1, DARK = 2
  };

  enum Purpose
  {
    UPDATE = 1, REFINE = 2
  };

  enum Result
  {
    COARSE = 1, FINE = 2
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
    Word(string word, float score) :
      word(word), score(score)
    {
    }
    string word;
    float score;
  };

  /* pipeline for detecting black/white words*/
  void detect();

  void preprocess(Mat &image);

  void pipeline(int blackWhite);

  void disposal();

  void strokeWidthTransform(const Mat &image, Mat &swtmap, int searchDirection);

  /* for each edge point, search along gradient
   * direction compute stroke width
   * searchDirection: 1 for along gradient, -1 for opposite
   * purpose: 1 for compute, 2 for refine
   */
  void updateStrokeWidth(Mat &swtmap, vector<Point> &startPoints, vector<Point> &strokePoints, int searchDirection,
                         Purpose purpose);

  int connectComponentAnalysis(const Mat& swtmap, Mat& ccmap);

  void identifyLetters(const Mat& swtmap, const Mat& ccmap);

  void groupLetters(const Mat& swtmap, const Mat& ccmap);

  void chainPairs(Mat& ccmap);

  void findRotationangles();

  void filterBoundingBoxes(vector<Rect>& boundingBoxes, Mat& ccmap, int rejectRatio);

  void chainToBox(vector<vector<int> >& chain, vector<Rect>& boundingBox);

  void overlapBoundingBoxes(vector<Rect>& boundingBoxes);

  void overlayText(vector<Rect>& box, vector<string>& text);

  void ocrRead(vector<Rect>& boundingBoxes);

  float ocrRead(const Mat& imagePatch, string& output, int actual);

  float spellCheck(string& str, string& output, int method);

  Mat filterPatch(const Mat& patch);

  // helper functions
  int ImageAdjust(IplImage* src, IplImage* dst, double low, double high, double bottom, double top, double gamma);

  int countInnerLetterCandidates(bool* array);

  float getMeanIntensity(const Mat& ccmap, const Rect& rect, int element);

  float getMedianStrokeWidth(const Mat& ccmap, const Mat& swtmap, const Rect& rect, int element);

  void mergePairs(const vector<Pair>& groups, vector<vector<int> >& chains);

  bool mergePairs(const vector<vector<int> >& initialChains, vector<vector<int> >& chains);

  void merge(const vector<int>& token, vector<int>& chain);

  static int editDistance(const string& s, const string& t);

  float editDistanceFont(const string& s, const string& t);

  int getCorrelationIndex(char letter);

  void getNearestWord(const string& str, string& nearestWord);

  void getTopkWords(const string& str, const int k, vector<Word>& words);

  float insertToList(vector<Word>& words, Word& word);

  string& trim(string& str);

  static bool spaticalOrder(Rect a, Rect b);

  // display intermidate results
  void showEdgeMap();

  void showCcmap(Mat& ccmap);

  void showSwtmap(Mat& swtmap);

  void showLetterDetection();

  void showLetterGroup();

  void showBoundingBoxes(vector<Rect>& boxes);

  void showBoundingBoxes(vector<Rect>& boxes, vector<bool>& text);
  // tests
  void testEdgePoints(vector<Point> &edgepoints);

  /***** variables *******/

  // these variables stays for the same image
  Mat originalImage_;
  Mat image_; // gray scale to be processed
  Mat detection_;
  float maxStrokeWidth_;
  float initialStrokeWidth_;
  Mat edgemap_;
  Mat theta_;
  bool firstPass_; //  white: 1, black : 0
  vector<Point> edgepoints_;

  Mat correlation_; // read from arg[1]
  vector<string> wordList_; // read from arg[2]
  Mode mode_; // streaming or images

  vector<Rect> boxesBothSides_;
  vector<string> wordsBothSides_;
  vector<float> boxesScores_;

  vector<bool> boxInbox_;

  FontColor fontColor_;
  Result result_;
  // these variables should be cleaned between calculations
  vector<Rect> componentsRoi_;
  bool *isLetterComponects_;
  bool *isGrouped_;
  vector<bool*> innerComponents_;

  vector<Pair> horizontalLetterGroups_;
  vector<Pair> verticalLetterGroups_;
  vector<vector<int> > horizontalChains_;
  vector<vector<int> > verticalChains_;

  vector<Rect> boundingBoxes_;

  float *componentsMeanIntensity_;
  float *componentsMedianStrokeWidth_;

  size_t nComponent_;
  float maxLetterHeight_;
  float minLetterHeight_;

  string filename_;
  string outputPrefix_;

  int textDisplayOffset_;

};

#endif
