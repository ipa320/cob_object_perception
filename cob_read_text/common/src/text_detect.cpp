/**
 * Implementation based on "Detecting Text in Natural Scenes with  
 * Stroke Width Transform", Boris Epshtein, Eyal Ofek, Yonatan Wexler
 * CVPR 2010
 *
 */

/** \author Menglong Zhu */

#include <cob_read_text/text_detect.h>
#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <stack>

/*#include <sensor_msgs/PointCloud2.h>
 #include <pcl/ros/conversions.h>
 #include <pcl/point_cloud.h>
 #include <pcl/point_types.h>*/

using namespace cv;
using namespace std;

DetectText::DetectText() :
  maxStrokeWidth_(0), initialStrokeWidth_(0), firstPass_(true), result_(COARSE), nComponent_(0), maxLetterHeight_(0),
      minLetterHeight_(0), textDisplayOffset_(1), eval(true)
{
}
DetectText::~DetectText()
{
}

void DetectText::detect(string filename)
{
  filename_ = filename;
  originalImage_ = imread(filename_);

  if (!originalImage_.data)
  {
    ROS_ERROR("Cannot read image input...");
    return;
  }
  mode_ = IMAGE;
  detect();
}
void DetectText::detect(Mat& image)
{
  filename_ = string("streaming.jpg");
  originalImage_ = image;
  mode_ = STREAM;
  detect();
}

void DetectText::detect()
{
  double start_time;
  double time_in_seconds;
  start_time = clock();

  Mat imGray(originalImage_.size(), CV_8UC1, Scalar(0));
  cvtColor(originalImage_, imGray, CV_RGB2GRAY);
  boundingBoxes_.clear();
  boxesBothSides_.clear();
  wordsBothSides_.clear();
  boxesScores_.clear();

  preprocess(imGray);
  firstPass_ = true;
  pipeline(1); //bright font
  cout << "Second pass" << endl;
  firstPass_ = false;
  pipeline(-1); //dark font

  overlapBoundingBoxes(boundingBoxes_);
  ocrRead(boundingBoxes_);
  showBoundingBoxes(boxesBothSides_);
  std::cout << "1\n";
  overlayText(boxesBothSides_, wordsBothSides_);
  std::cout << "1\n";

  if (eval == true)
  {
    ofstream myfile, myfile2;
    std::string textname = outputPrefix_ + ".txt";
    std::string textname2 = outputPrefix_ + "t.txt";
    myfile.open(textname.c_str());
    for (int i = 0; i < boxesBothSides_.size(); i++)
    {
      myfile << boxesBothSides_[i].x << "\n" << boxesBothSides_[i].y << "\n" << boxesBothSides_[i].width << "\n"
          << boxesBothSides_[i].height << "\n";
    }
    myfile.close();
    myfile2.open(textname2.c_str());
    for (int i = 0; i < wordsBothSides_.size(); i++)
    {
      myfile2 << wordsBothSides_[i] << "\n";
    }
    myfile2.close();

  }

  imwrite(outputPrefix_ + "_detection.jpg", detection_);

  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s total in process\n" << endl;

  textDisplayOffset_ = 1;
}

void DetectText::preprocess(Mat& image)
{
  int maxStrokeWidthParameter = 50;
  maxLetterHeight_ = 300;
  minLetterHeight_ = 5;

  cout << "preprocessing: " << filename_ << endl;
  cout << "image size:" << image.cols << "X" << image.rows << endl;

  //outputPrefix_: without extension
  int slashIndex = -1;
  int dotIndex = -1;
  for (size_t i = filename_.length() - 1; i != 0; i--)
  {
    if (dotIndex == -1 && filename_[i] == '.')
      dotIndex = i;
    if (slashIndex == -1 && filename_[i] == '/')
      slashIndex = i;
  }
  outputPrefix_ = filename_.substr(slashIndex + 1, dotIndex - slashIndex - 1);
  cout << "outputPrefix: " << outputPrefix_ << endl;

  //setting initialStrokeWidth_ for SWT
  image_ = image;

  // hack: was turned off
  //    bilateralFilter(image, image_, 7, 20, 50);// prosilica sensor noise
  maxStrokeWidth_ = round((float)(max(image.cols, image.rows)) / maxStrokeWidthParameter);
  initialStrokeWidth_ = maxStrokeWidth_ * 2;

  // 600 pixel side for displaying results
  IplImage *img2 = new IplImage(originalImage_);
  IplImage *img1 = cvCreateImage(cvSize(image.cols + 600, image.rows), img2->depth, img2->nChannels);
  cvSet(img1, cvScalar(0, 0, 0));
  cvSetImageROI(img1, cvRect(0, 0, image.cols, image.rows));
  cvCopy(img2, img1, NULL);
  cvResetImageROI(img1);
  detection_ = Mat(img1).clone();
  cvReleaseImage(&img1);
  delete img1;
  delete img2;
}

Mat& DetectText::getDetection()
{
  return detection_;
}
vector<string>& DetectText::getWords()
{
  return wordsBothSides_;
}

vector<Rect>& DetectText::getBoxesBothSides()
{
  return boxesBothSides_;
}

void DetectText::pipeline(int blackWhite)
{
  if (blackWhite == 1)
  {
    fontColor_ = BRIGHT;
  }
  else if (blackWhite == -1)
  {
    fontColor_ = DARK;
  }
  else
  {
    cout << "blackwhite should only be +/-1" << endl;
    assert(false);
  }
  double start_time;
  double time_in_seconds;

  start_time = clock();
  Mat swtmap(image_.size(), CV_32FC1, Scalar(initialStrokeWidth_));

  strokeWidthTransform(image_, swtmap, blackWhite);
  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s in strokeWidthTransform" << endl;

  start_time = clock();
  Mat ccmap(image_.size(), CV_32FC1, Scalar(-1));
  componentsRoi_.clear();
  nComponent_ = connectComponentAnalysis(swtmap, ccmap);
  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s in connectComponentAnalysis" << endl;

  start_time = clock();
  identifyLetters(swtmap, ccmap); //!
  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s in identifyLetters" << endl;

  start_time = clock();
  groupLetters(swtmap, ccmap);
  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s in groupLetters" << endl;

  start_time = clock();
  chainPairs(ccmap);
  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s in chainPairs" << endl;

  start_time = clock();
  //findRotationangles(blackWhite);
  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s in findRotationsangles" << endl;

  /*showEdgeMap();
   showSwtmap(swtmap);
   showCcmap(ccmap);
   showLetterGroup();
   */
  disposal();
  cout << "finish clean up" << endl;

}

void DetectText::strokeWidthTransform(const Mat& image, Mat& swtmap, int searchDirection)
{
  if (firstPass_)
  {
    // compute edge map
    Canny(image_, edgemap_, 50, 120);

    // compute partial derivatives
    Mat dx, dy;
    Sobel(image_, dx, CV_32FC1, 1, 0, 3);
    Sobel(image_, dy, CV_32FC1, 0, 1, 3);

    theta_ = Mat(image_.size(), CV_32FC1);

    if (edgepoints_.size())
    {
      edgepoints_.clear();
    }

    for (int y = 0; y < edgemap_.rows; y++)
    {
      for (int x = 0; x < edgemap_.cols; x++)
      {
        if (edgemap_.at<unsigned char> (y, x) == 255) // In case (x,y) is an edge
        {
          theta_.at<float> (y, x) = atan2(dy.at<float> (y, x), dx.at<float> (y, x)); //rise = arctan dy/dx
          edgepoints_.push_back(Point(x, y)); //Save edge as point in edgepoints
        }
      }
    }
  }
  // Second Pass (SWT is not performed again):
  vector<Point> strokePoints;
  updateStrokeWidth(swtmap, edgepoints_, strokePoints, searchDirection, UPDATE);
  updateStrokeWidth(swtmap, strokePoints, strokePoints, searchDirection, REFINE);
}

void DetectText::updateStrokeWidth(Mat& swtmap, vector<Point>& startPoints, vector<Point>& strokePoints,
                                   int searchDirection, Purpose purpose)
{
  //loop through all edgepoints, compute stroke width
  //startPoints = edgepoints_
  vector<Point>::iterator itr = startPoints.begin();

  vector<Point> pointStack;
  vector<float> SwtValues;

  for (; itr != startPoints.end(); ++itr)
  {
    pointStack.clear();
    SwtValues.clear();
    float step = 1;
    float iy = (*itr).y;
    float ix = (*itr).x;
    float currY = iy;
    float currX = ix;
    bool isStroke = false;
    float iTheta = theta_.at<float> (*itr);

    pointStack.push_back(Point(currX, currY));
    SwtValues.push_back(swtmap.at<float> (currY, currX));
    while (step < maxStrokeWidth_)
    {
      //going one pixel in the direction of the gradient to check if next pixel is an edge too
      float nextY = round(iy + sin(iTheta) * searchDirection * step);
      float nextX = round(ix + cos(iTheta) * searchDirection * step);

      if (nextY < 0 || nextX < 0 || nextY >= edgemap_.rows || nextX >= edgemap_.cols)
        break;

      step = step + 1;

      if (currY == nextY && currX == nextX)
        continue;

      currY = nextY;
      currX = nextX;

      pointStack.push_back(Point(currX, currY));
      SwtValues.push_back(swtmap.at<float> (currY, currX));
      if (edgemap_.at<unsigned char> (currY, currX) == 255)
      {
        float jTheta = theta_.at<float> (currY, currX);
        //if opposite point of stroke is found with roughly opposite gradient Theta: ...
        if (abs(abs(iTheta - jTheta) - 3.14) < 3.14 / 2)
        {
          isStroke = true;
          if (purpose == UPDATE)
          {
            strokePoints.push_back(Point(ix, iy));
          }
        }
        break;
      }
    }

    // ... then calculate newSwtVal for all Points between the two stroke points
    if (isStroke)
    {
      float newSwtVal;
      if (purpose == UPDATE)// update swt based on dist between edges
      {
        newSwtVal = sqrt((currY - iy) * (currY - iy) + (currX - ix) * (currX - ix));
      }
      else if (purpose == REFINE) // refine swt based on median
      {
        nth_element(SwtValues.begin(), SwtValues.begin() + SwtValues.size() / 2, SwtValues.end());
        newSwtVal = SwtValues[SwtValues.size() / 2];
      }
      // set all Points between to the newSwtVal except they are smaller because of another stroke
      for (size_t i = 0; i < pointStack.size(); i++)
      {
        swtmap.at<float> (pointStack[i]) = min(swtmap.at<float> (pointStack[i]), newSwtVal);
        //cout << "swtmap.at<float> (" << pointStack[i] << ") = min(" << swtmap.at<float> (pointStack[i]) << ", " << newSwtVal << ");" << endl;
      }
    }

  }// end loop through edge points

  // set initial upchanged value back to 0

  for (int y = 0; y < swtmap.rows; y++)
  {
    for (int x = 0; x < swtmap.cols; x++)
    {
      if (swtmap.at<float> (y, x) == initialStrokeWidth_)
      {
        swtmap.at<float> (y, x) = 0;
      }
    }
  }

}

int DetectText::connectComponentAnalysis(const Mat& swtmap, Mat& ccmap)
{

  int ccmapInitialVal = ccmap.at<float> (0, 0);
  int offsetY[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  int offsetX[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  int nNeighbors = 8;
  int label = 0;

  int vectorSize = ccmap.rows * ccmap.cols;

  int *pStack = new int[vectorSize * 2];
  int stackPointer;

  int *pVector = new int[vectorSize * 2];
  int vectorPointer;

  int currentPointX;
  int currentPointY;

  for (int y = 0; y < ccmap.rows; y++)
  {
    for (int x = 0; x < ccmap.cols; x++)
    {
      bool connected = false;
      if (ccmap.at<float> (y, x) == ccmapInitialVal)
      {
        vectorPointer = 0;
        stackPointer = 0;
        pStack[stackPointer] = x;
        pStack[stackPointer + 1] = y;

        while (stackPointer >= 0)
        {
          currentPointX = pStack[stackPointer];
          currentPointY = pStack[stackPointer + 1];
          stackPointer -= 2;

          pVector[vectorPointer] = currentPointX;
          pVector[vectorPointer + 1] = currentPointY;
          vectorPointer += 2;
          //check which one of the neighbors have similiar sw and then label the regions belonging together
          for (int i = 0; i < nNeighbors; i++)
          {
            int ny = currentPointY + offsetY[i];
            int nx = currentPointX + offsetX[i];

            if (ny < 0 || nx < 0 || ny >= ccmap.rows || nx >= ccmap.cols)
              continue;

            if (swtmap.at<float> (ny, nx) == 0)
            {
              ccmap.at<float> (ny, nx) = -2;
              continue;
            }

            if (ccmap.at<float> (ny, nx) == ccmapInitialVal)
            {
              float sw1 = swtmap.at<float> (ny, nx);
              float sw2 = swtmap.at<float> (y, x);

              if (max(sw1, sw2) / min(sw1, sw2) <= 3)
              {
                ccmap.at<float> (ny, nx) = label;
                stackPointer += 2;
                pStack[stackPointer] = nx;
                pStack[stackPointer + 1] = ny;
                connected = true;

              }
            }
          }// loop through neighbors
        }

        if (connected)
        {
          //	assert(vectorPointer <= vectorSize);
          //	assert(vectorPointer > 0);

          int minY = ccmap.rows, minX = ccmap.cols, maxY = 0, maxX = 0;
          int width, height;
          for (int i = 0; i < vectorPointer; i += 2)
          {
            // ROI for each component
            minY = min(minY, pVector[i + 1]);
            minX = min(minX, pVector[i]);
            maxY = max(maxY, pVector[i + 1]);
            maxX = max(maxX, pVector[i]);
          }
          width = maxX - minX + 1; // width = 1
          height = maxY - minY + 1; // height = 1
          Rect letterRoi(minX, minY, width, height);
          componentsRoi_.push_back(letterRoi);
          label++;
        }
        else
        {
          ccmap.at<float> (y, x) = -2;
        }
      }
    }// loop through ccmap
  }
  delete[] pStack;
  delete[] pVector;
  return label;
}

void DetectText::identifyLetters(const Mat& swtmap, const Mat& ccmap)
{

  int showCcmap = 0;
  Mat output = originalImage_.clone();

  assert(static_cast<size_t>(nComponent_) == componentsRoi_.size());
  isLetterComponects_ = new bool[nComponent_];
  vector<float> iComponentStrokeWidth;
  cout << nComponent_ << "componets" << endl;
  bool *innerComponents = new bool[nComponent_];

  for (size_t i = 0; i < nComponent_; i++)
  {
    float maxStrokeWidth = 0;
    float sumStrokeWidth = 0;
    float currentStrokeWidth;
    bool isLetter = true;

    Rect *itr = &componentsRoi_[i];

    // is height enough for being a letter?
    if (itr->height > maxLetterHeight_ || itr->height < minLetterHeight_)
    {
      isLetterComponects_[i] = false;
      continue;
    }

    float maxY = itr->y + itr->height;
    float minY = itr->y;
    float maxX = itr->x + itr->width;
    float minX = itr->x;
    float increment = abs(itr->width - itr->height) / 2;

    // reset the inner components
    memset(innerComponents, 0, nComponent_ * sizeof(bool));

    if (itr->width > itr->height) // increase box height

    {
      maxY = min(maxY + increment, static_cast<float> (ccmap.rows));
      minY = max(minY - increment, static_cast<float> (0.0));
    }
    else // increase box width

    {
      maxX = min(maxX + increment, static_cast<float> (ccmap.cols));
      minX = max(minX - increment, static_cast<float> (0.0));
    }

    for (int y = minY; y < maxY; y++)
      for (int x = minX; x < maxX; x++)
      {
        int component = static_cast<int> (ccmap.at<float> (y, x)); //ccmap-Label = -2 in case no Region; 0,1,2,3... for every region
        if (component == static_cast<int> (i))

        {
          currentStrokeWidth = swtmap.at<float> (y, x);
          iComponentStrokeWidth.push_back(currentStrokeWidth);
          maxStrokeWidth = max(maxStrokeWidth, currentStrokeWidth);
          sumStrokeWidth += currentStrokeWidth;
        }
        else if (component >= 0)
          innerComponents[component] = true;
      }

    float pixelCount = static_cast<float> (iComponentStrokeWidth.size());
    float mean = sumStrokeWidth / pixelCount;
    float variance = 0;

    for (size_t ii = 0; ii < pixelCount; ii++)
    {
      variance += (iComponentStrokeWidth[ii] - mean, 2) * (iComponentStrokeWidth[ii] - mean, 2); // variance += (SW[i]-mean)²;
    }
    variance = variance / pixelCount;

    // rules & parameters goes here:

    isLetter = isLetter && (variance / mean < 1.5); //Variance has to be small, so that for example leaves can be recognized as no letters

    isLetter = isLetter && (sqrt(((itr->width) * (itr->width) + (itr->height) * (itr->height))) / maxStrokeWidth < 10);

    // additional rules:
    isLetter = isLetter && (pixelCount / maxStrokeWidth > 5);

    isLetter = isLetter && (itr->width < 2.5 * itr->height);

    if (countInnerLetterCandidates(innerComponents) - 1 > 5)
    {
      isLetter = false;
    }
    isLetterComponects_[i] = isLetter;

    iComponentStrokeWidth.clear();
  }

  if (showCcmap == 1)
  {
    for (size_t i = 0; i < nComponent_; i++)
    {
      if (isLetterComponects_[i] == true)
      {
        rectangle(output, Point(componentsRoi_[i].x, componentsRoi_[i].y), Point(componentsRoi_[i].x
            + componentsRoi_[i].width, componentsRoi_[i].y + componentsRoi_[i].height), cvScalar((150), (110), (50)), 1);
      }
      if (fontColor_ == 1)
        cv::imshow("identify bright letters=ccmap[after]", output);
      else
        cv::imshow("identify dark letters=ccmap[after]", output);
    }
    waitKey(0);
  }
  delete[] innerComponents;
}

void DetectText::groupLetters(const Mat& swtmap, const Mat& ccmap)
{
  int showGroupedLetters = 0;
  componentsMeanIntensity_ = new float[nComponent_];
  componentsMedianStrokeWidth_ = new float[nComponent_];
  isGrouped_ = new bool[nComponent_];
  memset(componentsMeanIntensity_, 0, nComponent_ * sizeof(float));
  memset(componentsMedianStrokeWidth_, 0, nComponent_ * sizeof(float));
  memset(isGrouped_, false, nComponent_ * sizeof(bool));

  Mat output = originalImage_.clone();

  for (size_t i = 0; i < nComponent_; i++)
  {
    if (!isLetterComponects_[i]) //all in groupLetters as not-letters recognized componects ignored
      continue;

    Rect iRect = componentsRoi_[i];

    float iMeanIntensity = getMeanIntensity(ccmap, iRect, static_cast<int> (i));
    float iMedianStrokeWidth = getMedianStrokeWidth(ccmap, swtmap, iRect, static_cast<int> (i));

    for (size_t j = i + 1; j < nComponent_; j++)
    {
      if (!isLetterComponects_[j])
        continue;

      Rect jRect = componentsRoi_[j];

      // check if horizontal or vertical
      bool horizontal = (iRect.y < jRect.y + jRect.height && jRect.y < iRect.y + iRect.height);
      bool vertical = (iRect.x < jRect.x + jRect.width && jRect.x < iRect.x + iRect.width);

      if ((!horizontal) && (!vertical))
        continue;

      // if there is a tie between horizontal/vertical
      if (horizontal && vertical)
      {
        if (abs((iRect.x + iRect.width / 2) - (jRect.x + jRect.width / 2)) >= abs((iRect.y + iRect.height / 2)
            - (jRect.y + jRect.height / 2)))
        {
          horizontal = true;
          vertical = false;
        }
        else
        {
          horizontal = false;
          vertical = true;
        }
      }

      // rule 3: distance between characters
      float distance = sqrt((iRect.x + iRect.width / 2 - jRect.x - jRect.width / 2) * (iRect.x + iRect.width / 2
          - jRect.x - jRect.width / 2) + (iRect.y + iRect.height / 2 - jRect.y - jRect.height / 2) * (iRect.y
          + iRect.height / 2 - jRect.y - jRect.height / 2));
      int distanceRatio = 4; //4
      if (horizontal)
      {
        if (distance / max(iRect.width, jRect.width) > distanceRatio)
          continue;
      }
      else
      {
        if (distance / max(iRect.height, jRect.height) > distanceRatio)
          continue;
      }

      float jMeanIntensity = getMeanIntensity(ccmap, jRect, static_cast<int> (j));
      float jMedianStrokeWidth = getMedianStrokeWidth(ccmap, swtmap, jRect, static_cast<int> (j));

      bool isGroup = true;

      //!!! without rule 1 and 3, on the second picture more is recognized
      // rule 1: median of stroke width ratio
      isGroup = isGroup && (max(iMedianStrokeWidth, jMedianStrokeWidth) / min(iMedianStrokeWidth, jMedianStrokeWidth))
          < 2;
      // rule 2: height ratio
      isGroup = isGroup && (max(iRect.height, jRect.height) / min(iRect.height, jRect.height)) < 2;
      // rule 4: average color of letters
      isGroup = isGroup && abs(iMeanIntensity - jMeanIntensity) < 10;

      if (isGroup)
      {
        isGrouped_[i] = true;
        isGrouped_[j] = true;

        if (horizontal)
        {
          horizontalLetterGroups_.push_back(Pair(i, j));
        }

        //if (vertical)
        //{
        //  verticalLetterGroups_.push_back(Pair(i, j));
        // verticalLetterGroups are never used again, so the Pairs should be pushed into horizonzalLetterGroups instead
        //horizontalLetterGroups_.push_back(Pair(i, j));
        //}

        if (showGroupedLetters == 1)
        {
          rectangle(output, Point(iRect.x, iRect.y), Point(iRect.x + iRect.width, iRect.y + iRect.height),
                    cvScalar(0, 0, 255), 2);
          rectangle(output, Point(jRect.x, jRect.y), Point(jRect.x + jRect.width, jRect.y + jRect.height),
                    cvScalar(0, 0, 255), 2);
        }

      }
    }// end for loop j
  }// end for loop i

  if (showGroupedLetters == 1)
  {
    if (firstPass_)
      cv::imshow("bright", output);
    else
      cv::imshow("dark", output);
    waitKey(0);
  }
}

void DetectText::chainPairs(Mat& ccmap)
{
  int showpairs = 0;
  int showchains = 0;
  if (showpairs == 1)
  {
    Mat output = originalImage_.clone();
    for (int i = 0; i < horizontalLetterGroups_.size() - 1; i++)
    {
      rectangle(output, Point(componentsRoi_.at(horizontalLetterGroups_[i].left).x,
                              componentsRoi_.at(horizontalLetterGroups_[i].left).y),
                Point(componentsRoi_.at(horizontalLetterGroups_[i].left).x
                    + componentsRoi_.at(horizontalLetterGroups_[i].left).width,
                      componentsRoi_.at(horizontalLetterGroups_[i].left).y
                          + componentsRoi_.at(horizontalLetterGroups_[i].left).height), cvScalar(255, 255, 255), 2);
      rectangle(output, Point(componentsRoi_.at(horizontalLetterGroups_[i].right).x,
                              componentsRoi_.at(horizontalLetterGroups_[i].right).y),
                Point(componentsRoi_.at(horizontalLetterGroups_[i].right).x
                    + componentsRoi_.at(horizontalLetterGroups_[i].right).width,
                      componentsRoi_.at(horizontalLetterGroups_[i].right).y
                          + componentsRoi_.at(horizontalLetterGroups_[i].right).height), cvScalar(255, 255, 255), 2);
    }
    cv::imshow("pairs", output);
    waitKey(0);
  }

  mergePairs(horizontalLetterGroups_, horizontalChains_);

  // horizontalChains
  vector<Rect> initialHorizontalBoxes;
  chainToBox(horizontalChains_, initialHorizontalBoxes); //initialHorizontalBoxes contains rects for every chaincomponent with more than to components(letters)

  if (showchains == 1)
  {
    Mat output = originalImage_.clone();
    for (int i = 0; i < initialHorizontalBoxes.size(); i++)
    {
      rectangle(output, Point(initialHorizontalBoxes[i].x, initialHorizontalBoxes[i].y),
                Point(initialHorizontalBoxes[i].x + initialHorizontalBoxes[i].width, initialHorizontalBoxes[i].y
                    + initialHorizontalBoxes[i].height), cvScalar(155, 55, 255), 2);
    }
    cv::imshow("chains", output);
    waitKey(0);
  }

  filterBoundingBoxes(initialHorizontalBoxes, ccmap, 4);

  // boundingBoxes_ are filled with initialHorizontalBoxes:
  boundingBoxes_.insert(boundingBoxes_.end(), initialHorizontalBoxes.begin(), initialHorizontalBoxes.end());

  if (showchains == 1)
  {
    Mat output = originalImage_.clone();
    for (int i = 0; i < boundingBoxes_.size(); i++)
    {
      rectangle(output, Point(boundingBoxes_[i].x, boundingBoxes_[i].y), Point(boundingBoxes_[i].x
          + boundingBoxes_[i].width, boundingBoxes_[i].y + boundingBoxes_[i].height), cvScalar(0, 55, 105), 2);
    }
    cv::imshow("chains2", output);
    waitKey(0);
  }
}

void DetectText::findRotationangles(int blackWhite)
{
  int showHistogram = 1;
  int showRects = 0;
  int padding = 10;

  bgr whiteClr;
  whiteClr.r = 255;
  whiteClr.g = 255;
  whiteClr.b = 255;

  // Gradient Histogram
  //*****************************************************************************************************

  cv::Mat canvas;
  canvas.create(125, 360, CV_8UC3);
  int maxValue = 0, maxAngle = 0, angles = 360;
  int hist[angles], newHistogram[angles];
  double scale;

  for (unsigned int i = 0; i < boundingBoxes_.size(); i++)
  {
    // Show boundingBoxes if necessary
    if (showRects)
    {
      cv::Mat output = originalImage_.clone();
      cv::rectangle(output, cv::Point(boundingBoxes_.at(i).x, boundingBoxes_.at(i).y), cv::Point(boundingBoxes_.at(i).x
          + boundingBoxes_.at(i).width, boundingBoxes_.at(i).y + boundingBoxes_.at(i).height), cvScalar(250, 210, 150),
                    2);
      cv::imshow("right rectangles", output);
      cv::waitKey(0);
    }

    // Reset Values
    maxValue = 0;
    maxAngle = 0;

    for (int y = 0; y < canvas.rows; y++)
      for (int x = 0; x < canvas.cols; x++)
      {
        canvas.at<bgr> (y, x) = whiteClr;
      }

    for (int j = 0; j < angles - 1; j++)
      hist[j] = 0;

    // If there is an edge at (y,x),
    // get the angle[-180,180] at (y,x) and add 180 degrees. (because histogram range is [0,360])
    // Then add one point in the histogram at the found angle.
    for (int y = boundingBoxes_[i].y; y < boundingBoxes_[i].y + boundingBoxes_[i].height; y++)
    {
      for (int x = boundingBoxes_[i].x; x < boundingBoxes_[i].x + boundingBoxes_[i].width; x++)
      {
        if (edgemap_.at<unsigned char> (y, x) == 255)
        {
          int angle = (int)((180 / 3.141592) * theta_.at<float> (y, x)) + 180;
          hist[angle]++;
        }
      }
    }

    // Smoothing the histogram
    double mask[3];
    mask[0] = 0.25;
    mask[1] = 0.5;
    mask[2] = 0.25;

    for (int bin = 1; bin < 359; bin++)
    {
      double smoothedValue = 0;
      for (int i = 0; i < 3; i++)
      {
        smoothedValue += hist[bin - 1 + i] * mask[i];
      }
      newHistogram[bin] = smoothedValue;
    }

    newHistogram[0] = hist[0] * (2 / 3) + hist[1] * (1 / 3);
    newHistogram[359] = hist[358] * (1 / 3) + hist[359] * (2 / 3);

    for (int bin = 1; bin < 360; bin++)
    {
      hist[bin] = newHistogram[bin];
    }

    // Get maxValue and max angle
    for (int j = 0; j < angles - 1; j++)
      maxValue = hist[j] > maxValue ? hist[j] : maxValue;

    for (int j = 0; j < angles - 1; j++)
      if (maxValue == hist[j])
        maxAngle = j;

    // Fit histogram to the canvas height
    scale = maxValue > canvas.rows ? (double)canvas.rows / maxValue : 1.;

    //Draw histogram
    if (showHistogram)
    {
      for (int j = 0; j < angles - 1; j++)
      {
        cv::Point pt1(j, canvas.rows - (hist[j] * scale));
        cv::Point pt2(j, canvas.rows);
        if (j == maxAngle)
          cv::line(canvas, pt1, pt2, cv::Scalar(200, 160, 100), 2, 8, 0);
        else
          cv::line(canvas, pt1, pt2, cv::Scalar(250, 210, 150), 1, 8, 0);
      }
      cv::putText(canvas, "0", cv::Point(180, 122), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0, 0), 1.5, 8,
                  false);
      cv::putText(canvas, "-90", cv::Point(90, 122), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0, 0), 1.5, 8,
                  false);
      cv::putText(canvas, "-180", cv::Point(0, 122), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0, 0), 1.5, 8,
                  false);
      cv::putText(canvas, "90", cv::Point(260, 122), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0, 0), 1.5, 8,
                  false);
      cv::putText(canvas, "180", cv::Point(335, 122), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0, 0), 1.5, 8,
                  false);
      cv::imshow("Gradients", canvas);
      std::cout << "blackWhite:" << blackWhite << endl;
      std::cout << "maxAngle:" << maxAngle << "(" << maxAngle - 180 << ")" << endl;
      //cv::waitKey(0);
    }

    // Rotationangles
    //*****************************************************************************************************

    cv::Rect rectWithPadding;
    rectWithPadding.x = max(boundingBoxes_[i].x - padding, 0);
    rectWithPadding.y = max(boundingBoxes_[i].y - padding, 0);
    boundingBoxes_[i].y + boundingBoxes_[i].height > image_.rows ? rectWithPadding.height = image_.rows
        : rectWithPadding.height = boundingBoxes_[i].height + padding;
    boundingBoxes_[i].x + boundingBoxes_[i].width > image_.cols ? rectWithPadding.width = image_.cols
        : rectWithPadding.width = boundingBoxes_[i].width + padding;
    cv::Mat smallImg = image_(rectWithPadding);

    // Average of whole image
    cv::Scalar averageColor = mean(smallImg);

    // Average of background Pixels

    std::vector<int> bgColor;
    for (int y = 0; y < smallImg.rows; y++)
    {
      for (int x = 0; x < smallImg.cols; x++)
      {
        if (blackWhite == 1) //bright text
        {
          if ((unsigned int)smallImg.at<unsigned char> (y, x) > (unsigned int)averageColor.val[0])
          {
            bgColor.push_back((unsigned int)smallImg.at<unsigned char> (y, x));
          }
        }
        else //dark text
        {
          if ((unsigned int)smallImg.at<unsigned char> (y, x) < (unsigned int)averageColor.val[0])
          {
            bgColor.push_back((unsigned int)smallImg.at<unsigned char> (y, x));
          }
        }
      }
    }
    int average_bg = 0;
    for (int i = 0; i < bgColor.size(); i++)
    {
      average_bg += bgColor[i];
    }
    average_bg = average_bg / bgColor.size();

    // Rotation
    //*****************************************************************************************************

    cv::Point2f center;
    center.x = smallImg.cols * 0.5;
    center.y = smallImg.rows * 0.5;

    cv::Mat mapMatrix = cv::getRotationMatrix2D(center, maxAngle - 180, 1.0);
    cv::Mat rotatedImage;
    rotatedImage.create(smallImg.rows, smallImg.cols, CV_8UC1);

    for (int y = 0; y < rotatedImage.rows; y++)
      for (int x = 0; x < rotatedImage.cols; x++)
      {
        rotatedImage.at<unsigned char> (y, x) = average_bg;
      }

    cv::warpAffine(smallImg, rotatedImage, mapMatrix, smallImg.size(), INTER_CUBIC, BORDER_CONSTANT,
                   cv::Scalar(average_bg));
    cv::imshow("rotated", rotatedImage);
    //cv::waitKey(0);

    Rotated * r = new Rotated(rotatedImage, cv::Rect(boundingBoxes_[i].x, boundingBoxes_[i].y, boundingBoxes_[i].width,
                                                     boundingBoxes_[i].height));
    rotated.push_back(*r);
  }
}

void DetectText::chainToBox(vector<vector<int> >& chain, vector<Rect>& boundingBox)
{
  for (size_t i = 0; i < chain.size(); i++)
  {
    if (chain[i].size() < 3) //Only words with more than 2 letters
    {
      continue;
    }

    int minX = image_.cols, minY = image_.rows, maxX = 0, maxY = 0;
    int letterAreaSum = 0;
    int padding = 5;

    for (size_t j = 0; j < chain[i].size(); j++)
    {
      Rect *itr = &componentsRoi_[chain[i][j]];
      letterAreaSum += itr->width * itr->height;
      minX = min(minX, itr->x);
      minY = min(minY, itr->y);
      maxX = max(maxX, itr->x + itr->width);
      maxY = max(maxY, itr->y + itr->height);
    }

    // add padding around each box
    minX = max(0, minX - padding);
    minY = max(0, minY - padding);
    maxX = min(image_.cols, maxX + padding);
    maxY = min(image_.rows, maxY + padding);

    boundingBox.push_back(Rect(minX, minY, maxX - minX, maxY - minY));
  }
}

bool DetectText::spaticalOrder(Rect a, Rect b)
{
  return a.y < b.y;
}

void DetectText::filterBoundingBoxes(vector<Rect>& boundingBoxes, Mat& ccmap, int rejectRatio)
{
  vector<Rect> qualifiedBoxes;
  vector<int> components;

  for (size_t i = 0; i < boundingBoxes.size(); i++)
  {
    int isLetterCount = 0;
    int letterArea = 0;
    int nonLetterArea = 0;
    Rect *rect = &boundingBoxes[i];

    float width = static_cast<float> (rect->width);
    float height = static_cast<float> (rect->height);
    if (width < 20) // Only these rects/words with width > 20 -> maybe to be changed when rotated
      continue;
    if (max(width, height) / min(width, height) > 20) // maybe to be changed
      continue;

    for (int y = rect->y; y < rect->y + rect->height; y++)
      for (int x = rect->x; x < rect->x + rect->width; x++)
      {
        int componetIndex = static_cast<int> (ccmap.at<float> (y, x));

        if (componetIndex < 0) //padding has no label
          continue;

        if (isLetterComponects_[componetIndex])
          letterArea++;
        else
          nonLetterArea++;

        if (find(components.begin(), components.end(), componetIndex) == components.end())
        {
          components.push_back(componetIndex);
          if (isLetterComponects_[componetIndex])
            isLetterCount++;
        }
      }

    // accept patch with few noise inside
    if (letterArea > 3 * nonLetterArea || static_cast<int> (components.size()) < rejectRatio * isLetterCount)
    {
      qualifiedBoxes.push_back(*rect);
    }
    components.clear();
  }
  boundingBoxes = qualifiedBoxes;
}

void DetectText::overlapBoundingBoxes(vector<Rect>& boundingBoxes)
{
  vector<Rect> bigBoxes;
  // Merging BoundingBoxes
  Mat tempMap(image_.size(), CV_32FC1, Scalar(0));
  for (size_t i = 0; i < boundingBoxes.size(); i++)
  {
    Rect *rect = &boundingBoxes[i];
    for (int y = rect->y; y < rect->y + rect->height; y++)
      for (int x = rect->x; x < rect->x + rect->width; x++)
      {
        tempMap.at<float> (y, x) = 50;
      }
  }

  for (size_t i = 0; i < boundingBoxes.size(); i++)
  {
    if (tempMap.at<float> (boundingBoxes[i].y + 1, boundingBoxes[i].x + 1) != 50)
      continue;

    Rect rect;
    floodFill(tempMap, Point(boundingBoxes[i].x, boundingBoxes[i].y), i + 100, &rect);
    int padding = 5;

    // add padding around each box
    int minX = max(0, rect.x - padding);
    int minY = max(0, rect.y - padding);
    int maxX = min(image_.cols, rect.x + rect.width + padding);
    int maxY = min(image_.rows, rect.y + rect.height + padding);
    bigBoxes.push_back(Rect(minX, minY, maxX - minX, maxY - minY));
  }

  boundingBoxes = bigBoxes;

  cout << endl;
  cout << "888" << endl;
}

void DetectText::overlayText(vector<Rect>& box, vector<string>& text)
{
  assert(box.size() == text.size());
  Scalar color(0, 255, 0);
  size_t lineWidth = 25;
  int indent = 50;
  int count = 1;
  for (size_t i = 0; i < box.size(); i++)
  {
    if (count > 9)
      indent = 70;
    string output = text[i];
    if (output.compare("") == 0)
      continue;

    std::string s;
    std::stringstream out;
    out << count;
    count++;
    string prefix = "[";
    prefix = prefix + out.str() + "]";
    putText(detection_, prefix, Point(box[i].x + box[i].width, box[i].y + box[i].height), FONT_HERSHEY_DUPLEX, 1,
            color, 2);
    putText(detection_, prefix, Point(image_.cols, textDisplayOffset_ * 35), FONT_HERSHEY_DUPLEX, 1, color, 2);
    while (output.length() > lineWidth)
    {
      putText(detection_, output.substr(0, lineWidth), Point(image_.cols + indent, textDisplayOffset_ * 35),
              FONT_HERSHEY_DUPLEX, 1, color, 2);
      output = output.substr(lineWidth);
      textDisplayOffset_++;
    }
    putText(detection_, output, Point(image_.cols + indent, textDisplayOffset_ * 35), FONT_HERSHEY_DUPLEX, 1, color, 2);
    textDisplayOffset_ += 2;
  }
}

void DetectText::ocrRead(vector<Rect>& boundingBoxes)
{

  sort(boundingBoxes.begin(), boundingBoxes.end(), DetectText::spaticalOrder);
  for (size_t i = 0; i < boundingBoxes.size(); i++)
  {
    string result;
    float score = ocrRead(originalImage_(boundingBoxes[i]), result, -1);
    if (score > 0)
    {
      boxesBothSides_.push_back(boundingBoxes[i]);
      wordsBothSides_.push_back(result);
      boxesScores_.push_back(score);
    }
  }
  /*rotated
   for (size_t i = 0; i < rotated.size() - 1; i++)
   {
   string result;
   //cout << "ROTATION [" << i << "]: ";

   float score = ocrRead(rotated.at(i).rotated_img, result, i);
   if (score > 0)
   {
   boxesBothSides_.push_back(rotated.at(i).coords);
   wordsBothSides_.push_back(result);
   boxesScores_.push_back(score);
   }
   cout << "score:" << score << endl;
   imshow("actual image", rotated.at(i).rotated_img);
   //waitKey(0);
   }
   */
}

float DetectText::ocrRead(const Mat& imagePatch, string& output, int actual)
{
  float score = 0;
  Mat scaledImage;
  /*
   if (actual >= 0)
   {


   cvPoint

   cout << "imGray:" << imGray.cols << "x" << imGray.rows << endl;

   for (int y = 0; y < imGray.rows; y++)
   {
   for (int x = 0; x < imGray.cols; x++)
   {
   if ((unsigned int)imGray.at<unsigned char> (y, x) != 0)
   {
   a = x;
   b = y;
   color = (unsigned int)imGray.at<unsigned char> (y + 3, x + 3);
   x = imGray.cols;
   y = imGray.rows;
   cout << "[" << a << "|" << b << "||" << color << "]" << endl;
   }
   }
   cout << endl;
   }

   //getcolor
   floodFill(imGray, Point(1, 1), color, &rect);
   floodFill(imGray, Point(1, 1), 0, &rect);
   floodFill(imGray, Point(1, 1), color, &rect);
   //s = cvGet2D(img, yy,xx); // get the (i,j) pixel value
   //floodFill(tempMap, Point(1, 1), s, &rect);
   cv::imshow("OCR", imGray);
   waitKey(0);
   }*/

  if (imagePatch.rows < 30)
  {
    double scale = 1.5;
    resize(imagePatch, scaledImage, Size(0, 0), scale, scale, INTER_LANCZOS4);
    imwrite("patch.tiff", scaledImage);
  }
  else
  {
    imwrite("patch.tiff", imagePatch);
  }

  system("$(cp patch.tiff ~)");
  int result;
  result = system("$(rospack find tesseract)/bin/tesseract patch.tiff patch"); //OCR in patch
  cout << "result" << result << endl;
  assert(!result);
  ifstream fin("patch.txt");
  string str;
  while (fin >> str)
  {
    cout << "in ocrRead:" << endl;
    cout << "[" << str << "]" << endl;
    string tempOutput;
    score += spellCheck(str, tempOutput, 2);
    output += tempOutput;
  }
  result = system("$(rm patch.txt patch.tiff)");
  //waitKey(0);
  return score;
}

// two option: 1 for aspell, 2 for correlation edit distance
// return the score for the input
float DetectText::spellCheck(string& str, string& output, int method)
{ // Example
  int letterCount = 0, errorCount = 0, lNoiseCount = 0, digitCount = 0; //str=Day, output=[], method=2
  string withoutStrangeMarks;
  float score = 0;
  str = trim(str);

  cout << "in spellCheck" << endl;

  for (size_t i = 0; i < str.length(); i++)
  {
    if (isupper(str[i]) || islower(str[i]))
    {
      withoutStrangeMarks += str[i]; // withoutStrangeMarks = Day;
      letterCount++; //letterCount=3
      if (str[i] == 'l' || str[i] == 'L' || str[i] == 'I')
        lNoiseCount++;
    }
    else if (isdigit(str[i]))
    {
      digitCount++;
      //	withoutStrangeMarks += str[i];
    }
    else if (str[i] == '|' || str[i] == '/' || str[i] == '\\')
    {
      if ((i && isdigit(str[i - 1])) || ((i < str.length() - 1) && isdigit(str[i + 1]))) // is there a digit following or before the actual letter?
      {
        withoutStrangeMarks += '1'; //change to an '1'
        str[i] = '1';
        digitCount++;
      }
      else
      {
        withoutStrangeMarks += 'l'; //else think of the letter as a 'l'
        errorCount++;
        letterCount++;
      }
    }
    else if (str[i] == '[')
    {
      withoutStrangeMarks += 'L';
      errorCount++;
      letterCount++;
    }
    else if (str[i] == ']')
    {
      withoutStrangeMarks += 'I';
      errorCount++;
      letterCount++;
    }
    else
    {
      str[i] = ' ';
    }
  }

  //Maybe here Rotation 180° in every if and else if (and not in else):

  if (digitCount > 0 && letterCount == 0)
  {
    if (digitCount <= 5)
      output = str + " ";
  }
  else if (letterCount < 2)
  {
    if (result_ == FINE)
      output = str + " ";
  }
  else if ((errorCount + lNoiseCount) * 2 > letterCount)
  {
    // do nothing
  }
  else if (letterCount < static_cast<int> (str.length()) / 2)
  {
    // don't show up garbbige
  }
  else
  {
    if (method == 1)
    {
      const string command("echo " + withoutStrangeMarks + " | aspell -a >> output");
      int r = system(command.c_str());
      fstream fin("output");
      string result;
      int count = 0;

      while (fin >> result)
      {
        if (count)
        {
          count++;
          if (count >= 5)
          {
            output += result + " ";
          }
          if (count == 10)
          {
            if ((output)[output.length() - 2] == ',')
              ((output)[output.length() - 2] = ' ');
            break;
          }
        }
        if (result[0] == '&')
        {
          count++;
          output += "{";
        }
        else if (result[0] == '*')
        {
          output += " " + str;
          break;
        }
      }
      if (count)
        output += "}";
      r = system("rm output");
    }

    // dictionary search
    if (method == 2)
    {
      cout << "METHOD==2" << endl;
      vector<Word> topk;
      string nearestWord;
      getTopkWords(withoutStrangeMarks, 3, topk);
      if (result_ == COARSE)
      {
        string topWord = topk[0].word;
        output = topk[0].word + " ";

        if (topWord.length() < 3)
        {
          if (topk[0].score == 0)
            score++;
          else
            output = "";
        }
        else if (topWord.length() < 6)
        {
          if (topk[0].score * 5 <= topWord.length())
            score++;
          else
            output = "";
        }
        else
        {
          if (topk[0].score == 0)
            score = topWord.length() * 2;
          else if (topk[0].score <= topWord.length())
            score = topWord.length();
        }
      }
      else if (result_ == FINE)
      {
        if (topk[0].score == 0)
        {
          output = topk[0].word + " ";
          score += topk[0].word.length() * 2;
        }
        else
        {
          output = "{" + withoutStrangeMarks + "->";
          // pick top 3 results
          for (int i = 0; i < 3; i++)
          {
            stringstream ss;
            ss << topk[i].score;
            string s = ss.str();
            output = output + topk[i].word + ":" + s + " ";
          }
          output += "} ";
        }
      }
    }
  }

  return score;
}

Mat DetectText::filterPatch(const Mat& patch)
{
  Mat result;
  Mat element = getStructuringElement(MORPH_ELLIPSE, Size(patch.cols / 3, patch.rows / 3));
  morphologyEx(patch, result, MORPH_TOPHAT, element);
  return result;
}

void DetectText::disposal()
{
  delete[] isLetterComponects_;
  delete[] isGrouped_;
  delete[] componentsMeanIntensity_;
  delete[] componentsMedianStrokeWidth_;

  componentsRoi_.clear();
  innerComponents_.clear();
  horizontalLetterGroups_.clear();
  verticalLetterGroups_.clear();
  horizontalChains_.clear();
  verticalChains_.clear();
}

/********************* helper functions ***************************/
/*----------------------------------------- readLetterCorrelation - Correlation in correlation_ einlesen ----------------------------------*/
void DetectText::readLetterCorrelation(const char* file)
{
  ifstream fin(file);
  correlation_ = Mat(62, 62, CV_32F, Scalar(0));
  float number;
  for (int i = 0; i < 62; i++)
    for (int j = 0; j < 62; j++)
    {
      assert(fin >> number);
      correlation_.at<float> (i, j) = number;
    }
}

/*----------------------------------------- readWordList - Directory in wordList_ einlesen -----------------------------------------------*/
void DetectText::readWordList(const char* filename)
{
  ifstream fin(filename);
  string word;
  wordList_.clear();
  while (fin >> word)
  {
    wordList_.push_back(word);
  }
  assert(wordList_.size());
  cout << "read in " << wordList_.size() << " words from " << string(filename) << endl;
}

string& DetectText::trim(string& str)
{
  // Trim Both leading and trailing spaces

  // Find the first character position after
  // excluding leading blank spaces
  size_t startpos = str.find_first_not_of(" \t");
  // Find the first character position from reverse af
  size_t endpos = str.find_last_not_of(" \t");
  // if all spaces or empty return an empty string
  if ((string::npos == startpos) || (string::npos == endpos))
    str = "";
  else
    str = str.substr(startpos, endpos - startpos + 1);
  return str;
}

void DetectText::getNearestWord(const string& str, string& nearestWord)
{
  cout << "start searching match for " << str << endl;
  float score, lowestScore = 100;
  int referenceScore;
  size_t index = 0;
  for (size_t i = 0; i < wordList_.size(); ++i)
  {
    cout << "matching...." << wordList_[i];
    score = editDistanceFont(str, wordList_[i]);
    referenceScore = editDistance(str, wordList_[i]);
    cout << " " << score << " " << referenceScore << endl;
    if (score < lowestScore)
    {
      lowestScore = score;
      cout << "AHA! better!" << endl;
      index = i;
    }
  }
  nearestWord = wordList_[index];
  cout << nearestWord << " got the lowest score: " << lowestScore << endl;
}

void DetectText::getTopkWords(const string& str, const int k, vector<Word>& words) //k=3
{
  float score, lowestScore = 100;
  words.clear();
  words.resize(k);

  cout << "in getTopkWords with [" << str << "]" << endl;

  for (size_t i = 0; i < wordList_.size(); i++)
  {
    score = editDistanceFont(str, wordList_[i]); //compare every word in dictionary, score=0 -> perfect
    if (score < lowestScore)
    {
      Word w = Word(wordList_[i], score);
      lowestScore = insertToList(words, w);
    }
  }
  cout << "lowestScore:" << lowestScore << endl;
}

// return lowest score in the list
float DetectText::insertToList(vector<Word>& words, Word& word) // for example word = ("able",3.7)
{
  // first search for the position
  size_t index = 0;

  for (size_t i = 0; i < words.size(); i++)
  {
    index = i;
    if (word.score < words[i].score) //in case score of actual dictionaryword is smaller than any entry in words: break, remember index
    {
      break;
    }
  }
  if (index != words.size())
  {
    for (size_t i = words.size() - 1; i > index; i--)
    {
      words[i] = words[i - 1];
    }
    words[index] = word;
  }
  return words[words.size() - 1].score; // return last score, the smallest
}

/*--------------------------------------------------------*\
 *	display functions
 \*--------------------------------------------------------*/

void DetectText::showEdgeMap()
{
  if (firstPass_)
    imwrite("edgemap.png", edgemap_);
}

void DetectText::showCcmap(Mat& ccmap)
{

  Mat ccmapLetters = ccmap * (1.0 / static_cast<float> (nComponent_));
  for (size_t i = 0; i < nComponent_; ++i)
  {
    Rect *itr = &componentsRoi_[i];
    rectangle(ccmapLetters, Point(itr->x, itr->y), Point(itr->x + itr->width, itr->y + itr->height), Scalar(0.5));
  }
  if (firstPass_)
    imwrite("ccmap1.jpg", ccmapLetters * nComponent_);
  else
    imwrite("ccmap2.jpg", ccmapLetters * nComponent_);
}

void DetectText::showSwtmap(Mat& swtmap)
{
  if (firstPass_)
    imwrite("swtmap1.jpg", swtmap * 10);
  else
    imwrite("swtmap2.jpg", swtmap * 10);
}

void DetectText::showLetterDetection()
{
  Mat output = originalImage_.clone();
  Scalar scalar;
  if (firstPass_)
    scalar = Scalar(0, 255, 0);
  else
    scalar = Scalar(0, 0, 255);

  for (size_t i = 0; i < nComponent_; ++i)
  {
    if (isLetterComponects_[i])
    {
      Rect *itr = &componentsRoi_[i];
      rectangle(output, Point(itr->x, itr->y), Point(itr->x + itr->width, itr->y + itr->height), scalar, 2);
      stringstream ss;
      string s;
      ss << i;
      s = ss.str() + ".tiff";
      imwrite(s, originalImage_(*itr));
    }
  }
  if (firstPass_)
    imwrite(outputPrefix_ + "_letters1.jpg", output);
  else
    imwrite(outputPrefix_ + "_letters2.jpg", output);
}

void DetectText::showLetterGroup()
{
  Mat output = originalImage_.clone();
  Scalar scalar;
  if (firstPass_)
    scalar = Scalar(0, 255, 0);
  else
    scalar = Scalar(0, 0, 255);

  for (size_t i = 0; i < nComponent_; ++i)
  {
    if (isGrouped_[i])
    {
      Rect *itr = &componentsRoi_[i];
      rectangle(output, Point(itr->x, itr->y), Point(itr->x + itr->width, itr->y + itr->height), scalar, 2);
    }
  }
  if (firstPass_)
    imwrite(outputPrefix_ + "_group1.jpg", output);
  else
    imwrite(outputPrefix_ + "_group2.jpg", output);
}

void DetectText::showBoundingBoxes(vector<Rect>& boundingBoxes)
{
  Scalar scalar(0, 0, 255);

  for (size_t i = 0; i < boundingBoxes.size(); i++)
  {
    Rect *rect = &boundingBoxes[i];
    rectangle(detection_, Point(rect->x, rect->y), Point(rect->x + rect->width, rect->y + rect->height), scalar, 3);
  }
}

void DetectText::showBoundingBoxes(vector<Rect>& boundingBoxes, vector<bool>& boxInbox)
{
  assert(boundingBoxes.size() == boxInbox.size());
  Scalar scalar;
  scalar = Scalar(0, 0, 255);

  for (size_t i = 0; i < boundingBoxes.size(); i++)
  {
    if (boxInbox[i] == true)
      continue;
    Rect *rect = &boundingBoxes[i];
    rectangle(detection_, Point(rect->x, rect->y), Point(rect->x + rect->width, rect->y + rect->height), scalar, 3);
  }
}

inline int DetectText::countInnerLetterCandidates(bool* array)
{
  int count = 0;
  for (size_t i = 0; i < nComponent_; i++)
  {
    if (array[i] && isLetterComponects_[i])
    {
      count++;
    }
  }
  return count;
}

float DetectText::getMeanIntensity(const Mat& ccmap, const Rect& rect, int element)
{
  assert(element >= 0);
  if (componentsMeanIntensity_[element] == 0)
  {
    float sum = 0;
    float count = 0;
    float felement = static_cast<float> (element);
    for (int y = rect.y; y < rect.y + rect.height; y++)
      for (int x = rect.x; x < rect.x + rect.width; x++)
      {
        if (ccmap.at<float> (y, x) == felement)
        {
          sum += static_cast<float> (image_.at<unsigned char> (y, x));
          count = count + 1;
        }
      }
    componentsMeanIntensity_[element] = sum / count;
  }
  return componentsMeanIntensity_[element];
}

float DetectText::getMedianStrokeWidth(const Mat& ccmap, const Mat& swtmap, const Rect& rect, int element)
{

  assert(element >= 0);
  assert(isLetterComponects_[element]);
  if (componentsMedianStrokeWidth_[element] == 0)
  {
    vector<float> SwtValues;

    float felement = static_cast<float> (element);
    for (int y = rect.y; y < rect.y + rect.height; y++)
      for (int x = rect.x; x < rect.x + rect.width; x++)
      {
        if (ccmap.at<float> (y, x) == felement)
        {
          SwtValues.push_back(swtmap.at<float> (y, x));
        }
      }

    nth_element(SwtValues.begin(), SwtValues.begin() + SwtValues.size() / 2, SwtValues.end());

    componentsMedianStrokeWidth_[element] = SwtValues[SwtValues.size() / 2];

  }
  return componentsMedianStrokeWidth_[element];
}

void DetectText::mergePairs(const vector<Pair>& groups, vector<vector<int> >& chains)
{
  /* groups looks like this:
   *  4 5
   *  12 14
   *  44 45
   *   ...
   */
  vector<vector<int> > initialChains;
  initialChains.resize(groups.size());
  for (size_t i = 0; i < groups.size(); i++)
  {
    vector<int> temp;
    temp.push_back(groups[i].left);
    temp.push_back(groups[i].right);
    initialChains[i] = temp;
  }

  /* initialChains looks like this:
   * [0]  [1]  [2]
   *  4    12   44   ...
   *  5    14   45
   */

  while (mergePairs(initialChains, chains))
  {
    initialChains = chains;
    chains.clear();
  }
}

bool DetectText::mergePairs(const vector<vector<int> >& initialChains, vector<vector<int> >& chains)
{
  if (chains.size())
    chains.clear();

  bool merged = false;
  int *mergedToChainBitMap = new int[initialChains.size()];
  memset(mergedToChainBitMap, -1, initialChains.size() * sizeof(int));

  for (size_t i = 0; i < initialChains.size(); i++)
  {
    if (mergedToChainBitMap[i] != -1)
      continue;

    for (size_t j = i + 1; j < initialChains.size(); j++)
    {
      // match elements in chain i,j
      for (size_t ki = 0; ki < initialChains[i].size(); ki++)
      {
        for (size_t kj = 0; kj < initialChains[j].size(); kj++)
        {
          // found match
          if (initialChains[i][ki] == initialChains[j][kj]) // Does any other initialChains[x] contain a identical componect?
          {
            merged = true;
            // j already merged with others
            if (mergedToChainBitMap[j] != -1)
            {
              merge(initialChains[i], chains[mergedToChainBitMap[j]]);

              mergedToChainBitMap[i] = mergedToChainBitMap[j];
            }
            else // start a new chain
            {
              vector<int> newChain;
              merge(initialChains[i], newChain);
              merge(initialChains[j], newChain);
              chains.push_back(newChain);
              mergedToChainBitMap[i] = chains.size() - 1;
              mergedToChainBitMap[j] = chains.size() - 1;
            }
            break;
          }
        }
        if (mergedToChainBitMap[i] != -1 && mergedToChainBitMap[j] != -1)
          break;
      }
    }

    // comparing with all other chains, not found a match
    if (mergedToChainBitMap[i] == -1)
    {
      chains.push_back(initialChains[i]);
      mergedToChainBitMap[i] = chains.size() - 1;
    }

  }

  if (!merged)
  {
    chains = initialChains;
  }
  // dispose resourse
  delete[] mergedToChainBitMap;
  return merged;
}

void DetectText::merge(const vector<int>& token, vector<int>& chain)
{
  vector<int>::iterator it;
  for (size_t i = 0; i < token.size(); i++)
  {
    it = find(chain.begin(), chain.end(), token[i]);
    if (it == chain.end())
    {
      chain.push_back(token[i]);
    }
  }
}

// use correlation as indicator of distance
float DetectText::editDistanceFont(const string& s, const string& t) //s =word, t = dictionary compare word
{
  float penalty = 0.7;
  int n = s.length();
  int m = t.length();

  if (n == 0)
    return m;
  if (m == 0)
    return n;

  float **d = new float*[n + 1]; // float d [][str_laenge+1] = float[4][]
  for (int i = 0; i < n + 1; i++) // 4mal
  {
    d[i] = new float[m + 1];
    memset(d[i], 0, (m + 1) * sizeof(float)); // float d[4][5]
  }

  for (int i = 0; i < n + 1; i++)
    d[i][0] = i;
  /*d[0][0] = 0;
   d[1][0] = 1;
   d[2][0] = 2;
   d[3][0] = 3;*/
  for (int j = 0; j < m + 1; j++)
    d[0][j] = j;
  /*d[0][0] = 0;
   d[0][1] = 1;
   d[0][2] = 2;
   d[0][3] = 3;
   d[0][4] = 4;*/

  /* d:
   * 0 1 2 3
   * 1 0 0 0
   * 2 0 0 0
   * 3 0 0 0
   * 4 0 0 0
   */

  for (int i = 1; i < n + 1; i++)
  {
    char sc = s[i - 1]; // sc = 'D';
    for (int j = 1; j < m + 1; j++)
    {
      float v = d[i - 1][j - 1]; //v=d[0][0]=0;    ->   v=d[0][1]=1;
      if ((t[j - 1] != sc)) //        // t[0]='a' != 'D'  -> t[1]='b' != 'D';
      {
        // correlate = correlation_.at[0,29] = 0.21212; -> correlate=correlation_at[1,29] = 0.31865
        //!
        int a = getCorrelationIndex(t[j - 1]);
        int b = getCorrelationIndex(sc);
        if (a < 0)
        {
          cout << "Wort:" << t << endl;
        }
        float correlate = correlation_.at<float> (a, b);
        v = v + 1 - correlate; // v=0+1-0.2121=0.78  -> v = 0.78 + 1 - 0.3128 = 1.47
      }
      d[i][j] = min(min(d[i - 1][j] + penalty, d[i][j - 1] + penalty), v); //d[0][0]=min( min([0][1]+0.7, d[1][0]+0.7), v); -> d[0][2]=min(min[0][2]+0.7,[1][1]+0.7
    } //d[0][0]=min( min(1.7, 1.7), 0.78) = 0.78
  }
  float result = d[n][m];
  for (int i = 0; i < n + 1; i++)
    delete[] d[i];
  delete[] d;
  return result;
}

// get index in correlation matrix for given char
int DetectText::getCorrelationIndex(char letter)
{
  if (islower(letter))
  {
    return letter - 'a';
  }
  else if (isupper(letter))
  {
    return letter - 'A' + 26;
  }
  else if (isdigit(letter))
  {
    return letter - '0' + 52;
  }
  cout << "illigal letter: " << letter << endl;
  // assert(false);
  return -1;
}

// regular editDistance
int DetectText::editDistance(const string& s, const string& t)
{
  int n = s.length();
  int m = t.length();

  if (n == 0)
    return m;
  if (m == 0)
    return n;

  int **d = new int*[n + 1];
  for (int i = 0; i < n + 1; i++)
  {
    d[i] = new int[m + 1];
    memset(d[i], 0, (m + 1) * sizeof(int));
  }

  for (int i = 0; i < n + 1; i++)
    d[i][0] = i;
  for (int j = 0; j < m + 1; j++)
    d[0][j] = j;

  for (int i = 1; i < n + 1; i++)
  {
    char sc = s[i - 1];
    for (int j = 1; j < m + 1; j++)
    {
      int v = d[i - 1][j - 1];
      if (t[j - 1] != sc)
        v++;
      d[i][j] = min(min(d[i - 1][j] + 1, d[i][j - 1] + 1), v);
    }
  }
  return d[n][m];
}

/*------------- test functions-------------------*/
void DetectText::testGetCorrelationIndex()
{
  assert(getCorrelationIndex('a') == 0);
  assert(getCorrelationIndex('c') == 2);
  assert(getCorrelationIndex('A') == 26);
  assert(getCorrelationIndex('0') == 52);
  assert(getCorrelationIndex('9') == 61);
  cout << "pass getCorrelationIndex test" << endl;
}

void DetectText::testEditDistance()
{
  string a("hello");
  string b("helo");
  assert(editDistance(a,b)==1);
  string c("hello");
  string d("xello");
  cout << "distance betweeen " << c << " & " << d << ": " << editDistance(c, d) << endl;
  cout << "distance with font betweeen " << c << " & " << d << ":" << editDistanceFont(c, d) << endl;
}

void DetectText::testInsertToList()
{
  vector<Word> list;
  list.resize(10);

  for (int i = 0; i < 10; i++)
  {
    float score = rand() % 50;
    Word w = Word("", score);
    insertToList(list, w);
    for (size_t i = 0; i < 10; i++)
    {
      cout << list[i].score << " <= ";
    }
    cout << endl;
  }

}
void DetectText::testMergePairs()
{
  int a[] = {1, 2, 3};
  int b[] = {2, 3, 9};
  int c[] = {7, 5};
  int d[] = {2, 4, 6};

  vector<vector<int> > initialChain;
  vector<vector<int> > outputChain;
  vector<int> va(a, a + 3);
  vector<int> vb(b, b + 3);
  vector<int> vc(c, c + 2);
  vector<int> vd(d, d + 3);
  initialChain.push_back(va);
  initialChain.push_back(vb);
  initialChain.push_back(vc);
  initialChain.push_back(vd);

  while (mergePairs(initialChain, outputChain))
  {
    initialChain = outputChain;
    outputChain.clear();
  }

  for (size_t i = 0; i < outputChain.size(); i++)
  {
    for (size_t j = 0; j < outputChain[i].size(); j++)
    {
      cout << outputChain[i][j] << " ";
    }
    cout << endl;
  }

}

void DetectText::testEdgePoints(vector<Point>& edgepoints)
{
  Mat temp(edgemap_.size(), CV_8UC1);
  vector<Point>::iterator itr = edgepoints.begin();
  for (; itr != edgepoints.end(); ++itr)
  {
    temp.at<unsigned char> (*itr) = 255;
  }

  imshow("test edge", temp);
  waitKey();
}
