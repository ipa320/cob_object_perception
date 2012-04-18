/**
 * Implementation based on "Detecting Text in Natural Scenes with  
 * Stroke Width Transform", Boris Epshtein, Eyal Ofek, Yonatan Wexler
 * CVPR 2010
 *
 */

/** \author Menglong Zhu */

#include <read_text/text_detect.h>
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
      minLetterHeight_(0), textDisplayOffset_(1)
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

void DetectText::detect()
{
  // Head-Method
  //*****************************************************************************************************
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
  pipeline(1);
  cout << "Second pass" << endl;
  firstPass_ = false; // nur für SWT relevant, damit auch schwarzer Text auf weissen Grund erkannt wird
  pipeline(-1);

  cout << "size von rotated:" << rotated.size();
  //overlapBoundingBoxes(boundingBoxes_);
  ocrRead(boundingBoxes_);
  showBoundingBoxes(boxesBothSides_);
  overlayText(boxesBothSides_, wordsBothSides_);
  std::cout << "1\n";
  /*
   for (size_t i = 0; i < rotated.size(); i++)
   {
   string result;
   float score = ocrRead(rotated.at(i).rotated_img, result);
   if (score > 0)
   {
   cout << "CARMEN" << endl;
   boxesBothSides_.push_back(rotated.at(i).coords);
   wordsBothSides_.push_back(result);
   boxesScores_.push_back(score);
   }
   }

   /* vector<Rect> boundingBoxesRotated_;
   for (size_t i = 0; i < rotated.size(); i++)
   {
   boundingBoxesRotated_.push_back(rotated.at(i).coords);
   }

   cout << " boundingBoxesRotated_ vor overlap: " << boundingBoxesRotated_.size() << endl;

   overlapBoundingBoxes(boundingBoxesRotated_);

   cout << " boundingBoxesRotated_ nach overlap: " << boundingBoxesRotated_.size() << endl;

   char* window_title = "gradients2";
   IplImage *canvas;
   int angles = 360;
   int hist[angles];
   double scale;
   int max = 0, maxAngle = 0;
   canvas = cvCreateImage(cvSize(360, 125), IPL_DEPTH_8U, 3);
   cvSet(canvas, CV_RGB(255,255,255), NULL);
   IplImage t = image_;
   cout << "111" << endl;
   for (int i = 0; i < boundingBoxesRotated_.size(); i++)
   {
   cout << "222" << endl;
   for (int j = 0; j < angles - 1; j++)
   hist[j] = 0;
   max = 0;
   maxAngle = 0;

   for (int y = boundingBoxesRotated_.at(i).y; y < boundingBoxesRotated_.at(i).y + boundingBoxesRotated_.at(i).height; y++)
   {
   for (int x = boundingBoxesRotated_.at(i).x; x < boundingBoxesRotated_.at(i).x + boundingBoxesRotated_.at(i).width; x++)
   {
   if (edgemap_.at<unsigned char> (y, x) == 255) // Falls bei (x,y) eine Kante ist
   {
   int a = (int)((180 / 3.141592) * theta_.at<float> (y, x)) + 180;
   hist[a]++;
   }
   }
   }

   cout << "333" << endl;

   for (int j = 0; j < angles - 1; j++)
   max = hist[j] > max ? hist[j] : max;

   for (int j = 0; j < angles - 1; j++)
   if (max == hist[j])
   maxAngle = j;

   cout << "444" << endl;

   // Get scale so the histogram fit the canvas height
   scale = max > canvas->height ? (double)canvas->height / max : 1.;

   //Draw histogram
   for (int j = 0; j < angles - 1; j++)
   {
   CvPoint pt1 = cvPoint(j, canvas->height - (hist[j] * scale));
   CvPoint pt2 = cvPoint(j, canvas->height);
   cvLine(canvas, pt1, pt2, CV_RGB(0,0,0), 1, 8, 0);
   }

   cout << "555" << endl;
   CvFont font;
   cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.3, 0.3, 0, 1, CV_AA);
   cvPutText(canvas, "0", cvPoint(180, 122), &font, cvScalar(140, 160, 200, 0));
   cvPutText(canvas, "-90", cvPoint(90, 122), &font, cvScalar(140, 160, 200, 0));
   cvPutText(canvas, "-180", cvPoint(0, 122), &font, cvScalar(140, 160, 200, 0));
   cvPutText(canvas, "90", cvPoint(260, 122), &font, cvScalar(140, 160, 200, 0));
   cvPutText(canvas, "180", cvPoint(340, 122), &font, cvScalar(140, 160, 200, 0));
   cout << "666" << endl;
   // cvShowImage("gradients2", canvas);

   // rectangle(image_, Point(boundingBoxesRotated_.at(i).x, boundingBoxesRotated_.at(i).y), Point(boundingBoxesRotated_.at(i).x
   //     + boundingBoxesRotated_.at(i).width, boundingBoxesRotated_.at(i).y + boundingBoxesRotated_.at(i).height), 150, 3);
   // cv::imshow("...", image_);

   cout << "max at:" << maxAngle - 180 << endl;
   if (maxAngle - 180 > 10 || maxAngle - 180 < -10)
   { // Start Rotation
   cout << "rotation:" << endl;
   IplImage *rotatedImage = cvCreateImage(cvSize(image_.rows, image_.cols), IPL_DEPTH_8U, t.nChannels);

   cvSet(rotatedImage, cvScalar(255, 255, 255));
   CvPoint2D32f center;
   center.x = boundingBoxesRotated_.at(i).x;
   center.y = boundingBoxesRotated_.at(i).y;
   CvMat *mapMatrix = cvCreateMat(2, 3, CV_32FC1 );

   int angle = maxAngle - 180 - 180;
   cout << "777" << endl;
   cv2DRotationMatrix(center, angle, 1.0, mapMatrix);
   cout << "888" << endl;
   cvSetImageROI(&t, cvRect(boundingBoxesRotated_.at(i).x, boundingBoxesRotated_.at(i).y, boundingBoxesRotated_.at(i).x
   + boundingBoxesRotated_.at(i).width, boundingBoxesRotated_.at(i).y + boundingBoxesRotated_.at(i).height));
   cout << "999" << endl;
   cvWarpAffine(&t, rotatedImage, mapMatrix, CV_INTER_LINEAR, cvScalarAll(0));
   cvResetImageROI(&t);
   cvReleaseMat(&mapMatrix);
   //cvShowImage("mainWin", rotatedImage);

   Mat mat_img(rotatedImage);
   rotated.push_back(Rotated(mat_img, cvRect(boundingBoxesRotated_.at(i).x, boundingBoxesRotated_.at(i).y, boundingBoxesRotated_.at(i).x
   + boundingBoxesRotated_.at(i).width, boundingBoxesRotated_.at(i).y + boundingBoxesRotated_.at(i).height)));

   // End Rotation
   }
   //waitKey(0);
   }

   /*********************************************************************************************

   for (size_t i = 0; i < boundingBoxesRotated_.size(); i++)
   {
   string result;
   float score = ocrRead(rotated.at(i).rotated_img, result);
   if (score > 0)
   {
   boxesBothSides_.push_back(rotated.at(i).coords);
   wordsBothSides_.push_back(result);
   boxesScores_.push_back(score);
   }
   }

   showBoundingBoxes(boxesBothSides_);
   overlayText(boxesBothSides_, wordsBothSides_);
   */

  imwrite(outputPrefix_ + "_detection.jpg", detection_);

  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s total in process\n" << endl;

  textDisplayOffset_ = 1;
}

void DetectText::preprocess(Mat& image)
{
  // rescale, convert into gray scale

  cout << "preprocessing: " << filename_ << endl;
  cout << "image size:" << image.cols << "X" << image.rows << endl;

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

  image_ = image;

  // hack: was turned off
  //	bilateralFilter(image, image_, 7, 20, 50);// prosilica sensor noise


  maxStrokeWidth_ = round(20 * (float)(max(image.cols, image.rows)) / 1000);

  initialStrokeWidth_ = maxStrokeWidth_ * 2;
  maxLetterHeight_ = 600;
  minLetterHeight_ = 10;

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

void DetectText::pipeline(int blackWhite)
{
  if (blackWhite == 1)
  {
    fontColor_ = BRIGHT; //usable for rotation?
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

  strokeWidthTransform(image_, swtmap, blackWhite); //SWT!
  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s in strokeWidthTransform" << endl;

  start_time = clock();
  Mat ccmap(image_.size(), CV_32FC1, Scalar(-1));
  componentsRoi_.clear();
  nComponent_ = connectComponentAnalysis(swtmap, ccmap);
  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s in connectComponentAnalysis" << endl;

  start_time = clock();
  identifyLetters(swtmap, ccmap);
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
  findRotationangles();
  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s in findRotationsangles" << endl;

  /*showEdgeMap();
   showSwtmap(swtmap);
   showCcmap(ccmap);
   showLetterGroup();*/

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

    /* cout << endl << "Orginal Image:" << endl;
     cout << "----------------------------------" << endl;
     for (int y = 0; y < image.rows; y++)
     {
     for (int x = 0; x < image.cols; x++)
     {
     cout << (unsigned int)image.at<unsigned char> (y, x) << " ";
     }
     cout << endl;
     }

     cout << endl << "dx:" << endl;
     cout << "----------------------------------" << endl;
     for (int y = 0; y < image.rows; y++)
     {
     for (int x = 0; x < image.cols; x++)
     {
     cout << abs(round((float)dx.at<float> (y, x))) << " ";
     }
     cout << endl;
     }

     cout << endl << "dy:" << endl;
     cout << "----------------------------------" << endl;
     for (int y = 0; y < image.rows; y++)
     {
     for (int x = 0; x < image.cols; x++)
     {
     cout << round((float)dy.at<float> (y, x)) << " ";
     }
     cout << endl;
     }

     cout << endl << "Canny:" << endl;
     cout << "----------------------------------" << endl;
     for (int y = 0; y < edgemap_.rows; y++)
     {
     for (int x = 0; x < edgemap_.cols; x++)
     {
     cout << (unsigned int)edgemap_.at<unsigned char> (y, x) << " ";
     }
     cout << endl;
     }

     // cv::imshow("canny_img",edgemap_);
     // cv::waitKey(0);

     cout << endl << "theta_:" << endl;
     cout << "----------------------------------" << endl;

     */

    for (int y = 0; y < edgemap_.rows; y++)
    {
      for (int x = 0; x < edgemap_.cols; x++)
      {
        if (edgemap_.at<unsigned char> (y, x) == 255) // Falls bei (x,y) eine Kante ist
        {
          theta_.at<float> (y, x) = atan2(dy.at<float> (y, x), dx.at<float> (y, x)); //Anstieg = arctan gy/gx
          edgepoints_.push_back(Point(x, y)); //Kante als Punkt in edgepoints speichern
        }
      }
    }
    //imwrite(outputPrefix_ + "_5_theta_.jpg", theta_);
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

    pointStack.push_back(Point(currX, currY)); // Anfangspunkt im pointStack speichern
    SwtValues.push_back(swtmap.at<float> (currY, currX));
    while (step < maxStrokeWidth_)
    {
      float nextY = round(iy + sin(iTheta) * searchDirection * step); //anfangs:
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
        float jTheta = theta_.at<float> (currY, currX); //currY=nextY
        if (abs(abs(iTheta - jTheta) - 3.14) < 3.14 / 2)
        {
          isStroke = true;
          if (purpose == UPDATE)
          {
            strokePoints.push_back(Point(ix, iy)); //gegenüberliegender Punkt gefunden
          }
        }
        break;
      }
    }

    if (isStroke)
    {
      float newSwtVal;
      if (purpose == UPDATE)// update swt based on dist between edges
      {
        newSwtVal = sqrt((currY - iy) * (currY - iy) + (currX - ix) * (currX - ix)); //Abstand der Punkte
        //cout << "newSwtVal: " << newSwtVal << ", currY: " << currY << ", iy:" << iy << ", currX:" << currX << ", ix:" << ix << endl;
      }
      else if (purpose == REFINE) // refine swt based on median
      {
        nth_element(SwtValues.begin(), SwtValues.begin() + SwtValues.size() / 2, SwtValues.end());
        newSwtVal = SwtValues[SwtValues.size() / 2];
        //cout << "newSwtVal_median: " << newSwtVal << endl;
      }

      for (size_t i = 0; i < pointStack.size(); i++) //pointStack = wieviele Punkte liegen dazwischen
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

  int ccmapInitialVal = ccmap.at<float> (0, 0); // ccmapInitialVal = -1
  int offsetY[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  int offsetX[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  int nNeighbors = 8;
  int label = 0;

  int vectorSize = ccmap.rows * ccmap.cols; // vectorSize = 25

  int *pStack = new int[vectorSize * 2]; // int pStack[50]
  int stackPointer;

  int *pVector = new int[vectorSize * 2]; // int pVector[50]
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
        pStack[stackPointer] = x; //pstack[0] = 0; pStack[1] = 0
        pStack[stackPointer + 1] = y;

        while (stackPointer >= 0) // stackPointer = 0
        {
          currentPointX = pStack[stackPointer]; // currentPointX = pstack[0] = 0
          currentPointY = pStack[stackPointer + 1]; // currentPointY = pstack[1] = 0
          stackPointer -= 2; // stackPointer = -2

          pVector[vectorPointer] = currentPointX; // pVector[0] = 0;
          pVector[vectorPointer + 1] = currentPointY; // pVector[1] = 0;
          vectorPointer += 2; // vectorPointer = 2;
          for (int i = 0; i < nNeighbors; i++)
          {
            int ny = currentPointY + offsetY[i]; // ny = currentPointY + 0 = 0
            int nx = currentPointX + offsetX[i]; // nx = currentPointX + 1 = 1


            if (ny < 0 || nx < 0 || ny >= ccmap.rows || nx >= ccmap.cols)
              continue;

            if (swtmap.at<float> (ny, nx) == 0)
            {
              ccmap.at<float> (ny, nx) = -2;
              continue;
            }

            if (ccmap.at<float> (ny, nx) == ccmapInitialVal)
            {
              float sw1 = swtmap.at<float> (ny, nx); // sw1 = 3
              float sw2 = swtmap.at<float> (y, x); // sw2 = 3

              if (max(sw1, sw2) / min(sw1, sw2) <= 3) // sw1/sw2 = 1 <= 3
              {
                ccmap.at<float> (ny, nx) = label; // ccmap[0,1] = 0;
                stackPointer += 2; // stackPointer = 0;
                pStack[stackPointer] = nx; // pStack[0] = 1;
                pStack[stackPointer + 1] = ny; // pStack[1] = 0;
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
          Rect letterRoi(minX, minY, width, height); // Rect(0,0,1,1);
          componentsRoi_.push_back(letterRoi);
          //assert(label == componentsRoi_.size()-1);

          label++; // label = 1

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
        else
        {
          if (component >= 0)

          {
            innerComponents[component] = true;
          }
        }
      }

    float pixelCount = static_cast<float> (iComponentStrokeWidth.size());
    float mean = sumStrokeWidth / pixelCount;
    float variance = 0;

    for (size_t ii = 0; ii < pixelCount; ii++)
    {
      variance += pow(iComponentStrokeWidth[ii] - mean, 2); // variance += (SW[i]-mean)²;
    }
    variance = variance / pixelCount;

    // rules & parameters goes here:

    isLetter = isLetter && (variance / mean < 1.5); //Variance has to be small, so that for example leaves can be recognized as no letters

    isLetter = isLetter && (sqrt((pow(itr->width, 2) + pow(itr->height, 2))) / maxStrokeWidth < 10);

    // additional rules:
    isLetter = isLetter && (pixelCount / maxStrokeWidth > 5);

    isLetter = isLetter && (itr->width < 2.5 * itr->height);

    if (countInnerLetterCandidates(innerComponents) - 1 > 5)
    {
      isLetter = false;
    }
    isLetterComponects_[i] = isLetter;

    //!
    if (isLetter == true)
    {
      //!!!
      Mat output = originalImage_.clone();
      rectangle(output, Point(minX, minY), Point(maxX, maxY), cvScalar(0, 255, 0), 2);
      cv::imshow("rectangles", output);
      //waitKey(0);
    }

    iComponentStrokeWidth.clear();
  }

  /*******************************************
   double distance, distance_dummy;
   Rect iRect;
   Rect jRect;
   int partner[nComponent_];

   for (int a = 0; a < nComponent_ - 1; a++)
   {
   iRect = componentsRoi_[a];
   distance = 1000;
   for (int b = a + 1; b < nComponent_; b++)
   {
   jRect = componentsRoi_[b];
   distance_dummy = min(distance, sqrt(pow(iRect.x + iRect.width / 2 - jRect.x - jRect.width / 2, 2) + pow(iRect.y
   + iRect.height / 2 - jRect.y - jRect.height / 2, 2)));
   if (distance_dummy < distance)
   partner[a] = b;
   distance = distance_dummy;
   }
   }

   for (int z = 0; z < nComponent_; z++)
   cout << "[" << z << "]:" << partner[z] << endl;

   int plusminus = 0;
   int all_alpha = 0;

   float alpha[nComponent_];
   for (size_t j = 0; j < nComponent_; j++)
   {
   if (isLetterComponects_[j] == true)
   {
   Rect * rect = &componentsRoi_[j];
   cout << "rect[" << j << "] x:" << rect->x << ",y:" << rect->y << ",height:" << rect->height << ",width:"
   << rect->width;
   // rectangle(image_, Point(rect->x, rect->y), Point(rect->x + rect->width, rect->y + rect->height), 150, 3);
   alpha[j] = (180 / 3.141592) * atan2((rect->x - componentsRoi_[partner[j]].x), (rect->y
   - componentsRoi_[partner[j]].y));
   cout << ",alpha:" << alpha[j] << endl;
   if (alpha[j] < 0)
   plusminus--;
   if (alpha[j] > 0)
   plusminus++;
   }
   }
   int k = 0;

   for (int j = 0; j < nComponent_; j++)
   {
   if (plusminus > 0)
   {
   if (alpha[j] > 0)
   {
   all_alpha += alpha[j];
   k++;
   }
   }
   else if (alpha[j] < 0)
   {
   all_alpha += alpha[j];
   k++;
   }
   }

   int Baseline = all_alpha / k;

   IplImage t = image_;

   //cv::line(image_,
   //         Point(componentsRoi_[0].x + componentsRoi_[0].width, componentsRoi_[0].y + componentsRoi_[0].height),
   //         Point(componentsRoi_[nComponent_ - 1].x + componentsRoi_[nComponent_ - 1].width, componentsRoi_[nComponent_
   //            - 1].y + componentsRoi_[nComponent_ - 1].height), CV_RGB(0,255,0 ), 5);

   cv::imshow("...", image_);
   cv::waitKey(0);

   IplImage *rotatedImage = cvCreateImage(cvSize(image_.rows, image_.cols), IPL_DEPTH_8U, t.nChannels);

   cvSet(rotatedImage, cvScalar(255, 255, 255));
   CvPoint2D32f center;
   center.x = image_.rows;
   center.y = 0;
   CvMat *mapMatrix = cvCreateMat(2, 3, CV_32FC1 );

   cv2DRotationMatrix(center, all_alpha, 1.0, mapMatrix);
   cvSetImageROI(&t, cvRect(componentsRoi_[0].x - 50, componentsRoi_[0].y, componentsRoi_[nComponent_ - 1].x
   + componentsRoi_[nComponent_ - 1].width, componentsRoi_[nComponent_ - 1].y
   + componentsRoi_[nComponent_ - 1].height));

   cvWarpAffine(&t, rotatedImage, mapMatrix, CV_INTER_LINEAR, cvScalarAll(0));
   cvResetImageROI(&t);
   cvReleaseMat(&mapMatrix);
   cvShowImage("mainWin", rotatedImage);
   cv::waitKey(0);

   /**********************************************/

  delete[] innerComponents;
}

void DetectText::groupLetters(const Mat& swtmap, const Mat& ccmap)
{
  componentsMeanIntensity_ = new float[nComponent_];
  componentsMedianStrokeWidth_ = new float[nComponent_];
  isGrouped_ = new bool[nComponent_];
  memset(componentsMeanIntensity_, 0, nComponent_ * sizeof(float));
  memset(componentsMedianStrokeWidth_, 0, nComponent_ * sizeof(float));
  memset(isGrouped_, false, nComponent_ * sizeof(bool));
  //Mat debug = originalImage_.clone();

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

      /* Example
       * -2 0 0 0-2      iRect.y = 0 iRect.x = 1 iRect.height = 3 iRect.width = 3
       * -2 0 0 0-2
       * -2 0 0 0-2
       * -2-2-2-2-2
       * -2-2 1 1-2      jRect.y = 4 jRect.x = 2 jRect.height = 2 jRect.width = 2
       * -2-2 1 1-2
       */

      // check if horizontal
      bool horizontal = !(iRect.y > jRect.y + jRect.height || jRect.y > iRect.y + iRect.height);
      //= bool horizontal = (iRect.y < jRect.y + jRect.height && jRect.y < iRect.y + iRect.height);
      //bool horizontal = (    0   <    4    +      2       &&    4    <   0     +      3      );
      //bool horizontal = false

      // check if vertical
      bool vertical = !(iRect.x > jRect.x + jRect.width || jRect.x > iRect.x + iRect.width);
      //=bool vertical =  (iRect.x < jRect.x + jRect.width && jRect.x < iRect.x + iRect.width);
      //bool vertical =  (   1    <    2    +      2      &&     2   <    1    +       3    );
      //bool vertical = true

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
      float distance = sqrt(pow(iRect.x + iRect.width / 2 - jRect.x - jRect.width / 2, 2) + pow(iRect.y + iRect.height
          / 2 - jRect.y - jRect.height / 2, 2));
      int distanceRatio = 4;
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

        if (vertical)
        {
          //!!! important change
          //verticalLetterGroups_.push_back(Pair(i, j));
          // verticalLetterGroups are never used again, so the Pairs are pushed into horizonzalLetterGroups instead
          horizontalLetterGroups_.push_back(Pair(i, j));
        }
        //
        //Mat output = originalImage_.clone();
        //rectangle(output, Point(iRect.x, iRect.y), Point(iRect.x + iRect.width, iRect.y + iRect.height), cvScalar(0, 0,255      ),      2);
      //cv::imshow("rectangles", output);
      //waitKey(0);
    }
  }// end for loop j
}// end for loop i
}

void DetectText::chainPairs(Mat& ccmap)
{
  /*
   for (int i = 200; i < 300; i++)
   {
   cout << "Pair: " << i << endl;
   cout << componentsRoi_[horizontalLetterGroups_[i].left].x << ","
   << componentsRoi_[horizontalLetterGroups_[i].left].y << ","
   << componentsRoi_[horizontalLetterGroups_[i].left].width << ","
   << componentsRoi_[horizontalLetterGroups_[i].left].height << endl;
   cout << componentsRoi_[horizontalLetterGroups_[i].right].x << ","
   << componentsRoi_[horizontalLetterGroups_[i].right].y << ","
   << componentsRoi_[horizontalLetterGroups_[i].right].width << ","
   << componentsRoi_[horizontalLetterGroups_[i].right].height << endl;
   }*/
  mergePairs(horizontalLetterGroups_, horizontalChains_);

  // horizontalChains
  vector<Rect> initialHorizontalBoxes;
  chainToBox(horizontalChains_, initialHorizontalBoxes); //initialHorizontalBoxes contains rects for every chaincomponent with more than to components(letters)
  filterBoundingBoxes(initialHorizontalBoxes, ccmap, 4);

  // boundingBoxes_ are filled with initialHorizontalBoxes:
  boundingBoxes_.insert(boundingBoxes_.end(), initialHorizontalBoxes.begin(), initialHorizontalBoxes.end());

}

void DetectText::findRotationangles()
{
  int showHistogram = 1;

  // Gradient Histogram
  //*****************************************************************************************************

  rotated.clear();
  char* window_title = "gradients";
  IplImage *canvas = cvCreateImage(cvSize(360, 125), IPL_DEPTH_8U, 3);
  int angles = 360;
  int hist[angles];
  double scale;
  int maxValue = 0, maxAngle = 0;

  Mat output = originalImage_.clone();
  IplImage * final = new IplImage(output);
  cvSet(final, cvScalar(0, 0, 0));
  IplImage * t;
  Mat end;
  Mat imGray(originalImage_.size(), CV_8UC1, Scalar(0));

  for (int i = 0; i < boundingBoxes_.size(); i++)
  {
    output = originalImage_.clone();
    t = new IplImage(output);
    maxValue = 0;
    maxAngle = 0;
    cvSet(canvas, CV_RGB(255,255,255), NULL);
    for (int j = 0; j < angles - 1; j++)
      hist[j] = 0;

    // If there is an edge at (y,x),
    // get the angle[-180,180] at (y,x) and add 180 degrees. (because histogram range is [0,360])
    // Then add one point in the histogram at the found angle.
    for (int y = boundingBoxes_.at(i).y; y < boundingBoxes_.at(i).y + boundingBoxes_.at(i).height; y++)
    {
      for (int x = boundingBoxes_.at(i).x; x < boundingBoxes_.at(i).x + boundingBoxes_.at(i).width; x++)
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

    int newHistogram[angles];

    for (int bin = 1; bin < 359; bin++)
    {
      double smoothedValue = 0;
      for (int i = 0; i < 3; i++)
      {
        smoothedValue += hist[bin - 1 + i] * mask[i];
      }
      newHistogram[bin] = smoothedValue;
    }
    IplImage *canvas2 = cvCreateImage(cvSize(360, 125), IPL_DEPTH_8U, 3);
    cvSet(canvas2, CV_RGB(255,255,255), NULL);
    for (int j = 0; j < angles - 1; j++)
    {
      CvPoint pt1 = cvPoint(j, canvas2->height - (newHistogram[j] * scale));
      CvPoint pt2 = cvPoint(j, canvas2->height);
      if (j == maxAngle)
        cvLine(canvas2, pt1, pt2, CV_RGB(255,0,0), 1.5, 8, 0);
      else
        cvLine(canvas2, pt1, pt2, CV_RGB(0,0,0), 1, 8, 0);
    }
    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.3, 0.3, 0, 1, CV_AA);
    cvPutText(canvas2, "0", cvPoint(180, 122), &font, cvScalar(140, 160, 200, 0));
    cvPutText(canvas2, "-90", cvPoint(90, 122), &font, cvScalar(140, 160, 200, 0));
    cvPutText(canvas2, "-180", cvPoint(0, 122), &font, cvScalar(140, 160, 200, 0));
    cvPutText(canvas2, "90", cvPoint(260, 122), &font, cvScalar(140, 160, 200, 0));
    cvPutText(canvas2, "180", cvPoint(340, 122), &font, cvScalar(140, 160, 200, 0));
    cvShowImage("gradients smoothed", canvas2);
    // waitKey(0);

    //***************************

    // Get maxValue and max angle
    for (int j = 0; j < angles - 1; j++)
      maxValue = hist[j] > maxValue ? hist[j] : maxValue;

    for (int j = 0; j < angles - 1; j++)
      if (maxValue == hist[j])
        maxAngle = j;

    // Fit histogram to the canvas height
    scale = maxValue > canvas->height ? (double)canvas->height / maxValue : 1.;

    //Draw histogram
    for (int j = 0; j < angles - 1; j++)
    {
      CvPoint pt1 = cvPoint(j, canvas->height - (hist[j] * scale));
      CvPoint pt2 = cvPoint(j, canvas->height);
      if (j == maxAngle)
        cvLine(canvas, pt1, pt2, CV_RGB(255,0,0), 1.5, 8, 0);
      else
        cvLine(canvas, pt1, pt2, CV_RGB(0,0,0), 1, 8, 0);
    }
    // CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.3, 0.3, 0, 1, CV_AA);
    cvPutText(canvas, "0", cvPoint(180, 122), &font, cvScalar(140, 160, 200, 0));
    cvPutText(canvas, "-90", cvPoint(90, 122), &font, cvScalar(140, 160, 200, 0));
    cvPutText(canvas, "-180", cvPoint(0, 122), &font, cvScalar(140, 160, 200, 0));
    cvPutText(canvas, "90", cvPoint(260, 122), &font, cvScalar(140, 160, 200, 0));
    cvPutText(canvas, "180", cvPoint(340, 122), &font, cvScalar(140, 160, 200, 0));
    cvShowImage(window_title, canvas);

    rectangle(output, Point(boundingBoxes_.at(i).x, boundingBoxes_.at(i).y), Point(boundingBoxes_.at(i).x
        + boundingBoxes_.at(i).width, boundingBoxes_.at(i).y + boundingBoxes_.at(i).height), cvScalar(255, 0, 0), 2);
    //cv::imshow("rectangles", output);
    //cout << "maxAngle:" << maxAngle << "(" << maxAngle - 180 << ")" << endl;
    //waitKey(0);

    // Rotation
    //*****************************************************************************************************

    if (maxAngle < 183 && maxAngle > 177)
      maxAngle = 0;

    /*  output = originalImage_.clone();
     t = new IplImage(output);
     IplImage *imago = cvCreateImage(cvSize(t->width, t->height), t->depth, t->nChannels);
     cvSet(imago, cvScalar(0, 0, 0));

     //Crop boundingBox from t and put into imago
     cvSetImageROI(t, cvRect(boundingBoxes_.at(i).x, boundingBoxes_.at(i).y, boundingBoxes_.at(i).width,
     boundingBoxes_.at(i).height));
     cvSetImageROI(imago, cvRect(boundingBoxes_.at(i).x, boundingBoxes_.at(i).y, boundingBoxes_.at(i).width,
     boundingBoxes_.at(i).height));
     cvCopy(t, imago, NULL);
     cvResetImageROI(t);
     cvResetImageROI(imago);
     */

    //cut small image out
    Mat small = image_.colRange(max(boundingBoxes_.at(i).x - 10, 0), min(boundingBoxes_.at(i).x
        + boundingBoxes_.at(i).width + 10, image_.cols));
    small = small.rowRange(max(boundingBoxes_.at(i).y - 10, 0), min(boundingBoxes_.at(i).y
        + boundingBoxes_.at(i).height + 10, image_.rows));

    IplImage *Image1 = new IplImage(small);

    // Average of whole image
    CvScalar averageColor = cvAvg(Image1);

    vector<int> bgColor;

    for (int y = 0; y < small.rows; y++)
    {
      for (int x = 0; x < small.cols; x++)
      {
        if (fontColor_ == DARK)
        {
          if ((unsigned int)small.at<unsigned char> (y, x) > (unsigned int)averageColor.val[0])
          {
            bgColor.push_back((unsigned int)small.at<unsigned char> (y, x));
          }
        }
        else
        {
          if ((unsigned int)small.at<unsigned char> (y, x) < (unsigned int)averageColor.val[0])
          {
            bgColor.push_back((unsigned int)small.at<unsigned char> (y, x));
          }
        }
      }
    }

    // Average of background Pixels
    int average_dummy = 0;
    for (int i = 0; i < bgColor.size(); i++)
    {
      average_dummy += bgColor.at(i);
    }
    average_dummy = average_dummy / bgColor.size();

    //Rotation
    CvPoint2D32f center;
    center.x = small.cols * 0.5;
    center.y = small.rows * 0.5;
    CvMat *mapMatrix = cvCreateMat(2, 3, CV_32FC1 );
    cv2DRotationMatrix(center, maxAngle - 180, 1.0, mapMatrix);

    IplImage *rotatedImage = cvCreateImage(cvSize((small.cols), (small.rows)), IPL_DEPTH_8U, 1);
    cvSet(rotatedImage, Scalar(average_dummy));
    cvWarpAffine(Image1, rotatedImage, mapMatrix, CV_INTER_LINEAR, Scalar(average_dummy));


    //cvShowImage("rotatedImage", rotatedImage);
    // waitKey(0);

    //cvShowImage("outcut", imago);
    //waitKey(0);

    // Move picture extract to the middle of the image before rotating
    /*
     CvPoint2D32f srcSquare[4], dstSquare[4];
     srcSquare[0].x = boundingBoxes_.at(i).x;
     srcSquare[0].y = boundingBoxes_.at(i).y;
     srcSquare[1].x = boundingBoxes_.at(i).x + boundingBoxes_.at(i).width;
     srcSquare[1].y = boundingBoxes_.at(i).y;
     srcSquare[2].x = boundingBoxes_.at(i).x;
     srcSquare[2].y = boundingBoxes_.at(i).y + boundingBoxes_.at(i).height;
     srcSquare[3].x = boundingBoxes_.at(i).x + boundingBoxes_.at(i).width;
     srcSquare[3].y = boundingBoxes_.at(i).y + boundingBoxes_.at(i).height;

     dstSquare[0].x = originalImage_.cols * 0.5 - boundingBoxes_.at(i).width * 0.5;
     dstSquare[0].y = originalImage_.rows * 0.5 - boundingBoxes_.at(i).height * 0.5;
     dstSquare[1].x = originalImage_.cols * 0.5 + boundingBoxes_.at(i).width * 0.5;
     dstSquare[1].y = originalImage_.rows * 0.5 - boundingBoxes_.at(i).height * 0.5;
     dstSquare[2].x = originalImage_.cols * 0.5 - boundingBoxes_.at(i).width * 0.5;
     dstSquare[2].y = originalImage_.rows * 0.5 + boundingBoxes_.at(i).height * 0.5;
     dstSquare[3].x = originalImage_.cols * 0.5 + boundingBoxes_.at(i).width * 0.5;
     dstSquare[3].y = originalImage_.rows * 0.5 + boundingBoxes_.at(i).height * 0.5;

     CvMat* warp_mat = cvCreateMat(2, 3, CV_32FC1);
     cvGetAffineTransform(srcSquare, dstSquare, warp_mat);
     IplImage *middle = cvCreateImage(cvSize(t->width, t->height), t->depth, t->nChannels);
     cvWarpAffine(imago, middle, warp_mat);
     //cvShowImage("middle", middle);
     //waitKey(0);


     // Rotate the positioned picture extract
     CvPoint2D32f center;
     center.x = originalImage_.cols * 0.5;
     center.y = originalImage_.rows * 0.5;
     CvMat *mapMatrix = cvCreateMat(2, 3, CV_32FC1 );
     cv2DRotationMatrix(center, maxAngle - 180, 1.0, mapMatrix);

     //cvSetImageROI(imago, cvRect(originalImage_.cols * 0.5 - boundingBoxes_.at(i).width * 0.5, originalImage_.rows
     //    * 0.5 - boundingBoxes_.at(i).height * 0.5, boundingBoxes_.at(i).width, boundingBoxes_.at(i).height));
     IplImage *rotatedImage = cvCreateImage(cvSize((originalImage_.cols * 1.25), (originalImage_.rows * 1.25)), //*1.25
     IPL_DEPTH_8U, 3);
     cvSet(rotatedImage, cvScalar(255, 255, 255));
     cvWarpAffine(imago, rotatedImage, mapMatrix);
     cvShowImage("rotatedImage", rotatedImage);
     //cvResetImageROI(imago);
     cvReleaseMat(&mapMatrix);
     waitKey(0);;*/

    Mat dummy(rotatedImage);
    Rotated * r = new Rotated(dummy, cvRect(boundingBoxes_.at(i).x, boundingBoxes_.at(i).y, boundingBoxes_.at(i).width,
                                            boundingBoxes_.at(i).height));
    rotated.push_back(*r);
  }
}

void DetectText::chainToBox(vector<vector<int> >& chain, vector<Rect>& boundingBox)
{
  for (size_t i = 0; i < chain.size(); i++)
  {
    if (chain[i].size() < 3) //Only words with more than 2 letters
    {
      //continue;         //commented for rotation test
    }

    int minX = image_.cols, minY = image_.rows, maxX = 0, maxY = 0;
    int letterAreaSum = 0;
    int padding = 15; //=5, for rotation =15 because often letter parts lie on the boundary

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
    /*
     minX = minX - padding;
     minY = minY - padding;
     maxX = maxX + padding;
     maxY = maxY + padding;     */

    boundingBox.push_back(Rect(minX, minY, maxX - minX, maxY - minY));
    // rectangle(image_, Point(minX, minY), Point(maxX, maxY), 150, 3);
  }

  // imshow("...",image_);
  //waitKey(0);
  //exit(0);

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

        if (componetIndex < 0) //padding-pixel z.B sind <0
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
    //!!! to be changed back
    // rotated is at the moment a class and couldnt be used here directly
    int r = 1;
    cout << text[i] << endl;
    cout << "--------------" << endl;
    if (i > 0)
    {
      for (size_t j = i - 1; j > 0; j--)
      {
        cout << text[j] << " ";
        if (output == text[j])
          r = 0;
      }
      cout << endl;
      cout << "r:" << r << endl;
    }
    if (r == 1)
    {
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
      putText(detection_, output, Point(image_.cols + indent, textDisplayOffset_ * 35), FONT_HERSHEY_DUPLEX, 1, color,
              2);
      textDisplayOffset_ += 2;
    }
    r = 1;
  }
}

void DetectText::ocrRead(vector<Rect>& boundingBoxes)
{
  /*
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
   }*/
  //rotated
  for (size_t i = 0; i < rotated.size() - 1; i++)
  {
    string result;
    //cout << "ROTATION [" << i << "]: ";
    imshow("actual image", rotated.at(i).rotated_img);
    waitKey(0);
    float score = ocrRead(rotated.at(i).rotated_img, result, i);
    if (score > 0)
    {
      boxesBothSides_.push_back(rotated.at(i).coords);
      wordsBothSides_.push_back(result);
      boxesScores_.push_back(score);
    }
    cout << "score:" << score << endl;
  }

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
  /* groups:
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

  /* initialChains:
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
  /* initialChains:
   * [0]  [1]  [2]  [3]
   *  4    12   44  45
   *  5    14   45  46
   */
  for (size_t i = 0; i < initialChains.size(); i++) // chain i          i<4
  {
    if (mergedToChainBitMap[i] != -1)
      continue;

    for (size_t j = i + 1; j < initialChains.size(); j++) // chain j    j=1, j<4
    {
      // match elements in chain i,j
      for (size_t ki = 0; ki < initialChains[i].size(); ki++) // ki<initialChains[0].size()=2
      {
        for (size_t kj = 0; kj < initialChains[j].size(); kj++)
        {
          // found match
          if (initialChains[i][ki] == initialChains[j][kj]) // Does any other initialChains[x] contain a identical componect?
          { //i=2 j=3 ki=1 kj=0
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
              merge(initialChains[i], newChain); // newChain = {44,45}
              merge(initialChains[j], newChain); // newChain = {44,45,46}
              chains.push_back(newChain); // chains= {44,45,46},{},{}...
              mergedToChainBitMap[i] = chains.size() - 1; //mergedToChainBitMap[2]=2;
              mergedToChainBitMap[j] = chains.size() - 1; //mergedToChainBitMap[3]=2;
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
  /*chains:
   chains[0][0]: 47 chains[0][1]: 59
   chains[1][0]: 137 chains[1][1]: 149
   chains[2][0]: 228 chains[2][1]: 239
   */
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
