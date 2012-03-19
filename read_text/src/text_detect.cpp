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

using namespace cv;
using namespace std;

/*--------------------------------------------------------- Konstruktor/Destruktor ---------------------------------------------------------*/
DetectText::DetectText() :
  maxStrokeWidth_(0), initialStrokeWidth_(0), firstPass_(true), result_(COARSE), nComponent_(0), maxLetterHeight_(0),
      minLetterHeight_(0), textDisplayOffset_(1)
{
}
DetectText::~DetectText()
{
}

/*-------------------------------------------------- detect(string) / detect(Mat&) ----------------------------------------------------*/
/*-------------------------------------------------- File einlesen, detect() aufrufen--------------------------------------------------*/
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

/*----------------------------------------- Mat& getDetection() / vector<string>& getWords() -----------------------------------------*/
Mat& DetectText::getDetection()
{
  return detection_;
}
vector<string>& DetectText::getWords()
{
  return wordsBothSides_;
}

/*----------------------------------------- HAUPTFUNKTION ----------------------------------------------------------------------------*/
void DetectText::detect()
{
  double start_time;
  double time_in_seconds;
  start_time = clock();

  // neues cvMat: Graustufenbild. Mat(Size _size, int _type, const Scalar& _s);
  // 8-bit unsigned single-channel, gefüllt mit 0en
  Mat imGray(originalImage_.size(), CV_8UC1, Scalar(0));
  // convertColor, von Originalimage zu imGray, RGB_to_GRAY
  cvtColor(originalImage_, imGray, CV_RGB2GRAY);
  // Vectors für Boxen clearen
  boundingBoxes_.clear();
  boxesBothSides_.clear();
  wordsBothSides_.clear();
  boxesScores_.clear();

  preprocess(imGray);
  // imwrite(outputPrefix_ + "_1_imGray_.jpg", imGray);
  // imwrite(outputPrefix_ + "_2_detection_.jpg", detection_);
  firstPass_ = true;
  pipeline(1);
  firstPass_ = false; // nur für SWT relevant, damit auch schwarzer Text auf weissen Grund erkannt wird
  pipeline(-1);

  overlapBoundingBoxes(boundingBoxes_);
  ocrRead(boundingBoxes_);
  showBoundingBoxes(boxesBothSides_);
  std::cout << "1\n";
  overlayText(boxesBothSides_, wordsBothSides_);
  std::cout << "1\n";

  imwrite(outputPrefix_ + "_detection.jpg", detection_);

  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
  cout << time_in_seconds << "s total in process\n" << endl;

  textDisplayOffset_ = 1;
}

/*----------------------------------------- preprocess wird von Detect aufgerufen mit imGray: ----------------------------------------------------*/
/* rescale, convert into gray scale */
void DetectText::preprocess(Mat& image)
{
  cout << "preprocessing: " << filename_ << endl;
  cout << "image size:" << image.cols << "X" << image.rows << endl;

  int slashIndex = -1;
  int dotIndex = -1;
  for (size_t i = filename_.length() - 1; i != 0; i--) //~i = Länge des Strings filename_ -1
  {
    //Beispiel images/SceneTrialTest/ryoungt_05.08.2002/wPICT0006.JPG
    if (dotIndex == -1 && filename_[i] == '.')
      // i = 53, keine der ifs aufgerufen bis i = 50
      // dann erstes if erfüllt: dotIndex = 50, i=49,48..40: zweites if erfüllt
      // slashIndex = 40, dann kein if mehr erfüllt bis i=0
      dotIndex = i;
    if (slashIndex == -1 && filename_[i] == '/')
      slashIndex = i;
  }
  outputPrefix_ = filename_.substr(slashIndex + 1, dotIndex - slashIndex - 1); //substr(40+1, 50-40-1)
  cout << "outputPrefix: " << outputPrefix_ << endl;

  image_ = image; //Vorhin (lokal) erstelles imGray wurde an preprocess() übergeben und jetzt image_ zugewiesen

  //	bilateralFilter(image, image_, 7, 20, 50);// prosilica sensor noise

	maxStrokeWidth_ = 10;
  // A  eigentlich:
  // | 
  //maxStrokeWidth_ = round(20 * (float)(max(image.cols, image.rows)) / 1000);

  initialStrokeWidth_ = maxStrokeWidth_ * 2;
  maxLetterHeight_ = 600;
  minLetterHeight_ = 10;

  IplImage *img2 = new IplImage(originalImage_);
  IplImage *img1 = cvCreateImage(cvSize(image.cols + 600, image.rows), img2->depth, img2->nChannels); //img1 ist 600 breiter als original

  cvSet(img1, cvScalar(0, 0, 0));
  cvSetImageROI(img1, cvRect(0, 0, image.cols, image.rows)); //Region of Interest
  cvCopy(img2, img1, NULL); //Kopiert originalbild in img1(ROI)
  cvResetImageROI(img1); //Region of Interest aus img1 gelöscht
  detection_ = Mat(img1).clone(); //detection = img1
  cvReleaseImage(&img1); // img1 releasen
  delete img1; // img1 löschen
  delete img2; // img2 löschen
  // image_ enthält sw bild
  // detection_ enthält farbbild + 600 breite
}
/*----------------------------------------- pipeline wird von Detect aufgerufen und setzt fontColor_: ----------------------------------------------*/
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
  // initialize swtmap with large values
  double start_time;
  double time_in_seconds;

  start_time = clock();
  Mat swtmap(image_.size(), CV_32FC1, Scalar(initialStrokeWidth_)); //swtmap mit swbild size, 32bitfloats, alle gefüllt mit initialStrokeWidth

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

  /*showEdgeMap();
   showSwtmap(swtmap);
   showCcmap(ccmap);
   showLetterGroup();*/

  disposal();
  cout << "finish clean up" << endl;

}

/*----------------------------------------- SWT wird von pipeline aufgerufen ----------------------------------------------*/
void DetectText::strokeWidthTransform(const Mat& image, Mat& swtmap, int searchDirection)
{
  if (firstPass_)
  {
    // compute edge map
    Canny(image_, edgemap_, 50, 120); //Canny-Algorithmus, T1=50, T2=120, output=edgemap_
    //edgemap(x,y) hat entweder Wert 0 oder 255
    //compute gradient direction
    //imwrite(outputPrefix_ + "_3_edgemap(canny)_.jpg", edgemap_);

    Mat dx, dy;
    Sobel(image_, dx, CV_32FC1, 1, 0, 3); //Partielle Ableitungen der einzelnen Pixel durch Faltung mit Sobeloperator
    Sobel(image_, dy, CV_32FC1, 0, 1, 3);
    //imwrite(outputPrefix_ + "_4_dx(Sobel)_.jpg", dx);
    // imwrite(outputPrefix_ + "_4_dy(Sobel)_.jpg", dy);

    theta_ = Mat(image_.size(), CV_32FC1);

    if (edgepoints_.size()) //Falls edgepoints noch gefüllt ist
    {
      edgepoints_.clear();
    }

    // edgemap_ = nach Canny_algorithmus
    // dx, dy = nach Faltung mit Sobeloperator in x und y Richtung


    /* cout << endl << "Originalbild:" << endl;
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
     cout << abs(round((float)dx.at<float> (y, x)))<< " ";
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
     for (int y = 0; y < 30; y++)
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
     cout << "----------------------------------" << endl;  */
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

  // Zweiter Durchgang:

  vector<Point> strokePoints;

  updateStrokeWidth(swtmap, edgepoints_, strokePoints, searchDirection, UPDATE);
      cout << endl << "swt-map:" << endl;
     cout << "----------------------------------" << endl;
     for (int y = 0; y < swtmap.rows; y++)
     {
     for (int x = 0; x < swtmap.cols; x++)
     {
     cout << (unsigned int)swtmap.at<float> (y, x) << " ";
     }
     cout << endl;
     }



  updateStrokeWidth(swtmap, strokePoints, strokePoints, searchDirection, REFINE);
  
    cout << endl << "swt-map:" << endl;
     cout << "----------------------------------" << endl;
     for (int y = 0; y < swtmap.rows; y++)
     {
     for (int x = 0; x < swtmap.cols; x++)
     {
     cout << (unsigned int)swtmap.at<float> (y, x) << " ";
     }
     cout << endl;
     }
     exit(0);
}

//swtmap ist noch leer, startPoints = edgepoints -> enthält alle Kantenpunkte aus canny, strokePoints noch leer(initialStrokeWidth)
// , searchDirection zunächst 1

void DetectText::updateStrokeWidth(Mat& swtmap, vector<Point>& startPoints, vector<Point>& strokePoints,
                                   int searchDirection, Purpose purpose)
{
  //loop through all edgepoints, compute stroke width
  //startPoints = edgepoints_ alle Punkte bei denen Kante
  vector<Point>::iterator itr = startPoints.begin();

  vector<Point> pointStack;
  vector<float> SwtValues;

  for (; itr != startPoints.end(); ++itr) // Jeder Punkt der bei Canny als Kante erkannt wurde
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
      if (ccmap.at<float> (y, x) == ccmapInitialVal) // falls noch auf -1
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

            if (ccmap.at<float> (ny, nx) == ccmapInitialVal) // erfüllt
            {
              float sw1 = swtmap.at<float> (ny, nx); // sw1 = 3
              float sw2 = swtmap.at<float> (y, x); // sw2 = 3

              if (max(sw1, sw2) / min(sw1, sw2) <= 3) // sw1/sw2 = 1 <= 3 erfüllt
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
        int component = static_cast<int> (ccmap.at<float> (y, x));
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
      variance += pow(iComponentStrokeWidth[ii] - mean, 2);
    }
    variance = variance / pixelCount;

    // rules & parameters goes here:

    isLetter = isLetter && (variance / mean < 1.5); //Varianz darf nicht zu gross sein, um z.B. Laub von Buchstaben zu unterscheiden

    isLetter = isLetter && (sqrt((pow(itr->width, 2) + pow(itr->height, 2))) / maxStrokeWidth < 10);

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
  Mat debug = originalImage_.clone();

  for (size_t i = 0; i < nComponent_; i++)
  {
    if (!isLetterComponects_[i])
      continue;

    Rect iRect = componentsRoi_[i];

    float iMeanIntensity = getMeanIntensity(ccmap, iRect, static_cast<int> (i)); //Durchschnittsintensität in der jeweiligen Komponente
    float iMedianStrokeWidth = getMedianStrokeWidth(ccmap, swtmap, iRect, static_cast<int> (i)); //Median-SW in der jeweiligen Komponente

    for (size_t j = i + 1; j < nComponent_; j++)
    {
      if (!isLetterComponects_[j])
        continue;

      Rect jRect = componentsRoi_[j];

      /* Beispiel
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
        if (distance / max(iRect.width, jRect.width) > distanceRatio) // distance darf nicht 4mal groesser als breite des breiteren letters sein, sonst kein letter
          continue;
      }
      else
      {
        if (distance / max(iRect.height, jRect.height) > distanceRatio) // distance darf nicht 4mal groesser als höhe des höheren letters sein, sonst kein letter
          continue;
      }

      float jMeanIntensity = getMeanIntensity(ccmap, jRect, static_cast<int> (j)); //Durchschnittsintensität in der j - Komponente
      float jMedianStrokeWidth = getMedianStrokeWidth(ccmap, swtmap, jRect, static_cast<int> (j)); //Median-SW in der j - Komponente

      bool isGroup = true;

      // rule 1: median of stroke width ratio
      isGroup = isGroup && (max(iMedianStrokeWidth, jMedianStrokeWidth) / min(iMedianStrokeWidth, jMedianStrokeWidth))
          < 2;
      // MedianSW einer Komponente darf nicht 2x oder mehr stärker sein als MedianSW der anderen Komponente

      // rule 2: height ratio
      isGroup = isGroup && (max(iRect.height, jRect.height) / min(iRect.height, jRect.height)) < 2;
      // Height Ratio muss kleiner 2.0 sein, um z.B noch Gross- und Kleinbuchstaben nebeneinander nicht zu trennen

      // rule 4: average color of letters
      isGroup = isGroup && abs(iMeanIntensity - jMeanIntensity) < 10;
      // Buchstaben eines Wortes sind meistens in der selben Farbe

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
          verticalLetterGroups_.push_back(Pair(i, j));
        }
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
  chainToBox(horizontalChains_, initialHorizontalBoxes); //initialHorizontalBoxes enthält danach Rechtecke für jede Chainkomponente die aus mehr als 2 Buchstaben besteht

  filterBoundingBoxes(initialHorizontalBoxes, ccmap, 4);

  // boundingBoxes_ füllen mit initialHorizontalBoxes:
  boundingBoxes_.insert(boundingBoxes_.end(), initialHorizontalBoxes.begin(), initialHorizontalBoxes.end());
}

void DetectText::chainToBox(vector<vector<int> >& chain, vector<Rect>& boundingBox)
{
  for (size_t i = 12; i < chain.size(); i++)
  {
    if (chain[i].size() < 3)
      continue;

    int minX = image_.cols, minY = image_.rows, maxX = 0, maxY = 0;
    int letterAreaSum = 0;
    int padding = 5;

    for (size_t j = 0; j < chain[i].size(); j++)
    {
      Rect *itr = &componentsRoi_[chain[i][j]];
      letterAreaSum += itr->width * itr->height; // Fläche jedes einzelnen Rechtecks einer Chainkomponente mit mehr als 2 Rechtecken(Buchstaben)
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
    if (width < 20)
      continue;
    if (max(width, height) / min(width, height) > 20) //? bei einem Satz könnte die Breite auch mehr als 20x mehr sein als die Höhe
      continue;

    for (int y = rect->y; y < rect->y + rect->height; y++)
      for (int x = rect->x; x < rect->x + rect->width; x++)
      {
        int componetIndex = static_cast<int> (ccmap.at<float> (y, x));

        if (componetIndex < 0)
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

// Rahmen für BoundingBoxes
void DetectText::overlapBoundingBoxes(vector<Rect>& boundingBoxes)
{
  vector<Rect> bigBoxes;

  Mat tempMap(image_.size(), CV_32FC1, Scalar(0));
  //BoundingBoxes auf tempMap markieren
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
    if (tempMap.at<float> (boundingBoxes[i].y + 1, boundingBoxes[i].x + 1) != 50) //unnötig? eine boundingBox ist doch immer groesser 0
      continue;

    Rect rect;
    floodFill(tempMap, Point(boundingBoxes[i].x, boundingBoxes[i].y), i + 100, &rect); // warum nicht einfach Rect *rect = &boundingBoxes[i]; ???

    //ATM

    int padding = 5;

    // add padding around each box
    int minX = max(0, rect.x - padding);
    int minY = max(0, rect.y - padding);
    int maxX = min(image_.cols, rect.x + rect.width + padding);
    int maxY = min(image_.rows, rect.y + rect.height + padding);

    bigBoxes.push_back(Rect(minX, minY, maxX - minX, maxY - minY));
  }

  boundingBoxes = bigBoxes;
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
  sort(boundingBoxes.begin(), boundingBoxes.end(), DetectText::spaticalOrder); //boundingBoxes nach y-Werten sortieren
  for (size_t i = 0; i < boundingBoxes.size(); i++)
  {
    string result;
    float score = ocrRead(originalImage_(boundingBoxes[i]), result);
    if (score > 0)
    {
      boxesBothSides_.push_back(boundingBoxes[i]);
      wordsBothSides_.push_back(result);
      boxesScores_.push_back(score);
    }
  }
}

float DetectText::ocrRead(const Mat& imagePatch, string& output)
{
  float score = 0;
  Mat scaledImage;
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
  int result;
  result = system("$(rospack find tesseract)/bin/tesseract patch.tiff patch");
  assert(!result);
  ifstream fin("patch.txt");
  string str;
  while (fin >> str)
  {
    string tempOutput;
    score += spellCheck(str, tempOutput, 2);
    output += tempOutput;
  }
  result = system("$(rm patch.txt patch.tiff)");
  return score;
}

// two option: 1 for aspell, 2 for correlation edit distance 
// return the score for the input
float DetectText::spellCheck(string& str, string& output, int method)
{
  int letterCount = 0, errorCount = 0, lNoiseCount = 0, digitCount = 0;
  string withoutStrangeMarks;
  float score = 0;
  str = trim(str);
  for (size_t i = 0; i < str.length(); i++)
  {
    if (isupper(str[i]) || islower(str[i]))
    {
      withoutStrangeMarks += str[i];
      letterCount++;
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
      if ((i && isdigit(str[i - 1])) || ((i < str.length() - 1) && isdigit(str[i + 1])))
      {
        withoutStrangeMarks += '1';
        str[i] = '1';
        digitCount++;
      }
      else
      {
        withoutStrangeMarks += 'l';
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
  correlation_ = Mat(62, 62, CV_32F, Scalar(0)); // 62*62 - 32bit_float - gefüllt mit 0en
  float number;
  for (int i = 0; i < 62; i++)
    for (int j = 0; j < 62; j++)
    {
      assert(fin >> number); // nächste Zahl aus fin in number
      correlation_.at<float> (i, j) = number; // Zahl aus number in correlation_ speichern
    }
}

/*----------------------------------------- readWordList - Directory in wordList_ einlesen -----------------------------------------------*/
void DetectText::readWordList(const char* filename)
{
  ifstream fin(filename);
  string word;
  wordList_.clear(); //vector<string>
  while (fin >> word) // von dictionary ein Wort in string word schreiben
  {
    wordList_.push_back(word); // in wordList_ das word aufnehmen und index erhöhen
  }
  assert(wordList_.size()); // size>0?
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

void DetectText::getTopkWords(const string& str, const int k, vector<Word>& words)
{
  float score, lowestScore = 100;
  words.clear();
  words.resize(k);

  for (size_t i = 0; i < wordList_.size(); i++)
  {
    score = editDistanceFont(str, wordList_[i]); //Jedes Wort im Wörterbuch
    if (score < lowestScore)
    {
      Word w = Word(wordList_[i], score);
      lowestScore = insertToList(words, w);
    }
  }
}

// return lowest score in the list
float DetectText::insertToList(vector<Word>& words, Word& word)
{
  // first search for the position
  size_t index = 0;

  for (size_t i = 0; i < words.size(); i++)
  {
    index = i;
    if (word.score < words[i].score)
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
  return words[words.size() - 1].score;
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
          if (initialChains[i][ki] == initialChains[j][kj]) // Enthält irgendeine andere initialChains[x] eine gleiche Komponente?
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
float DetectText::editDistanceFont(const string& s, const string& t) //s = eigentliches Wort, t = wörterbuchwort
{
  float penalty = 0.7;

  int n = s.length();
  int m = t.length();

  if (n == 0)
    return m;
  if (m == 0)
    return n;

  //Beispiel "Day"
  float **d = new float*[n + 1]; // float d [][str_laenge+1] = float[][4]
  for (int i = 0; i < n + 1; i++) // 4mal
  {
    d[i] = new float[m + 1];
    memset(d[i], 0, (m + 1) * sizeof(float));
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
      float v = d[i - 1][j - 1];
      if ((t[j - 1] != sc))
      {
        float correlate = correlation_.at<float> (getCorrelationIndex(t[j - 1]), getCorrelationIndex(sc));
        v = v + 1 - correlate;
      }
      d[i][j] = min(min(d[i - 1][j] + penalty, d[i][j - 1] + penalty), v);
    }
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
  assert(false);
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
