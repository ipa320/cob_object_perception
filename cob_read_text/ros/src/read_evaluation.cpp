/**
 * Evaluation for cob_read_text based on precision and recall
 */

#include "opencv2/core/core.hpp"
#include "cv.h"
#include "highgui.h"

#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <fstream>
#include <stack>
#include <cstdio>
#include <dirent.h>

#include <cob_read_text/text_detect.h>

// computes the area of the minimum bounding box (possibly rotated) of two rotated rectangles
double rotatedRectangleMinBox(const cv::RotatedRect& rect1, const cv::RotatedRect& rect2);

// computes the area of intersection of two rotated rectangles
double rotatedRectangleIntersection(const cv::RotatedRect& rect1, const cv::RotatedRect& rect2);

// process one rectangle for rotated rectangle intersection (search for inner points of rect1 in rect2 and intersections, i.e. clip rect1 with rect2)
void rotatedRectangleIntersectionOneWay(const std::vector<cv::Point2f>& vertices1,
                                        const std::vector<cv::Point2f>& vertices2,
                                        const std::vector<cv::Point2f>& lines1, const std::vector<cv::Point2f>& lines2,
                                        std::vector<cv::Point2f>& intersectionContour);

// adds a point to the list of points if it is not already contained in the list
void addContourPoint(const cv::Point2f& point, std::vector<cv::Point2f>& contour);

// determines whether point is an inner point of the given rectangle (= one corner as origin and two vectors representing the sides pointing away from the origin)
bool isInnerPoint(const cv::Point2f& rectangleOrigin, const cv::Point2f& rectangleVectorX,
                  const cv::Point2f& rectangleVectorY, const cv::Point2f& point);

// determines whether two line segments intersect and provides the intersection point if available
bool lineSegmentIntersection(const cv::Point2f& basePoint1, const cv::Point2f& lineSegmentVector1,
                             const cv::Point2f& basePoint2, const cv::Point2f& lineSegmentVector2,
                             cv::Point2f& intersectionPoint);


class img
{
public:
  img(std::string img_name) :
    img_name(img_name)
  {
    count++;
  }
  ~img()
  {
  }
  void setRect(cv::RotatedRect r)
  {
    correctRects.push_back(r);
  }
  void setText(std::string t)
  {
    correctTexts.push_back(t);
  }
  void setEstimatedRect(cv::RotatedRect r)
  {
    estimatedRects.push_back(r);
  }
  void setEstimatedText(std::string t)
  {
    estimatedTexts.push_back(t);
  }

  std::string img_name;
  std::vector<cv::RotatedRect> correctRects, estimatedRects;
  std::vector<std::string> correctTexts, estimatedTexts;
  // correct = ground truth data, each entry contains exactly one word, estimated = may contain several words that become broken into single words

  std::vector<bool> wordWasFound;
  //label which words of the solution are in the estimated set

  float precision;
  // correct estimates / total number of estimates for bounding boxes

  float recall;
  // correct estimates / total number of targets for bounding boxes

  float f;
  // standard f measure combines precision and recall

  float foundWordsRatio;
  // correct found words / total number of words that could've been found

  float wrongWordsRatio;
  // words found that don't exist in the solution / total number of words found

  static int count;
};

int img::count = 0;

int cutout(std::string label, std::string output)
{
  //cutout x,y,width,height
  if (output.find(label) != std::string::npos)
    return atoi((output.substr(output.find(label) + label.length() + 1, output.length() - 2 - label.length())).c_str());
  else
    return -1;
}

int readInEstimates(std::vector<img> &images, std::string path)
{
  for (unsigned int imageIndex = 0; imageIndex < images.size(); imageIndex++)
  {
    // Show progress (for large image sets)
    std::cout << std::endl;
    std::cout << std::endl;
    std::cout << "Processing: image " << imageIndex << "/" << images.size() << std::endl;
    std::cout << "[";
    for (unsigned int i = 0; i < floor((imageIndex / (float)images.size()) * 30); i++)
      std::cout << "|";
    for (unsigned int i = 0; i < ceil(((images.size() - imageIndex) / (float)images.size()) * 30); i++)
      std::cout << " ";
    std::cout << "]" << std::endl;
    std::cout << std::endl;

    // run read_text
    std::string imgpath = path.substr(0, path.find_last_of("/") + 1);
    imgpath.append(images[imageIndex].img_name);
    std::string cmd_ = ros::package::getPath("cob_read_text") + "/bin/run_detect " + imgpath + " "
        + ros::package::getPath("cob_read_text_data") + "/fonts/new_correlation.txt "
        + ros::package::getPath("cob_read_text_data") + "/dictionary/full-dictionary_ger";//_ger";	//todo: make dictionary path a parameter

    cmd_.append(" eval");

    std::cout << "cmd_: " << cmd_ << std::endl;

    if (system(cmd_.c_str()) != 0)
      std::cout << "Error occurred while running text_detect" << std::endl;

    // get read_text results
    std::ifstream ocrfile;
    std::string textname = path.substr(0, path.find_last_of("/") + 1)
        + images[imageIndex].img_name.substr(0, images[imageIndex].img_name.find_last_of(".")) + ".txt";
    ocrfile.open(textname.c_str());
    if (!ocrfile)
    {
      std::cout << "#1 While opening read_text results file an error is encountered" << std::endl;
      std::cout << "The image has to be in same folder as the .xml-file!" << std::endl;
      return -1;
    }

    std::string input;

    // read x,y,width,height
    if (ocrfile.is_open())
    {
      for (int line = 0; !ocrfile.eof(); line++)
      {
        ocrfile >> input;
        static int x, y, width, height;

        std::cout << "input: " << input << std::endl;

        // every fifth line the structure repeats
        if (line % 4 == 0)
          x = atoi(input.c_str());
        if (line % 4 == 1)
          y = atoi(input.c_str());
        if (line % 4 == 2)
          width = atoi(input.c_str());
        if (line % 4 == 3)
        {
          height = atoi(input.c_str());
          cv::RotatedRect * r = new cv::RotatedRect(cv::Point2f(x + 0.5 * width, y + 0.5 * height), cv::Size(width,
                                                                                                             height),
                                                    0.0);
          // images[imageIndex].setEstimatedRect(*r);
          std::cout << std::endl;
        }
      }
      ocrfile.close();
    }

    // delete text file
    cmd_ = "rm " + textname;
    if (system(cmd_.c_str()) != 0)
      std::cout << "Error occurred while deleting textfile with results!" << std::endl;

    // open file with texts

    textname = path.substr(0, path.find_last_of("/") + 1)
        + images[imageIndex].img_name.substr(0, images[imageIndex].img_name.find_last_of(".")) + "t.txt";
    ocrfile.open(textname.c_str());
    if (!ocrfile)
    {
      std::cout << "#2 While opening read_text results file an error is encountered" << std::endl;
      std::cout << "The image has to be in same folder as the .xml-file!" << std::endl;
      return -1;
    }

    // read texts
    if (ocrfile.is_open())
    {
      std::string s;
      while (getline(ocrfile, s))
      {
        if (s.size() > 0)
          s.resize(s.size() - 1);
        images[imageIndex].setEstimatedText(s);
        std::cout << "input text: " << s << std::endl;
      }
      ocrfile.close();
    }

    // delete text file
    cmd_ = "rm " + textname;
    if (system(cmd_.c_str()) != 0)
      std::cout << "Error occured while deleting textfile with results!" << std::endl;

    // open file with rotated rect

    textname = path.substr(0, path.find_last_of("/") + 1)
        + images[imageIndex].img_name.substr(0, images[imageIndex].img_name.find_last_of(".")) + "r.txt";
    ocrfile.open(textname.c_str());
    if (!ocrfile)
    {
      std::cout << "#2 While opening read_text results file an error is encountered" << std::endl;
      std::cout << "The image has to be in same folder as the .xml-file!" << std::endl;
      return -1;
    }

    if (ocrfile.is_open())
    {
      for (int line = 0; !ocrfile.eof(); line++)
      {
        ocrfile >> input;
        static int x, y, width, height, angle;

        std::cout << "input: " << input << std::endl;

        // every sixth line the structure repeats
        if (line % 5 == 0)
          x = atoi(input.c_str());
        if (line % 5 == 1)
          y = atoi(input.c_str());
        if (line % 5 == 2)
          width = atoi(input.c_str());
        if (line % 5 == 3)
        {
          height = atoi(input.c_str());
        }
        if (line % 5 == 4)
        {
          angle = atoi(input.c_str());
          cv::RotatedRect * r = new cv::RotatedRect(cv::Point2f(x, y), cv::Size(width, height), angle);
          images[imageIndex].setEstimatedRect(*r);
          std::cout << std::endl;
        }
      }
      ocrfile.close();
    }

    // delete text file
    cmd_ = "rm " + textname;
    if (system(cmd_.c_str()) != 0)
      std::cout << "Error occured while deleting textfile with results!" << std::endl;

    std::vector<cv::Point> abc;
    abc.push_back(cv::Point(0, 0));
    abc.push_back(cv::Point(10, 0));
    abc.push_back(cv::Point(0, 10));
    std::cout << "contour: " << cv::contourArea(abc) << std::endl;

  }
  return 0;
}

void readInSolution(std::vector<img> &images, std::string filename)
{
  //xml-labels
  std::string label[] = {"<imageName>", "center_x=", "center_y=", "width=", "height=", "text=", "angle="};
  std::ifstream imgxml;
  imgxml.open(filename.c_str());
  if (imgxml.is_open())
  {
    int x = 0, y = 0, width = 0, height = 0, angle = 0;
    std::string filename, text, output;
    while (!imgxml.eof())
    {
      imgxml >> output;

      // label[0] = "<imageName>"  => new image is described in xmlfile, copy filename
      if (output.find(label[0]) != std::string::npos)
      {
        filename
            = output.substr(output.find(label[0]) + label[0].length(), output.length() - 2 * label[0].length() - 1);
        img * i = new img(filename);
        images.push_back(*i);
      }

      // label[1]..label[4]: x=, y=, width=, height= in xmlfile,, copy them
      if (cutout(label[1], output) != -1)
        x = cutout(label[1], output);
      if (cutout(label[2], output) != -1)
        y = cutout(label[2], output);
      if (cutout(label[3], output) != -1)
        width = cutout(label[3], output);
      if (cutout(label[4], output) != -1)
        height = cutout(label[4], output);
      if (cutout(label[6], output) != -1)
        angle = cutout(label[6], output);

      // label[5]: text= in xmlfile
      if (output.find(label[5]) != std::string::npos)
      {
        if ((output[output.length() - 1] != '"'))
        {
          text.append(output.substr(output.find(label[5]) + label[5].length() + 1, output.length() - label[5].length()
              - 1));
          text.append(" ");
          imgxml >> output;
          while ((output[output.length() - 1] != '"'))
          {
            text.append(output);
            text.append(" ");
            imgxml >> output;
          }
          text.append(output.substr(0, output.length() - 1));
        }
        else
          text.append(output.substr(output.find(label[5]) + label[5].length() + 1, output.length() - label[5].length()
              - 2));

        cv::RotatedRect * r = new cv::RotatedRect(cv::Point2f(x, y), cv::Size(width, height), angle);
        images[img::count - 1].setRect(*r);
        images[img::count - 1].setText(text);
        text = "";
      }
    }
    imgxml.close();
  }
}

void showRects(std::vector<img> &images, std::string path)
{
  bool showWords = true;
  // show also witch words were found, only in combination with OCR software

  cv::Scalar green(50, 200, 50, 0), blue(200, 50, 50, 0), red(50, 50, 200, 0);
  for (unsigned int imageIndex = 0; imageIndex < images.size(); imageIndex++)
  {
    //show image with all boxes
    std::string imgpath = path.substr(0, path.find_last_of("/") + 1);
    imgpath.append(images[imageIndex].img_name);
    cv::Mat OriginalImage_ = cv::imread(imgpath);

    int newCols = 600;

    //if OriginalImage is too high or there are so many words that they'll not fit on the image -> more columns
    cv::Size textSize = getTextSize("[", cv::FONT_HERSHEY_SIMPLEX, 0.75, 2, 0);
    if ((textSize.height * images[imageIndex].estimatedTexts.size() + textSize.height
        * images[imageIndex].correctTexts.size() + 600) > (unsigned int)OriginalImage_.rows
        || (unsigned int)OriginalImage_.rows > 1000)
      newCols = 1200;

    cv::Mat Image_(OriginalImage_.rows, OriginalImage_.cols + newCols, OriginalImage_.type());
    Image_.setTo(cv::Scalar(0, 0, 0, 0));
    cv::Mat roi = cv::Mat(Image_, cv::Rect(cv::Point(0, 0), OriginalImage_.size()));
    OriginalImage_.copyTo(roi);

    //  Show correct ones
    cv::putText(Image_, "Correct:", cv::Point(OriginalImage_.cols + 25, 25), cv::FONT_HERSHEY_SIMPLEX, 0.75, green,
                1.5, 8, false);
    for (unsigned int i = 0; i < images[imageIndex].correctRects.size(); i++)
    {
      std::string output;
      std::stringstream out;
      out << i;
      output = out.str();

      cv::Point2f vertices[4];
      images[imageIndex].correctRects[i].points(vertices);
      for (int j = 0; j < 4; j++)
        cv::line(Image_, vertices[j], vertices[(j + 1) % 4], green);

      cv::putText(Image_, output, cv::Point(images[imageIndex].correctRects[i].center.x + 0.5
          * images[imageIndex].correctRects[i].size.width, images[imageIndex].correctRects[i].center.y - 0.5
          * images[imageIndex].correctRects[i].size.height), cv::FONT_HERSHEY_SIMPLEX, 0.75, green, 2, 8, false);
      out << ": [ " << images[imageIndex].correctRects[i].center.x << " | "
          << images[imageIndex].correctRects[i].center.y << " | " << images[imageIndex].correctRects[i].size.width
          << " | " << images[imageIndex].correctRects[i].size.height << " | "
          << images[imageIndex].correctRects[i].angle << "]";
      output = out.str();
      cv::putText(Image_, output, cv::Point(OriginalImage_.cols + 5, 25 + 25 * (i + 1)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  green, 1.5, 8, false);
      cv::putText(Image_, images[imageIndex].correctTexts[i], cv::Point(OriginalImage_.cols + 300, 25 + 25 * (i + 1)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, green, 1.5, 8, false);

      if (showWords)
        if (images[imageIndex].wordWasFound[i] == true)
          cv::putText(Image_, " [found]", cv::Point(OriginalImage_.cols + 400, 25 + 25 * (i + 1)),
                      cv::FONT_HERSHEY_SIMPLEX, 0.5, green, 1.5, 8, false);

    }

    // Show estimated ones
    int offset = 25 * (images[imageIndex].correctRects.size() + 1) + 75;
    cv::putText(Image_, "Estimates:", cv::Point(OriginalImage_.cols + 25, offset), cv::FONT_HERSHEY_SIMPLEX, 0.75,
                blue, 1.5, 8, false);
    for (unsigned int i = 0; i < images[imageIndex].estimatedRects.size(); i++)
    {
      std::string output;
      std::stringstream out;
      out << i;
      output = out.str();

      cv::Point2f vertices[4];
      images[imageIndex].estimatedRects[i].points(vertices);
      for (int j = 0; j < 4; j++)
        cv::line(Image_, vertices[j], vertices[(j + 1) % 4], blue);

      cv::putText(Image_, output, cv::Point(images[imageIndex].estimatedRects[i].center.x + 0.5
          * images[imageIndex].estimatedRects[i].size.width, images[imageIndex].estimatedRects[i].center.y - 0.5
          * images[imageIndex].estimatedRects[i].size.height), cv::FONT_HERSHEY_SIMPLEX, 0.75, blue, 2, 8, false);
      out << ": [ " << (int)images[imageIndex].estimatedRects[i].center.x << " | "
          << (int)images[imageIndex].estimatedRects[i].center.y << " | "
          << (int)images[imageIndex].estimatedRects[i].size.width << " | "
          << (int)images[imageIndex].estimatedRects[i].size.height << " | "
          << (int)images[imageIndex].estimatedRects[i].angle << "]";

      output = out.str();
      cv::putText(Image_, output, cv::Point(OriginalImage_.cols + 5, offset + 25 * (i + 1)), cv::FONT_HERSHEY_SIMPLEX,
                  0.5, blue, 1.5, 8, false);
      if (showWords)
        if (images[imageIndex].estimatedTexts.size() > i)
          cv::putText(Image_, images[imageIndex].estimatedTexts[i], cv::Point(OriginalImage_.cols + 300, offset + 25
              * (i + 1)), cv::FONT_HERSHEY_SIMPLEX, 0.5, blue, 1.5, 8, false);

    }

    // Show results
    offset += 25 * (images[imageIndex].estimatedRects.size() + 1) + 75;

    int offset_x = 0;
    //if image was made wider at the beginning of the function, results has to be placed at another location:
    if (newCols == 1200)
    {
      offset = 0;
      offset_x = 600;
    }
    cv::putText(Image_, "Results:", cv::Point(OriginalImage_.cols + 25 + offset_x, offset), cv::FONT_HERSHEY_SIMPLEX,
                0.75, red, 1.5, 8, false);
    cv::putText(Image_, "Words recognized:", cv::Point(OriginalImage_.cols + 5 + offset_x, offset + 50),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 1.5, 8, false);
    cv::putText(Image_, "Wrong words found:", cv::Point(OriginalImage_.cols + 5 + offset_x, offset + 100),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 1.5, 8, false);
    cv::putText(Image_, "Precision:", cv::Point(OriginalImage_.cols + 5 + offset_x, offset + 150),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 1.5, 8, false);
    cv::putText(Image_, "Recall:", cv::Point(OriginalImage_.cols + 5 + offset_x, offset + 200),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 1.5, 8, false);
    cv::putText(Image_, "standard f:", cv::Point(OriginalImage_.cols + 5 + offset_x, offset + 250),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 1.5, 8, false);

    std::string output;
    std::stringstream out;
    out << std::setprecision(4);
    if (showWords)
    {
      out << images[imageIndex].foundWordsRatio * 100 << "%";
      output = out.str();
      cv::putText(Image_, output, cv::Point(OriginalImage_.cols + 200 + offset_x, offset + 50),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 1.5, 8, false);
      out.str("");
      out << images[imageIndex].wrongWordsRatio * 100 << "%";
      output = out.str();
      cv::putText(Image_, output, cv::Point(OriginalImage_.cols + 200 + offset_x, offset + 100),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 1.5, 8, false);
      out.str("");
    }
    out << images[imageIndex].precision * 100 << "%";
    output = out.str();
    cv::putText(Image_, output, cv::Point(OriginalImage_.cols + 200 + offset_x, offset + 150),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 1.5, 8, false);
    out.str("");
    out << images[imageIndex].recall * 100 << "%";
    output = out.str();
    cv::putText(Image_, output, cv::Point(OriginalImage_.cols + 200 + offset_x, offset + 200),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 1.5, 8, false);
    out.str("");
    out << images[imageIndex].f * 100 << "%";
    output = out.str();
    cv::putText(Image_, output, cv::Point(OriginalImage_.cols + 200 + offset_x, offset + 250),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, red, 1.5, 8, false);

    // Show/Save result images:
    imgpath = path.substr(0, path.find_last_of("/") + 1);
    imgpath.append("results/");
    static bool folderCreated = false;
    if (folderCreated == false)
    {
      std::string cmd = "mkdir " + imgpath + " -p";
      if (system(cmd.c_str()) != 0)
        std::cout << "Error occured while executing mkdir!" << std::endl;
      folderCreated = true;
    }
    cv::imwrite(imgpath + images[imageIndex].img_name, Image_);
    std::string winName = "Evaluation: " + images[imageIndex].img_name;
    cv::imshow(winName, Image_);
    cvMoveWindow(winName.c_str(), 0, 0);
    cv::waitKey(0);
  }
}

void calculateBoxResults(std::vector<img> &images, std::string path, float alpha)
{
  for (unsigned int imageIndex = 0; imageIndex < images.size(); imageIndex++)
  {
    unsigned int numberEstimatedRects = images[imageIndex].estimatedRects.size();
    unsigned int numberCorrectRects = images[imageIndex].correctRects.size();
    float allMatches = 0;

    //int inside[images[imageIndex].correctRects.size()];
    //which correct rects are completely inside an estimated rect

    for (unsigned int rectIndex = 0; rectIndex < images[imageIndex].estimatedRects.size(); rectIndex++)
    {
      // compare match from every found rectangle j in ocrImages[imageIndex] and rectangle k in images[imageIndex]
      float bestMatch = 0;
      int bestK = 0;

      // Precision
      for (unsigned int k = 0; k < images[imageIndex].correctRects.size(); k++)
      {
        // Calculate bestMatch: which one of the estimated rects has highest ratio: (intersection)/(min.possible box containing both rects)

        // for upright rectangle
        //        float intersection = (images[imageIndex].estimatedRects[rectIndex] & images[imageIndex].correctRects[k]).area();
        //        float minBox = (images[imageIndex].estimatedRects[rectIndex] | images[imageIndex].correctRects[k]).area();

        // for rotated rectangle
        float intersection = rotatedRectangleIntersection(images[imageIndex].estimatedRects[rectIndex],
                                                          images[imageIndex].correctRects[k]);
        float minBox = rotatedRectangleMinBox(images[imageIndex].estimatedRects[rectIndex],
                                              images[imageIndex].correctRects[k]);

        float match = intersection / minBox;
        if (match > bestMatch)
        {
          bestMatch = match;
          bestK = k;
        }
        //
        //        if (intersection / images[imageIndex].correctRects[k].area() == 1.0)
        //          inside[k] = 1;
      }

      // bestMatch for ocrImages[imageIndex].rects[rectIndex] and images[imageIndex].rects[bestK]
      allMatches += bestMatch;
    }

    float precision;
    if (numberEstimatedRects > 0)
      precision = allMatches / numberEstimatedRects;
    else
      precision = 1.f; // todo: makes sense?
    images[imageIndex].precision = precision;

    allMatches = 0;

    // Recall
    for (unsigned int j = 0; j < images[imageIndex].correctRects.size(); j++)
    {
      // compare match from every rectangle j in images[imageIndex] and found rectangle k in ocrImages[imageIndex]
      float bestMatch = 0;
      int bestK = 0;

      for (unsigned int k = 0; k < images[imageIndex].estimatedRects.size(); k++)
      {
        // for upright rectangle
        //        float intersection = (images[imageIndex].estimatedRects[k] & images[imageIndex].correctRects[j]).area();
        //        float minBox = (images[imageIndex].estimatedRects[k] | images[imageIndex].correctRects[j]).area();

        // for rotated rectangle
        float intersection = rotatedRectangleIntersection(images[imageIndex].estimatedRects[k],
                                                          images[imageIndex].correctRects[j]);
        float minBox = rotatedRectangleMinBox(images[imageIndex].estimatedRects[k], images[imageIndex].correctRects[j]);

        float match = intersection / minBox;
        if (match > bestMatch)
        {
          bestMatch = match;
          bestK = k;
        }
      }
      // bestMatch for ocrImages[imageIndex].rects[bestK]
      allMatches += bestMatch;
    }
    float recall;
    if (numberCorrectRects > 0)
      recall = allMatches / numberCorrectRects;
    else
      recall = 1.f; // do not punish the algorithm on text-free images

    images[imageIndex].recall = recall;

    // standard f measure combines precision and recall
    float f = 1 / ((alpha / precision) + ((1 - alpha) / recall));
    images[imageIndex].f = f;
  }
}

double rotatedRectangleMinBox(const cv::RotatedRect& rect1, const cv::RotatedRect& rect2)
{
  // compute corners of rotated rectangles
  std::vector<cv::Point2f> vertices(8);
  cv::Point2f v1[4], v2[4];
  rect1.points(v1);
  rect2.points(v2);
  for (int i = 0; i < 4; i++)
  {
    vertices[i] = v1[i];
    vertices[4 + i] = v2[i];
  }

  // determine minimum bounding box
  cv::RotatedRect minBox = cv::minAreaRect(vertices);

  // debug output
  if (false)
  {
    cv::Mat image(300, 500, CV_8UC3);
    image.setTo(cv::Scalar(0, 0, 0));
    cv::Point2f mb[4];
    minBox.points(mb);
    for (int i = 0; i < 4; i++)
    {
      line(image, v1[i], v1[(i + 1) % 4], cv::Scalar(255, 0, 0));
      line(image, v2[i], v2[(i + 1) % 4], cv::Scalar(0, 0, 255));
    }
    for (int i = 0; i < 4; i++)
      line(image, mb[i], mb[(i + 1) % 4], cv::Scalar(0, 255, 0));
    cv::imshow("minBox", image);
    std::cout << "minBox.area: " << minBox.size.height * minBox.size.width << std::endl;
  }

  return minBox.size.height * minBox.size.width;
}

double rotatedRectangleIntersection(const cv::RotatedRect& rect1, const cv::RotatedRect& rect2)
{
  std::vector<cv::Point2f> intersectionContour; // will contain all contour points of the intersection

  // compute corners of rotated rectangles
  std::vector<cv::Point2f> vertices1(4), vertices2(4);
  cv::Point2f v1[4], v2[4];
  rect1.points(v1);
  rect2.points(v2);
  for (int i = 0; i < 4; i++)
  {
    vertices1[i] = v1[i];
    vertices2[i] = v2[i];
  }

  // compute line vectors (direction from vertices[i] to vertices[(i+1)%4])
  std::vector<cv::Point2f> lines1(4), lines2(4);
  for (int i = 0; i < 4; i++)
  {
    lines1[i] = vertices1[(i + 1) % 4] - vertices1[i];
    lines2[i] = vertices2[(i + 1) % 4] - vertices2[i];
  }

  // process first rectangle (search for inner points of rect1 in rect2 and intersections, i.e. clip rect1 with rect2)
  rotatedRectangleIntersectionOneWay(vertices1, vertices2, lines1, lines2, intersectionContour);

  // process second rectangle (search for inner points of rect2 in rect1 and intersections, i.e. clip rect2 with rect1)
  rotatedRectangleIntersectionOneWay(vertices2, vertices1, lines2, lines1, intersectionContour);

  // make intersectionContour an ordered list
  std::vector<cv::Point2f> intersectionContourOrdered;
  cv::convexHull(intersectionContour, intersectionContourOrdered);

  // compute area
  double intersectionArea = cv::contourArea(intersectionContourOrdered);

  // debug output
  if (false)
  {
    cv::Mat image(300, 500, CV_8UC3);
    image.setTo(cv::Scalar(0, 0, 0));
    for (int i = 0; i < 4; i++)
    {
      cv::line(image, v1[i], v1[(i + 1) % 4], cv::Scalar(255, 0, 0));
      cv::line(image, v2[i], v2[(i + 1) % 4], cv::Scalar(0, 0, 255));
    }
    for (int i = 0; i < (int)intersectionContourOrdered.size(); i++)
    {
      cv::line(image, intersectionContourOrdered[i], intersectionContourOrdered[(i + 1)
          % (int)intersectionContourOrdered.size()], cv::Scalar(0, 255, 0));
    }
    cv::imshow("intersection", image);
    std::cout << "intersection.area: " << intersectionArea << std::endl;
  }

  return intersectionArea;
}

// process one rectangle (search for inner points of rect1 in rect2 and intersections, i.e. clip rect1 with rect2)
void rotatedRectangleIntersectionOneWay(const std::vector<cv::Point2f>& vertices1,
                                        const std::vector<cv::Point2f>& vertices2,
                                        const std::vector<cv::Point2f>& lines1, const std::vector<cv::Point2f>& lines2,
                                        std::vector<cv::Point2f>& intersectionContour)
{
  bool innerPoints1[4]; // stores whether the corners of rect1 are inner points in rect2
  // define a local coordinate system for rectangle 2
  cv::Point2f rectangleOrigin = vertices2[0];
  cv::Point2f rectangleVectorX = lines2[0];
  cv::Point2f rectangleVectorY = lines2[1];
  // check whether vertices1[i] is an inner point of rect2
  for (int i = 0; i < 4; i++)
    innerPoints1[i] = isInnerPoint(rectangleOrigin, rectangleVectorX, rectangleVectorY, vertices1[i]);
  // check for intersection points
  for (int i = 0; i < 4; i++)
  {
    if (innerPoints1[i] == true)
    {
      addContourPoint(vertices1[i], intersectionContour);
      if (innerPoints1[(i + 1) % 4] == false)
      {
        // there is exactly one intersection
        cv::Point2f intersectionPoint;
        for (int j = 0; j < 4; j++)
          if (true == lineSegmentIntersection(vertices1[i], lines1[i], vertices2[j], lines2[j], intersectionPoint))
            addContourPoint(intersectionPoint, intersectionContour);
      }
    }
    else
    {
      if (innerPoints1[(i + 1) % 4] == true)
      {
        // there is exactly one intersection
        cv::Point2f intersectionPoint;
        for (int j = 0; j < 4; j++)
          if (true == lineSegmentIntersection(vertices1[i], lines1[i], vertices2[j], lines2[j], intersectionPoint))
            addContourPoint(intersectionPoint, intersectionContour);
      }
      else
      {
        // there are either no or two intersections
        cv::Point2f intersectionPoint;
        for (int j = 0; j < 4; j++)
          if (true == lineSegmentIntersection(vertices1[i], lines1[i], vertices2[j], lines2[j], intersectionPoint))
            addContourPoint(intersectionPoint, intersectionContour);
      }
    }
  }
}

void addContourPoint(const cv::Point2f& point, std::vector<cv::Point2f>& contour)
{
  bool pointInList = false;
  for (int i = 0; i < (int)contour.size(); i++)
  {
    if (contour[i] == point)
    {
      pointInList = true;
      break;
    }
  }

  if (pointInList == false)
    contour.push_back(point);
}

bool isInnerPoint(const cv::Point2f& rectangleOrigin, const cv::Point2f& rectangleVectorX,
                  const cv::Point2f& rectangleVectorY, const cv::Point2f& point)
{
  cv::Point2f PA0 = point - rectangleOrigin;
  cv::Mat A(2, 2, CV_32F), B(2, 1, CV_32F), X(2, 1, CV_32F);
  A.at<float> (0, 0) = rectangleVectorX.x;
  A.at<float> (1, 0) = rectangleVectorX.y;
  A.at<float> (0, 1) = rectangleVectorY.x;
  A.at<float> (1, 1) = rectangleVectorY.y;
  B.at<float> (0, 0) = PA0.x;
  B.at<float> (1, 0) = PA0.y;
  cv::solve(A, B, X);
  if (X.at<float> (0, 0) >= 0.f && X.at<float> (0, 0) <= 1.f && X.at<float> (1, 0) >= 0.f && X.at<float> (1, 0) <= 1.f)
    return true; // point is inner point in rectangle

  return false;
}

bool lineSegmentIntersection(const cv::Point2f& basePoint1, const cv::Point2f& lineSegmentVector1,
                             const cv::Point2f& basePoint2, const cv::Point2f& lineSegmentVector2,
                             cv::Point2f& intersectionPoint)
{
  // parallel line segments do not have one unique intersection
  if (abs(lineSegmentVector1.y - lineSegmentVector1.x / lineSegmentVector2.x * lineSegmentVector2.y) < 1e-8)
    return false;

  cv::Point2f P1P2 = basePoint1 - basePoint2;
  cv::Mat A(2, 2, CV_32F), B(2, 1, CV_32F), X(2, 1, CV_32F);
  A.at<float> (0, 0) = -lineSegmentVector1.x;
  A.at<float> (1, 0) = -lineSegmentVector1.y;
  A.at<float> (0, 1) = lineSegmentVector2.x;
  A.at<float> (1, 1) = lineSegmentVector2.y;
  B.at<float> (0, 0) = P1P2.x;
  B.at<float> (1, 0) = P1P2.y;
  cv::solve(A, B, X);
  if (X.at<float> (0, 0) >= 0.f && X.at<float> (0, 0) <= 1.f && X.at<float> (1, 0) >= 0.f && X.at<float> (1, 0) <= 1.f)
  {
    // both line segments intersect
    intersectionPoint = basePoint1 + X.at<float> (0, 0) * lineSegmentVector1;
    return true;
  }

  return false;
}

void calculateWordResults(std::vector<img> &images)
{
  for (unsigned int imageIndex = 0; imageIndex < images.size(); imageIndex++)
  {
    unsigned int foundWords = 0;
    std::vector<std::string> brokenWords; // single words

    //Breaking into words
    for (unsigned int j = 0; j < images[imageIndex].estimatedTexts.size(); j++)
    {
      std::string dummyString(images[imageIndex].estimatedTexts[j]);
      if (dummyString.find(" ") != std::string::npos) // find empty spaces in estimated words -> separate ĺine into words
        while (dummyString.find(" ") != std::string::npos) // more than one empty space possible
        {
          if ((dummyString.substr(0, dummyString.find(" ")).compare("") != 0)) // if it is not an empty string
            brokenWords.push_back(dummyString.substr(0, dummyString.find(" ")));
          dummyString = dummyString.substr(dummyString.find(" ") + 1, dummyString.length() - dummyString.find(" "));
          if (dummyString.find(" ") == std::string::npos) //if there is no other empty space remaining -> just save the last word
            if (dummyString.compare("") != 0)
              brokenWords.push_back(dummyString);
        }
      else
        brokenWords.push_back(images[imageIndex].estimatedTexts[j]); // if there are no empty spaces, push the whole text in brokenWords
    }

    // Take solution word and look up whether its in the estimated set
    for (unsigned int j = 0; j < images[imageIndex].correctTexts.size(); j++)
    {
      bool found = false;
      for (unsigned int k = 0; k < images[imageIndex].estimatedTexts.size(); k++)
        if ((images[imageIndex].estimatedTexts[k]).find(images[imageIndex].correctTexts[j]) != std::string::npos)
        {
          foundWords++;
          images[imageIndex].wordWasFound.push_back(true);
          found = true;
          break;
        }
      if (found == false)
        images[imageIndex].wordWasFound.push_back(false);
    }

    images[imageIndex].foundWordsRatio = foundWords / (float)images[imageIndex].correctTexts.size();
    images[imageIndex].wrongWordsRatio = brokenWords.size() > 0 ? (brokenWords.size() - foundWords)
        / (float)brokenWords.size() : 0;
  }
}

std::vector<float> printAverageResults(std::vector<img> &images, std::string path)
{
  std::vector<float> results;
  //calculate average of all images:
  float averagePrecision = 0;
  float averageRecall = 0;
  float averageWords = 0;
  for (unsigned int i = 0; i < images.size(); i++)
  {
    averagePrecision += images[i].precision;
    averageRecall += images[i].recall;
    averageWords += images[i].foundWordsRatio;
  }
  averagePrecision /= images.size();
  averageRecall /= images.size();
  averageWords /= images.size();

  // output
  std::cout << "------------------" << std::endl;
  std::cout << "PRECISION: " << std::setprecision(4) << averagePrecision * 100 << "%" << std::endl;
  std::cout << "RECALL: " << std::setprecision(4) << averageRecall * 100 << "%" << std::endl;
  std::cout << "WORDS RECOGNIZED: " << std::setprecision(4) << averageWords * 100 << "%" << std::endl;
  std::cout << "------------------" << std::endl;

  cv::destroyAllWindows();
  cv::Mat Image_(450, 450, CV_8UC3);
  Image_.setTo(cv::Scalar(0, 0, 0, 0));
  cv::putText(Image_, "PRECISION: ", cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200, 0), 1.5,
              8, false);
  cv::putText(Image_, "RECALL: ", cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200, 0), 1.5,
              8, false);
  cv::putText(Image_, "WORDS RECOGNIZED: ", cv::Point(10, 300), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200,
                                                                                                        0), 1.5, 8,
              false);
  std::string output;
  std::stringstream out;
  out << std::setprecision(4) << averagePrecision * 100 << "%";
  output = out.str();
  cv::putText(Image_, output, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200, 0), 1.5, 8,
              false);
  out.str("");
  out << std::setprecision(4) << averageRecall * 100 << "%";
  output = out.str();
  cv::putText(Image_, output, cv::Point(10, 240), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200, 0), 1.5, 8,
              false);
  out.str("");
  out << std::setprecision(4) << averageWords * 100 << "%";
  output = out.str();
  cv::putText(Image_, output, cv::Point(10, 360), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(200, 200, 200, 0), 1.5, 8,
              false);
  std::string resultPath = path.substr(0, path.find_last_of("/") + 1);
  resultPath.append("results/");
  cv::imwrite(resultPath + "allResults.jpg", Image_);
  cv::imshow("Results", Image_);
  cv::waitKey(0);
  results.push_back(averagePrecision);
  results.push_back(averageRecall);
  results.push_back(averageWords);
  return results;
}

bool fOrder(img a, img b)
{
  return a.f > b.f;
}

void writeAllResultsInTxt(std::vector<float> results, std::vector<img> &images, std::string path)
{
  std::ofstream resultFile;
  std::string textname = path.substr(0, path.find_last_of("/") + 1);
  textname.append("results/results.txt");
  resultFile.open(textname.c_str());

  resultFile << std::setprecision(4);

  // Average for all
  resultFile << "Average for all images: " << std::endl;
  resultFile << "--------------------------------" << std::endl;
  resultFile << "Precision: " << results[0] << std::endl;
  resultFile << "Recall: " << results[1] << std::endl;
  resultFile << "Words recognized: " << results[2] << std::endl;
  resultFile << std::endl;
  resultFile << std::endl;

  // Sort standard f measure
  std::sort(images.begin(), images.end(), fOrder);

  // Every image:
  for (unsigned int i = 0; i < images.size(); i++)
  {
    resultFile << images[i].img_name << ":" << std::endl;
    resultFile << "--------------------------------" << std::endl;
    resultFile << "Precision: " << images[i].precision << std::endl;
    resultFile << "Recall: " << images[i].recall << std::endl;
    resultFile << "Standard f measure: " << images[i].f << std::endl;
    resultFile << "Words recognized: " << images[i].foundWordsRatio << std::endl;
    resultFile << std::endl;
  }
  resultFile.close();

}

int main(int argc, char **argv)
{
  // rectangle intersection test
  if (false)
  {
    char key = 0;
    while (key != 'q')
    {
      for (int angle = 0; angle < 361; angle += 30)
      {
        for (int width = 20; width < 150; width += 40)
        {
          cv::RotatedRect rect1 = cv::RotatedRect(cv::Point2f(100, 100), cv::Size2f(100, 50), 0);
          cv::RotatedRect rect2 = cv::RotatedRect(cv::Point2f(100, 100), cv::Size2f(width, 50), angle);

          rotatedRectangleIntersection(rect1, rect2);
          rotatedRectangleMinBox(rect1, rect2);

          key = cv::waitKey();
        }
      }
    }
  }

  if (argc < 2)
  {
    ROS_ERROR( "not enought input: eval_read_text <img_list_name.xml>");
    return 1;
  }

  float alpha = 0.5;

  std::vector<img> images;

  //read everything from img_list.xml in images
  readInSolution(images, argv[1]);

  //run read_text and write results in ocrImages
  readInEstimates(images, argv[1]);

  std::cout << "Number of images that were processed: " << images.size() << std::endl;

  std::cout << images[0].correctRects.size() << " ground truth rects: " << std::endl;
  for (unsigned int i = 0; i < images[0].correctRects.size(); i++)
    std::cout << images[0].correctRects[i].center.x << " " << images[0].correctRects[i].center.y << " "
        << images[0].correctRects[i].size.width << " " << images[0].correctRects[i].size.height << " "
        << images[0].correctRects[i].angle << " " << images[0].correctTexts[i] << std::endl;

  std::cout << "Cob_read_text found " << images[0].estimatedRects.size() << " rects: " << std::endl;
  for (unsigned int i = 0; i < images[0].estimatedRects.size(); i++)
    std::cout << images[0].estimatedRects[i].center.x << " " << images[0].estimatedRects[i].center.y << " "
        << images[0].estimatedRects[i].size.width << " " << images[0].estimatedRects[i].size.height << " "
        << images[0].estimatedRects[i].angle << " " << images[0].estimatedTexts[i] << std::endl;

  //calculate precision, recall, f
  calculateBoxResults(images, argv[1], alpha);

  //calculate how many words got recognized
  calculateWordResults(images);

  //show everything
  showRects(images, argv[1]);

  //print everything to stdout and show final result image for all images
  std::vector<float> results = printAverageResults(images, argv[1]);

  //save all results in a txt file in results directory
  writeAllResultsInTxt(results, images, argv[1]);

  return 0;
}
