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
  void setRect(cv::Rect r)
  {
    correctRects.push_back(r);
  }
  void setText(std::string t)
  {
    correctTexts.push_back(t);
  }
  void setEstimatedRect(cv::Rect r)
  {
    estimatedRects.push_back(r);
  }
  void setEstimatedText(std::string t)
  {
    estimatedTexts.push_back(t);
  }

  std::string img_name;
  std::vector<cv::Rect> correctRects, estimatedRects;
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
        + ros::package::getPath("cob_read_text_data") + "/fonts/correlation.txt "
        + ros::package::getPath("cob_read_text_data") + "/dictionary/full-dictionary";//_ger";	//todo: make dictionary path a parameter

    if (system(cmd_.c_str()) != 0)
      std::cout << "Error occurred while running text_detect" << std::endl;
    ;

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
          cv::Rect * r = new cv::Rect(x, y, width, height);
          images[imageIndex].setEstimatedRect(*r);
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
      }
      ocrfile.close();
    }

    // delete text file
    cmd_ = "rm " + textname;
    if (system(cmd_.c_str()) != 0)
      std::cout << "Error occured while deleting textfile with results!" << std::endl;
  }
  return 0;
}

void readInSolution(std::vector<img> &images, std::string filename)
{
  //xml-labels
  std::string label[] = {"<imageName>", "x=", "y=", "width=", "height=", "text="};
  std::ifstream imgxml;
  imgxml.open(filename.c_str());
  if (imgxml.is_open())
  {
    int x = 0, y = 0, width = 0, height = 0;
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
        cv::Rect * r = new cv::Rect(x, y, width, height);
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

      cv::rectangle(Image_, cv::Point(images[imageIndex].correctRects[i].x, images[imageIndex].correctRects[i].y),
                    cv::Point(images[imageIndex].correctRects[i].x + images[imageIndex].correctRects[i].width,
                              images[imageIndex].correctRects[i].y + images[imageIndex].correctRects[i].height), green,
                    1);
      cv::putText(Image_, output, cv::Point(images[imageIndex].correctRects[i].x
          + images[imageIndex].correctRects[i].width + 10, images[imageIndex].correctRects[i].y - 10),
                  cv::FONT_HERSHEY_SIMPLEX, 0.75, green, 2, 8, false);
      out << ": [ " << images[imageIndex].correctRects[i].x << " | " << images[imageIndex].correctRects[i].y << " | "
          << images[imageIndex].correctRects[i].width << " | " << images[imageIndex].correctRects[i].height << " ]";
      output = out.str();
      cv::putText(Image_, output, cv::Point(OriginalImage_.cols + 5, 25 + 25 * (i + 1)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  green, 1.5, 8, false);
      cv::putText(Image_, images[imageIndex].correctTexts[i], cv::Point(OriginalImage_.cols + 250, 25 + 25 * (i + 1)),
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

      cv::rectangle(Image_, cv::Point(images[imageIndex].estimatedRects[i].x, images[imageIndex].estimatedRects[i].y),
                    cv::Point(images[imageIndex].estimatedRects[i].x + images[imageIndex].estimatedRects[i].width,
                              images[imageIndex].estimatedRects[i].y + images[imageIndex].estimatedRects[i].height),
                    blue, 1);
      cv::putText(Image_, output, cv::Point(images[imageIndex].estimatedRects[i].x
          + images[imageIndex].estimatedRects[i].width, images[imageIndex].estimatedRects[i].y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.75, blue, 2, 8, false);

      out << ": [ " << images[imageIndex].estimatedRects[i].x << " | " << images[imageIndex].estimatedRects[i].y
          << " | " << images[imageIndex].estimatedRects[i].width << " | "
          << images[imageIndex].estimatedRects[i].height << " ]";
      output = out.str();
      cv::putText(Image_, output, cv::Point(OriginalImage_.cols + 5, offset + 25 * (i + 1)), cv::FONT_HERSHEY_SIMPLEX,
                  0.5, blue, 1.5, 8, false);
      if (showWords)
        if (images[imageIndex].estimatedTexts.size() > i)
          cv::putText(Image_, images[imageIndex].estimatedTexts[i], cv::Point(OriginalImage_.cols + 250, offset + 25
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
        float intersection = (images[imageIndex].estimatedRects[rectIndex] & images[imageIndex].correctRects[k]).area();
        float minBox = (images[imageIndex].estimatedRects[rectIndex] | images[imageIndex].correctRects[k]).area();
        float match = intersection / minBox;
        if (match > bestMatch)
        {
          bestMatch = match;
          bestK = k;
        }

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
      precision = 1.f;		// todo: makes sense?
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
        float intersection = (images[imageIndex].estimatedRects[k] & images[imageIndex].correctRects[j]).area();
        float minBox = (images[imageIndex].estimatedRects[k] | images[imageIndex].correctRects[j]).area();
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
      recall = 1.f;		// do not punish the algorithm on text-free images

    images[imageIndex].recall = recall;

    // standard f measure combines precision and recall
    float f = 1 / ((alpha / precision) + ((1 - alpha) / recall));
    images[imageIndex].f = f;
  }
}


void calculateWordResults(std::vector<img> &images)
{
  for (unsigned int imageIndex = 0; imageIndex < images.size(); imageIndex++)
  {
    unsigned int foundWords = 0;
    std::vector<std::string> brokenWords;		// single words

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
  if (argc < 2)
  {
    ROS_ERROR( "not enought input: eval_read_text <img_list_name.xml>");
    return 1;
  }

  float alpha = 0.5;

  std::cout << "test123" << std::endl;

  std::vector<img> images;

  //read everything from img_list.xml in images
  readInSolution(images, argv[1]);

  //run read_text and write results in ocrImages
  readInEstimates(images, argv[1]);

  std::cout << "Number of images that were processed: " << images.size() << std::endl;

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
