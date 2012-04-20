#include "opencv2/core/core.hpp"
#include "cv.h"
#include "highgui.h"

#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <stack>
#include <stdio.h>
#include <dirent.h>

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
    rects.push_back(r);
  }
  void setText(std::string t)
  {
    texts.push_back(t);
  }

  std::string img_name;
  std::vector<cv::Rect> rects;
  std::vector<std::string> texts;

  static int count;
};

int img::count = 0;

int cutout(std::string label, std::string output)
{
  //cutout x,y,width,height
  if (output.find(label) != std::string::npos)
  {
    return atoi((output.substr(output.find(label) + label.length() + 1, output.length() - 2 - label.length())).c_str());
  }
  else
    return -1;
}

int readInEstimates(std::vector<img> &images, std::vector<img> &ocrImages, std::string path)
{
  for (int i = 0; i < images.size(); i++)
  {
    // new ocrImage
    img * im = new img(images[i].img_name);
    ocrImages.push_back(*im);

    // run read_text
    std::string imgpath = path.substr(0, path.find_last_of("/") + 1);
    imgpath.append(images[i].img_name);
    std::string cmd_ = "$(rospack find read_text)/bin/run_detect " + imgpath
        + " $(rospack find read_text)/fonts/correlation.txt $(rospack find read_text)/dictionary/full-dictionary";
    //system(cmd_.c_str());

    // get read_text results
    std::ifstream ocrfile;
    std::string textname = images[i].img_name.substr(0, images[i].img_name.find_last_of(".")) + ".txt";
    ocrfile.open(textname.c_str());
    if (!ocrfile)
    {
      std::cout << "While opening read_text results file an error is encountered" << std::endl;
      return -1;
    }

    std::string input;

    if (ocrfile.is_open())
    {
      for (int a = 0; !ocrfile.eof(); a++)
      {
        ocrfile >> input;
        static int x, y, width, height;

        // read x,y,width,height
        if (a % 4 == 0)
          x = atoi(input.c_str());
        if (a % 4 == 1)
          y = atoi(input.c_str());
        if (a % 4 == 2)
          width = atoi(input.c_str());
        if (a % 4 == 3)
        {
          height = atoi(input.c_str());
          cv::Rect * r = new cv::Rect(x, y, width, height);
          ocrImages[img::count - 1].setRect(*r);
        }
      }
      ocrfile.close();
    }
    textname = images[i].img_name.substr(0, images[i].img_name.find_last_of(".")) + "t.txt";
    ocrfile.open(textname.c_str());
    if (!ocrfile)
    {
      std::cout << "While opening read_text results file an error is encountered" << std::endl;
      return -1;
    }

    if (ocrfile.is_open())
    {
      std::string s;
      while (getline(ocrfile, s))
      {
        ocrImages[img::count - 1].setText(s);
      }
    }
  }
  return 0;
}

void readInSolution(std::vector<img> &images, std::string filename)
{
  std::string label[] = {"<imageName>", "x=", "y=", "width=", "height=", "text="};

  std::ifstream imgxml;
  imgxml.open(filename.c_str());
  if (imgxml.is_open())
  {
    while (!imgxml.eof())
    {
      std::string filename;
      int x, y, width, height;
      std::string text;

      std::string output;
      imgxml >> output;

      // label[0] = <imageName>  => new image starts in xmlfile
      if (output.find(label[0]) != std::string::npos)
      {
        filename
            = output.substr(output.find(label[0]) + label[0].length(), output.length() - 2 * label[0].length() - 1);
        img * i = new img(filename);
        images.push_back(*i);
      }

      // x=, y=, width=, height= in xmlfile
      if (cutout(label[1], output) != -1)
        x = cutout(label[1], output);
      if (cutout(label[2], output) != -1)
        y = cutout(label[2], output);
      if (cutout(label[3], output) != -1)
        width = cutout(label[3], output);
      if (cutout(label[4], output) != -1)
        height = cutout(label[4], output);

      // text= in xmlfile
      if (output.find(label[5]) != std::string::npos)
      {
        text = output.substr(output.find(label[5]) + label[5].length() + 1, output.length() - label[5].length() - 2);
        cv::Rect * r = new cv::Rect(x, y, width, height);
        images[img::count - 1].setRect(*r);
        images[img::count - 1].setText(text);
      }
    }
    imgxml.close();
  }
}

void deleteIdentical(std::vector<img> &ocrImages)
{
  //delete identical found rects
  std::vector<cv::Rect>::iterator it;
  for (int h = 0; h < ocrImages.size(); h++)
  {
    for (int i = 0; i < ocrImages[h].rects.size(); i++)
    {
      for (int j = i + 1; j < ocrImages[h].rects.size(); j++)
      {
        if (ocrImages[h].rects[i].x == ocrImages[h].rects[j].x && ocrImages[h].rects[i].y == ocrImages[h].rects[j].y
            && ocrImages[h].rects[i].height == ocrImages[h].rects[j].height && ocrImages[h].rects[i].width
            == ocrImages[h].rects[j].width)
        {
          it = find(ocrImages[h].rects.begin(), ocrImages[h].rects.end(), ocrImages[h].rects[j]);
          ocrImages[h].rects.erase(it);
        }
      }
    }
  }
}

void showRects(std::vector<img> &images, std::vector<img> &ocrImages, std::string path)
{
  for (int h = 0; h < ocrImages.size(); h++)
  {
    //show image with all boxes
    std::string imgpath = path.substr(0, path.find_last_of("/") + 1);
    imgpath.append(ocrImages[h].img_name);
    cv::Mat OriginalImage_ = cv::imread(imgpath);
    cv::Mat Image_(OriginalImage_.rows, OriginalImage_.cols + 600, OriginalImage_.type());
    Image_.setTo(cv::Scalar(0, 0, 0, 0));
    cv::Mat roi = cv::Mat(Image_, cv::Rect(cv::Point(0, 0), OriginalImage_.size()));
    OriginalImage_.copyTo(roi);

    cv::putText(Image_, "Found:", cv::Point(OriginalImage_.cols + 25, OriginalImage_.rows / 2 + 25),
                cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 0, 0, 0), 1.5, 8, false);
    cv::putText(Image_, "Correct:", cv::Point(OriginalImage_.cols + 25, 25), cv::FONT_HERSHEY_SIMPLEX, 0.75,
                cv::Scalar(0, 255, 0, 0), 1.5, 8, false);
    cv::putText(Image_, "Evaluation:", cv::Point(OriginalImage_.cols + 375, 25), cv::FONT_HERSHEY_SIMPLEX, 0.75,
                cv::Scalar(0, 0, 255, 0), 1.5, 8, false);
    cv::line(Image_, cv::Point(OriginalImage_.cols + 250, 0), cv::Point(OriginalImage_.cols + 250, Image_.rows),
             cv::Scalar(50, 50, 50, 0), 1, 1, 0);

    for (int i = 0; i < ocrImages[h].rects.size(); i++)
    {
      cv::rectangle(Image_, cv::Point(ocrImages[h].rects[i].x, ocrImages[h].rects[i].y),
                    cv::Point(ocrImages[h].rects[i].x + ocrImages[h].rects[i].width, ocrImages[h].rects[i].y
                        + ocrImages[h].rects[i].height), cvScalar((255 - 10 * i), (0), (0)), 2);

      std::string output;
      std::stringstream out;
      out << i << ": [ " << ocrImages[h].rects[i].x << " | " << ocrImages[h].rects[i].y << " | "
          << ocrImages[h].rects[i].width << " | " << ocrImages[h].rects[i].height << " ]";
      output = out.str();

      cv::putText(Image_, output, cv::Point(OriginalImage_.cols + 5, OriginalImage_.rows / 2 + 25 + 25 * (i + 1)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar((255 - 10 * i), 0, 0, 0), 1.5, 8, false);
    }

    for (int i = 0; i < images[h].rects.size(); i++)
    {
      cv::rectangle(Image_, cv::Point(images[h].rects[i].x, images[h].rects[i].y), cv::Point(images[h].rects[i].x
          + images[h].rects[i].width, images[h].rects[i].y + images[h].rects[i].height), cvScalar((0), (255 - 50 * i),
                                                                                                  (0)), 2);
      std::string output;
      std::stringstream out;
      out << i << ": [ " << images[h].rects[i].x << " | " << images[h].rects[i].y << " | " << images[h].rects[i].width
          << " | " << images[h].rects[i].height << " ]";
      output = out.str();

      cv::putText(Image_, output, cv::Point(OriginalImage_.cols + 5, 25 + 25 * (i + 1)), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                  cv::Scalar(0, (255 - 50 * i), 0, 0), 1.5, 8, false);
    }

    //cv::putText(Image_, out.str(), cv::Point(OriginalImage_.cols + 260, 25 + 25 * (i + 1)),
    //            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255, 0), 1.5, 8, false);
    cv::imshow(ocrImages[h].img_name, Image_);
    cv::waitKey(0);
  }
}

void calculateResults(std::vector<img> &images, std::vector<img> &ocrImages)
{
  for (int i = 0; i < images.size(); i++)
  {
    std::cout << images[i].img_name << ":" << std::endl;
    for (int j = 0; j < images[i].rects.size(); j++)
    {
      std::cout << images[i].rects[j].x << "|" << images[i].rects[j].y << "|" << images[i].rects[j].width << "|"
          << images[i].rects[j].height << " [" << images[i].texts[j] << "]" << std::endl;
    }
  }

  for (int i = 0; i < images.size(); i++)
  {
    std::cout << ocrImages[i].img_name << ":" << std::endl;
    for (int j = 0; j < ocrImages[i].rects.size(); j++)
    {
      std::cout << ocrImages[i].rects[j].x << "|" << ocrImages[i].rects[j].y << "|" << ocrImages[i].rects[j].width
          << "|" << ocrImages[i].rects[j].height << " [" << ocrImages[i].texts[j] << "]" << std::endl;
    }
  }

  for (int i = 0; i < ocrImages.size(); i++)
  {
    int retrieved = (int)ocrImages[i].rects.size();
    int relevant = (int)images[i].rects.size();

    for (int j = 0; j < ocrImages[i].rects.size(); j++)
      for (int k = 0; k < images[i].rects.size(); k++)
      {
        {
          int intersectionPixels = (ocrImages[i].rects[j] & images[i].rects[k]).area();
          unsigned int percent = (intersectionPixels / (images[i].rects[k].area())) * 100;

          if (ocrImages[i].rects[j].area() > images[i].rects[k].area())
          {

            percent -= (((ocrImages[i].rects[j].area() - images[i].rects[k].area()) / images[i].rects[k].area()) / 8)
                * 100;
          }
          std::string output;
          std::stringstream out;
          out << "overlap: correct[" << i << "] & found[" << j << "]: " << percent << "%";
        }
      }
  }
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    ROS_ERROR( "not enought input: eval_read_text <img_list.xml>");
    return 1;
  }

  std::vector<img> images;
  std::vector<img> ocrImages;

  //read everything from img_list.xml in images
  readInSolution(images, argv[1]);

  img::count = 0;

  //run read_text and write results in ocrImages
  readInEstimates(images, ocrImages, argv[1]);

  calculateResults(images, ocrImages);

  showRects(images, ocrImages, argv[1]);

  return 0;
}
