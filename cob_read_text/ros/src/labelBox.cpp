#include "opencv2/core/core.hpp"
#include "cv.h"
#include "highgui.h"

#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <stack>
#include <cstdio>
#include <dirent.h>
#include <time.h>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/fstream.hpp"

#include <time.h>

// ASCII KEY CODES
#define LEFT 1113937
#define LEFT2 65361
#define UP 65362
#define UP2 1113938
#define RIGHT 65363
#define RIGHT2 1113939
#define DOWN 65364
#define DOWN2 1113940
#define RETURN 10
#define RETURN2 1048586
#define C 99
#define C2 1048675
#define H 104
#define H2 1048680
#define S 115
#define S2 1048691
#define R 114
#define R2 1048690
#define Z 122
#define Z2 1048698
#define ESC 27
#define ESC2 1048603
#define LSHIFT 65505
#define LSHIFT2 1114081
#define RSHIFT 65506
#define RSHIFT2 1114082
#define BACKSPACE 65288
#define BACKSPACE2 1113864
#define SPACE 32
#define SPACE2 1048608
class labelImage
{
public:
  labelImage(std::string winName, cv::Mat imageWithRects, cv::Mat originalImage, cv::Scalar clr) :
    name(winName), img(imageWithRects), originalImage(originalImage), actualClr(clr), actualRect(cv::Rect(1, 1, 1, 1)),
        textMode(false)
  {

  }
  ~labelImage()
  {

  }

  std::string name;

  cv::Mat img;
  cv::Mat originalImage;
  cv::Mat temporaryImage;

  std::vector<cv::Rect> allRects;
  std::vector<std::string> allTexts;
  std::vector<cv::Scalar> allClrs;

  cv::Scalar actualClr;
  cv::Rect actualRect;

  bool textMode;

  // moves box 1 pixel in given direction and draws resulting box on parameter Mat img
  void move_box(cv::Mat img, int direction)
  {
    switch (direction)
    {
      case 1:
        if (actualRect.x > 0)
          actualRect = cv::Rect(actualRect.x - 1, actualRect.y, actualRect.width, actualRect.height);
        break;
      case 2:
        if (actualRect.y > 0)
          actualRect = cv::Rect(actualRect.x, actualRect.y - 1, actualRect.width, actualRect.height);
        break;
      case 3:
        if (actualRect.x + actualRect.width < img.cols)
          actualRect = cv::Rect(actualRect.x + 1, actualRect.y, actualRect.width, actualRect.height);
        break;
      case 4:
        if (actualRect.y + actualRect.height < img.rows)
          actualRect = cv::Rect(actualRect.x, actualRect.y + 1, actualRect.width, actualRect.height);
        break;
    }
    cv::rectangle(img, actualRect, actualClr, 1);
  }

  void resize_box(cv::Mat img, int direction)
  {
    switch (direction)
    {
      case 1:
        if (actualRect.width > 1)
          actualRect = cvRect(actualRect.x, actualRect.y, actualRect.width - 1, actualRect.height);
        break;
      case 2:
        if (actualRect.height > 1)
          actualRect = cvRect(actualRect.x, actualRect.y, actualRect.width, actualRect.height - 1);
        break;
      case 3:
        if (actualRect.x + actualRect.width < img.cols)
          actualRect = cvRect(actualRect.x, actualRect.y, actualRect.width + 1, actualRect.height);
        break;
      case 4:
        if (actualRect.y + actualRect.height < img.rows)
          actualRect = cvRect(actualRect.x, actualRect.y, actualRect.width, actualRect.height + 1);
        break;
    }
    cv::rectangle(img, actualRect, actualClr, 1);
  }

};

int input_text(cv::Mat img, std::string winName, std::string * text, cv::Rect rect)
{
  std::string completeText;
  int completeTextWidth = 0;
  cv::waitKey(10);
  int space = 0;

  for (;;)
  {
    int c = cv::waitKey(0);
    if (c == ESC || c == ESC2)
      return 0;
    else if (c == RETURN || c == RETURN2)
      break;
    else if (c == BACKSPACE || c == BACKSPACE2)
    {
      if (completeText.length() > 0)
      {
        std::string sub = completeText.substr(completeText.length() - 1, 1);
        cv::Size textSize = cv::getTextSize(sub, cv::FONT_HERSHEY_SIMPLEX, 0.75, 1, 0);
        cv::rectangle(img, cv::Rect(rect.x + completeTextWidth - textSize.width + 1, rect.y - 60 + 1, 200
            + textSize.width - completeTextWidth - 2, /*r.height-2*/30 - 2), cv::Scalar(255, 255, 255), -1);
        completeText = completeText.substr(0, completeText.length() - 1);
        cv::imshow(winName, img);
        space -= textSize.width;
        completeTextWidth -= textSize.width;
      }
    }
    else if (c != LSHIFT && c != RSHIFT && c != LSHIFT2 && c != RSHIFT2)
    {
      std::stringstream ss;
      std::string s;
      if (c == SPACE)
        s = " ";
      else
      {
        ss << (char)c;
        ss >> s;
      }
      cv::putText(img, s, cv::Point(rect.x + 1 + space, rect.y - 40 + 1), cv::FONT_HERSHEY_SIMPLEX, 0.75,
                  cv::Scalar(0, 0, 0), 1, 8, false);
      cv::imshow(winName, img);
      cv::Size textSize = cv::getTextSize(s, cv::FONT_HERSHEY_SIMPLEX, 0.75, 1, 0);
      space += textSize.width;
      completeText += s;
      completeTextWidth += textSize.width;
    }
  }
  std::cout << "Text: " << completeText << std::endl;
  *text = completeText.data();
  return 1;
}

void draw_box(cv::Mat img, cv::Rect rect, cv::Scalar clr)
{
  cv::rectangle(img, rect, clr, 1);
}

void showInfo(std::string winName, cv::Mat img, cv::Rect r, std::string text, cv::Scalar clr)
{
  cv::Mat infoBox = img.clone();
  cv::rectangle(infoBox, cv::Rect(r.x, r.y - 60, 200, 30), cv::Scalar(255, 255, 255), -1);
  cv::rectangle(infoBox, cv::Rect(r.x, r.y - 60, 200, 30), clr);
  cv::putText(infoBox, text, cv::Point(r.x + 1, r.y - 40 + 1), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 0), 1,
              8, false);
  cv::imshow(winName, infoBox);
  cv::waitKey(0);
}

void onMouse(int event, int x, int y, int flags, void* param)
{
  static bool onRect = false;
  static int firstX = 0, firstY = 0;
  static int action = 0;

  cv::Mat image = ((labelImage *)param)->img.clone();
  std::string winName = ((labelImage *)param)->name;
  std::vector<cv::Rect> * allRects = &((labelImage *)param)->allRects;

  // if no text is written at the moment
  if (((labelImage *)param)->textMode == false)
  {
    // Left mouse button clicked
    if (event == cv::EVENT_LBUTTONDOWN)
    {
      onRect = false;
      int whichRect = -1;

      // check if other rect was clicked
      for (unsigned int i = 0; i < allRects->size(); i++)
      {
        if (x >= (((labelImage *)param)->allRects)[i].x && x <= (((labelImage *)param)->allRects)[i].x
            + (*allRects)[i].width && y >= (((labelImage *)param)->allRects)[i].y && y
            <= (((labelImage *)param)->allRects)[i].y + (((labelImage *)param)->allRects)[i].height)
        {
          onRect = true;
          whichRect = i;
        }
      }
      // no other rect was clicked -> start new box, remember x,y
      if (!onRect)
      {
        action = 1;
        firstX = x;
        firstY = y;
      }
      // other rect was clicked -> show what text was written in there
      else
      {
        showInfo(winName, image, (((labelImage *)param)->allRects)[whichRect],
                 (((labelImage *)param)->allTexts)[whichRect], ((labelImage*)param)->allClrs[whichRect]);
      }
    }
    if (event == cv::EVENT_LBUTTONUP)
    {
      if (!onRect)
      {
        action = 0;
        ((labelImage *)param)->actualRect = cv::Rect(firstX > x ? x : firstX, firstY > y ? y : firstY, firstX > x
            ? (firstX - x) : (x - firstX), firstY > y ? (firstY - y) : (y - firstY));
        draw_box(image, ((labelImage*)param)->actualRect, ((labelImage*)param)->actualClr);
        cv::imshow(winName, image);
      }

    }
    if (event == cv::EVENT_MOUSEMOVE)
    {
      if (action == 1)
      {
        ((labelImage*)param)->actualRect = cv::Rect(firstX > x ? x : firstX, firstY > y ? y : firstY, firstX > x
            ? (firstX - x) : (x - firstX), firstY > y ? (firstY - y) : (y - firstY));
        draw_box(image, ((labelImage*)param)->actualRect, ((labelImage*)param)->actualClr);
        cv::imshow(winName,/* *((cv::Mat*)param)*/image);
      }
      else
      {
        for (unsigned int i = 0; i < allRects->size(); i++)
        {
          if (x >= (((labelImage *)param)->allRects)[i].x && x <= (((labelImage *)param)->allRects)[i].x
              + (((labelImage *)param)->allRects)[i].width && y >= (((labelImage *)param)->allRects)[i].y && y
              <= (((labelImage *)param)->allRects)[i].y + (((labelImage *)param)->allRects)[i].height)
          {
            //std::cout << "show info" << std::endl;
          }
        }
      }
    }
    if (event == cv::EVENT_RBUTTONDOWN) //DELETE BOX
    {
      for (unsigned int i = 0; i < allRects->size(); i++)
      {
        if (x >= (((labelImage *)param)->allRects)[i].x && x <= (((labelImage *)param)->allRects)[i].x
            + (((labelImage *)param)->allRects)[i].width && y >= (((labelImage *)param)->allRects)[i].y && y
            <= (((labelImage *)param)->allRects)[i].y + (((labelImage *)param)->allRects)[i].height)
        {
          cv::Mat roi = cv::Mat(((labelImage *)param)->img, (((labelImage *)param)->allRects)[i]);
          (((labelImage *)param)->originalImage((((labelImage *)param)->allRects)[i])).copyTo(roi);
          std::cout << "deleted text: " << (((labelImage *)param)->allTexts)[i] << std::endl;
          ((labelImage *)param)->allRects.erase(((labelImage *)param)->allRects.begin() + i);
          ((labelImage *)param)->allTexts.erase(((labelImage *)param)->allTexts.begin() + i);
          ((labelImage *)param)->allClrs.erase(((labelImage *)param)->allClrs.begin() + i);
          for (unsigned int j = 0; j < ((labelImage *)param)->allRects.size(); j++)
          {
            cv::rectangle(((labelImage*)param)->img, (((labelImage *)param)->allRects)[j],
                          ((labelImage*)param)->allClrs[j], 1);
          }
          cv::imshow(winName, ((labelImage *)param)->img);
          break;
        }
      }
    }
  }
}

void writeTxt(std::vector<labelImage> all, std::string path, std::string actTime)
{
  std::ifstream inn;
  std::string str;
  std::string word;

  std::cout << "path: " << path << std::endl;

  std::string newFileName = path + "/";
 // newFileName.append(actTime);
  newFileName.append("train.xml");

  std::ofstream newFile;
  newFile.open(newFileName.c_str());
  newFile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>";
  newFile << std::endl;
  newFile << "<tagset>";
  newFile << std::endl;
  for (unsigned int j = 0; j < all.size(); j++)
  {
    std::vector<std::string> words = all[j].allTexts;
    std::vector<cv::Rect> rects = all[j].allRects;
    std::string imgName = all[j].name;
    imgName = imgName.substr(imgName.find_last_of(' ') + 1, imgName.size() - imgName.find_last_of(' '));
    imgName = imgName.substr(imgName.find_last_of('/') + 1, imgName.size() - imgName.find_last_of('/'));
    newFile << "<image>" << std::endl << " <imageName>" << imgName << "</imageName>" << std::endl
        << "<taggedRectangles>" << std::endl;
    for (unsigned int i = 0; i < rects.size(); i++)
    {
      newFile << "<taggedRectangle x=\"" << rects[i].x << "\" y=\"" << rects[i].y << "\" width=\"" << rects[i].width
          << "\" height=\"" << rects[i].height << "\" modelType=\"1\" text=\"" << words[i] << "\" />" << std::endl;
    }
    newFile << "</taggedRectangles>" << std::endl << "</image>" << std::endl;
  }
  newFile << "</tagset>";
}

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cout << "error: not enough input parameters!" << std::endl;
    return -1;
  }

  std::vector<std::string> allImageNames;

  std::string arg(argv[1]);
  std::string imgpath = arg.substr(0, arg.find_last_of("/") + 1);
  if (imgpath.empty())
  {
    boost::filesystem::path full_path(boost::filesystem::current_path());
    imgpath = full_path.directory_string();
  }


  cv::Mat test(1000, 1000, CV_8UC3);
  cv::putText(test, arg, cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(100, 100, 100), 2, 8, false);
  cv::imwrite("/home/rmb-rh/test.png", test);

  // read all image names in the folder
  boost::filesystem::path input_path(argv[1]);
  if (boost::filesystem::is_directory(input_path))
  {
    DIR *pDIR;
    struct dirent *entry;
    if ((pDIR = opendir(argv[1])))
      while ((entry = readdir(pDIR)))
        if (strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0)
        {
          std::string s = argv[1];
          s.append("/");
          s.append(entry->d_name);
          allImageNames.push_back(s);
        }
  }
  else
    allImageNames.push_back(argv[1]); //single image to be processed

  std::cout << "Images to be processed: " << allImageNames.size() << std::endl;

  std::vector<labelImage> allImages;

  for (unsigned int i = 0; i < allImageNames.size(); i++)
  {
    cv::Mat originalImage = cv::imread(allImageNames[i]); // originalImage like it used to be, never modify
    cv::Mat imageWithRects = cv::imread(allImageNames[i]); // image with all rects the user selected

    // Show originalImage
    std::string winName = "labelBox - ";
    winName.append(allImageNames[i]);
    cv::imshow(winName.c_str(), originalImage);
    cvMoveWindow(winName.c_str(), 0, 0);

    // labelImage object for everything
    labelImage labelImg(winName, imageWithRects, originalImage, cv::Scalar(0, 0, 0));

    // Mouse CB function
    cv::setMouseCallback(winName, onMouse, (void*)&labelImg);

    bool active = true;

    // loop till next image
    while (active)
    {
      cv::Mat temporaryImg;
      static bool shift = true;

      int c = cv::waitKey(0);

      temporaryImg = labelImg.img.clone();
      labelImg.temporaryImage = labelImg.img.clone();

      // Keyboard Input
      switch (c)
      {
        // Direction Keys for moving or resizing the box
        case LEFT:
        case LEFT2:
        {
          if (shift)
          {
            labelImg.move_box(temporaryImg, 1);
            cv::imshow(winName, temporaryImg);
          }
          else
          {
            labelImg.resize_box(temporaryImg, 1);
            cv::imshow(winName, temporaryImg);
          }
          break;
        }

        case UP:
        case UP2:
        {
          if (shift)
          {
            labelImg.move_box(temporaryImg, 2);
            cv::imshow(winName, temporaryImg);
          }
          else
          {
            labelImg.resize_box(temporaryImg, 2);
            cv::imshow(winName, temporaryImg);
          }
          break;
        }
        case RIGHT:
        case RIGHT2:
          if (shift)
          {
            labelImg.move_box(temporaryImg, 3);
            cv::imshow(winName, temporaryImg);
          }
          else
          {
            labelImg.resize_box(temporaryImg, 3);
            cv::imshow(winName, temporaryImg);
          }
          break;

        case DOWN:
        case DOWN2:
          if (shift)
          {
            labelImg.move_box(temporaryImg, 4);
            cv::imshow(winName, temporaryImg);
          }
          else
          {
            labelImg.resize_box(temporaryImg, 4);
            cv::imshow(winName, temporaryImg);
          }
          break;

          // Enter Text Mode
        case RETURN:
        case RETURN2:
        {
          cv::Mat textBox = labelImg.img.clone();
          cv::rectangle(textBox, cv::Rect(labelImg.actualRect.x, labelImg.actualRect.y - 60, 200, 30), cv::Scalar(255,
                                                                                                                  255,
                                                                                                                  255),
                        -1);
          cv::rectangle(textBox, cv::Rect(labelImg.actualRect.x, labelImg.actualRect.y - 60, 200, 30),
                        labelImg.actualClr);
          cv::rectangle(textBox, cv::Rect(labelImg.actualRect.x, labelImg.actualRect.y, labelImg.actualRect.width,
                                          labelImg.actualRect.height), labelImg.actualClr);
          cv::imshow(winName, textBox);

          labelImg.textMode = true;
          std::string text;
          if (input_text(textBox, winName, &text, labelImg.actualRect))
          {
            cv::rectangle(labelImg.img, cv::Rect(labelImg.actualRect.x, labelImg.actualRect.y,
                                                 labelImg.actualRect.width, labelImg.actualRect.height),
                          labelImg.actualClr);
            labelImg.allRects.push_back(labelImg.actualRect);
            labelImg.allTexts.push_back(text);
            labelImg.allClrs.push_back(labelImg.actualClr);
            shift = true;
          }
          else
          {
            cv::rectangle(temporaryImg, cv::Rect(labelImg.actualRect.x, labelImg.actualRect.y,
                                                 labelImg.actualRect.width, labelImg.actualRect.height),
                          labelImg.actualClr);
          }
          labelImg.textMode = false;

          temporaryImg = labelImg.img.clone();
          cv::imshow(winName, temporaryImg);
          break;
        }

          // [C]olor change
        case C:
        case C2:
        {
          srand(time(NULL));
          int c1, c2, c3;
          c1 = rand() % 255;
          c2 = (c1 * c1) % 255;
          c3 = (c1 * c2) % 255;
          labelImg.actualClr = cv::Scalar(c1, c2, c3);
          draw_box(temporaryImg, labelImg.actualRect, labelImg.actualClr);
          cv::imshow(winName, temporaryImg);
          break;
        }

          //[S]hift between Moving Box Mode and Resizing Box Mode
        case S:
        case S2:
          if (shift)
            shift = false;
          else
            shift = true;
          break;

          /* [Z]oom
           case Z:
           cv::Mat zoomedImg = input2(r);
           cv::pyrUp(zoomedImg, zoomedImg, cv::Size(zoomedImg.cols * 2, zoomedImg.rows * 2));
           cv::imshow(winName, zoomedImg);
           break;
           */

          // [R]eset complete image
        case R:
        case R2:
          labelImg.img = labelImg.originalImage.clone();
          cv::imshow(winName, labelImg.img);
          labelImg.allRects.clear();
          labelImg.allTexts.clear();

          // [H]elp
        case H:
        case H2:
        {
          std::cout << std::endl;
          std::cout << "labelBox Help" << std::endl;
          std::cout << std::endl;
          std::cout << "* Draw box with mouse." << std::endl;
          std::cout << "* Press direction keys to move/resize drawn box before entering text." << std::endl;
          std::cout << "* Press Return for entering text in drawn and resized box." << std::endl;
          std::cout << "* Press right mouse button to delete a box (after text was written inside)." << std::endl;
          std::cout << "* Press left mouse button on box to show text." << std::endl;
          std::cout << "* Press [s] for switching between moving and resizing with direction keys." << std::endl;
          std::cout << "* Press [c] for color change." << std::endl;
          std::cout << "* Press [r] to reset complete image." << std::endl;
          std::cout << "* Press [ESC] to quit/move to next image." << std::endl;
          std::cout << std::endl;
          break;
        }
          // [Esc]ape to next image
        case ESC:
        case ESC2:
          active = false;
          break;
        case Z:
        case Z2:
        {
          std::cout << "Rects: " << labelImg.allRects.size() << std::endl;
        }
      }

    }
    allImages.push_back(labelImg);
  }
  std::cout << "Saving Data." << std::endl;

  time_t t;
  t = time(NULL);
  std::stringstream ss;
  std::string actTime;
  ss << t;
  ss >> actTime;

  writeTxt(allImages, imgpath, actTime);
  return 0;

}
