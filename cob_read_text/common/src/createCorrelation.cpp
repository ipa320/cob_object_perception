#include <iostream>
#include <fstream>
#include <sstream>
#include <stack>
#include <cstdio>
#include <dirent.h>

// OpenCV includes
#include "opencv2/core/core.hpp"
#include "cv.h"
#include "highgui.h"

#define HAVE_QT

int main(int argc, char* argv[])
{
  cv::Mat fontImage(100, 100, CV_8UC3, cv::Scalar(255, 255, 255));
  //cv::Foncv::fontQt("Times",12, cv::Scalar(0,0,0), 50, CV_STYLE_NORMAL, 0);
  //for(unsigned int i = 48; i < 110; i++)
  //  {
  //    std::string s;
  //    std::stringstream ss;
  //    ss << "ä";
  //    s = ss.str();
  //    std::cout << "s: " << s << std::endl;
  //    cv::putText(fontImage, s, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 4, cv::Scalar(0, 0, 0), 3, 1, 0);
  //    cv::imshow("fontImage", fontImage);
  //    cv::waitKey(0);
  //    fontImage.setTo(cv::Scalar(255, 255, 255));
  //  }
  return 0;
}
