#include <opencv2/opencv.hpp>

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
#define A 97
#define A2 666665
#define D 100
#define D2 666666

void onMouse(int event, int x, int y, int flags, void* param);

class labelImage
{
protected:
	int input_text(cv::Mat img, std::string winName, std::string * text, cv::Rect rect);

public:
	labelImage(std::string winName, cv::Mat imageWithRects, cv::Mat originalImage, cv::Scalar clr) :
		name(winName), img(imageWithRects), originalImage(originalImage), actualClr(clr),
		actualRect(cv::Rect_<float>(1.0, 1.0, 1.0, 1.0)), middlePoint(0.0, 0.0), textMode(false), rotationMode(false)
	{

	}
	~labelImage()
	{

	}

	void labelingLoop();

	static void writeTxt(std::vector<labelImage> all, std::string path, std::string actTime);

	static bool checkPointInsideRect(cv::RotatedRect rect, cv::Point2f p);

	static void showInfo(std::string winName, cv::Mat img, cv::RotatedRect r, std::string text, cv::Scalar clr);

	static void drawRect(cv::Mat & img, cv::RotatedRect rrect, cv::Scalar clr);

	std::string name;

	cv::Mat img;
	cv::Mat originalImage;
	//cv::Mat temporaryImage;

	std::vector<cv::RotatedRect> allRects;
	std::vector<std::string> allTexts;
	std::vector<cv::Scalar> allClrs;

	cv::Scalar actualClr;
	cv::Rect_<float> actualRect;
	cv::RotatedRect actualRotatedRect;

	cv::Point2f middlePoint;

	bool textMode;

	bool rotationMode;
};
