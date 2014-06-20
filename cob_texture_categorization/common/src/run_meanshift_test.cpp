#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc_c.h"


#include "cob_texture_categorization/run_meanshift_test.h"
#include "cob_texture_categorization/meanshift.h"
#include "cob_texture_categorization/meanshift_3d.h"

using namespace cv;


run_meanshift_test::run_meanshift_test()
{

}

void run_meanshift_test::run_test(cv::Mat* image, cv::Mat depth, std::vector < std::vector<cv::Mat> >* regions)
{
//IplImage *img = cvLoadImage("/home/rmb-dh/obst.jpg");
IplImage *img = new IplImage(*image);

cv::Mat imageO = (*image).clone();
IplImage *imgO = new IplImage(imageO);


// Mean shift
int **ilabels = new int *[img->height];
for(int i=0;i<img->height;i++) ilabels[i] = new int [img->width];
int regionCount = MeanShift3D(img, ilabels, &depth);
vector<int> color(regionCount);
CvRNG rng= cvRNG(cvGetTickCount());
for(int i=0;i<regionCount;i++)
color[i] = cvRandInt(&rng);

// Mean shift3
//int **ilabelsO = new int *[img->height];
//for(int i=0;i<imgO->height;i++) ilabelsO[i] = new int [imgO->width];
//int regionCountO = MeanShift(imgO, ilabelsO, &depth);
//vector<int> colorO(regionCountO);
//CvRNG rngO= cvRNG(cvGetTickCount());
//for(int i=0;i<regionCountO;i++)
//colorO[i] = cvRandInt(&rngO);

(*regions).resize(regionCount);
for(int i=0;i<regionCount;i++)
{
	(*regions)[i].resize(2);
	(*regions)[i][0] = cv::Mat::zeros(480,640,CV_8UC3);
	(*regions)[i][1] = cv::Mat::zeros(480,640,CV_32F);
}



// Draw random color
for(int i=0;i<img->height;i++)
for(int j=0;j<img->width;j++)
{
//int cl = ilabels[i][j];
//((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 0] = (color[cl])&255;
//((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 1] = (color[cl]>>8)&255;
//((uchar *)(img->imageData + i*img->widthStep))[j*img->nChannels + 2] = (color[cl]>>16)&255;

//int clO = ilabelsO[i][j];
//((uchar *)(imgO->imageData + i*imgO->widthStep))[j*imgO->nChannels + 0] = (colorO[clO])&255;
//((uchar *)(imgO->imageData + i*imgO->widthStep))[j*imgO->nChannels + 1] = (colorO[clO]>>8)&255;
//((uchar *)(imgO->imageData + i*imgO->widthStep))[j*imgO->nChannels + 2] = (colorO[clO]>>16)&255;

int reg = ilabels[i][j];
(*regions)[reg][0].at<cv::Vec3b>(i,j)[0]=(*image).at<cv::Vec3b>(i,j)[0];
(*regions)[reg][0].at<cv::Vec3b>(i,j)[1]=(*image).at<cv::Vec3b>(i,j)[1];
(*regions)[reg][0].at<cv::Vec3b>(i,j)[2]=(*image).at<cv::Vec3b>(i,j)[2];

(*regions)[reg][1].at<float>(i,j)=depth.at<float>(i,j);


}


//cvNamedWindow("MeanShift",CV_WINDOW_AUTOSIZE);
//cvShowImage("MeanShift3D",img);
//cv::moveWindow("MeanShift3D", 10,10);
//cvShowImage("MeanShift",imgO);
//cv::moveWindow("MeanShift", 800,10);


//cvWaitKey();

//cvDestroyWindow("MeanShift");
//
//cvReleaseImage(&img);


}
