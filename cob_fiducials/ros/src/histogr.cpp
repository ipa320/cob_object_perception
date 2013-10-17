/*
 * histogr.cpp
 *
 *
 *  Created on: 27.03.2013
 *      Author: Matthias Nösner
 *
 * Used to test the usage of histogramms for pre segementation of marker area
 */

#include <ros/ros.h>
#include <opencv/cv.h>
#include "opencv/highgui.h"
#include <stdlib.h>
#include "opencv2/imgproc/imgproc.hpp"


static const char WINDOW[] = "Image window";
using namespace cv;


cv::MatND generateHistogramm(cv::Mat img);
cv::Mat visualizeHistogramm(cv::MatND hist);

void showthumb(cv::Mat img, float scale, char* title);
void fourierTrafo(cv::Mat img);
Mat createGausFilterMask(Size mask_size, int x, int y, int ksize, bool normalization, bool invert);
void shift(Mat magI);
void cosinusTrafo(cv::Mat I);
size_t getOptimalDCTSize(size_t N);
void init();

int histSize[1];
float hranges[2];
const float* ranges[1];
int channels[1];

//-------init---------------------------------------------------------------------------
void init(){
	histSize[0] = 256;
	hranges[0] = 0.0;
	hranges[1] = 255.0;
	ranges[0] = hranges;
	channels[0] = 0;
}
// -----------------Discrete Cosinus Transform------------------------------------
void cosinusTrafo(cv::Mat I){

    Mat padded;                            //expand input image to optimal size
    int m = getOptimalDCTSize( I.rows );
    int n = getOptimalDCTSize( I.cols ); // on the border add zero values
    copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));

    Mat src;
    padded.copyTo(src);
    //padded.convertTo(src,CV_64FC1);
	Mat dst = I.clone();

	cv::dct(src,dst);
	imshow("Input Image", src);
	imshow("Cosinus Trafo", dst);
}
size_t getOptimalDCTSize(size_t N) { return 2*getOptimalDFTSize((N+1)/2); }

//------------------Discrete Fourier----------------------------------------------

void fourierTrafo(cv::Mat I){

	//DFT
    Mat padded;                            //expand input image to optimal size
    int m = getOptimalDFTSize( I.rows );
    int n = getOptimalDFTSize( I.cols ); // on the border add zero values
    copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, BORDER_CONSTANT, Scalar::all(0));


    Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
    Mat complexI;
    merge(planes, 2, complexI);         // Add to the expanded another plane with zeros

    dft(complexI, complexI);            // this way the result may fit in the source matrix
    Mat fft;
    complexI.copyTo(fft);

    // Prozessing in Frequency space
    //fft

    Mat mask = createGausFilterMask(fft.size(), 5, 5, 20, true, true);
    shift(mask);  // rearrange quadrants of mask

    Mat planes_p[] = {Mat::zeros(fft.size(), CV_32F), Mat::zeros(fft.size(), CV_32F)};
    Mat kernel_spec;
    planes_p[0] = mask; // real
    planes_p[1] = mask; // imaginar
    merge(planes_p, 2, kernel_spec);

    mulSpectrums(fft, kernel_spec, fft, DFT_ROWS); //only DFT_ROWS flag is accepted


    //IDFT
    std::vector<Mat> chans;
    cv::split(fft, chans);
    chans[1].zeros(I.size(), CV_64FC1);
    cv::merge(chans, fft);
    cv::Mat invFFT;
    cv::idft(fft, invFFT, DFT_REAL_OUTPUT + DFT_SCALE);

    cv::Rect roi(0,0,I.cols,I.rows);
    Mat arangedimg;
    invFFT(roi).copyTo(arangedimg);
    showthumb(arangedimg,0.3,"After Inverse Output");

//    cv::Rect roii(10,10,10,10);
//    fft(roi).copyTo(complexI);

    //Visualize DFT
    // compute the magnitude and switch to logarithmic scale
    // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
    split(complexI, planes);                   // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
    magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
    Mat magI = planes[0];

    magI += Scalar::all(1);                    // switch to logarithmic scale
    log(magI, magI);

    // crop the spectrum, if it has an odd number of rows or columns
    magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));

    // rearrange the quadrants of Fourier image  so that the origin is at the image center
    int cx = magI.cols/2;
    int cy = magI.rows/2;

    Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
    Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
    Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right

    Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);

    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);

    normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a
                                            // viewable image form (float between values 0 and 1).


    showthumb(I,0.3,"Input Image");
    showthumb(magI,0.3,"spectrum magnitude");

}

Mat createGausFilterMask(Size mask_size, int x, int y, int ksize, bool normalization, bool invert) {
	// Some corrections if out of bounds
	if(x < (ksize / 2)) {
		ksize = x * 2;
	}
	if(y < (ksize / 2)) {
		ksize = y * 2;
	}
	if(mask_size.width - x < ksize / 2 ) {
		ksize = (mask_size.width - x ) * 2;
	}
	if(mask_size.height - y < ksize / 2 ) {
		ksize = (mask_size.height - y) * 2;
	}

	// call openCV gaussian kernel generator
	double sigma = -1;
	Mat kernelX = getGaussianKernel(ksize, sigma, CV_32F);
	Mat kernelY = getGaussianKernel(ksize, sigma, CV_32F);
	// create 2d gaus
	Mat kernel = kernelX * kernelY.t();
	// create empty mask
	Mat mask = Mat::zeros(mask_size, CV_32F);
	Mat maski = Mat::zeros(mask_size, CV_32F);

	// copy kernel to mask on x,y
	Mat pos(mask, Rect(x - ksize / 2, y - ksize / 2, ksize, ksize));
	kernel.copyTo(pos);

	// create mirrored mask
	Mat posi(maski, Rect(( mask_size.width - x) - ksize / 2, (mask_size.height - y) - ksize / 2, ksize, ksize));
	kernel.copyTo(posi);
	// add mirrored to mask
	add(mask, maski, mask);

	// transform mask to range 0..1
	if(normalization) {
		normalize(mask, mask, 0, 1, NORM_MINMAX);
	}

	// invert mask
	if(invert) {
		mask = Mat::ones(mask.size(), CV_32F) - mask;
	}

	return mask;
}

void shift(Mat magI) {

    // crop if it has an odd number of rows or columns
	magI = magI(Rect(0, 0, magI.cols & -2, magI.rows & -2));

	int cx = magI.cols/2;
    int cy = magI.rows/2;

    Mat q0(magI, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    Mat q1(magI, Rect(cx, 0, cx, cy));  // Top-Right
    Mat q2(magI, Rect(0, cy, cx, cy));  // Bottom-Left
    Mat q3(magI, Rect(cx, cy, cx, cy)); // Bottom-Right

    Mat tmp;                            // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);                     // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);
}


//-------showthumbs----------------------------------------------------------------------
void showthumb(cv::Mat img, float scale, char* title){

	cv::Size size(cvRound(img.cols*scale),cvRound(img.rows*scale));

	cv::Mat thumbnail;
	cv::resize(img,thumbnail,size);
	cv::imshow(title,thumbnail);
}

//// -- Transform to HSV - Space ------
//cv::Mat hsv;
//std::vector<cv::Mat> v;
//cvtColor(thumbnail,hsv,CV_BGR2HSV);
//cv::split(hsv,v);
//
//cv::imshow("HSV - h", v[0]);
//cv::imshow("HSV - s", v[1]);
//cv::imshow("HSV - v", v[2]);
//
////Transform to Gray
//cv::Mat grayscale;
//cv::cvtColor(thumbnail,grayscale, CV_RGB2GRAY );
//cv::imshow("grayscale",grayscale);

//-------generateHistogramm----------------------------------------------------------------------------------------------------------
cv::MatND generateHistogramm(cv::Mat img){

	cv::MatND hist;
	cv::calcHist(&img,1,channels,cv::Mat(),hist,1,histSize,ranges);

	return hist;
}

//-------visualizeHistogramm-----------------------------------------------------------------
cv::Mat visualizeHistogramm(cv::MatND hist){

	double maxVal = 0;
	double minVal = 0;

	cv::minMaxLoc(hist,&minVal,&maxVal,0,0);

	cv::Mat histimg(histSize[0],histSize[0],CV_8U,cv::Scalar(0,0,255));

	int hpt = static_cast<int>(0.9*histSize[0]);

	for(int i = 0; i < 256 ; i++){

		float binVal = hist.at<float>(i);
		int intensity = static_cast<int>(binVal*hpt/maxVal);

		cv::line(histimg,cv::Point(i,histSize[0]),cv::Point(i,histSize[0]-intensity),cv::Scalar(255,255,255));
	}

	return histimg;
}



//------------main------------------------------------------------------------------------
int main(int argc, char** argv)
{
	init();

    ros::init(argc, argv, "histogr");
    ros::NodeHandle nh;

   // cv::namedWindow(WINDOW);
    float scale = 0.3;

    while(ros::ok()){

    	cv::Mat img = cv::imread("/home/matthias/Bilder/people+smallmarker.jpg", CV_LOAD_IMAGE_COLOR);
//   	cv::Mat img = cv::imread("/home/matthias/Bilder/marker.png", CV_LOAD_IMAGE_COLOR);
//     	cv::Mat img = cv::imread("/home/matthias/Bilder/people+marker.jpg", CV_LOAD_IMAGE_COLOR);
//    	cv::Mat img = cv::imread("/home/matthias/Bilder/planarmarker.png", CV_LOAD_IMAGE_COLOR);

       	cv::Mat grayscale;
    	cv::cvtColor(img,grayscale, CV_RGB2GRAY );
    	fourierTrafo(grayscale);
//    	cosinusTrafo(grayscale);

    	cv::MatND hist = generateHistogramm(img);
//    	cv::Mat histimg = visualizeHistogramm(hist);
//
//    	cv::imshow("Histogram",histimg);
//
//    	showthumbs(img,scale);
    	cv::waitKey(10);
    	ros::spinOnce();
    }
}
