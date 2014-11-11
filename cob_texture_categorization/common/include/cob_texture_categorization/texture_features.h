/*
 * texture_features.h
 *
 *  Created on: 18.12.2013
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef TEXTURE_FEATURES_H_
#define TEXTURE_FEATURES_H_

//#include <cob_texture_categorization/texture_categorization.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

struct color_vals
{
	double s_mean;
	double s_std;
	double v_mean;
	double v_std;
	int colorfulness;
	double dom_color;
	double dom_color2;
};
struct amadasun_values
{
	double coars;
	double contr;
	double busyn;
	double compl_val;
	double stren;
};
struct feature_results
{
	double colorfulness;		//Value 1: colorfulness
	double dom_color;			//Value 2: dominant color
	double dom_color2;			//Value 3: secondary dominant color
	double v_mean;				//Value 4: value/lightness/brightness
	double v_std;				//Value 5: variety of value/lightness/brightness
	double s_mean;				//Value 6: saturation
	double s_std;				//Value 7: variety of saturation
	double avg_size;			//Value 8: average primitive size
	double prim_num;			//Value 9: number of primitives
	double prim_strength;		//Value 10: strength of primitives
	double prim_regularity;		//Value 11: regularity of primitives
	double contrast;			//Value 12: contrast
	double line_likeness;		//Value 13: line-likeness
	double roughness;			//Value 14: 3D roughness
	double direct_reg;			//Value 15: directionality/regularity
	double lined;				//Value 16: lined
	double checked;				//Value 17: checked

	void setTo(double val)
	{
		colorfulness = val;
		dom_color = val;
		dom_color2 = val;
		v_mean = val;
		v_std = val;
		s_mean = val;
		s_std = val;
		avg_size = val;
		prim_num = val;
		prim_strength = val;
		prim_regularity = val;
		contrast = val;
		line_likeness = val;
		roughness = val;
		direct_reg = val;
		lined = val;
		checked = val;
	}
};


class texture_features
{
public:
	texture_features();
	~texture_features();
	void compute_151617(cv::Mat *image,  struct feature_results *results, std::vector <std::vector <cv::Point> > *circle_contours, cv::Mat *edge_pixels, std::vector<double> *eccentricity, std::vector<cv::RotatedRect> *ellipse_ecc, std::vector< std::vector<double> > *centroid);
	void primitive_size(cv::Mat *img, struct feature_results *results, cv::Mat* raw_features=0);
	void compute_features(cv::Mat *img, struct feature_results *results);
	void size_of_primitive(std::vector <std::vector <cv::Point> > *contours, struct feature_results *results, std::vector<int> *numPixels, double *big_comp_val);
	void compute_contours(cv::Mat *img, int canny_op1, int canny_op2, float resize_op1, float resize_op2, bool center, std::vector<int>* numPixels, std::vector<int> *idx, cv::Mat *detected_edges, std::vector <std::vector <cv::Point> > *contours, bool sort);
	void number_of_primitives(std::vector <std::vector <cv::Point> > *contours, struct feature_results *results, cv::Mat *edge_pixels, cv::Mat *img, std::vector<int>* numPixels, std::vector<int>* idx, double *big_comp);
	void strength_of_primitives(std::vector<int> *idx,std::vector <std::vector <cv::Point> > *contours, struct feature_results *results, cv::Mat *edge_pixels, cv::Mat *img, std::vector<int>* numPixels);
	void compute_regularity_of_primitives(std::vector <std::vector <cv::Point> > *contours, std::vector<int>* numPixels, cv::Mat *edge_pixels, struct feature_results *results, std::vector< std::vector<double> > *centroid, std::vector<int> *idx );
	void linelikeness_of_primitives(cv::Mat image, std::vector <std::vector <cv::Point> > *contours, cv::Mat *edge_pixels, struct feature_results *results, std::vector<cv::RotatedRect> *ellipse_ecc, std::vector<double> *eccentricity);

	void compute_texture_features(const cv::Mat& img, struct feature_results& results, cv::Mat* raw_features);
};
#endif /* TEXTURE_FEATURES_H_ */
