/*
 * texture_features.h
 *
 *  Created on: 18.12.2013
 *      Author: rbormann, Daniel Hundsdoerfer
 */

#ifndef TEXTURE_FEATURES_H_
#define TEXTURE_FEATURES_H_

#include <cob_texture_categorization/texture_categorization.h>

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

};


class texture_features
{
public:
	texture_features();
	void primitive_size(cv::Mat img, struct feature_results *results);
};
#endif /* TEXTURE_FEATURES_H_ */
