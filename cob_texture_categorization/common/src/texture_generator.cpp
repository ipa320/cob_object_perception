#include <cob_texture_categorization/texture_generator.h>
#include <cob_texture_categorization/texture_features.h>
#include "cob_texture_categorization/amadasun.h"
#include <sstream>
#include <fstream>
#include <set>


TextureGenerator::TextureGenerator()
{
	srand(0);	// for repeatability
}


void TextureGenerator::generate_textures()
{
	const std::string path = "/home/rbormann/.ros/texture_generator/";
	char key = 0;

	while (key != 'q')
	{
		cv::Mat sample_image_hsv = cv::Mat::zeros(200, 266, CV_8UC3);
		ColorData color = random_color();
		sample_image_hsv.setTo(color.color_hsv);

		sample_image_hsv = random_checked_pattern(sample_image_hsv.cols, sample_image_hsv.rows);
		std::vector<ColorData> colors;
		distinct_random_colors(colors, 2);
		int dom_color_index, dom_color2_index;
		colorize_gray_image(sample_image_hsv, colors, dom_color_index, dom_color2_index);

		cv::Mat sample_image_rgb;
		cv::cvtColor(sample_image_hsv, sample_image_rgb, CV_HSV2BGR);

		unsigned char c = (colors[dom_color_index].color_hsv.val[2] > 200 ? 0 : 255);
		std::stringstream ss;
		//ss << color.color_name << " (" << 2*(int)color.color_hsv.val[0] << "," << std::setprecision(2) << (1./255.*(double)color.color_hsv.val[1]) << "," << (1./255.*(double)color.color_hsv.val[2]) << ")";
		ss << colors[dom_color_index].color_name << ", " << colors[dom_color2_index].color_name;
		cv::putText(sample_image_rgb, ss.str(), cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(c,c,c), 1);
		cv::imshow("image", sample_image_rgb);
		key = cv::waitKey();
	}


	// ------------------------------------------------------
	const int templates_number = 1700;
	const int image_width = 266, image_height = 200;
	int image_counter = 1;

	std::stringstream annotation_data;
	annotation_data << "1" << std::endl << "generated\t1" << std::endl << "\t" << templates_number << std::endl;

	// generate monochrome templates
	for (; image_counter <= templates_number/17; ++image_counter)
	{
		// image
		cv::Mat sample_image_hsv = cv::Mat::zeros(image_height, image_width, CV_8UC3);
		ColorData color = random_color();
		sample_image_hsv.setTo(color.color_hsv);
		cv::Mat sample_image_rgb;
		cv::cvtColor(sample_image_hsv, sample_image_rgb, CV_HSV2BGR);

		// attributes
		feature_results attributes;
		attributes.setTo(1.0);
		attributes.dom_color = (double)color.color_code;
		attributes.dom_color2 = 0.0;
		attributes.v_mean = 1. + color.color_hsv.val[2]*4./255.;
		attributes.s_mean = 1. + color.color_hsv.val[1]*4./255.;

		// save generated data
		save_generated_data(path, sample_image_rgb, image_counter, annotation_data, attributes);
	}
	// generate lined templates
	for (; image_counter <= 2*templates_number/17; ++image_counter)
	{
		// image
		double line_thickness_value=0, line_spacing_value=0;	// relative values in [0,1] representing the ratio of chosen value to possible range
		cv::Mat sample_image_hsv = random_lined_pattern(image_width, image_height, &line_thickness_value, &line_spacing_value);
		std::vector<ColorData> colors;
		distinct_random_colors(colors, 2);
		int dom_color_index=0, dom_color2_index=0;
		colorize_gray_image(sample_image_hsv, colors, dom_color_index, dom_color2_index);
		cv::Mat sample_image_bgr;
		post_processing(sample_image_hsv, sample_image_bgr, line_spacing_value);

		// attributes
		feature_results attributes;
		attributes.setTo(1.0);
		color_parameter cp;
		cp.get_color_parameter(sample_image_bgr, &attributes);
		double contrast_raw;
		double d = 1;
		amadasun amadasun_fkt2;
		amadasun_fkt2.get_amadasun(sample_image_bgr, d, &attributes, contrast_raw);
		attributes.colorfulness = 2.;
		attributes.dom_color = colors[dom_color_index].color_code;
		attributes.dom_color2 = colors[dom_color2_index].color_code;
		attributes.avg_size = 1. + 4.*line_spacing_value;
		attributes.prim_num = 5. - 4.*line_spacing_value;
		attributes.prim_strength = 2. + attributes.avg_size/5. + attributes.v_std/5. + attributes.contrast/5.;
		attributes.prim_regularity = 4.5 + 0.5*rand()/(double)RAND_MAX;
		attributes.line_likeness = 5. - 0.5*(line_thickness_value + line_spacing_value + abs(line_thickness_value-line_spacing_value));
		attributes.roughness = 2. + 2.*rand()/(double)RAND_MAX;
		attributes.direct_reg = 5. - 0.5*rand()/(double)RAND_MAX;
		attributes.lined = 5. - 0.5*rand()/(double)RAND_MAX;
		attributes.checked = 1. + 0.5*rand()/(double)RAND_MAX;

		// save generated data
		save_generated_data(path, sample_image_bgr, image_counter, annotation_data, attributes);
	}
	// generate checked templates
	for (; image_counter <= 3*templates_number/17; ++image_counter)
	{
		// image
		double line_thickness_value=0, line_spacing_value=0;	// relative values in [0,1] representing the ratio of chosen value to possible range
		cv::Mat sample_image_hsv = random_checked_pattern(image_width, image_height, &line_thickness_value, &line_spacing_value);
		std::vector<ColorData> colors;
		distinct_random_colors(colors, 2);
		int dom_color_index=0, dom_color2_index=0;
		colorize_gray_image(sample_image_hsv, colors, dom_color_index, dom_color2_index);
		cv::Mat sample_image_bgr;
		post_processing(sample_image_hsv, sample_image_bgr, line_spacing_value);

		// attributes
		feature_results attributes;
		attributes.setTo(1.0);
		color_parameter cp;
		cp.get_color_parameter(sample_image_bgr, &attributes);
		double contrast_raw;
		double d = 1;
		amadasun amadasun_fkt2;
		amadasun_fkt2.get_amadasun(sample_image_bgr, d, &attributes, contrast_raw);
		attributes.colorfulness = 2.;
		attributes.dom_color = colors[dom_color_index].color_code;
		attributes.dom_color2 = colors[dom_color2_index].color_code;
		attributes.avg_size = 1. + 4.*line_spacing_value;
		attributes.prim_num = 5 - 4*line_spacing_value;
		attributes.prim_strength = 2. + attributes.avg_size/5. + attributes.v_std/5. + attributes.contrast/5.;
		attributes.prim_regularity = 4.5 + 0.5*rand()/(double)RAND_MAX;
		attributes.line_likeness = 3 - (line_thickness_value + line_spacing_value);
		attributes.roughness = 2 + 2*rand()/(double)RAND_MAX;
		attributes.direct_reg = 5. - 0.5*rand()/(double)RAND_MAX;
		attributes.lined = 5. - 0.5*rand()/(double)RAND_MAX;
		attributes.checked = 5. - 0.5*rand()/(double)RAND_MAX;

		// save generated data
		save_generated_data(path, sample_image_bgr, image_counter, annotation_data, attributes);
	}
	// generate templates with primitives
	for (; image_counter <= templates_number; ++image_counter)
	{
		// choose some attribute values
		feature_results attributes;
		double colorfulness = 0., directionality = 0.;
		attributes.setTo(1.0);
		if (rand()%3 == 0)
			colorfulness = 2.;
		else
			colorfulness = 2. + rand()%4;
		attributes.prim_num = 1. + 4.*rand()/(double)RAND_MAX;
		attributes.avg_size = 1. + (5.-attributes.prim_num)*rand()/(double)RAND_MAX;
		attributes.prim_regularity = 1. + 4.*rand()/(double)RAND_MAX;
		attributes.line_likeness = 1. + 4.*rand()/(double)RAND_MAX;
		directionality = 1. + 4.*rand()/(double)RAND_MAX;

		// image
		std::vector<cv::Mat> primitives;
		int l1 = (int)(0.2*attributes.avg_size*0.5*std::min(image_width, image_height));
		int l2 = l1 * 0.2*(6.-attributes.line_likeness);
		distinct_random_primitives(primitives, cvRound(6-attributes.prim_regularity), l1, l2);
		std::vector<ColorData> colors;
		distinct_random_colors(colors, (int)colorfulness);
		cv::Mat sample_image_hsv = cv::Mat::zeros(image_height, image_width, CV_8UC3);
		sample_image_hsv.setTo(colors[0].color_hsv);
		int dom_color_index=0, dom_color2_index=0;
		place_primitives(sample_image_hsv, primitives, colors, directionality, attributes.prim_num, dom_color_index, dom_color2_index);
		cv::Mat sample_image_bgr;
		post_processing(sample_image_hsv, sample_image_bgr, 0.5);

		// determine remaining attributes
		color_parameter cp;
		cp.get_color_parameter_new(sample_image_bgr, &attributes);
		double contrast_raw;
		double d = 1;
		amadasun amadasun_fkt2;
		amadasun_fkt2.get_amadasun(sample_image_bgr, d, &attributes, contrast_raw);
		attributes.colorfulness = colorfulness;
		attributes.dom_color = colors[dom_color_index].color_code;
		attributes.dom_color2 = colors[dom_color2_index].color_code;
		attributes.prim_strength = 2. + attributes.avg_size/5. + attributes.v_std/5. + attributes.contrast/5.;
		attributes.roughness = 1. + 0.2*attributes.avg_size + 2*rand()/(double)RAND_MAX;		// todo: this is a quite random choice
		attributes.direct_reg = std::max(1., 5.*directionality*0.2 * attributes.prim_num*0.2);
		attributes.lined = 1. + 0.25*(attributes.line_likeness-1.) + 0.25*(attributes.direct_reg-1.) + 0.5*rand()/(double)RAND_MAX;
		attributes.checked = 1. + 0.25*(attributes.direct_reg-1.) + 0.5*rand()/(double)RAND_MAX;
		// save generated data
		save_generated_data(path, sample_image_bgr, image_counter, annotation_data, attributes);
	}

	// store annotations to file
	std::string filename = path + "generated_database_attributes_new.txt";
	std::ofstream file(filename.c_str(), std::ios::out);
	if (file.is_open() == true)
		file << annotation_data.str();
	file.close();
}


void TextureGenerator::save_generated_data(const std::string path, const cv::Mat& image, const int image_counter, std::stringstream& annotation_data, const feature_results& attributes)
{
	std::stringstream ss;
	ss << "generated_01_";
	if (image_counter < 10)
		ss << "00000" << image_counter;
	else if (image_counter < 100)
		ss << "0000" << image_counter;
	else if (image_counter < 1000)
		ss << "000" << image_counter;
	else if (image_counter < 10000)
		ss << "00" << image_counter;
	else if (image_counter < 100000)
		ss << "0" << image_counter;
	else
		ss << image_counter;
	ss << ".png";
	std::string filename = path + ss.str();
	cv::imwrite(filename, image);
	annotation_data << "\t\t" << ss.str() << std::endl;
	annotation_data << "\t\t" << attributes.colorfulness << "\t" << attributes.dom_color << "\t" << attributes.dom_color2 << "\t" << attributes.v_mean << "\t" << attributes.v_std
			<< "\t" << attributes.s_mean << "\t" << attributes.s_std << "\t" << attributes.avg_size << "\t" << attributes.prim_num << "\t" << attributes.prim_strength
			<< "\t" << attributes.prim_regularity << "\t" << attributes.contrast << "\t" << attributes.line_likeness << "\t" << attributes.roughness
			<< "\t" << attributes.direct_reg << "\t" << attributes.lined << "\t" << attributes.checked << std::endl;
}


void TextureGenerator::post_processing(const cv::Mat& image_hsv, cv::Mat& image_bgr, const double blur_factor)
{
	cv::cvtColor(image_hsv, image_bgr, CV_HSV2BGR);
	cv::Mat temp;
	int kernel_size = 2*(rand()%(std::max(2, (int)(7*blur_factor))))+3;
	cv::GaussianBlur(image_bgr, temp, cv::Size(kernel_size,kernel_size), -1, -1);
	image_bgr = temp;
}


void TextureGenerator::place_primitives(cv::Mat& image, const std::vector<cv::Mat>& primitives, const std::vector<ColorData>& colors, const double directionality, const double primitive_number, int& dominant_color_index, int& dominant_color2_index)
{
	// increase image size to allow for image rotation and clipping at the end
	cv::Mat temp_image;
	cv::resize(image, temp_image, cv::Size(), 2, 2, cv::INTER_LINEAR);

	// (regular) placement pattern
	std::vector<cv::Point> placement;
	int x_lines = 2 + 4*(primitive_number-1.)*(0.5+rand()/(double)RAND_MAX);
	int y_lines = 2 + 4*(primitive_number-1.)*(0.5+rand()/(double)RAND_MAX);
	for (int y=0; y<temp_image.rows; y+=temp_image.rows/y_lines)
		for (int x=0; x<temp_image.cols; x+=temp_image.cols/x_lines)
			placement.push_back(cv::Point(x,y));

	// randomize placement pattern
	double max_offset_x = 0.8*temp_image.cols/(double)x_lines;
	double max_offset_y = 0.8*temp_image.rows/(double)y_lines;
	for (size_t i=0; i<placement.size(); ++i)
	{
		placement[i].x += (int)((5.-directionality)/4. * max_offset_x * (-1. + 2.*rand()/(double)RAND_MAX));
		placement[i].y += (int)((5.-directionality)/4. * max_offset_y * (-1. + 2.*rand()/(double)RAND_MAX));
	}

	// place primitives
	for (size_t i=0; i<placement.size(); ++i)
	{
		cv::Mat prim = primitives[rand()%((int)primitives.size())];
		cv::Vec3b color = colors[1 + rand()%((int)colors.size()-1)].color_hsv;
		for (int pv=0; pv<prim.rows; ++pv)
		{
			for (int pu=0; pu<prim.cols; ++pu)
			{
				int u=placement[i].x+pu;
				int v=placement[i].y+pv;
				if (u>=0 && u<temp_image.cols && v>=0 && v<temp_image.rows && prim.at<uchar>(pv,pu)!=0)
					temp_image.at<cv::Vec3b>(v,u) = color;
			}
		}
	}

	// determine dominant colors
	std::map<int,int> color_occurrences;
	for (int v=0; v<temp_image.rows; ++v)
	{
		for (int u=0; u<temp_image.cols; ++u)
		{
			cv::Vec3b& val = temp_image.at<cv::Vec3b>(v,u);
			int gray_index = val.val[0] + (val.val[1]<<8) + (val.val[2]<<16);
			if (color_occurrences.find(gray_index) == color_occurrences.end())
			{
				// this color type has not yet been seen
				color_occurrences[gray_index] = 1;
			}
			else
				color_occurrences[gray_index]++;
		}
	}
	int max_occurrences = 0, max_occurrences2 = 0;
	dominant_color_index = 0;
	dominant_color2_index = 1;
	for (std::map<int,int>::iterator it=color_occurrences.begin(); it!=color_occurrences.end(); ++it)
	{
		if (it->second > max_occurrences)
		{
			max_occurrences2 = max_occurrences;
			max_occurrences = it->second;
			dominant_color2_index = dominant_color_index;
			dominant_color_index = it->first;
		}
		else if (it->second > max_occurrences2)
		{
			max_occurrences2 = it->second;
			dominant_color2_index = it->first;
		}
	}
	for (size_t i=0; i<colors.size(); ++i)
	{
		int val = colors[i].color_hsv.val[0] + (colors[i].color_hsv.val[1]<<8) + (colors[i].color_hsv.val[2]<<16);
		if (dominant_color_index == val)
			dominant_color_index = i;
		if (dominant_color2_index == val)
			dominant_color2_index = i;
	}
	if (dominant_color_index > (int)colors.size() || dominant_color2_index > (int)colors.size())
		std::cout << "Error: wrong color codes for dominant colors." << std::endl;

	// randomly rotate image
	random_image_rotation(temp_image);

	// roi
	image = temp_image(cv::Rect(image.cols/2, image.rows/2, image.cols, image.rows));

}


int TextureGenerator::count_non_zeros(const cv::Mat& image)
{
	int counter = 0;
	for (int v=0; v<image.rows; ++v)
		for (int u=0; u<image.cols; ++u)
			if (image.at<uchar>(v,u) != 0)
				++counter;

	return counter;
}


void TextureGenerator::colorize_gray_image(cv::Mat& image, const std::vector<ColorData>& colors, int& dominant_color_index, int& dominant_color2_index)
{
	std::vector<int> color_usage(colors.size(), 0);

	std::map<int, int> color_mapping;	// from gray_scale value to color index in colors
	color_mapping[0] = 0;

	// colorize
	for (int v=0; v<image.rows; ++v)
	{
		for (int u=0; u<image.cols; ++u)
		{
			cv::Vec3b& val = image.at<cv::Vec3b>(v,u);
			int gray_index = val.val[0] + (val.val[1] << 8) + (val.val[2] << 16);
			if (color_mapping.find(gray_index) == color_mapping.end())
			{
				// this pixel type has not yet received a color mapping
				color_mapping[gray_index] = 1+rand()%((int)colors.size()-1);
			}
			color_usage[color_mapping[gray_index]]++;
			val = colors[color_mapping[gray_index]].color_hsv;
		}
	}

	// find dominant colors
	int max_number = 0;
	int max_index = 0;
	for (size_t i=0; i<color_usage.size(); ++i)
	{
		if (color_usage[i] > max_number)
		{
			max_number = color_usage[i];
			max_index = i;
		}
	}
	dominant_color_index = max_index;
	color_usage[max_index] = 0;
	max_number = 0;
	max_index = 0;
	for (size_t i=0; i<color_usage.size(); ++i)
	{
		if (color_usage[i] > max_number)
		{
			max_number = color_usage[i];
			max_index = i;
		}
	}
	dominant_color2_index = max_index;
}


void TextureGenerator::distinct_random_primitives(std::vector<cv::Mat>& primitives, int number_primitves, int max_width, int max_height)
{
	primitives.clear();
	while (primitives.size() < (size_t)number_primitves)
	{
		int rand_pattern = rand()%30;
		double d1=0, d2=0;
		cv::Mat prim;
		if (rand_pattern < 28)
		{
			prim = random_primitive_mask(max_width, max_height);
		}
		else if (rand_pattern < 29)
		{
			cv::Mat temp = random_lined_pattern(2*max_width, 2*max_height, &d1, &d2);
			cv::Mat temp2;
			cv::cvtColor(temp, temp2, CV_BGR2GRAY);
			prim = temp2(cv::Rect(max_width/2, max_height/2, max_width, max_height));
		}
		else
		{
			cv::Mat temp = random_checked_pattern(2*max_width, 2*max_height, &d1, &d2);
			cv::Mat temp2;
			cv::cvtColor(temp, temp2, CV_BGR2GRAY);
			prim = temp2(cv::Rect(max_width/2, max_height/2, max_width, max_height));
		}
		if (count_non_zeros(prim) > 0.05*prim.cols*prim.rows)
			primitives.push_back(prim.clone());
	}
}


cv::Mat TextureGenerator::random_lined_pattern(int max_width, int max_height, double* line_thickness_value, double* line_spacing_value)
{
	int min_length = std::min(max_width, max_height);
	cv::Mat primitive = cv::Mat::zeros(2*max_height, 2*max_width, CV_8UC3);

	const int max_thickness = std::max(2, min_length/10);
	const int max_spacing = (std::max(2, min_length/4));
	int line_thickness = 1 + rand()%max_thickness;
	int line_spacing = 2*line_thickness + rand()%max_spacing;
	for (int v=line_spacing; v<primitive.rows; v += line_spacing)
		cv::line(primitive, cv::Point(0,v), cv::Point(primitive.cols-1,v), cv::Scalar(255), line_thickness);
	if (line_thickness_value != 0)
		*line_thickness_value = (double)line_thickness/(double)max_thickness;
	if (line_spacing_value != 0)
		*line_spacing_value = std::min(1., (double)(line_spacing-line_thickness)/(double)(max_thickness+max_spacing-1));

	// randomly rotate primitive
	random_image_rotation(primitive);

	// roi
	cv::Mat primitive_roi = primitive(cv::Rect(max_width/2, max_height/2, max_width, max_height));

	return primitive_roi;
}


cv::Mat TextureGenerator::random_checked_pattern(int max_width, int max_height, double* line_thickness_value, double* line_spacing_value)
{
	int min_length = std::min(max_width, max_height);
	cv::Mat primitive = cv::Mat::zeros(2*max_height, 2*max_width, CV_8UC3);

	const int max_thickness = std::max(2, min_length/10);
	const int max_spacing = (std::max(2, min_length/4));
	// line direction 1
	int line_thickness = 1 + rand()%max_thickness;
	int line_spacing = 2*line_thickness + rand()%max_spacing;
	for (int v=line_spacing; v<primitive.rows; v += line_spacing)
		cv::line(primitive, cv::Point(0,v), cv::Point(primitive.cols-1,v), cv::Scalar(255), line_thickness);
	// line direction 2
	double mean_line_thickness = line_thickness;
	double mean_line_spacing = line_spacing;
	if (rand()%3 == 0)	// use other line properties sometimes
	{
		line_thickness = 1 + rand()%max_thickness;
		line_spacing = 2*line_thickness + rand()%max_spacing;
	}
	mean_line_thickness = (mean_line_thickness+line_thickness)*0.5;
	mean_line_spacing = (mean_line_spacing+line_spacing)*0.5;
	for (int u=line_spacing; u<primitive.cols; u += line_spacing)
		cv::line(primitive, cv::Point(u,0), cv::Point(u,primitive.rows-1), cv::Scalar(255), line_thickness);
	if (line_thickness_value != 0)
		*line_thickness_value = (double)(mean_line_thickness)/(double)max_thickness;
	if (line_spacing_value != 0)
		*line_spacing_value = std::min(1., (double)(mean_line_spacing-mean_line_thickness)/(double)(max_thickness + max_spacing - 1));

	// randomly rotate primitive
	random_image_rotation(primitive);

	// roi
	cv::Mat primitive_roi = primitive(cv::Rect(max_width/2, max_height/2, max_width, max_height));

	return primitive_roi;
}


cv::Mat TextureGenerator::random_primitive_mask(int max_width, int max_height)
{
	int min_length = std::min(max_width, max_height);
	cv::Mat primitive = cv::Mat::zeros(2*max_height, 2*max_width, CV_8UC1);

	int primitive_type = rand()%5;
	if (primitive_type == 0)
	{
		// rectangle
		cv::rectangle(primitive, cv::Point(rand()%std::max(2, (int)(0.4*max_width)), rand()%std::max(2, (int)(0.4*max_height))), cv::Point(max_width-1-rand()%std::max(2, (int)(0.4*max_width)), max_height-1-rand()%std::max(2, (int)(0.4*max_height))), cv::Scalar(255), CV_FILLED);
	}
	else if (primitive_type == 1)
	{
		// line
		cv::line(primitive, cv::Point(rand()%std::max(2, (int)(0.2*max_width)), rand()%std::max(2, (int)(0.2*max_height))), cv::Point(max_width-1-rand()%std::max(2, (int)(0.2*max_width)), max_height-1-rand()%std::max(2, (int)(0.2*max_height))), cv::Scalar(255), 1+rand()%(2+(int)(max_width*0.1)));
	}
	else if (primitive_type == 2)
	{
		// circle
		cv::circle(primitive, cv::Point(max_width/2, max_height/2), min_length/2*(0.8+0.2*rand()/(double)RAND_MAX), cv::Scalar(255), CV_FILLED);
	}
	else if (primitive_type == 3)
	{
		// ellipse
		cv::ellipse(primitive, cv::Point(max_width/2, max_height/2), cv::Size(min_length/2*(0.8+0.2*rand()/(double)RAND_MAX), min_length/2*(0.2+0.8*rand()/(double)RAND_MAX)), 0, 0, 360, cv::Scalar(255), CV_FILLED);
	}
	else if (primitive_type == 4)
	{
		// polygon with 3-16 vertices
		int vertices = 3 + rand()%14;
		std::vector<std::vector<cv::Point> > polygon_points(1);
		for (int i=0; i<vertices; ++i)
			polygon_points[0].push_back(cv::Point(rand()%max_width, rand()%max_height));
		cv::fillPoly(primitive, polygon_points, cv::Scalar(255));
	}

	// randomly rotate primitive
	random_image_rotation(primitive);

	// roi
	cv::Mat primitive_roi = primitive(cv::Rect(max_width/2, max_height/2, max_width, max_height));

	return primitive_roi;
}


void TextureGenerator::random_image_rotation(cv::Mat& image)
{
	// randomly rotate primitive
	double angle = (rand()%360);
	cv::Mat rotation_matrix = cv::getRotationMatrix2D(cv::Point2f(image.cols/2, image.rows/2), angle, 1.0);
	cv::Mat temp;
	cv::warpAffine(image, temp, rotation_matrix, cv::Size(image.cols, image.rows));
	image = temp;
}

void TextureGenerator::distinct_random_colors(std::vector<ColorData>& colors, int number_colors)
{
	colors.clear();
	std::set<ColorCode> drawn_colors;
	while ((int)colors.size() < number_colors)
	{
		ColorData color = random_color();
		if (drawn_colors.find(color.color_code) == drawn_colors.end())
		{
			colors.push_back(color);
			drawn_colors.insert(color.color_code);
		}
	}
}


ColorData TextureGenerator::random_color()
{
	// generate a random color
	// 2. dominant color / 3. secondary dominant color
	//    0 = no dominant color (e.g. too colorful or transparent)	--> not generated here
	//    1 = brown (ocher): h in [20, 70], s > 0.15, 0.2 < v < 0.7
	//    2 = red: h in [0, 20] & [345, 360], s > 0.2, v > 0.3
	//    3 = orange: h in [20, 50], s > 0.3, v > 0.7
	//    4 = yellow: h in [50, 70], s > 0.2, v > 0.3
	//    5 = green: h in [70, 160], s > 0.2, v > 0.3
	//    6 = cyan (turquoise): h in [160, 180], s > 0.2, v > 0.3
	//    7 = blue: h in [180, 250], s > 0.2, v > 0.2
	//    8 = purple: h in [250, 295], s > 0.2, v > 0.3
	//    9 = pink (rose): h in [295, 345], s > 0.2, v > 0.3
	//   10 = white: s < 0.2, v >= 0.75
	//   11 = gray (silver/metallic): s < 0.2, 0.2 < v < 0.75 or s>0.2, 0.2<v<0.3
	//   12 = black: v <= 0.2
	// 4. value/brightness / 5. variety of value/brightness
	// 6. saturation/color strength / 7. variety of saturation/color strength

	ColorCode color = (ColorCode)(1 + rand() % 12);
	double h=0., s=0., v=0.;
	std::string color_name;

	if (color == BROWN)
	{
		// brown (ocher): h in [20, 70], s > 0.15, 0.2 < v < 0.7
		color_name = "brown";
		h = 20 + rand()%51;
		s = 0.15 + 0.85*rand()/(double)RAND_MAX;
		v = 0.2 + 0.5*rand()/(double)RAND_MAX;
	}
	else if (color == RED)
	{
		// red: h in [0, 20] & [345, 360], s > 0.2, v > 0.3
		color_name = "red";
		if (rand()%2 == 0)
			h = rand()%21;
		else
			h = 345 + rand()%16;
		s = 0.2 + 0.8*rand()/(double)RAND_MAX;
		v = 0.3 + 0.7*rand()/(double)RAND_MAX;
	}
	else if (color == ORANGE)
	{
		// orange: h in [20, 50], s > 0.3, v > 0.7
		color_name = "orange";
		h = 20 + rand()%31;
		s = 0.3 + 0.7*rand()/(double)RAND_MAX;
		v = 0.7 + 0.3*rand()/(double)RAND_MAX;
	}
	else if (color == YELLOW)
	{
		// yellow: h in [50, 70], s > 0.2, v > 0.3
		color_name = "yellow";
		h = 50 + rand()%21;
		s = 0.2 + 0.8*rand()/(double)RAND_MAX;
		v = 0.3 + 0.7*rand()/(double)RAND_MAX;
	}
	else if (color == GREEN)
	{
		// green: h in [70, 160], s > 0.2, v > 0.3
		color_name = "green";
		h = 70 + rand()%91;
		s = 0.2 + 0.8*rand()/(double)RAND_MAX;
		v = 0.3 + 0.7*rand()/(double)RAND_MAX;
	}
	else if (color == CYAN)
	{
		// cyan (turquoise): h in [160, 180], s > 0.2, v > 0.3
		color_name = "cyan";
		h = 160 + rand()%21;
		s = 0.2 + 0.8*rand()/(double)RAND_MAX;
		v = 0.3 + 0.7*rand()/(double)RAND_MAX;
	}
	else if (color == BLUE)
	{
		// blue: h in [180, 250], s > 0.2, v > 0.2
		color_name = "blue";
		h = 180 + rand()%71;
		s = 0.2 + 0.8*rand()/(double)RAND_MAX;
		v = 0.2 + 0.8*rand()/(double)RAND_MAX;
	}
	else if (color == PURPLE)
	{
		// purple: h in [250, 295], s > 0.2, v > 0.3
		color_name = "purple";
		h = 250 + rand()%46;
		s = 0.2 + 0.8*rand()/(double)RAND_MAX;
		v = 0.3 + 0.7*rand()/(double)RAND_MAX;
	}
	else if (color == PINK)
	{
		// pink (rose): h in [295, 345], s > 0.2, v > 0.3
		color_name = "pink";
		h = 295 + rand()%51;
		s = 0.2 + 0.8*rand()/(double)RAND_MAX;
		v = 0.3 + 0.7*rand()/(double)RAND_MAX;
	}
	else if (color == WHITE)
	{
		// white: s < 0.2, v >= 0.75
		color_name = "white";
		h = rand()%361;
		s = 0.2*rand()/(double)RAND_MAX;
		v = 0.75 + 0.25*rand()/(double)RAND_MAX;
	}
	else if (color == GRAY)
	{
		// gray (silver/metallic): s < 0.2, 0.2 < v < 0.75 or s>0.2, 0.2<v<0.3
		color_name = "gray";
		h = rand()%361;
		if (rand()%2 == 0)
		{
			s = 0.2*rand()/(double)RAND_MAX;
			v = 0.2 + 0.55*rand()/(double)RAND_MAX;
		}
		else
		{
			s = 0.2 + 0.8*rand()/(double)RAND_MAX;
			v = 0.2 + 0.1*rand()/(double)RAND_MAX;
		}
	}
	else if (color == BLACK)
	{
		// black: v <= 0.2
		color_name = "black";
		h = rand()%361;
		s = rand()/(double)RAND_MAX;
		v = 0.2*rand()/(double)RAND_MAX;
	}

	ColorData cd;
	cd.color_code = color;
	cd.color_name = color_name;
	cd.color_hsv = cv::Vec3b((unsigned char)(h*0.5), (unsigned char)(255*s), (unsigned char)(255*v));
	return cd;
}


int main()
{
	TextureGenerator tg;
	tg.generate_textures();
}
