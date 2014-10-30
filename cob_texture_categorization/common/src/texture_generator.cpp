#include <cob_texture_categorization/texture_generator.h>
#include <sstream>


TextureGenerator::TextureGenerator()
{
	srand(0);	// for repeatability
}


void TextureGenerator::generate_textures()
{
	char key = 0;

	while (key != 'q')
	{
		cv::Mat sample_image_hsv = cv::Mat::zeros(200, 266, CV_8UC3);
		ColorData color = random_color();
		sample_image_hsv.setTo(color.color_hsv);

		cv::Mat primitive = random_primitive_mask(sample_image_hsv.cols/2, sample_image_hsv.rows/2);


		cv::Mat sample_image_rgb;
		cv::cvtColor(sample_image_hsv, sample_image_rgb, CV_HSV2BGR);

		unsigned char c = (color.color_hsv.val[2] > 200 ? 0 : 255);
		std::stringstream ss;
		ss << color.color_name << " (" << 2*(int)color.color_hsv.val[0] << "," << std::setprecision(2) << (1./255.*(double)color.color_hsv.val[1]) << "," << (1./255.*(double)color.color_hsv.val[2]) << ")";
		cv::putText(sample_image_rgb, ss.str(), cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(c,c,c), 1);
		cv::imshow("image", sample_image_rgb);
		key = cv::waitKey();
	}

	// generate monochrome templates

	// generate
}


cv::Mat TextureGenerator::random_primitive_mask(int max_width, int max_height)
{
	int primitive_type = rand()%5;
	int min_length = std::min(max_width, max_height);

	cv::Mat primitive = cv::Mat::zeros(max_height, max_width, CV_8UC1);
	if (primitive_type == 0)
	{
		// rectangle
		cv::rectangle(primitive, cv::Point(rand()%(int)(0.4*max_width), rand()%(int)(0.4*max_height)), cv::Point(max_width-1-rand()%(int)(0.4*max_width), max_height-1-rand()%(int)(0.4*max_height)), cv::Scalar(255), CV_FILLED);
	}
	else if (primitive_type == 1)
	{
		// line
		cv::line(primitive, cv::Point(rand()%(int)(0.2*max_width), rand()%(int)(0.2*max_height)), cv::Point(max_width-1-rand()%(int)(0.2*max_width), max_height-1-rand()%(int)(0.2*max_height)), cv::Scalar(255), 1+rand()%(2+(int)(max_width*0.1)));
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
	double angle = (rand()%360);
	cv::Mat rotation_matrix = cv::getRotationMatrix2D(cv::Point2f(max_width/2, max_height/2), angle, 1.0);
	cv::Mat temp;
	cv::warpAffine(primitive, temp, rotation_matrix, cv::Size(max_width, max_height));
	primitive = temp;

	cv::imshow("primitive", primitive);

	return primitive;
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
