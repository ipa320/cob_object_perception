#include "splitandmerge.h"


struct region{
	int x, y, width, height;
	bool parent;
	int objekt;
};
int glob, val, num=0, num2;

bool hom_krit(cv::Mat image)
{
	double rows = image.rows;
	double cols = image.cols;
	int rmin, gmin, bmin, rmax, gmax, bmax;

	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)
		{
			if(rmin < image.at<cv::Vec3b>(i,j)[0])rmin=image.at<cv::Vec3b>(i,j)[0];
			if(gmin < image.at<cv::Vec3b>(i,j)[1])gmin=image.at<cv::Vec3b>(i,j)[1];
			if(bmin < image.at<cv::Vec3b>(i,j)[2])bmin=image.at<cv::Vec3b>(i,j)[2];

			if(rmax < image.at<cv::Vec3b>(i,j)[0])rmax=image.at<cv::Vec3b>(i,j)[0];
			if(gmax < image.at<cv::Vec3b>(i,j)[1])gmax=image.at<cv::Vec3b>(i,j)[1];
			if(bmax < image.at<cv::Vec3b>(i,j)[2])bmax=image.at<cv::Vec3b>(i,j)[2];
		}
	}
	if((rmax-rmin) < 20 && (gmax-gmin)<20 && (bmax-bmin)<20)
	{
		return true;
	}else
	{
		return false;
	}


}


void split(cv::Mat image, int x, int y, std::vector<region> *tree)
{
	int tree_length = (*tree).size();
	std::cout << tree_length << " tree ";
	(*tree).at(tree_length-1).x = x;
	(*tree).at(tree_length-1).y = y;
	double rows_old = image.rows;
	double cols_old = image.cols;
	(*tree).at(tree_length-1).width = cols_old;
	(*tree).at(tree_length-1).height = rows_old;


	if(true)//(!hom_krit(image))
	{
		(*tree).at(tree_length-1).parent = true;
		std::cout << num<<"anzahl1 ";
		num++;
		double rows = floor(rows_old/2);
		double cols = floor(cols_old/2);
		if(rows <10)return;
		if(cols<10)return;
		cv::Rect roi_1 = cv::Rect(0, 0, cols, rows);
		cv::Rect roi_2 = cv::Rect(cols, 0, cols, rows);
		cv::Rect roi_3 = cv::Rect(0, rows, cols, rows);
		cv::Rect roi_4 = cv::Rect(cols, rows, cols, rows);
		cv::Mat new_image1 = image(roi_1);
		cv::Mat new_image2 = image(roi_2);
		cv::Mat new_image3 = image(roi_3);
		cv::Mat new_image4 = image(roi_4);
		(*tree).resize(tree_length+4);
		split(new_image1, x, y, tree);
		split(new_image2, x+cols, y, tree);
		split(new_image3, x, y+rows, tree);
		split(new_image4, x+cols, y+rows, tree);
	}
	else
	{
		(*tree).at(tree_length-1).parent = false;
		std::cout << num2<<"anzahl2 ";
				num2++;
		//Test
		for(int i = y; i<y+rows_old;i++)
		{
			for(int j=x;j<x+cols_old;j++)
			{

//					image.at<cv::Vec3b>(i,j)[glob]=val;
//					glob++;
//					if(glob >=3)glob = 0;
//					val = val +33;
//					if(val >=255)val=val/67;

			}
		}

	}

}




splitandmerge::splitandmerge()
{

}


cv::Mat splitandmerge::categorize(cv::Mat image_in)
{



	std::vector<region> tree(1);
	split(image_in, 0, 0, &tree);



	return image_in;





}
