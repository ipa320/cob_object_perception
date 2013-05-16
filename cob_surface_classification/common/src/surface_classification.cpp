/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2013 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: cob_object_perception
 * \note
 * ROS package name: cob_surface_classification
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 07.08.2012
 *
 * \brief
 * functions for display of people detections
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <cob_surface_classification/surface_classification.h>

cv::Mat grad;


static void onMouse( int event, int x,int y,int,void*)
{
	cv::imwrite("Sobelimage.png", grad);
}


void SurfaceClassification::testFunction(cv::Mat& color_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, cv::Mat& depth_image)
{
	//&: Adresse der Variable
	// -> pointCloud selbst wird verändert! ohne & wird eine Kopie von pointcloud übergeben und nur die kopie verändert
	//bei Funktionsaufruf einfach das objekt übergeben, bei der Deklaration der Funktion ein & an den Datentyp -> die Funktion verwendet eine Referenz/einen Zeiger auf das Objekt!

	std::cout << "function call \n";

	std::vector<int> test;
	test.size();







	//--------------------------------------------------------------------
	//computations with depth_image
	//----------------------------------------------------------------------

	//compute Sobel and Laplacian
	//derivatives(color_image,depth_image);

	//draw two perpendicular lines into the image, and approximate the depth along these lines by linear regression.
	depth_along_lines(color_image, depth_image, pointcloud);


	//visualization
	cv::imshow("image", color_image);
	cv::waitKey(10);

	cv::imshow("depth_image", depth_image);
	cv::waitKey(10);

}

void SurfaceClassification::approximateLine(cv::Mat& depth_image,pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& abc, cv::Mat& coordinates, bool& step)
{
	//Achtung: coordinates-Matrix wird durch SVD verändert

	Timer timer;

	//std::cout << "approximateLine()\n";


	//dotIni is origin of local coordinate system, dotEnd the end point of the line


	/* consider depth coordinates along the line between dotIni and dotEnd
	 * write them into a matrix
	 * approximation of the function z(w), that is depth(z) over coordinate on the line(w)
	 *
	 * linear regression via least squares minimisation (-> SVD)
	 * ----------------------------------------------------------*/

	//write depth of points along the line into a matrix
	//format of one line: [coordinate along the line (w), depth coordinate (z) , 1]

	//parameters of the approximated line: aw+bz+1 = 0

	/*timer.start();
	for(int itim=0;itim<10000;itim++)
	{*/


	int xDist = dotEnd.x - dotIni.x;
	int yDist = dotEnd.y - dotIni.y;
	int lineLength = std::max(std::abs(xDist),std::abs(yDist)) + 1;
	int dist;

	int lineIter;	//index of current pixel on line
	int endIndex;	//index of last pixel on line
	int sign = 1;
	int xIter[lineLength];
	int yIter[lineLength];
	//std::cout << "xDist: " << xDist <<"\n";
	if(xDist != 0)
	{
		//line in x-direction
		//lineLength = std::abs(xDist) +1;
		lineIter = dotIni.x;
		endIndex = dotEnd.x;
		if(xDist <0)
			sign = -1;
		//include both points -> one coordinate more than distance


		for(int i=0; i<lineLength; i ++)
		{
			xIter[i] = dotIni.x + i * sign;
			yIter[i] = dotIni.y;
		}
	}
	else
	{
		//lineLength = std::abs(yDist) +1;
		lineIter = dotIni.y;
		endIndex = dotEnd.y;
		if(yDist <0)
			sign = -1;

		for(int i=0; i<lineLength; i ++)
		{
			xIter[i] = dotIni.x;
			yIter[i] = dotIni.y + i*sign;
		}
	}

	//std::cout << "xIter:\n" <<xIter << "\nyIter:\n" <<yIter << "\n";

	float x0,y0; //coordinates of reference point
	bool first = true;

	int iCoord;
	//timer.start();


	//iterate from dotIni to dotEnd:


	coordinates = cv::Mat::zeros(lineLength,3,CV_32FC1);
	//cv::Mat currentCoord (cv::Mat::zeros(1,3,CV_32FC1));

	iCoord = 0;
	//for(int v=0; v<xIter.count; v++, ++xIter)
	int iX = 0;
	int iY = 0;


	while(iX < lineLength && iY < lineLength)
	{
		//später nicht einfach x bzw y-Koordinate betrachten, sondern (x-x0)²+(y-y0)² als w nehmen


		//don't save points with nan-entries (no data available)
		if(!std::isnan(pointcloud->at(xIter[iX],yIter[iY]).x))
		{

			if(first)
			{
				//origin of local coordinate system is the first point with valid data. Express coordinates relatively:
				x0 = pointcloud->at(xIter[iX],yIter[iY]).x;		//x0 = pointcloud->at(xIter.pos().x,xIter.pos().y).x;
				y0 = pointcloud->at(xIter[iX],yIter[iY]).y;
				first = false;
				//std::cout << "coordinates: " <<coordinates << "\n";
			}
			//std::cout << "writing coordinates\n ";
			coordinates.at<float>(iCoord,0) = (pointcloud->at(xIter[iX],yIter[iY]).x - x0)
											+ (pointcloud->at(xIter[iX],yIter[iY]).y - y0);
			coordinates.at<float>(iCoord,1) = depth_image.at<float>(yIter[iY], xIter[iX]);	//depth coordinate
			coordinates.at<float>(iCoord,2) = 1.0;


			//detect steps in depth coordinates
			if(iCoord != 0 && std::abs(coordinates.at<float>(iCoord,1) - coordinates.at<float>(iCoord-1,1)) > 0.05)
			{
				step = true;
			}
			iCoord++;
		}
		iX++;
		iY++;
	}
	//std::cout << "coordinates: " <<coordinates << "\n";
	if(iCoord == 0)
		//no valid data
		abc = cv::Mat::zeros(1,3,CV_32FC1);
	else
	{
		//std::cout <<"iCoord "<< iCoord <<", lineLength "<<lineLength<< "\n";
		if(iCoord<lineLength)
		{
			//std::cout << "resizing\n ";
			coordinates.resize(iCoord);
			//std::cout << "coordinates after resize: " <<coordinates << "\n";
		}


		cv::Mat sv;	//singular values
		cv::Mat u;	//left singular vectors
		cv::Mat vt;	//right singular vectors, transposed, 3x3


		//if there have been valid coordinates available, perform SVD
		//last column of v = last row of v-transposed is x, so that y is minimal
		if(coordinates.rows >2)
		{
			cv::SVD::compute(coordinates,sv,u,vt,cv::SVD::MODIFY_A);
			abc =  vt.row(2);
		}
	}

	/*}

	timer.stop();
	std::cout << timer.getElapsedTimeInMilliSec() /10000<< " ms setting up coordinates-matrix (averaged over 10000 iterations)\n";*/


}

void SurfaceClassification::drawLines(cv::Mat& plotZW, cv::Mat& coordinates, cv::Mat& abc)
{
	// draw computed coordinates and approximates lines in plotZW
	//origin of coordinate system at the bottom in the middle
	//x-axis: w-coordinate	(first column of "coordinates")
	//y-axis: z-coordinate	(second column of "coordinates")
	// -----------------------------------------------------------


	//Skalierung des Tiefen-Wertes zur besseren Darstellung
	float scaleDepth = 400;


	int windowX = plotZW.cols;
	int windowY = plotZW.rows;

	//std::cout << "coordinates: \n" <<coordinates <<"\n";
	//std::cout << "abc: \n" <<abc <<"\n";


	//letzter Eintrag in der Spalte der w-Koordinaten ist die groesste w-Koordinate
	//alle Koordinaten werden durch diese groesste geteilt -> Projektion auf [0,1]
	float maxW = coordinates.col(0).at<float>(coordinates.rows -1);

	//take magnitude of maxW, so that normalization of coordinates won't affect the sign
	if(maxW <0)
		maxW = maxW * (-1);

	float w,z;	//w-coordinate along line,  depth coordinate z

	if(maxW == 0)
	{
		maxW = 1;
		std::cout << "SurfaceClassification::drawLines(): Prevented division by 0\n";
	}

	for(int v=0; v< coordinates.rows; v++)
	{
		//project w-coordinates on interval [0,windowX/2] and then shift to the middle windowX/2
		w = (coordinates.at<float>(v,0) / maxW) * windowX/2 + windowX/2;
		//scale z-value for visualization
		z = coordinates.at<float>(v,1) *scaleDepth;

		//invert depth-coordinate for visualization (coordinate system of image-matrix is in upper left corner,
		//whereas we want our coordinate system to be at the bottom with the positive z-axis going upward
		z = windowY - z;
		cv::circle(plotZW,cv::Point2f(w,z),1,CV_RGB(255,255,255),2);
	}

	//compute first and last point of the line using the parameters abc
	// P1(0,-c/b)
	// P2(xBoundary,(-c-a*xBoundary)/b);
	float x1 = 0;
	float z1 = -abc.at<float>(2)/abc.at<float>(1);	//line equation -> z-value for x1
	float x2 = coordinates.col(0).at<float>(coordinates.rows -1);
	float z2 = (-abc.at<float>(2) - abc.at<float>(0) * x2) / abc.at<float>(1);

	//scaling and shifting
	x1 =  windowX/2;
	x2 = (x2/ maxW) * windowX/2 + windowX/2;

	z1 = z1 * scaleDepth;
	z1 = windowY - z1;

	z2 = z2 *scaleDepth;
	z2 = windowY - z2;

	//draw line between the two points
	cv::line(plotZW,cv::Point2f(x1 ,z1 ), cv::Point2f(x2 , z2 ),CV_RGB(255,255,255),1);

}




void SurfaceClassification::scalarProduct(cv::Mat& abc1,cv::Mat& abc2,float& scalarProduct, int& concaveConvex, bool& step)
{

	//std::cout << "scalarProduct()\n";

	//detect steps
	//------------------------------------------------------------

	//compute z-value at w =0
	//compare value on left side of w=0 to value on right side
	// P[0, -c/b]
	float zLeft = -abc1.at<float>(2)/abc1.at<float>(1);
	float zRight = -abc2.at<float>(2)/abc2.at<float>(1);

	//there truly is a step at the central point only if there is none in the coordinates to its left and right
	if(!step && ((zRight - zLeft) > 0.05 || (zRight - zLeft) < -0.05))
	{
		//mark step as edge in depth data
		scalarProduct = 0;
	}

	else
	{
		//if depth data is continuous: compute scalar product
		//--------------------------------------------------------

		//normal vector n=[1,(-c-a)/b]
		float b1 = -(abc1.at<float>(0) + abc1.at<float>(2))/abc1.at<float>(1);
		float b2 = -(abc2.at<float>(0) + abc2.at<float>(2))/abc2.at<float>(1);

		//std::cout << "b1: " <<b1 << "\n";
		//std::cout << "b2: " <<b2<< "\n";


		//Richtungsvektoren der Geraden:
		cv::Mat n1 = (cv::Mat_<float>(1,2) << 1, b1);
		cv::Mat n2 = (cv::Mat_<float>(1,2) << 1, b2);

		cv::Mat n1Norm, n2Norm;
		cv::normalize(n1,n1Norm,1);
		cv::normalize(n2,n2Norm,1);
		//std::cout << "n1: " <<n1 << "\n";
		//std::cout << "n2: " <<n2<< "\n";


		//Skalarprodukt liegt zwischen 0 und 1
		//cv::Mat scalProd = n1Norm * n2Norm.t();
		scalarProduct = n1Norm.at<float>(0) * n2Norm.at<float>(0) + n1Norm.at<float>(1) * n2Norm.at<float>(1);

		//scalarProduct = scalProd.at<float>(0,0);
		//std::cout << "scalarProduct: " <<scalarProduct << "\n";


		//convex: gradient of right line is larger than gradient of left line
		//concave: gradient of right line is smaller than gradient of left line
		float offset = 1.5;	//how much the gradients need to differ
		if(b2 > (b1 + offset))
			//convex
			concaveConvex = 255;
		else if (b1 > b2 + offset)
			//concave
			concaveConvex = 125;

	}

}


void SurfaceClassification::thinEdges(cv::Mat& edgePicture, int xy)
{
	/* input + output: 	edgePicture - picture whose edges are to be thinned. It is changed during computation!
	 * input: 			xy			- 0 if edge running in y direction is to be thinned. (->neighbourhood in x-direction is considered), 1 if edge in x-direction.
	 */

	//detect edges as local minima of scalarProduct
	//--------------------------------------------------------------------------------

	int neighboursOnOneSide = 9;
	int neighbourhoodSize = neighboursOnOneSide*2 +1;
	cv::Mat neighbourhood = cv::Mat::zeros(1,neighbourhoodSize,CV_32FC1);

	float min;
	int minIdx;
	//loop over rows of edgePicture
	for(int iY = neighbourhoodSize; iY< edgePicture.rows - neighbourhoodSize; iY++)
	{
		//loop over columns of edgePicture
		for(int iX = neighbourhoodSize; iX< edgePicture.cols- neighbourhoodSize; iX++)
		{
			min = 2;
			minIdx = -1;
			//loop over neighbourhood
			for(int iNeigh= - neighboursOnOneSide ; iNeigh <= neighboursOnOneSide; iNeigh++)
			{
				//loop either over x-coordinate
				if(xy == 0)
				{
					//check if current value is smaller than hitherto assumed minimum
					if(edgePicture.at<float>(iY,iX + iNeigh) < min)
					{
						minIdx = iX + iNeigh;
						min = edgePicture.at<float>(iY,minIdx);
					}
				}
				//or loop over y-coordinate
				else if (xy == 1)
				{
					if(edgePicture.at<float>(iY + iNeigh,iX) < min)
					{
						minIdx = iY + iNeigh;
						min = edgePicture.at<float>(minIdx,iX);
					}
				}
			}
			//if valid minimum has been found
			if(min != 2 && minIdx != -1)
			{
				//set all values in neighbourhood to one, except for minimum value
				for(int iNeigh= - neighboursOnOneSide ; iNeigh <= neighboursOnOneSide; iNeigh++)
				{
					if(xy == 0)
					{
						if(iX + iNeigh != minIdx)
						{
							edgePicture.at<float>(iY,iX + iNeigh) = 1;
						}
					}
					else if (xy == 1)
					{
						if(iY + iNeigh != minIdx)
						{
							edgePicture.at<float>(iY + iNeigh,iX) = 1;
						}
					}

				}
			}
		}
	}
}




void SurfaceClassification::depth_along_lines(cv::Mat& color_image, cv::Mat& depth_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{
	/*Timer timerFunc;
	timerFunc.start();*/

	int lineLength = 20;	//depth coordinates along two lines with length lineLength/2 are considered

	//zeichne Fadenkreuz:
	cv::line(color_image,cv::Point2f(color_image.cols/2 -lineLength/2, color_image.rows/2),cv::Point2f(color_image.cols/2 +lineLength/2, color_image.rows/2),CV_RGB(0,1,0),1);
	cv::line(color_image,cv::Point2f(color_image.cols/2 , color_image.rows/2 +lineLength/2),cv::Point2f(color_image.cols/2 , color_image.rows/2 -lineLength/2),CV_RGB(0,1,0),1);


	//plot z over w, draw estimated lines
	int windowX = 600;	//size of window in x-direction
	int windowY = 600;

	cv::Mat plotZW (cv::Mat::zeros(windowX,windowY,CV_32FC1));
	cv::Mat scalarProducts (cv::Mat::ones(color_image.rows,color_image.cols,CV_32FC1));
	cv::Mat concaveConvex (cv::Mat::zeros(color_image.rows,color_image.cols,CV_8UC1)); 	//0:neither concave nor convex; 125:concave; 255:convex
	cv::Mat concaveConvexY (cv::Mat::zeros(color_image.rows,color_image.cols,CV_8UC1)); 	//0:neither concave nor convex; 125:concave; 255:convex
	cv::Mat scalarProductsY (cv::Mat::ones(color_image.rows,color_image.cols,CV_32FC1));
	cv::Mat scalarProductsX (cv::Mat::ones(color_image.rows,color_image.cols,CV_32FC1));

	int iX=color_image.cols/2;
	int iY=color_image.rows/2;

	Timer timer;

	int sampleStep = 0;	//computation only for every n-th pixel

	//	cout << timerFunc.getElapsedTimeInMilliSec() << " ms for initial definitions before loop\n";
	//	int countIterations = 0;

	//loop over rows
	for(int iY = lineLength/2; iY< color_image.rows-lineLength/2; iY++)
	{
		//loop over columns
		for(int iX = lineLength/2; iX< color_image.cols-lineLength/2; iX++)
		{
			/*sampleStep ++;
			if(sampleStep == 5)
			{
				sampleStep = 0;*/



			//scalarProduct of depth along lines in x-direction
			//------------------------------------------------------------------------
			cv::Point2f dotLeft(iX -lineLength/2, iY);
			cv::Point2f dotRight(iX +lineLength/2, iY);
			cv::Mat coordinates1;
			cv::Mat coordinates2;


			cv::Mat abc1 (cv::Mat::zeros(1,3,CV_32FC1));

			//	timer.start();
			//for(int i=0; i<10000; i++)
			//{
			//	coordinates1 = cv::Mat::zeros(1,3,CV_32FC1);
			//do not use point right at the center (would belong to both lines -> steps not correctly represented)

			bool step = false;
			approximateLine(depth_image,pointcloud, cv::Point2f(iX-1,iY),dotLeft, abc1, coordinates1, step);

			//}

			//	timer.stop();
			//	std::cout << timer.getElapsedTimeInMilliSec() /10000 << " ms for lineApproximation (averaged over 10000 iterations)\n";

			cv::Mat abc2 (cv::Mat::zeros(1,3,CV_32FC1));

			//besser den Pixel rechts bzw.links von dotMiddle betrachten, damit dotMiddle nicht zu beiden Seiten dazu gerechnet wird (sonst ungenau bei Sprung)
			approximateLine(depth_image,pointcloud, cv::Point2f(iX+1,iY),dotRight, abc2, coordinates2,step);

			//drawLines(plotZW,coordinates1,abc1);
			//drawLines(plotZW,coordinates2,abc2);


			float scalProdX = 1;
			int concConv = 0;
			if(abc1.at<float>(0,0) == 0 || abc2.at<float>(0,0) == 0)
			{
				//abc konnte nicht approximiert werden
				scalProdX = 1;
			}
			else
			{
				scalarProduct(abc1,abc2,scalProdX,concConv,step);
			}

			//std::cout << "scalarProduct: " <<scalProdX << "\n";
			//std::cout << "concave 125 grau, konvex 255 weiß: " <<concConv << "\n";

			//compute magnitude of scalar product (sign only depends on which angle between the lines was considered)
			if(scalProdX < 0)
				scalProdX = scalProdX * (-1);
			scalarProductsX.at<float>(iY,iX) = scalProdX;


			concaveConvex.at<unsigned char>(iY,iX) = concConv;








			//scalarProduct of depth along lines in y-direction
			//------------------------------------------------------------------------
			cv::Point2f dotDown(iX , iY-lineLength/2);
			cv::Point2f dotUp(iX , iY+lineLength/2);
			cv::Point2f dotMiddle(iX , iY );
			cv::Mat coordinates1Y = cv::Mat::zeros(1,3,CV_32FC1);
			cv::Mat coordinates2Y = cv::Mat::zeros(1,3,CV_32FC1);


			cv::Mat abc1Y (cv::Mat::zeros(1,3,CV_32FC1));

			//timer.start();



			//boolean step will be set to true in approximateLine(), if there is a step either in coordinates1 or coordinates2
			//-> needs to be set to false before calling approximateLine() for both sides.
			//in scalarProduct(), boolean step will be considered when detecting a step at the central point.
			//if there is a step at the central point, the coordinates on the right and left to it should be continuous without step!
			//(else the step would be detected in the neighbourhood as well, leading to inaccuracies)
			step = false;

			//do not use point right at the center (would belong to both lines -> steps not correctly represented)
			approximateLine(depth_image,pointcloud, cv::Point2f(iX,iY-1),dotDown, abc1Y, coordinates1Y, step);

			//timer.stop();
			//cout << timer.getElapsedTimeInMilliSec() << " ms for lineApproximation\n";

			cv::Mat abc2Y (cv::Mat::zeros(1,3,CV_32FC1));

			//besser den Pixel rechts bzw.links von dotMiddle betrachten, damit dotMiddle nicht zu beiden Seiten dazu gerechnet wird (sonst ungenau bei Sprung)
			approximateLine(depth_image,pointcloud, cv::Point2f(iX,iY+1),dotUp, abc2Y, coordinates2Y,step);

			//drawLines(plotZW,coordinates1Y,abc1Y);
			//drawLines(plotZW,coordinates2Y,abc2Y);

			//std::cout << "step: " << step << "\n";


			float scalProdY = 1;
			int concConvY = 0;
			if(abc1Y.at<float>(0,0) == 0 || abc2Y.at<float>(0,0) == 0)
			{
				//abc konnte nicht approximiert werden
				scalProdY = 1;
			}
			else
			{
				//timer.start();
				scalarProduct(abc1Y,abc2Y,scalProdY,concConvY, step);
				//timer.stop();
				//cout << timer.getElapsedTimeInMilliSec() << " ms for scalarProduct\n";
			}

			//std::cout << "scalarProduct: " <<scalProdX << "\n";
			//std::cout << "concave 125 grau, konvex 255 weiß: " <<concConv << "\n";

			//compute magnitude of scalar product (sign only depends on which angle between the lines was considered)
			if(scalProdY < 0)
				scalProdY = scalProdY * (-1);
			scalarProductsY.at<float>(iY,iX) = scalProdY;


			concaveConvexY.at<unsigned char>(iY,iX) = concConvY;

/*
			//Minimum:
			scalarProducts.at<float>(iY,iX) = (scalProdX < scalProdY)? scalProdX : scalProdY;*/




			//}
			//countIterations++;
		}
	}

	//thin edges in x- and y-direction separately
	thinEdges(scalarProductsX, 0);
	thinEdges(scalarProductsY, 1);

	for(int iY = lineLength/2; iY< color_image.rows-lineLength/2; iY++)
		{
			//loop over columns
			for(int iX = lineLength/2; iX< color_image.cols-lineLength/2; iX++)
			{
				//Minimum:
				scalarProducts.at<float>(iY,iX) = std::min(scalarProductsX.at<float>(iY,iX), scalarProductsY.at<float>(iY,iX));
			}
		}



	//cout << timerFunc.getElapsedTimeInMilliSec() << " ms directly after loop\n";
	//cout << "countIterations: " <<countIterations <<"\n";

	//std::cout << "scalarProduct: " <<scalarProductsX << "\n";

	//cv::imshow("depth over coordinate along line", plotZW);
	//cv::waitKey(10);


	//cv::Mat scalarProductsNorm;
	//cv::normalize(scalarProducts,scalarProductsNorm,0,1,cv::NORM_MINMAX);



	cv::imshow("Skalarprodukt", scalarProducts);
	cv::waitKey(10);



	//cv::imshow("Skalarprodukt with thinned edges", scalarProducts);
	//cv::waitKey(10);

	//cv::imshow("concave = grey, convex = white", concaveConvexY);
	//cv::waitKey(10);

	//timerFunc.stop();
	//cout << timerFunc.getElapsedTimeInMilliSec() << " ms for depth_along_lines()\n";

}

void SurfaceClassification::derivatives(cv::Mat& color_image, cv::Mat& depth_image)
{
	//smooth image
	cv::Mat depth_image_smooth = depth_image.clone();
	cv::GaussianBlur( depth_image, depth_image_smooth, cv::Size(13,13), -1, -1);

	//first derivatives with Sobel operator
	//-----------------------------------------------------------------------
	cv::Mat x_deriv;
	cv::Mat y_deriv;

	int scale = 1;
	int delta = 0;
	int ddepth = CV_32F;//CV_16S; //depth (format) of output data
	//x-derivative:
	cv::Sobel(depth_image_smooth,x_deriv,ddepth,1,0,11,scale,delta);	//order of derivative in x-direction: 1, in y-direction: 0

	//y-derivative
	cv::Sobel(depth_image_smooth,y_deriv,ddepth,0,1,11,scale,delta);	//order of derivative in y-direction: 1, in x-direction: 0

	//compute gradient magnitude
	cv::Mat  x_pow2, y_pow2;
	//cv::magnitude(x_deriv,y_deriv,grad);
	cv::pow(x_deriv,2.0, x_pow2);
	cv::pow(y_deriv,2.0, y_pow2);
	cv::sqrt(x_pow2 + y_pow2, grad);


	cv::imshow("depth image smoothed", depth_image_smooth);
	cv::waitKey(10);

	/*	cv::imshow("x gradient",x_deriv);
		cv::waitKey(10);

		cv::imshow("y gradient",y_deriv);
		cv::waitKey(10);*/

	cv::normalize(grad,grad,0,1,cv::NORM_MINMAX);
	cv::setMouseCallback("Sobel",onMouse,0);
	cv::imshow("Sobel", grad);
	cv::waitKey(10);



	//second derivatives with Laplace operator
	//------------------------------------------------------------------------
	cv::Mat deriv_2,temp;
	int kernel_size = 7;

	//cv::Laplacian( depth_image_smooth, temp, ddepth, kernel_size, scale, delta );
	//cv::normalize(deriv_2,deriv_2,-1,2,cv::NORM_MINMAX);

	cv::Mat kernel = (cv::Mat_<float>(5,5) <<  0,          0,          1,          0,          0,
			0,          2,         -8,         2,          0,
			1,         -8,         20,         -8,          1,
			0,          2,         -8,          2,          0,
			0,          0,          1,          0,          0);


	/*cv::Mat kernel = (cv::Mat_< float >(7,7) << 0.1, 0.2, 0.5, 0.8, 0.5, 0.2, 0.1,
			0.2, 1.1, 2.5, 2.7, 2.5, 1.1, 0.2,
			0.5, 2.5, 0, -6.1, 0, 2.5, 0.5,
			0.8, 2.7, -6.1, -20, -6.1, 2.7, 0.8,
			0.5, 2.5, 0, -6.1, 0, 2.5, 0.5,
			0.2, 1.1, 2.5, 2.7, 2.5, 1.1, 0.2,
			0.1, 0.2, 0.5, 0.8, 0.5, 0.2, 0.1);*/
	//LoG (Laplacian and Gaussian) with sigma = 1.4
	/*cv::Mat kernel = (cv::Mat_< float >(9,9) << 0,0,3,2, 2, 2,3,0,0,
													0,2,3,5, 5, 5,3,2,0,
													3,3,5,3, 0, 3,5,3,3,
													2,5,3,-12, -23, -12,3,5,2,
													2,5,0,-23, -40, -23,0,5,2,
													2,5,3,-12, -23, -12,3,5,2,
													3,3,5,3, 0, 3,5,3,3,
													0,2,3,5, 5, 5,3,2,0,
													0,0,3,2, 2, 2,3,0,0);*/


	cv::filter2D(depth_image_smooth, temp, ddepth, kernel);

	float sat = 0.5;

	//Bild immer zeilenweise durchgehen
	for(int v=0; v<temp.rows; v++ )
	{
		for(int u=0; u< temp.cols; u++)
		{
			if(temp.at<float>(v,u) < -sat)
			{
				temp.at<float>(v,u) = - sat;
			}
			else if(temp.at<float>(v,u) > sat)
			{
				temp.at<float>(v,u) =  sat;
			}
		}
	}
	//normalize values to range between 0 and 1, so that all values can be depicted in an image
	cv::normalize(temp,deriv_2,0,1,cv::NORM_MINMAX);

	cv::imshow("Laplacian", deriv_2);
	cv::waitKey(10);
}



void SurfaceClassification::computeFPFH(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

	//Ptr in constPtr umwandeln
	//pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_cptr = pointcloud;
	//pcl::PCLBase< pcl::PointXYZ >::PointCloudConstPtr cloudWithoutColor;
	// pcl::copyPointCloud(*pointcloud, *cloudWithoutColor);
	pcl::PCLBase< pcl::PointXYZRGB >::PointCloudConstPtr cloud_cptr = pointcloud;
	ne.setInputCloud (cloud_cptr);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).

	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 1cm
	ne.setRadiusSearch (0.01);

	// Compute the features
	ne.compute (*cloud_normals);


	// cloud_normals->points.size () should have the same size as the input cloud->points.size ()

	std::cout << "normal estimation finished \n";

	// visualize normals
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	viewer.setBackgroundColor (0.0, 0.0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud);


	viewer.addPointCloud<pcl::PointXYZRGB> (pointcloud, rgb, "sample cloud");
	viewer.addPointCloudNormals<pcl::PointXYZRGB,pcl::Normal>(pointcloud, cloud_normals,10,0.005,"normals");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer.addCoordinateSystem (1.0);
	viewer.initCameraParameters ();



	//--------------------------------------------------------------------------------
	//compute FPFH

	// Create the FPFH estimation class, and pass the input dataset+normals to it
	//one histogram = concatenation of the four features
	//array of 33 values
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (cloud_cptr);
	fpfh.setInputNormals (cloud_normals);

	// Create an empty kdtree representation, and pass it to the PFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr treeFPFH (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
	fpfh.setSearchMethod (treeFPFH);

	// Output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch (0.01);

	// Compute the features
	fpfh.compute (*fpfhs);

	// pfhs->points.size () should have the same size as the input cloud->points.size ()*


	//visualise histogram
	pcl::visualization::PCLHistogramVisualizer histogramViewer;
	histogramViewer.setBackgroundColor(0,0,0);
	histogramViewer.addFeatureHistogram(*fpfhs,33,"FPFH",640,200);


	//for updating, the window has to be closed manually
	while (!viewer.wasStopped ())
	{
		viewer.spinOnce();
		histogramViewer.spinOnce();
	}
	viewer.removePointCloud("sample cloud");
}


