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
	//<> template (generisch); XYZRGB: Tiefendaten + Farbbild
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

void SurfaceClassification::approximateLine(cv::Mat& depth_image,pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& abc, cv::Mat& coordinates)
{
	//std::cout << "approximateLine()\n";


	//dotIni is origin of local coordinate system, dotEnd the end point of the line


	/* consider depth coordinates along the line between dotIni and dotEnd
	 * write them into a matrix
	 * approximation of the function z(w), that is depth(z) over coordinate on the line(w)
	 *
	 * linear regression via least squares minimisation (-> SVD)
	 * ----------------------------------------------------------*/

	//write depth of points along the line into a matrix
	//format of one line: [coordinate along the line, depth coordinate, 1]


	//iterate from dotIni to dotEnd:
	cv::LineIterator xIter (depth_image,dotIni,dotEnd);

	float x0,y0; //coordinates of reference point
	bool first = true;

	int v;
	cv::Mat currentCoord (cv::Mat::zeros(1,3,CV_32FC1));
	for( v=0; v<xIter.count; v++, ++xIter)
	{
		//später nicht einfach x bzw y-Koordinate betrachten, sondern (x-x0)²+(y-y0)² als w nehmen


		//dont't save points with nan-entries (no data available)
		if(!std::isnan(pointcloud->at(xIter.pos().x, xIter.pos().y).x))
		{
			if(first)
			{
				//origin of local coordinate system is the first point with valid data. Express coordinates relatively:
				x0 = pointcloud->at(xIter.pos().x,xIter.pos().y).x;
				y0 = pointcloud->at(xIter.pos().x,xIter.pos().y).y;
				coordinates.at<float>(0,0) = x0;
				coordinates.at<float>(0,1) = depth_image.at<float>(xIter.pos().y, xIter.pos().x);	//depth coordinate
				coordinates.at<float>(0,2) = 1.0;
				first = false;
				//std::cout << "coordinates: " <<coordinates << "\n";
			}
			else
			{
				currentCoord.at<float>(0,0) = pointcloud->at(xIter.pos().x, xIter.pos().y).x - x0;
				//ACHTUNG: Matrix depth_image: zuerst Zeilenindex (y), dann Spaltenindex (x) !!!!
				currentCoord.at<float>(0,1) = depth_image.at<float>(xIter.pos().y, xIter.pos().x);	//depth coordinate
				currentCoord.at<float>(0,2) = 1.0;
				coordinates.push_back(currentCoord);
				//std::cout << "coordinates: " <<coordinates << "\n";
				/*
							coordinates.at<float>(v,0) = pointcloud->at(xIter.pos().x, xIter.pos().y).x - x0;
							//ACHTUNG: Matrix depth_image: zuerst Zeilenindex (y), dann Spaltenindex (x) !!!!
							coordinates.at<float>(v,1) = depth_image.at<float>(xIter.pos().y, xIter.pos().x);	//depth coordinate
							coordinates.at<float>(v,2) = 1.0;*/
			}


		}
		/*else
		{
			v--;	//no row added to coordinates-matrix
			//coordinates.at<float>(v,0) = 0;
			//coordinates.at<float>(v,1) = 0;
			//coordinates.at<float>(v,2) = 0;
		}*/


	}

	//Reihen unterhalb von v löschen?

	//std::cout << "coordinates: " <<coordinates << "\n";
	/*std::cout << "v: " <<v << "\n";
	std::cout << "coordinates.rows: " <<coordinates.rows-1 << "\n";

	cv::Mat coordSmall;
	if(v<coordinates.rows-1)
	{
		//coordinates.copyTo(coordSmall,coordinates);
		coordinates.pop_back(coordinates.rows -1 - v);
	}

	std::cout << "coordSmall: " <<coordinates << "\n";
	//coordinates = coordSmall;
	//~coordSmall;*/

	cv::Mat sv;	//singular values
		cv::Mat u;	//left singular vectors
		cv::Mat vt;	//right singular vectors, transposed, 3x3

	//if there were valid coordinates available, perform SVD
	if(coordinates.rows >2)
	{


			cv::SVD::compute(coordinates,sv,u,vt);
			abc =  vt.row(2);
			//std::cout << "SVD performed\n";
	}

	//std::cout << "vt: \n"<<vt << "\n";

	//last column of v = last row of vt is x, so that y is minimal
	//parameters of the approximated line: aw+bz+1 = 0
	/*if(vt.rows !=3)
	{
		std::cout << "SVD failed\n";
	}
	else
		abc =  vt.row(2);

	if(std::isnan(abc.at<float>(0,0)) || std::isnan(abc.at<float>(0,1)))
	{
		std::cout << "abc is nan\n";
		std::cout << "coordinates: " <<coordinates << "\n";
	}*/



	//std::cout << "abc: " <<abc << "\n";

}

void SurfaceClassification::drawLines(cv::Mat& plotZW, cv::Mat& coordinates, cv::Mat& abc)
{
	// draw computed coordinates and approximates lines in plotZW
	//origin of coordinate system at the bottom in the middle
	//x-axis: w-coordinate	(first column of "coordinates")
	//y-axis: z-coordinate	(second column of "coordinates")
	// -----------------------------------------------------------


	//Skalierung des Tiefen-Wertes zur besseren Darstellung
	float scaleDepth = 300;


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




void SurfaceClassification::scalarProduct(cv::Mat& abc1,cv::Mat& abc2,float& scalarProduct, int& concaveConvex)
{
	//std::cout << "scalarProduct()\n";

	//compute scalar product


	//normal vector n=[1,(-c-a)/b]
	float b1 = -(abc1.at<float>(0) + abc1.at<float>(2))/abc1.at<float>(1);
	float b2 = -(abc2.at<float>(0) + abc2.at<float>(2))/abc2.at<float>(1);

	//std::cout << "b1: " <<b1 << "\n";
	//std::cout << "b2: " <<b2<< "\n";

	//in der Realität gibt es keine unendlich spitzen Kanten
	//erstmal begrenzen
	/*float maxInclination = 1000;
	if(std::isinf(b1))
		b1 = maxInclination;
	if(std::isinf(b2))
		b2 = maxInclination;*/

	//Richtungsvektoren der Geraden:
	cv::Mat n1 = (cv::Mat_<float>(1,2) << -1, -b1);
	cv::Mat n2 = (cv::Mat_<float>(1,2) << 1, b2);

	cv::Mat n1Norm, n2Norm;
	cv::normalize(n1,n1Norm,1);
	cv::normalize(n2,n2Norm,1);
	std::cout << "n1: " <<n1Norm << "\n";
	std::cout << "n2: " <<n2Norm<< "\n";

	//abc ist schon normiert, da Ergebnis der SVD eine orthonormale Matrix ist
	//Skalarprodukt muss zwischen 0 und 1 sein!
	cv::Mat scalProd = n1Norm * n2Norm.t();

	scalarProduct = scalProd.at<float>(0,0);
	//std::cout << "scalarProduct: " <<scalarProduct << "\n";


	//convex: second component of both normal vectors is positive
	//concave: negative
	if(n1.at<float>(1) > 0.05 && n2.at<float>(1) > 0.05)
	{
		//convex
		concaveConvex = 255;
	}
	else if (n1.at<float>(1) < 0.05 && n2.at<float>(1) < 0.05)
	{
		//concave
		concaveConvex = 125;
	}

}


void SurfaceClassification::depth_along_lines(cv::Mat& color_image, cv::Mat& depth_image, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{
	int lineLength = 30;

	/*cv::Point2f dotLeft(color_image.cols/2 -lineLength/2, color_image.rows/2);
	cv::Point2f dotRight(color_image.cols/2 +lineLength/2, color_image.rows/2);

	cv::Point2f dotMiddle(color_image.cols/2 , color_image.rows/2 );
	cv::Point2f dotUp(color_image.cols/2 , color_image.rows/2 +lineLength/2);
	cv::Point2f dotDown(color_image.cols/2 , color_image.rows/2 -lineLength/2);*/


	//zeichne Fadenkreuz:
	cv::line(color_image,cv::Point2f(color_image.cols/2 -lineLength/2, color_image.rows/2),cv::Point2f(color_image.cols/2 +lineLength/2, color_image.rows/2),CV_RGB(0,1,0),1);
	cv::line(color_image,cv::Point2f(color_image.cols/2 , color_image.rows/2 +lineLength/2),cv::Point2f(color_image.cols/2 , color_image.rows/2 -lineLength/2),CV_RGB(0,1,0),1);


	//plot z over w, draw estimated lines
	int windowX = 600;	//size of window in x-direction
	int windowY = 600;

	cv::Mat plotZW (cv::Mat::zeros(windowX,windowY,CV_32FC1));
	cv::Mat scalarProducts (cv::Mat::ones(color_image.rows,color_image.cols,CV_32FC1));
	cv::Mat concaveConvex (cv::Mat::zeros(color_image.rows,color_image.cols,CV_8UC1)); 	//0:neither concave nor convex; 1:concave; 2:convex
	cv::Mat scalarProductsY (cv::Mat::ones(color_image.rows,color_image.cols,CV_32FC1));
	cv::Mat scalarProductsX (cv::Mat::ones(color_image.rows,color_image.cols,CV_32FC1));

	int iX=color_image.cols/2;
	int iY=color_image.rows/2;

	int sampleStep = 0;	//computation only for every n-th pixel
	//loop over rows
	/*for(int iY = lineLength/2; iY< color_image.rows-lineLength/2; iY++)
	{
		//loop over columns
		for(int iX = lineLength/2; iX< color_image.cols-lineLength/2; iX++)
		{*/
			/*sampleStep ++;
			if(sampleStep == 5)
			{
				sampleStep = 0;*/



			//scalarProduct of depth along lines in x-direction
			//------------------------------------------------------------------------
			cv::Point2f dotLeft(iX -lineLength/2, iY);
			cv::Point2f dotRight(iX +lineLength/2, iY);
			//cv::line(color_image,dotLeft,dotRight,CV_RGB(0,1,0),1);
			cv::Point2f dotMiddle(iX , iY );
			cv::Mat coordinates1 = cv::Mat::zeros(1,3,CV_32FC1); //= cv::Mat::zeros(dotMiddle.x - dotLeft.x,3,CV_32FC1);
			cv::Mat coordinates2 = cv::Mat::zeros(1,3,CV_32FC1); //= cv::Mat::zeros(dotRight.x - dotMiddle.x,3,CV_32FC1);


			cv::Mat abc1 (cv::Mat::zeros(1,3,CV_32FC1));

			//approximateLine(depth_image,pointcloud, dotMiddle,dotLeft, abc1, coordinates1);
			approximateLine(depth_image,pointcloud, cv::Point2f(iX-1,iY),dotLeft, abc1, coordinates1);


			cv::Mat abc2 (cv::Mat::zeros(1,3,CV_32FC1));
			//approximateLine(depth_image,pointcloud, dotMiddle,dotRight, abc2, coordinates2);
			//besser den Pixel rechts bzw.links von dotMiddle betrachten, damit dotMiddle nicht zu beiden Seiten dazu gerechnet wird (sonst ungenau bei Sprung)
			approximateLine(depth_image,pointcloud, cv::Point2f(iX+1,iY),dotRight, abc2, coordinates2);
			//std::cout<< "coordinates2" <<coordinates2 << "\n";

			drawLines(plotZW,coordinates1,abc1);
			drawLines(plotZW,coordinates2,abc2);


			float scalProdX = 1;
			int concConv = 0;
			if(abc1.at<float>(0,0) == 0 || abc2.at<float>(0,0) == 0)
			{
				//abc konnte nicht approximiert werden
				scalProdX = 1;
			}
			else
				scalarProduct(abc1,abc2,scalProdX,concConv);

			std::cout << "scalarProduct: " <<scalProdX << "\n";
			std::cout << "concave 125 grau, konvex 255 weiß: " <<concConv << "\n";
			//scalarProductsX.at<float>(iY,iX) = scalProdX;


			//if (scalProdX < 0.8)
			//concaveConvex.at<float>(iY,iX) = concConv;

			/*



			//alte Variablen hier löschen?!


			//scalarProduct of depth along lines in y-direction
			//-----------------------------------------------------------------------

/*


				cv::Point2f dotDown(iX, iY -lineLength/2);
				cv::Point2f dotUp(iX, iY +lineLength/2);
				//cv::line(color_image,dotLeft,dotRight,CV_RGB(0,1,0),1);
				//cv::Point2f dotMiddle(iX , iY );
				cv::Mat coordinates1y = cv::Mat::zeros(dotMiddle.y - dotDown.y,3,CV_32FC1);
				cv::Mat coordinates2y = cv::Mat::zeros(dotUp.y - dotMiddle.y,3,CV_32FC1);

				cv::Mat abc1y (cv::Mat::zeros(1,3,CV_32FC1));
				approximateLine(depth_image,dotDown,dotMiddle, abc1y, coordinates1y,1);

				cv::Mat abc2y (cv::Mat::zeros(1,3,CV_32FC1));
				approximateLine(depth_image,dotMiddle,dotUp, abc2y, coordinates2y,1);


				//drawLines(plotZW,coordinates1,abc1,dotDown,dotMiddle,0);
				//drawLines(plotZW,coordinates2,abc2,dotMiddle,dotUp,1);

				float scalProdY = 1;
				int concConvy = 0;
				scalarProduct(abc1y,abc2y,scalProdY,concConvy);
				//scalarProductsY.at<float>(iY,iX) = scalProdY;

				//	concaveConvex.at<float>(iY,iX) = concConv;


				//Maximum:
				scalarProducts.at<float>(iY,iX) = (scalProdX > scalProdY)? scalProdX : scalProdY;


			 */


			//}
		/*}
	}*/
	//std::cout << "scalarProduct: " <<scalarProductsX << "\n";

	cv::imshow("depth over coordinate along line", plotZW);
	cv::waitKey(10);


	//cv::Mat scalarProductsNorm;
	//cv::normalize(scalarProducts,scalarProductsNorm,0,1,cv::NORM_MINMAX);

	//scalarproduct = 1 for flat surfaces, = 0 for perpendicular lines
	//cv::Mat vis (cv::Mat::ones(color_image.rows,color_image.cols,CV_32FC1));
	//cv::Mat sub = vis - scalarProductsX;
	//cv::normalize(scalarProductsX,vis,0,1,cv::NORM_MINMAX);

	//cv::imshow("Skalarprodukt", scalarProductsX);
	//cv::waitKey(10);

	//cv::imshow("concave = grey, convex = white", concaveConvex);
	//cv::waitKey(10);



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


