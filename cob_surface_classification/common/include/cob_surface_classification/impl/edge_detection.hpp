/*
 * edge_detection.hpp
 *
 *  Created on: May 17, 2013
 *      Author: rmb-ce
 */

#ifndef IMPL_EDGE_DETECTION_HPP_
#define IMPL_EDGE_DETECTION_HPP_

template <typename PointInT> void
EdgeDetection<PointInT>::approximateLine
(cv::Mat& depth_image, PointCloudInPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& abc, cv::Mat& n, cv::Mat& coordinates, bool& step)
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




	int xDist = dotEnd.x - dotIni.x;
	int yDist = dotEnd.y - dotIni.y;
	//include both points -> one coordinate more than distance
	int lineLength = std::max(std::abs(xDist),std::abs(yDist)) + 1;
	int sign = 1;	//iteration in positive or negative direction
	int xIter[lineLength];
	int yIter[lineLength];

	//iterate from dotIni to dotEnd. Set up indices for iteration:
	if(xDist != 0)
	{
		//line in x-direction
		if(xDist <0)
			sign = -1;

		for(int i=0; i<lineLength; i ++)
		{
			xIter[i] = dotIni.x + i * sign;
			yIter[i] = dotIni.y;
		}
	}
	else
	{
		//line in y-direction
		if(yDist <0)
			sign = -1;

		for(int i=0; i<lineLength; i ++)
		{
			xIter[i] = dotIni.x;
			yIter[i] = dotIni.y + i*sign;
		}
	}


	float x0,y0; //coordinates of reference point


	int iCoord;

	//---------------------------------------------------------------------------------------------------
	//SVD version
	//---------------------------------------------------------------------------------------------------

	/*timer.start();
		for(int itim=0;itim<10000;itim++)
		{*/

	coordinates = cv::Mat::zeros(lineLength,3,CV_32FC1);
	iCoord = 0;
	int iX = 0;
	int iY = 0;
	bool first = true;



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
			}
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

	if(iCoord == 0)
		//no valid data
		abc = cv::Mat::zeros(1,3,CV_32FC1);
	else
	{
		if(iCoord<lineLength)
		{
			coordinates.resize(iCoord);
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
		std::cout << timer.getElapsedTimeInMilliSec() /10000<< " ms for setting up coordinates-matrix + SVD (averaged over 10000 iterations)\n";*/



	/*

		//---------------------------------------------------------------------------------------------------
		//PCA version
		//---------------------------------------------------------------------------------------------------


		//timer.start();
		//for(int itim=0;itim<10000;itim++)
		//{

		iCoord = 0;
		int iX = 0;
		int iY = 0;
		bool first = true;
		coordinates = cv::Mat::zeros(lineLength,2,CV_32FC1);
		cv::PCA pca;


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
				}
				coordinates.at<float>(iCoord,0) = (pointcloud->at(xIter[iX],yIter[iY]).x - x0)
																			+ (pointcloud->at(xIter[iX],yIter[iY]).y - y0);
				coordinates.at<float>(iCoord,1) = depth_image.at<float>(yIter[iY], xIter[iX]);	//depth coordinate
				//coordinates.at<float>(iCoord,2) = 1.0;


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

		if(iCoord == 0)
			//no valid data
			n = cv::Mat::zeros(1,2,CV_32FC1);
		else if (iCoord == 1)
		{
			//coordinates already is its eigenvector
			n = coordinates;
		}
		else
		{
			//std::cout << "iCoord: " <<iCoord << "\n";
			if(iCoord<lineLength)
			{
				coordinates.resize(iCoord);
			}


			//PCA
			pca(coordinates, // pass the data
					cv::Mat(), // there is no pre-computed mean vector,
					// so let the PCA engine to compute it
					CV_PCA_DATA_AS_ROW, // input samples are stored as matrix rows
					1 // retain only the first principal component (the eigenvector to the largest eigenvalue of the covariance matrix)
			);
			n = pca.eigenvectors.row(0); 	//eigenvector to largest eigenvalue

			//directional vector should point from dotIni to dotEnd
			if(coordinates.at<float>(1,0) < 0)
				n *= -1;

		}

	 */
	/*}
		timer.stop();
		std::cout << timer.getElapsedTimeInMilliSec() /10000<< " ms for setting up coordinates-matrix + PCA (averaged over 10000 iterations)\n";*/

	//std::cout << "eigenvectors:\n"<< pca.eigenvectors <<"\n";
}

//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------





template <typename PointInT> void
EdgeDetection<PointInT>::approximateLine
(cv::Mat& depth_image, PointCloudInPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& abc)
{
	/* approximate depth coordinates between dotIni and dotEnd as a line */
	/* linear approximation using only the two points
	 * line equation: a*w + b*z + c =  0  (w refers to coordinate on the line between dotIni and dotEnd, z to depth coordinate) */
	/* -------------------------------------------------------------------------------------------------------------------------*/

	abc.at<float>(1) = 1;		//b=1
	float z1 = depth_image.at<float>(dotIni.y, dotIni.x);


	// c = -z1
	abc.at<float>(2) = -z1;

	float w2;

	//don't save points with nan-entries (no data available)
	if(!std::isnan(pointcloud->at(dotEnd.x,dotEnd.y).z))
	{
		float x0 = pointcloud->at(dotIni.x,dotIni.y).x;
		float y0 = pointcloud->at(dotIni.x,dotIni.y).y;
		//dotIni and dotEnd lie on line in either x- or y-direction
		w2 = (pointcloud->at(dotEnd.x,dotEnd.y).x - x0)
																+ (pointcloud->at(dotEnd.x,dotEnd.y).y - y0);
		if(w2 != 0)
		{
			float z2 = depth_image.at<float>(dotEnd.y, dotEnd.x);
			// a = -(z2-z1) /(w2-w1);
			abc.at<float>(0) = (z1-z2) /w2;
		}
		else
		{
			std::cout << "dotIni and dotEnd are the same!\n";
			abc = cv::Mat::zeros(1,3,CV_32FC1);
		}
	}
	else

		//no valid data
		abc = cv::Mat::zeros(1,3,CV_32FC1);


}

//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------




template <typename PointInT> void
EdgeDetection<PointInT>::scalarProduct(cv::Mat& abc1,cv::Mat& abc2,float& scalarProduct, int& concaveConvex, bool& step)
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


		//Richtungsvektoren der Geraden:
		cv::Mat n1 = (cv::Mat_<float>(1,2) << 1, b1);
		cv::Mat n2 = (cv::Mat_<float>(1,2) << 1, b2);

		cv::Mat n1Norm, n2Norm;
		cv::normalize(n1,n1Norm,1);
		cv::normalize(n2,n2Norm,1);

		//Skalarprodukt liegt zwischen 0 und 1
		scalarProduct = n1Norm.at<float>(0) * n2Norm.at<float>(0) + n1Norm.at<float>(1) * n2Norm.at<float>(1);


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


//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------


template <typename PointInT> void
EdgeDetection<PointInT>::approximateLineFullAndHalfDist
(cv::Mat& depth_image, PointCloudInPtr pointcloud, cv::Point2f dotIni, cv::Point2f dotEnd, cv::Mat& abc)
{
	//check if lines to dotEnd and dotEnd/2 go in the same direction. Only then dotIni is on an edge, else it is next to one or there is an outlier in the data.

	cv::Mat abc1 (cv::Mat::zeros(1,3,CV_32FC1));	//first approximation, full distance from dotIni to dotEnd
	cv::Mat abc2 (cv::Mat::zeros(1,3,CV_32FC1));	//second approximation, half distance from dotIni to dotEnd
	approximateLine(depth_image,pointcloud, dotIni,dotEnd, abc1);
	if(dotEnd.x == dotIni.x)
		dotEnd.y = (int) (dotEnd.y - (dotEnd.y-dotIni.y)/2);
	else if(dotEnd.y == dotEnd.y)
		dotEnd.x = (int) (dotEnd.x - (dotEnd.x-dotIni.x)/2);

	approximateLine(depth_image,pointcloud, dotIni,dotEnd, abc2);

	float scalProd = 1;
	int concConv = 0; //sinnlos, evtl noch neue Funktion scalarProduct() ohne concConv schreiben
	bool step = false;
	scalarProduct(abc1,abc2,scalProd,concConv,step);
	//std::cout <<"scalProd: " <<scalProd <<endl;
	if(scalProd > 0.95 || scalProd < -0.95)
	{
		abc = abc1;
	}
	else
		//lines not going in the same direction -> no edge
		abc = cv::Mat::zeros(1,3,CV_32FC1);

}

//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------

template <typename PointInT> void
EdgeDetection<PointInT>::thinEdges(cv::Mat& edgePicture, int xy)
{
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

//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------

template <typename PointInT> void
EdgeDetection<PointInT>::drawLines(cv::Mat& plotZW, cv::Mat& coordinates, cv::Mat& abc)
{
	// draw computed coordinates and approximates lines in plotZW
	//origin of coordinate system at the bottom in the middle
	//x-axis: w-coordinate	(first column of "coordinates")
	//y-axis: z-coordinate	(second column of "coordinates")
	// -----------------------------------------------------------


	//Skalierung des Tiefen-Wertes zur besseren Darstellung
	float scaleDepth = 600;



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
		w = (coordinates.at<float>(v,0) / maxW) * windowX_/2 + windowX_/2;
		//scale z-value for visualization
		z = coordinates.at<float>(v,1) *scaleDepth;

		//invert depth-coordinate for visualization (coordinate system of image-matrix is in upper left corner,
		//whereas we want our coordinate system to be at the bottom with the positive z-axis going upward
		z = windowY_ - z;
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
	x1 =  windowX_/2;
	x2 = (x2/ maxW) * windowX_/2 + windowX_/2;

	z1 = z1 * scaleDepth;
	z1 = windowY_ - z1;

	z2 = z2 *scaleDepth;
	z2 = windowY_ - z2;

	//draw line between the two points
	cv::line(plotZW,cv::Point2f(x1 ,z1 ), cv::Point2f(x2 , z2 ),CV_RGB(255,255,255),1);
}

//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------


template <typename PointInT> void
EdgeDetection<PointInT>::drawLineAlongN(cv::Mat& plotZW, cv::Mat& coordinates, cv::Mat& n)
{
	// draw computed coordinates and approximates lines in plotZW
	// use for visualization of the directional vectors computed by PCA version of approximateLine()
	// drawn line will only approximate the direction, not the absolute height!



	//origin of coordinate system at the bottom in the middle
	//x-axis: w-coordinate	(first column of "coordinates")
	//y-axis: z-coordinate	(second column of "coordinates")
	//
	//
	// -----------------------------------------------------------


	//Skalierung des Tiefen-Wertes zur besseren Darstellung
	float scaleDepth = 400;


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
		//project w-coordinates on interval [0,windowX_/2] and then shift to the middle windowX_/2
		w = (coordinates.at<float>(v,0) / maxW) * windowX_/2 + windowX_/2;
		//scale z-value for visualization
		z = coordinates.at<float>(v,1) *scaleDepth;

		//invert depth-coordinate for visualization (coordinate system of image-matrix is in upper left corner,
		//whereas we want our coordinate system to be at the bottom with the positive z-axis going upward
		z = windowY_ - z;
		cv::circle(plotZW,cv::Point2f(w,z),1,CV_RGB(255,255,255),2);
	}

	//line along n
	float x1 = 0;
	float z1 = 0;
	float x2 = n.at<float>(0,0);
	float z2 = n.at<float>(0,1);



	//scaling and shifting
	x1 =  windowX_/2;
	x2 = (x2/ maxW) * windowX_/2 + windowX_/2;

	z1 = z1 * scaleDepth;
	z1 = windowY_ - z1 - windowY_/2;

	z2 = z2 *scaleDepth ;
	z2 = windowY_ - z2 - windowY_/2;

	//draw line between the two points
	cv::line(plotZW,cv::Point2f(x1 ,z1 ), cv::Point2f(x2 , z2 ),CV_RGB(255,255,255),1);
}


//-----------------------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------

template <typename PointInT> void
EdgeDetection<PointInT>::computeDepthEdges
(cv::Mat depth_image, PointCloudInPtr pointcloud, cv::Mat& edgeImage)
{


	/*Timer timerFunc;
		timerFunc.start();*/

	//plot z over w, draw estimated lines
	cv::Mat plotZW (cv::Mat::zeros(windowX_,windowY_,CV_32FC1));
	//cv::Mat scalarProducts (cv::Mat::ones(depth_image.rows,depth_image.cols,CV_32FC1));
	cv::Mat concaveConvex (cv::Mat::zeros(depth_image.rows,depth_image.cols,CV_8UC1)); 	//0:neither concave nor convex; 125:concave; 255:convex
	cv::Mat concaveConvexY (cv::Mat::zeros(depth_image.rows,depth_image.cols,CV_8UC1)); 	//0:neither concave nor convex; 125:concave; 255:convex
	cv::Mat scalarProductsY (cv::Mat::ones(depth_image.rows,depth_image.cols,CV_32FC1));
	cv::Mat scalarProductsX (cv::Mat::ones(depth_image.rows,depth_image.cols,CV_32FC1));

	int iX=depth_image.cols/2;
	int iY=depth_image.rows/2;

	Timer timer;


	//	cout << timerFunc.getElapsedTimeInMilliSec() << " ms for initial definitions before loop\n";


	//loop over rows
	for(int iY = lineLength_/2; iY< depth_image.rows-lineLength_/2; iY++)
	{
		//loop over columns
		for(int iX = lineLength_/2; iX< depth_image.cols-lineLength_/2; iX++)
		{


			//scalarProduct of depth along lines in x-direction
			//------------------------------------------------------------------------
			cv::Point2f dotLeft(iX -lineLength_/2, iY);
			cv::Point2f dotRight(iX +lineLength_/2, iY);
			cv::Mat coordinates1;	//coordinates on the left side of the center point
			cv::Mat coordinates2;	//coordinates on the right
			cv::Mat abc1 (cv::Mat::zeros(1,3,CV_32FC1));	//line parameters a,b,c of line a*w+b*z+1=0, left line
			cv::Mat abc2 (cv::Mat::zeros(1,3,CV_32FC1));	//right line
			cv::Mat n1;
			cv::Mat n2;
bool step;

			/* line approximation using SVD
			 * ----------------------------------------------------------*/
			/*

	//boolean step will be set to true in approximateLine(), if there is a step either in coordinates1 or coordinates2
	//-> needs to be set to false before calling approximateLine() for both sides.
	//in scalarProduct(), boolean step will be considered when detecting a step at the central point.
	//if there is a step at the central point, the coordinates on the right and left to it should be continuous without step!
	//(else the step would be detected in the neighbourhood as well, leading to inaccuracies)
	step = false;
	//do not use point right at the center (would belong to both lines -> steps not correctly represented)
	approximateLine(depth_image,pointcloud, cv::Point2f(iX-1,iY),dotLeft, abc1,n1, coordinates1, step);
	//n1 wird nur gebraucht, falls PCA anstatt SVD

	//	timer.stop();
	//	std::cout << timer.getElapsedTimeInMilliSec() /10000 << " ms for lineApproximation (averaged over 10000 iterations)\n";



	//besser den Pixel rechts bzw.links von dotMiddle betrachten, damit dotMiddle nicht zu beiden Seiten dazu gerechnet wird (sonst ungenau bei Sprung)
	approximateLine(depth_image,pointcloud, cv::Point2f(iX+1,iY),dotRight, abc2,n2, coordinates2,step);
			 */

			/*	approximate line using only two points
			 * -------------------------------------------------------------------*/

			//no step detection in approximateLine from 2 points
			step = false;

			//std::cout << "left\n";
			approximateLineFullAndHalfDist(depth_image,pointcloud, cv::Point2f(iX-1,iY),dotLeft, abc1);
			//std::cout << "right\n";
			approximateLineFullAndHalfDist(depth_image,pointcloud, cv::Point2f(iX+1,iY),dotRight, abc2);




			/* -------------------------------------------------------------------*/



			//drawLines(plotZW,coordinates1,abc1);
			//drawLines(plotZW,coordinates2,abc2);

			//drawLineAlongN(plotZW,coordinates1,n1);
			//drawLineAlongN(plotZW,coordinates2,n2);


			float scalProdX = 1;
			int concConv = 0;
			if(abc1.at<float>(0,2) == 0 || abc2.at<float>(0,2) == 0)
			{
				//abc could not be approximated or no edge (see approximateLineFullAndHalfDist())
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

			/*

	//scalarProduct of depth along lines in y-direction
	//------------------------------------------------------------------------
	cv::Point2f dotDown(iX , iY-lineLength_/2);
	cv::Point2f dotUp(iX , iY+lineLength_/2);
	cv::Point2f dotMiddle(iX , iY );
	cv::Mat coordinates1Y = cv::Mat::zeros(1,3,CV_32FC1);
	cv::Mat coordinates2Y = cv::Mat::zeros(1,3,CV_32FC1);
	cv::Mat n1Y;
	cv::Mat n2Y;

	cv::Mat abc1Y (cv::Mat::zeros(1,3,CV_32FC1));
	cv::Mat abc2Y (cv::Mat::zeros(1,3,CV_32FC1));*/

			//timer.start();

			/* approximate lines using SVD
			 * -----------------------------------------------------------*/
			/*
	step = false;

	//do not use point right at the center (would belong to both lines -> steps not correctly represented)
	approximateLine(depth_image,pointcloud, cv::Point2f(iX,iY-1),dotDown, abc1Y, n1Y,coordinates1Y, step);

	//timer.stop();
	//cout << timer.getElapsedTimeInMilliSec() << " ms for lineApproximation\n";



	//besser den Pixel rechts bzw.links von dotMiddle betrachten, damit dotMiddle nicht zu beiden Seiten dazu gerechnet wird (sonst ungenau bei Sprung)
	approximateLine(depth_image,pointcloud, cv::Point2f(iX,iY+1),dotUp, abc2Y,n2Y, coordinates2Y,step);
			 */


			//std::cout << "step: " << step << "\n";


			/*	approximate line using only two points
			 * -------------------------------------------------------------------*/

			//no step detection in approximateLine from 2 points
			step = false;
			/*
	approximateLine(depth_image,pointcloud, cv::Point2f(iX,iY-1),dotDown, abc1Y);
	approximateLine(depth_image,pointcloud, cv::Point2f(iX,iY+1),dotUp, abc2Y);*/



			/* -------------------------------------------------------------------*/

			//drawLines(plotZW,coordinates1Y,abc1Y);
			//drawLines(plotZW,coordinates2Y,abc2Y);

			/*				float scalProdY = 1;
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

			 */
			//Minimum:
			//scalarProducts.at<float>(iY,iX) = (scalProdX < scalProdY)? scalProdX : scalProdY;
		}
	}

	/*	//thin edges in x- and y-direction separately
	thinEdges(scalarProductsX, 0);
	thinEdges(scalarProductsY, 1);

	for(int iY = lineLength_/2; iY< depth_image.rows-lineLength_/2; iY++)
	{
		for(int iX = lineLength_/2; iX< depth_image.cols-lineLength_/2; iX++)
		{
			//Minimum:
			edgeImage.at<float>(iY,iX) = std::min(scalarProductsX.at<float>(iY,iX), scalarProductsY.at<float>(iY,iX));
			if(edgeImage.at<float>(iY,iX) > edgeThreshold_)
				edgeImage.at<float>(iY,iX) = 1;
			else
				edgeImage.at<float>(iY,iX) = 0;
		}
	}*/





	//cv::imshow("depth over coordinate along line", plotZW);
	//cv::waitKey(10);

	cv::imshow("Skalarprodukt", scalarProductsX);
	cv::waitKey(10);



	//cv::imshow("concave = grey, convex = white", concaveConvexY);
	//cv::waitKey(10);

	//timerFunc.stop();
	//cout << timerFunc.getElapsedTimeInMilliSec() << " ms for depth_along_lines()\n";

}



#endif /* IMPL_EDGE_DETECTION_HPP_ */
