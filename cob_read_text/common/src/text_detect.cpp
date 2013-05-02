/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2012 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: cob_read_text
 * \note
 * ROS stack name: cob_object_perception
 * \note
 * ROS package name: cob_read_text
 *
 * \author
 * Author: Robert Heinze
 * \author
 * Supervised by: Richard Bormann
 *
 * \date
 * Date of creation: August 2012
 *
 * \brief
 * Converts color image with text on it into text-file containing found text.
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

/**

 * Implementation based on Menglong Zhu and "Detecting Text in Natural Scenes with
 * Stroke Width Transform", Boris Epshtein, Eyal Ofek, Yonatan Wexler
 * CVPR 2010*
 *
 */

#include <cob_read_text/text_detect.h>
#include <map>

DetectText::DetectText()
{
	eval_ = false;
	enableOCR_ = true;
}

DetectText::DetectText(bool eval, bool enableOCR)
{
	eval_ = eval;
	enableOCR_ = enableOCR;
}

DetectText::~DetectText()
{
}

void DetectText::detect(std::string filename)
{
	// Read image from file
	filename_ = filename;
	originalImage_ = cv::imread(filename_);

//	cv::resize(originalImage_, originalImage_, cv::Size(), 0.25, 0.25);

	if (!originalImage_.data)
	{
		ROS_ERROR("Cannot read image input.");
		return;
	}
	//mode_ = IMAGE;
	detect();
}

void DetectText::detect(cv::Mat& image)
{
	// Read image from topic
	filename_ = std::string("streaming.jpg");
	originalImage_ = image;
	//mode_ = STREAM;
	detect();
}

void DetectText::readLetterCorrelation(const char* file)
{
	std::cout << std::endl;
	std::cout << "Correlation:" << file << std::endl;
	std::ifstream fin(file);
	correlation_ = cv::Mat(256, 256, CV_32F, cv::Scalar(0));
	float number;
	for (int i = 0; i < 256; i++)
		for (int j = 0; j < 256; j++)
		{
			assert(fin >> number);
			correlation_.at<float> (i, j) = number;
		}
}

void DetectText::readWordList(const char* filename)
{
	std::ifstream fin(filename);
	std::string word;
	wordList_.clear();
	while (fin >> word)
		wordList_.push_back(word);
	assert(wordList_.size());
	std::cout << "read in " << wordList_.size() << " words from " << std::string(filename) << std::endl;
}

cv::Mat& DetectText::getDetection()
{
	return resultImage_;
}

std::vector<std::string>& DetectText::getWords()
{
	return finalTexts_;
}

std::vector<cv::RotatedRect>& DetectText::getBoxes()
{
	return finalBoxes_;
}

void DetectText::detect()
{
	if (processing_method_ == ORIGINAL_EPSHTEIN)
		detect_original_epshtein();
	else if (processing_method_ == BORMANN)
		detect_bormann();
	else
		std::cout << "DetectText::detect: Error: Desired processing method is not implemented." << std::endl;
}


void extractVChannel(cv::Mat& img, cv::Mat& V)
{
	cv::cvtColor(img, img, CV_BGR2HSV);

	std::vector<cv::Mat> channels;
	cv::split(img, channels);
	channels[2].copyTo(V);

	cv::cvtColor(img, img, CV_HSV2BGR);

	return;

}

void subVChannel(cv::Mat& img, cv::Mat& V)
{
	cv::cvtColor(img, img, CV_BGR2HSV);

	std::vector<cv::Mat> channels;
	cv::split(img, channels);
	channels[2] = V;
	cv::merge(channels, img);

	cv::cvtColor(img, img, CV_HSV2BGR);

	return;
}

void dct(cv::Mat& input_img)
{
	cv::Mat img = cv::Mat(input_img.rows, input_img.cols, CV_8UC1);
	if (input_img.channels() == 3)
	{
		extractVChannel(input_img, img);
		std::cout << "extracting" << std::endl;
	}
	else
	{
		img = input_img;
	}

	// Dct conversion on logarithmic image
	cv::resize(img, img, cv::Size(input_img.cols * 2, input_img.rows * 2));
	//float mask_arr[]={-1, -1, -1 , -1 , 9 , -1, -1 , -1 ,-1};
	//cv::Mat mask=cv::Mat(3,3,CV_32FC1,mask_arr);
	//cv::filter2D(img,img,-1,mask);
	cv::equalizeHist(img, img);
	img.convertTo(img, CV_32FC1);
	//cv::Scalar mu=cv::mean(img);
	cv::Scalar mu, sigma;
	cv::meanStdDev(img, mu, sigma);

	double C_00 = log(mu.val[0]) * sqrt(img.cols * img.rows);

	//img=img+1;
	//cv::log(img,img);
	//----------------------------
	cv::pow(img, 0.2, img);
	cv::dct(img, img);

	//---------------------------------------
	img.at<float>(0, 0) = C_00;
	img.at<float>(0, 1) /= 50;
	img.at<float>(0, 2) /= 50;

	img.at<float>(1, 0) /= 50;
	img.at<float>(1, 1) /= 50;

	//img.at<float>(1,0)=0;
	//--------------------------------------

	cv::idct(img, img);
	cv::normalize(img, img, 0, 255, cv::NORM_MINMAX);

	cv::resize(img, img, cv::Size(img.cols / 2, img.rows / 2));

	img.convertTo(img, CV_8UC1);
	//cv::blur(img,img,cv::Size(3,3));

	if (input_img.channels() == 3)
	{
		subVChannel(input_img, img);
	}
	else
	{
		input_img = img;
	}
}


void DetectText::detect_original_epshtein()
{
	// clear data fields
	finalBoundingBoxes_.clear();
	finalRotatedBoundingBoxes_.clear();
	finalBoundingBoxesQualityScore_.clear();

	// start timer
	double start_time;
	double time_in_seconds;
	start_time = std::clock();

	// Smooth image
	if (smoothImage) // default: turned off
	{
		if (debug["showEdge"] == true)
			cv::imshow("original", originalImage_);

//		dct(originalImage_);
//
//		if (debug["showEdge"] == true)
//			cv::imshow("original dct", originalImage_);

		cv::Mat dummy = originalImage_.clone();
//		cv::cvtColor(originalImage_, dummy, CV_BGR2Lab);	// BGR


//		cv::bilateralFilter(dummy, originalImage_, 7, 20, 50); // sensor noise
		cv::bilateralFilter(dummy, originalImage_, 13, 40, 10); // sensor noise
//		originalImage_ = sharpenImage(originalImage_);
//		dummy = originalImage_.clone();
//		cv::bilateralFilter(dummy, originalImage_, 9, 30, 10); // sensor noise

//		std::vector<cv::Mat> singleChannels;
//		cv::split(originalImage_, singleChannels);
//		for (int i=0; i<1; i++)
//			cv::equalizeHist(singleChannels[i], singleChannels[i]);
//		cv::merge(singleChannels, originalImage_);

		if (debug["showEdge"] == true)
		{
			cv::imshow("original filtered", originalImage_);
			cv::waitKey();
		}
	}

	// grayImage for SWT
	grayImage_ = cv::Mat(originalImage_.size(), CV_8UC1, cv::Scalar(0));
	cv::cvtColor(originalImage_, grayImage_, CV_BGR2GRAY);

	// Show image information
	std::cout << std::endl;
	std::cout << "Image: " << filename_ << std::endl;
	std::cout << "Size:" << grayImage_.cols << " x " << grayImage_.rows << std::endl << std::endl;
	preprocess();

	// bright font
	firstPass_ = true;
	pipeline();
	disposal();

	// dark font
	firstPass_ = false;
	pipeline();
	disposal();		// todo: check whether this harms any of the following processing



	if (processing_method_==ORIGINAL_EPSHTEIN)
	{
		// some feasibility checks
//		start_time = clock();
//		filterBoundingBoxes(boundingBoxes, ccmap, boundingBoxFilterParameter); // filters boxes based on height and width -> makes no sense when text is rotated
//		time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
//		std::cout << "[" << time_in_seconds << " s] in filterBoundingBoxes: " << boundingBoxes.size() << " boundingBoxes found" << std::endl;


		// merge bright and dark text lines
		std::vector<TreeNode> nodes(textRegions_.size());
		for (unsigned int i=0; i<textRegions_.size(); i++)
		{
			nodes[i].parent = -1;
			nodes[i].rank = 0;
			nodes[i].element = i;
		}

		for (unsigned int i=0; i<textRegions_.size(); i++)
		{
			int root = i;
			while (nodes[root].parent != -1)
				root = nodes[root].parent;
			for (unsigned int j=0; j<textRegions_.size(); j++)
			{
				if (i!=j && sameTextline(textRegions_[nodes[i].element], textRegions_[nodes[j].element])==true)
				{
//					cv::Mat mergeImage1 = originalImage_.clone();
//					cv::rectangle(mergeImage1, cv::Point(textRegions_[nodes[i].element].boundingBox.x,textRegions_[nodes[i].element].boundingBox.y), cv::Point(textRegions_[nodes[i].element].boundingBox.x+textRegions_[nodes[i].element].boundingBox.width,textRegions_[nodes[i].element].boundingBox.y+textRegions_[nodes[i].element].boundingBox.height), cv::Scalar(255,0,255));
//					cv::rectangle(mergeImage1, cv::Point(textRegions_[nodes[j].element].boundingBox.x,textRegions_[nodes[j].element].boundingBox.y), cv::Point(textRegions_[nodes[j].element].boundingBox.x+textRegions_[nodes[j].element].boundingBox.width,textRegions_[nodes[j].element].boundingBox.y+textRegions_[nodes[j].element].boundingBox.height), cv::Scalar(255,0,255));
//					cv::imshow("merged lines pre", mergeImage1);
//					cv::waitKey();

					int root2 = j;
					while (nodes[root2].parent != -1)
						root2 = nodes[root2].parent;

					if (root != root2)
					{
						if(nodes[root].rank > nodes[root2].rank)
							nodes[root2].parent = root;
						else
						{
							nodes[root].parent = root2;
							nodes[root2].rank += (nodes[root].rank==nodes[root2].rank ? 1 : 0);
							root = root2;
						}

						// collapse a branch to direct children of root
						int node = j;
						while(nodes[node].parent != -1)
						{
							int temp = nodes[node].parent;
							nodes[node].parent = root;
							node = temp;
						}
						node = i;
						while(nodes[node].parent != -1)
						{
							int temp = nodes[node].parent;
							nodes[node].parent = root;
							node = temp;
						}
					}
				}
			}
		}

		// merge textline pairs
//		cv::Mat mergeImage = originalImage_.clone();
		std::vector<TextRegion> mergedTextRegions;
		int classIndex = 0;
		for (unsigned int i=0; i<textRegions_.size(); i++)
		{
			int root = i;
			while (nodes[root].parent != -1)
				root = nodes[root].parent;
			if (nodes[root].rank >= 0)
				nodes[root].rank = ~classIndex++;

			int insertIndex = ~nodes[root].rank;

//			std::cout << "i=" << i << "  root=" << root << "   insertIndex=" << insertIndex << "   mergedTextRegions=" << mergedTextRegions.size() << std::endl;

			if (insertIndex < (int)mergedTextRegions.size())
			{
				// check whether to keep or replace the existing text region
//				cv::rectangle(mergeImage, cv::Point(textRegions_[i].boundingBox.x,textRegions_[i].boundingBox.y), cv::Point(textRegions_[i].boundingBox.x+textRegions_[i].boundingBox.width,textRegions_[i].boundingBox.y+textRegions_[i].boundingBox.height), cv::Scalar(255,0,255));
//				cv::rectangle(mergeImage, cv::Point(mergedTextRegions[insertIndex].boundingBox.x,mergedTextRegions[insertIndex].boundingBox.y), cv::Point(mergedTextRegions[insertIndex].boundingBox.x+mergedTextRegions[insertIndex].boundingBox.width,mergedTextRegions[insertIndex].boundingBox.y+mergedTextRegions[insertIndex].boundingBox.height), cv::Scalar(255,0,255));
//				cv::imshow("merged lines", mergeImage);
//				cv::waitKey();
				if (mergedTextRegions[insertIndex].boundingBox.width < textRegions_[i].boundingBox.width)
				{
//					cv::rectangle(mergeImage, cv::Point(textRegions_[i].boundingBox.x,textRegions_[i].boundingBox.y), cv::Point(textRegions_[i].boundingBox.x+textRegions_[i].boundingBox.width,textRegions_[i].boundingBox.y+textRegions_[i].boundingBox.height), cv::Scalar(0,255,0));
//					cv::rectangle(mergeImage, cv::Point(mergedTextRegions[insertIndex].boundingBox.x,mergedTextRegions[insertIndex].boundingBox.y), cv::Point(mergedTextRegions[insertIndex].boundingBox.x+mergedTextRegions[insertIndex].boundingBox.width,mergedTextRegions[insertIndex].boundingBox.y+mergedTextRegions[insertIndex].boundingBox.height), cv::Scalar(0,0,255));
//					cv::imshow("merged lines", mergeImage);
//					std::cout<< "replace\n";
//					cv::waitKey();
					mergedTextRegions[insertIndex] = textRegions_[i];
				}
//				else
//				{
//					cv::rectangle(mergeImage, cv::Point(textRegions_[i].boundingBox.x,textRegions_[i].boundingBox.y), cv::Point(textRegions_[i].boundingBox.x+textRegions_[i].boundingBox.width,textRegions_[i].boundingBox.y+textRegions_[i].boundingBox.height), cv::Scalar(0,0,255));
//					cv::rectangle(mergeImage, cv::Point(mergedTextRegions[insertIndex].boundingBox.x,mergedTextRegions[insertIndex].boundingBox.y), cv::Point(mergedTextRegions[insertIndex].boundingBox.x+mergedTextRegions[insertIndex].boundingBox.width,mergedTextRegions[insertIndex].boundingBox.y+mergedTextRegions[insertIndex].boundingBox.height), cv::Scalar(0,255,0));
//					cv::imshow("merged lines", mergeImage);
//					std::cout<< "keep\n";
//					cv::waitKey();
//				}
			}
			else
			{
				// create text region
//				cv::rectangle(mergeImage, cv::Point(textRegions_[i].boundingBox.x,textRegions_[i].boundingBox.y), cv::Point(textRegions_[i].boundingBox.x+textRegions_[i].boundingBox.width,textRegions_[i].boundingBox.y+textRegions_[i].boundingBox.height), cv::Scalar(255,0,0));
//				cv::imshow("merged lines", mergeImage);
//				cv::waitKey();
				mergedTextRegions.push_back(textRegions_[i]);
			}
		}
		textRegions_ = mergedTextRegions;


		// separating several lines of text
//		start_time = clock();
//		breakLines(textRegions_);
//		time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
//		std::cout << "[" << time_in_seconds << " s] in breakLines: " << textRegions_.size() << " textRegions_ after breaking blocks into lines" << std::endl << std::endl;

		// separate words on a single line
		start_time = clock();
		breakLinesIntoWords(textRegions_);
		time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
		std::cout << "[" << time_in_seconds << " s] in breakLinesIntoWords: " << textRegions_.size() << " textRegions_ after breaking blocks into lines" << std::endl << std::endl;

		// write found bounding boxes into the respective structures
		for (unsigned int i=0; i<textRegions_.size(); i++)
		{
			finalBoundingBoxes_.push_back(textRegions_[i].boundingBox);
			finalRotatedBoundingBoxes_.push_back(cv::RotatedRect(cv::Point2f(textRegions_[i].boundingBox.x+0.5*textRegions_[i].boundingBox.width, textRegions_[i].boundingBox.y+0.5*textRegions_[i].boundingBox.height),
					cv::Size2f(textRegions_[i].boundingBox.width, textRegions_[i].boundingBox.height), 0.f));
			finalBoundingBoxesQualityScore_.push_back(textRegions_[i].qualityScore);
		}
	}



	// filter boxes that lie completely inside others
	for (int i=(int)finalBoundingBoxes_.size()-1; i>=0; i--)
	{
		for (int j=0; j<(int)finalBoundingBoxes_.size(); j++)
		{
			if (j==i)
				continue;
			if ((finalBoundingBoxes_[i] & finalBoundingBoxes_[j]).area()>0.95*finalBoundingBoxes_[i].area())
//			if ((finalBoundingBoxes_[i] & finalBoundingBoxes_[j]).area()>0.85*std::min(finalBoundingBoxes_[i].area(), finalBoundingBoxes_[j].area()) && (finalBoundingBoxes_[i].area()<finalBoundingBoxes_[j].area()))
//			if (((finalBoundingBoxes_[i] & finalBoundingBoxes_[j]).area()>0.85*finalBoundingBoxes_[i].area() && finalBoundingBoxesQualityScore_[i]<=finalBoundingBoxesQualityScore_[j]) ||
//					((finalBoundingBoxes_[i] & finalBoundingBoxes_[j]).area()>0.85*finalBoundingBoxes_[j].area() && finalBoundingBoxesQualityScore_[i]<finalBoundingBoxesQualityScore_[j]))
			{
				finalBoundingBoxes_.erase(finalBoundingBoxes_.begin()+i);
				finalRotatedBoundingBoxes_.erase(finalRotatedBoundingBoxes_.begin()+i);
				finalBoundingBoxesQualityScore_.erase(finalBoundingBoxesQualityScore_.begin()+i);
				break;
			}
//			if ((finalBoundingBoxes_[i] & finalBoundingBoxes_[j]).area()>0.85*std::min(finalBoundingBoxes_[i].area(), finalBoundingBoxes_[j].area()))
//			{
//				finalBoundingBoxes_[j].x = std::min(finalBoundingBoxes_[i].x, finalBoundingBoxes_[j].x);
//				finalBoundingBoxes_[j].width = std::max(finalBoundingBoxes_[i].x+finalBoundingBoxes_[i].width, finalBoundingBoxes_[j].x+finalBoundingBoxes_[j].width) - finalBoundingBoxes_[j].x;
//				finalBoundingBoxes_[j].y = std::min(finalBoundingBoxes_[i].y, finalBoundingBoxes_[j].y);
//				finalBoundingBoxes_[j].height = std::max(finalBoundingBoxes_[i].y+finalBoundingBoxes_[i].height, finalBoundingBoxes_[j].y+finalBoundingBoxes_[j].height) - finalBoundingBoxes_[j].y;
//				finalBoundingBoxes_.erase(finalBoundingBoxes_.begin()+i);
//				finalRotatedBoundingBoxes_.erase(finalRotatedBoundingBoxes_.begin()+i);
//				finalBoundingBoxesQualityScore_.erase(finalBoundingBoxesQualityScore_.begin()+i);
//				break;
//			}
		}
	}

	std::cout << std::endl << "Found " << transformedImage_.size() << " boundingBoxes for OCR." << std::endl << std::endl;

	// OCR
	if (enableOCR_ == true)
	{
		//  ocrPreprocess(transformedBoundingBoxes_);
		//  ocrPreprocess(notTransformedBoundingBoxes_);

		// delete boxes that are complete inside other boxes
		//deleteDoubleBrokenWords(boundingBoxes_);


		// fill textImages_ with either rectangles or rotated rectangles
		for (size_t i = 0; i < transformedImage_.size(); i++)
		{
			if (transformImages)
			{
				cv::Mat rotatedFlippedBox;
				cv::flip(transformedImage_[i], rotatedFlippedBox, -1);
				textImages_.push_back(sharpenImage(transformedImage_[i]));
				textImages_.push_back(sharpenImage(rotatedFlippedBox));
			}
			else
			{
				textImages_.push_back(sharpenImage(notTransformedImage_[i]));
			}

			// sometimes tesseract likes gray images
			//    cv::Mat grayImage(notTransformedBoundingBoxes_[i].size(), CV_8UC1, cv::Scalar(0));
			//    cv::cvtColor(notTransformedBoundingBoxes_[i], grayImage, CV_RGB2GRAY);
			// textImages_.push_back(sharpenImage(grayImage));

			// binary image
			// textImages_.push_back(binarizeViaContrast(transformedBoundingBoxes_[i]));

			// binary image #2
		}

		ocrRead(textImages_);
	}
	else
	{
		finalBoxes_ = finalRotatedBoundingBoxes_;
		finalTexts_.resize(finalBoxes_.size(), "");
	}

	// Draw output on resultImage_
	showBoundingBoxes(finalBoxes_, finalTexts_);

	std::cout << "eval_: " << eval_ << std::endl;

	// Write results
	if (eval_)
		writeTxtsForEval();
	else
		cv::imwrite(outputPrefix_ + "_detection.jpg", resultImage_);

	// Show results
	if (debug["showResult"])
	{
		cv::imshow("detection", resultImage_);
		cvMoveWindow("detection", 0, 0);
		cv::waitKey(0);
	}

	time_in_seconds = (clock() - start_time) / (double) CLOCKS_PER_SEC;
	std::cout << std::endl << "[" << time_in_seconds << " s] total in process\n" << std::endl;
}

void DetectText::detect_bormann()
{
	// clear data fields
	finalBoundingBoxes_.clear();
	finalRotatedBoundingBoxes_.clear();
	finalBoundingBoxesQualityScore_.clear();

	// start timer
	double start_time;
	double time_in_seconds;
	start_time = std::clock();

	// grayImage for SWT
	grayImage_ = cv::Mat(originalImage_.size(), CV_8UC1, cv::Scalar(0));
	cv::cvtColor(originalImage_, grayImage_, CV_RGB2GRAY);

	// Show image information
	std::cout << std::endl;
	std::cout << "Image: " << filename_ << std::endl;
	std::cout << "Size:" << grayImage_.cols << " x " << grayImage_.rows << std::endl << std::endl;
	preprocess();

	// bright font
	firstPass_ = true;
	pipeline();
	disposal();

	// dark font
	firstPass_ = false;
	pipeline();
	disposal();		// todo: check whether this harms any of the following processing

	std::cout << std::endl << "Found " << transformedImage_.size() << " boundingBoxes for OCR." << std::endl << std::endl;

	// OCR
	if (enableOCR_ == true)
	{
		//  ocrPreprocess(transformedBoundingBoxes_);
		//  ocrPreprocess(notTransformedBoundingBoxes_);

		// delete boxes that are complete inside other boxes
		//deleteDoubleBrokenWords(boundingBoxes_);


		// fill textImages_ with either rectangles or rotated rectangles
		for (size_t i = 0; i < transformedImage_.size(); i++)
		{
			if (transformImages)
			{
				cv::Mat rotatedFlippedBox;
				cv::flip(transformedImage_[i], rotatedFlippedBox, -1);
				textImages_.push_back(sharpenImage(transformedImage_[i]));
				textImages_.push_back(sharpenImage(rotatedFlippedBox));
			}
			else
			{
				textImages_.push_back(sharpenImage(notTransformedImage_[i]));
			}

			// sometimes tesseract likes gray images
			//    cv::Mat grayImage(notTransformedBoundingBoxes_[i].size(), CV_8UC1, cv::Scalar(0));
			//    cv::cvtColor(notTransformedBoundingBoxes_[i], grayImage, CV_RGB2GRAY);
			// textImages_.push_back(sharpenImage(grayImage));

			// binary image
			// textImages_.push_back(binarizeViaContrast(transformedBoundingBoxes_[i]));

			// binary image #2
		}

		ocrRead(textImages_);
	}
	else
	{
		finalBoxes_ = finalRotatedBoundingBoxes_;
		finalTexts_.resize(finalBoxes_.size(), "");
	}

	// Draw output on resultImage_
	showBoundingBoxes(finalBoxes_, finalTexts_);

	std::cout << "eval_: " << eval_ << std::endl;

	// Write results
	if (eval_)
		writeTxtsForEval();
	else
		cv::imwrite(outputPrefix_ + "_detection.jpg", resultImage_);

	// Show results
	if (debug["showResult"])
	{
		cv::imshow("detection", resultImage_);
		cvMoveWindow("detection", 0, 0);
		cv::waitKey(0);
	}

	time_in_seconds = (clock() - start_time) / (double) CLOCKS_PER_SEC;
	std::cout << std::endl << "[" << time_in_seconds << " s] total in process\n" << std::endl;
}

void DetectText::preprocess()
{
	if (processing_method_==ORIGINAL_EPSHTEIN)
		maxStrokeWidth_ = maxStrokeWidthParameter; // * 640./(double)std::max(grayImage_.cols, grayImage_.rows);
	else
		maxStrokeWidth_ = round((std::max(grayImage_.cols, grayImage_.rows)) / (float) maxStrokeWidthParameter);
	initialStrokeWidth_ = maxStrokeWidth_ * 2;

	// outputPrefix_: filename without extension
	int slashIndex = -1;
	int dotIndex = -1;
	for (size_t i = filename_.length() - 1; i != 0; i--)
	{
		if (dotIndex == -1 && filename_[i] == '.')
			dotIndex = i;
		if (slashIndex == -1 && filename_[i] == '/')
			slashIndex = i;
	}
	outputPrefix_ = filename_.substr(slashIndex + 1, dotIndex - slashIndex - 1);

	// add 600 pixel width to have space for displaying results
	cv::Mat img1(originalImage_.rows, originalImage_.cols + 600, originalImage_.type(), cv::Scalar(0, 0, 0));
	cv::Mat tmp = img1(cv::Rect(0, 0, originalImage_.cols, originalImage_.rows));
	originalImage_.copyTo(tmp);
	resultImage_ = img1; //.clone();
}

void DetectText::pipeline()
{
	double start_time;
	double time_in_seconds;

	start_time = clock();

	int searchDirection = 0;
	if (firstPass_)
	{
		std::cout << "--- Bright Font ---" << std::endl;
		searchDirection = 1;
	}
	else
	{
		std::cout << "--- Dark Font ---" << std::endl;
		searchDirection = -1;
	}

//	for (cannyThreshold1 = 10; cannyThreshold1<250; cannyThreshold1+=10)
//	{
//		for (cannyThreshold2 = cannyThreshold1; cannyThreshold2<250; cannyThreshold2+=10)
//		{
//			searchDirection = -1;
//			std::cout << " Canny1=" << cannyThreshold1 << "    Canny2=" << cannyThreshold2 << std::endl;
//			cv::Mat swtmap(grayImage_.size(), CV_32FC1, cv::Scalar(initialStrokeWidth_));
//			strokeWidthTransform(grayImage_, swtmap, searchDirection);
//			time_in_seconds = (clock() - start_time) / (double) CLOCKS_PER_SEC;
//			std::cout << "[" << time_in_seconds << " s] in strokeWidthTransform" << std::endl;
//		}
//	}
//	getchar();

	cv::Mat swtmap(grayImage_.size(), CV_32FC1, cv::Scalar(initialStrokeWidth_));
	strokeWidthTransform(grayImage_, swtmap, searchDirection);
	time_in_seconds = (clock() - start_time) / (double) CLOCKS_PER_SEC;
	std::cout << "[" << time_in_seconds << " s] in strokeWidthTransform" << std::endl;

	start_time = clock();
	cv::Mat ccmap(grayImage_.size(), CV_32FC1, cv::Scalar(-1));
	labeledRegions_.clear();
	nComponent_ = connectComponentAnalysis(swtmap, ccmap);
	time_in_seconds = (clock() - start_time) / (double) CLOCKS_PER_SEC;
	std::cout << "[" << time_in_seconds << " s] in connectComponentAnalysis: " << nComponent_ << " components found" << std::endl;

	start_time = clock();
	identifyLetters(swtmap, ccmap);
	time_in_seconds = (clock() - start_time) / (double) CLOCKS_PER_SEC;
	std::cout << "[" << time_in_seconds << " s] in identifyLetters: " << nLetter_ << " letters found" << std::endl;

	start_time = clock();
	groupLetters(swtmap, ccmap);
	time_in_seconds = (clock() - start_time) / (double) CLOCKS_PER_SEC;
	std::cout << "[" << time_in_seconds << " s] in groupLetters: " << letterGroups_.size() << " groups found" << std::endl;

	start_time = clock();
	//std::vector<TextRegion> textRegions; //replace or fuse connectedComponents
	chainPairs(textRegions_);
	time_in_seconds = (clock() - start_time) / (double) CLOCKS_PER_SEC;
	std::cout << "[" << time_in_seconds << " s] in chainPairs: " << textRegions_.size() << " chains found" << std::endl;

	//  start_time = clock();
	//  combineNeighborBoxes(boundingBoxes);
	//  time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
	//  std::cout << "[" << time_in_seconds << " s] in combineNeighborBoxes: " << boundingBoxes.size() << " chains remain"
	//      << std::endl;

	if (firstPass_)
		ccmapBright_ = ccmap.clone();
	else
		ccmapDark_ = ccmap.clone();

//	if (processing_method_==ORIGINAL_EPSHTEIN)
//	{
//		// some feasibility checks
////		start_time = clock();
////		filterBoundingBoxes(boundingBoxes, ccmap, boundingBoxFilterParameter); // filters boxes based on height and width -> makes no sense when text is rotated
////		time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
////		std::cout << "[" << time_in_seconds << " s] in filterBoundingBoxes: " << boundingBoxes.size() << " boundingBoxes found" << std::endl;
//
//		// separating several lines of text
//		start_time = clock();
//		breakLines(textRegions_);
//		time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
//		std::cout << "[" << time_in_seconds << " s] in breakLines: " << textRegions_.size() << " textRegions_ after breaking blocks into lines" << std::endl << std::endl;
//
//		// separate words on a single line
//		start_time = clock();
//		breakLinesIntoWords(textRegions_);
//		time_in_seconds = (clock() - start_time) / (double)CLOCKS_PER_SEC;
//		std::cout << "[" << time_in_seconds << " s] in breakLinesIntoWords: " << textRegions_.size() << " textRegions_ after breaking blocks into lines" << std::endl << std::endl;
//
//		// write found bounding boxes into the respective structures
//		for (unsigned int i=0; i<textRegions_.size(); i++)
//		{
//			finalBoundingBoxes_.push_back(textRegions_[i].boundingBox);
//			finalRotatedBoundingBoxes_.push_back(cv::RotatedRect(cv::Point2f(textRegions_[i].boundingBox.x+0.5*textRegions_[i].boundingBox.width, textRegions_[i].boundingBox.y+0.5*textRegions_[i].boundingBox.height),
//					cv::Size2f(textRegions_[i].boundingBox.width, textRegions_[i].boundingBox.height), 0.f));
//			finalBoundingBoxesQualityScore_.push_back(textRegions_[i].qualityScore);
//		}
//	}
//	else
//	{
//		start_time = clock();
//		ransacPipeline(textRegions_);
//		time_in_seconds = (clock() - start_time) / (double) CLOCKS_PER_SEC;
//
//		if (firstPass_)
//		{
//			std::cout << "[" << time_in_seconds << " s] in Ransac and Bezier: " << transformedImage_.size() << " boundingBoxes remain" << std::endl;
//			fontColorIndex_ = transformedImage_.size();
//		}
//		else
//			std::cout << "[" << time_in_seconds << " s] in Ransac and Bezier: " << transformedImage_.size() - fontColorIndex_ << " boundingBoxes remain" << std::endl;
//	}
}

void DetectText::strokeWidthTransform(const cv::Mat& image, cv::Mat& swtmap, int searchDirection)
{
	// Calculate edges and gradients
	if (firstPass_)
	{
		// compute edge map
		edgemap_ = computeEdgeMap(useColorEdge);
		closeOutline(edgemap_);
//		if (debug["showEdge"] == true)
//		{
//			cv::imshow("gray color edgemap closed", edgemap_);
//		}

		// compute partial derivatives
		Sobel(grayImage_, dx_, CV_32FC1, 1, 0, 3);
		Sobel(grayImage_, dy_, CV_32FC1, 0, 1, 3);

		theta_ = cv::Mat::zeros(grayImage_.size(), CV_32FC1);

		edgepoints_.clear();

		for (int y = 0; y < edgemap_.rows; y++)
			for (int x = 0; x < edgemap_.cols; x++)
				if (edgemap_.at<unsigned char>(y, x) == 255) // In case (x,y) is an edge
				{
					theta_.at<float>(y, x) = atan2(dy_.at<float>(y, x), dx_.at<float>(y, x)); //rise = arctan dy/dx
					edgepoints_.push_back(cv::Point(x, y)); //Save edge as point in edgepoints
				}

	}

	// Second Pass (SWT is not performed again):
	std::vector<cv::Point> strokePoints;
	updateStrokeWidth(swtmap, edgepoints_, strokePoints, searchDirection, UPDATE);
	updateStrokeWidth(swtmap, strokePoints, strokePoints, searchDirection, REFINE);

	// todo: reactivate
//	cv::Mat temp;
//	cv::dilate(swtmap, temp, cv::Mat(), cv::Point(-1, -1), 1);
//	cv::erode(temp, swtmap, cv::Mat(), cv::Point(-1, -1), 2);
//	cv::dilate(swtmap, temp, cv::Mat());
//	swtmap = temp;

	if (debug["showSWT"])
	{
		cv::Mat output(originalImage_.size(), CV_8UC3);
		cv::Mat swtmapNormalized;
		cv::normalize(swtmap, swtmapNormalized, 0, 255, cv::NORM_MINMAX);
		for (int y = 0; y < swtmap.rows; y++)
			for (int x = 0; x < swtmap.cols; x++)
			{
				double val = (swtmap.at<float>(y, x)==0.f ? 255. : swtmapNormalized.at<float>(y, x));
				cv::rectangle(output, cv::Rect(x, y, 1, 1), cv::Scalar(val, val, val), 1, 1, 0);
			}

		if (searchDirection == 1)
		{
			cv::imshow("SWT-map for bright font", output);
			cvMoveWindow("SWT-map for bright font", 0, 0);
			cv::waitKey(0);
			//cv::erode(output, output, cv::Mat());
			//cv::dilate(output, output, cv::Mat());
			//cv::imshow("eroded bright swt", output);
			//cvMoveWindow("eroded bright swt", 0, 0);
			//swtmap = output;
		}
		else
		{
			cv::imshow("SWT-map for dark font", output);
			cvMoveWindow("SWT-map for dark font", 0, 0);
			cv::waitKey(0);
			// cv::erode(output, output, cv::Mat());
			// cv::dilate(output, output, cv::Mat());
			// cv::imshow("eroded dark swt", output);
			// cvMoveWindow("eroded dark swt", 0, 0);
		}
	}
}

cv::Mat DetectText::computeEdgeMap(bool rgbCanny)
{
	cv::Mat edgemap;

//	// edgemap with color segmentation
//	cv::Mat segmentation_(grayImage_.size(), CV_32FC1, cv::Scalar(-1));
//	std::vector<cv::Point3d> meanColorOfSegment;
//	std::vector<int> numberPixelsInSegment;
//
//	int segmentationInitialVal = segmentation_.at<float>(0, 0);
//	int offsetX8[] = {-1, 1, -1, 0, 1, -1, 0, 1};	//{ -2, -1,  0,  1,  2, -2, -1,  0,  1,  2, -2, -1,  1,  2, -2, -1,  0,  1,  2, -2, -1,  0,  1,  2 };
//	int offsetY8[] = {0, 0, -1, -1, -1, 1, 1, 1};	//{ -2, -2, -2, -2, -2, -1, -1, -1, -1, -1,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2 };
//	int nNeighbors = 8;//24;//8;
//	int label = 0;
//
//	int vectorSize = segmentation_.rows * segmentation_.cols;
//
//	std::vector<int> pStack(vectorSize * 2);
//	int stackPointer;
////	std::vector<int> pVector(vectorSize * 2);
////	int vectorPointer;
//
//	int currentPointX;
//	int currentPointY;
//	for (int y = 0; y < segmentation_.rows; y++)
//	{
//		for (int x = 0; x < segmentation_.cols; x++)
//		{
//			bool connected = false;
//			if (segmentation_.at<float>(y, x) == segmentationInitialVal) // if pixel wasn't processed up to now
//			{
////				vectorPointer = 0;
//				stackPointer = 0;
//				pStack[stackPointer] = x;
//				pStack[stackPointer + 1] = y;
//
//				cv::Point3d componentMeanColor(0.,0.,0.);
//				int addedPixels = 0;
//
//				while (stackPointer >= 0)
//				{
//					currentPointX = pStack[stackPointer];
//					currentPointY = pStack[stackPointer + 1];
//					stackPointer -= 2;
//
////					pVector[vectorPointer] = currentPointX;
////					pVector[vectorPointer + 1] = currentPointY;
////					vectorPointer += 2;
//
//					cv::Point3d color_xy((double)originalImage_.at<bgr>(currentPointY, currentPointX).b, (double)originalImage_.at<bgr>(currentPointY, currentPointX).g, (double)originalImage_.at<bgr>(currentPointY, currentPointX).r);
//
//					//check which one of the neighbors have similar color and then label the regions belonging together
//					for (int i = 0; i < nNeighbors; i++)
//					{
//						int ny = currentPointY + offsetY8[i];
//						int nx = currentPointX + offsetX8[i];
//
//						if (ny < 0 || nx < 0 || ny >= segmentation_.rows || nx >= segmentation_.cols)
//							continue;
//
//						if (segmentation_.at<float>(ny, nx) == segmentationInitialVal)
//						{
//							// compute mean color and intensity for pixel (nx,ny)
//							cv::Point3d color((double)originalImage_.at<bgr>(ny, nx).b, (double)originalImage_.at<bgr>(ny, nx).g, (double)originalImage_.at<bgr>(ny, nx).r);
//							if (componentMeanColor.x == 0. && componentMeanColor.y == 0. && componentMeanColor.z == 0.)
//								componentMeanColor = color;
//
//							// do the pixels have similar color?
////							if ((fabs(componentMeanColor.x-color.x) < colorCompareParameter) &&
////									(fabs(componentMeanColor.y-color.y) < colorCompareParameter) && (fabs(componentMeanColor.z-color.z) < colorCompareParameter))
//							if (((componentMeanColor.x-color.x)*(componentMeanColor.x-color.x)+(componentMeanColor.y-color.y)*(componentMeanColor.y-color.y)+
//									(componentMeanColor.z-color.z)*(componentMeanColor.z-color.z) < colorCompareParameter*colorCompareParameter) &&
//									((color_xy.x-color.x)*(color_xy.x-color.x)+(color_xy.y-color.y)*(color_xy.y-color.y)+
//									(color_xy.z-color.z)*(color_xy.z-color.z) < 0.5*0.5*colorCompareParameter*colorCompareParameter))
//							{
//								segmentation_.at<float>(ny, nx) = label;
//								stackPointer += 2;
//								pStack[stackPointer] = nx;
//								pStack[stackPointer + 1] = ny;
//								connected = true;
//
//								// update mean intensity and color
//								componentMeanColor = ((componentMeanColor*addedPixels) + color) * (1./(addedPixels+1.));
//								addedPixels++;
//							}
//						}
//					}// loop through neighbors
//				}
//
//				if (connected)
//				{
//					segmentation_.at<float>(y, x) = label;
//					meanColorOfSegment.push_back(componentMeanColor);
//					numberPixelsInSegment.push_back(addedPixels);
//					label++;
//				}
//				else
//					// pixel had no neighbor with similar color
//					segmentation_.at<float>(y, x) = -2;
//			}
//		}// loop through segmentation
//	}
//	// re-assign 1-pixel sized segments
//	for (int v=0; v<segmentation_.rows; v++)
//	{
//		for (int u=0; u<segmentation_.cols; u++)
//		{
//			if (segmentation_.at<float>(v,u)==-2.f)
//			{
//				std::map<float, int> labelCount;
//				for (int i = 0; i < nNeighbors; i++)
//				{
//					int nv = v + offsetY8[i];
//					int nu = u + offsetX8[i];
//
//					if (nv < 0 || nu < 0 || nv >= segmentation_.rows || nu >= segmentation_.cols)
//						continue;
//
//					float neighborValue = segmentation_.at<float>(nv,nu);
//					if (neighborValue!=-2.f)
//					{
//						if (labelCount.find(neighborValue)==labelCount.end())
//							labelCount[neighborValue] = 1;
//						else
//							labelCount[neighborValue]++;
//					}
//				}
//				float maxLabel = -2;
//				int maxCount = 0;
//				for (std::map<float, int>::iterator it=labelCount.begin(); it!=labelCount.end(); it++)
//				{
//					if (it->second > maxCount)
//					{
//						maxCount = it->second;
//						maxLabel = it->first;
//					}
//				}
//				segmentation_.at<float>(v,u) = maxLabel;
//			}
//		}
//	}
//	if (debug["showEdge"] == true)
//	{
//		std::cout << "label=" << label << std::endl;
//		cv::Mat segmentation_copy;
//		cv::normalize(segmentation_, segmentation_copy, 0, 1, cv::NORM_MINMAX);
//		cv::imshow("segmentation", segmentation_copy);
//	}
//	// find segment borders
//	edgemap = cv::Mat(grayImage_.size(), CV_8UC1, cv::Scalar(0));
//	for (int v=0; v<segmentation_.rows; v++)
//	{
//		for (int u=0; u<segmentation_.cols; u++)
//		{
//			for (int i = 0; i < nNeighbors; i++)
//			{
//				int nv = v + offsetY8[i];
//				int nu = u + offsetX8[i];
//
//				if (nv < 0 || nu < 0 || nv >= segmentation_.rows || nu >= segmentation_.cols)
//					continue;
//
//				if (numberPixelsInSegment[segmentation_.at<float>(v,u)]>numberPixelsInSegment[segmentation_.at<float>(nv,nu)]
//						/*&& segmentation.at<float>(v,u)!=-2.f && segmentation.at<float>(nv,nu)!=-2.f*/)
//					edgemap.at<uchar>(v,u) = 255;
//			}
//		}
//	}
//	if (debug["showEdge"] == true)
//	{
//		cv::imshow("color segmentation edgemap", edgemap);
//		//cvMoveWindow("color segmentation edgemap", 0, 0);
//	}
//
//	// compute gradients
//	cv::Mat color_segmentation(segmentation_.size(), CV_8UC3);
//	for (int v=0; v<segmentation_.rows; v++)
//	{
//		for (int u=0; u<segmentation_.cols; u++)
//		{
//			cv::Point3d& color = meanColorOfSegment[segmentation_.at<float>(v,u)];
//			color_segmentation.at<cv::Vec3b>(v,u) = cv::Vec3b((uchar)color.x, (uchar)color.y, (uchar)color.z);
//		}
//	}
//	cv::Mat gray_segmentation;
//	cv::cvtColor(color_segmentation, gray_segmentation, CV_BGR2GRAY);
//	if (debug["showEdge"] == true)
//	{
//		cv::imshow("segmentation gray scale", gray_segmentation);
//		cv::imshow("segmentation original colors", color_segmentation);
//	}
//	Sobel(gray_segmentation, dx_, CV_32FC1, 1, 0, 3);
//	Sobel(gray_segmentation, dy_, CV_32FC1, 0, 1, 3);



//	// edgemap with toggle-mapping
//	int minimalContrast = 16;
//	double p = 0.8;
//	cv::Mat dilation, erosion, segmentation(grayImage_.rows, grayImage_.cols, CV_8UC1);
//	cv::dilate(grayImage_, dilation, cv::Mat());
//	cv::erode(grayImage_, erosion, cv::Mat());
//
//	for (int v=0; v<grayImage_.rows; v++)
//	{
//		for (int u=0; u<grayImage_.cols; u++)
//		{
//			int contrast = std::abs<int>((int)erosion.at<uchar>(v,u)-(int)dilation.at<uchar>(v,u));
//			if (contrast < minimalContrast)
//				segmentation.at<uchar>(v,u) = 0;
//			else
//			{
//				if ((double)std::abs<int>((int)erosion.at<uchar>(v,u)-(int)grayImage_.at<uchar>(v,u)) < p*(double)std::abs<int>((int)dilation.at<uchar>(v,u)-(int)grayImage_.at<uchar>(v,u)))
//					segmentation.at<uchar>(v,u) = 128;
//				else
//					segmentation.at<uchar>(v,u) = 255;
//			}
//		}
//	}
//
//	cv::imshow("segmentation before", segmentation);
//	cv::waitKey();
//
////	// fill black regions inside white regions
////	for (int v=1; v<segmentation.rows-1; v++)
////	{
////		for (int u=1; u<segmentation.cols-1; u++)
////		{
////			if (segmentation.at<uchar>(v,u) == 0)
////			{
////				// check neighborhood for white pixel
////				bool foundWhitePixel = false;
////				for (int kv=-1; foundWhitePixel==false && kv<=1; kv++)
////					for (int ku=-1; foundWhitePixel==false && ku<=1; ku++)
////						if (segmentation.at<uchar>(v+kv,u+ku) == 255)
////							foundWhitePixel = true;
////				if (foundWhitePixel == true)
////					segmentation.at<uchar>(v,u) = 255;
////			}
////		}
////	}
//
////	std::vector< std::vector<cv::Point> > contours;
////	cv::Mat segmentation_copy = segmentation.clone();
////	for (int v=0; v<segmentation_copy.rows; v++)
////	{
////		for (int u=0; u<segmentation_copy.cols; u++)
////		{
////			if (segmentation_copy.at<uchar>(v,u) != 0)
////				segmentation_copy.at<uchar>(v,u) = 1;
////		}
////	}
////	cv::Mat segmentation_copy2 = 255*segmentation_copy.clone();
////	cv::imshow("segmentation_copy", segmentation_copy2);
////	cv::waitKey();
////	cv::findContours(segmentation_copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
////	cv::drawContours(segmentation_copy2, contours, -1, cv::Scalar(128), 1);
////	cv::imshow("segmentation_copy2", segmentation_copy2);
////	cv::waitKey();
//
//	std::vector< std::vector<cv::Point> > contours;
//	std::vector<cv::Vec4i> hierarchy;
//	cv::Mat segmentation_copy = segmentation.clone();
//	cv::findContours(segmentation_copy, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
//
//	for (unsigned int i=0; i<contours.size(); i++)
//	{
//		// only for inner components
//		if (hierarchy[i][3]!=-1)
//		{
//			int colorCounter128 = 0, colorCounter255 = 0;
//			for (unsigned int j=0; j<contours[i].size(); j++)
//			{
//				if (segmentation.at<uchar>(contours[i][j]) == 128)
//					colorCounter128++;
//				else if (segmentation.at<uchar>(contours[i][j]) == 255)
//					colorCounter255++;
//			}
//			if (colorCounter128 > colorCounter255)
//				cv::drawContours(segmentation, contours, i, cv::Scalar(128), CV_FILLED, 8, hierarchy, 2);
//			else
//				cv::drawContours(segmentation, contours, i, cv::Scalar(255), CV_FILLED, 8, hierarchy, 2);
//		}
////		if (hierarchy[i][3]!=-1)
////			cv::drawContours(segmentation, contours, i, cv::Scalar(64), 1, 8, hierarchy);
//
//		cv::imshow("segmentation", segmentation);
//		cv::waitKey();
//	}
//
//
//	cv::Canny(segmentation, edgemap, cannyThreshold2, cannyThreshold1);
//
//	return edgemap;



	// ====== edgemap with canny edge ======

	//  cannyThreshold1 = 120; // default: 120
	//  cannyThreshold2 = 50; // default: 50 , cannyThreshold1 > cannyThreshold2

	// cv::blur(grayImage_, edgemap, cv::Size(3, 3));
//	cv::Mat temp;
//	cv::adaptiveThreshold(grayImage_, temp, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 31, 0);
//	cv::imshow("adaptive", temp);
//	cv::waitKey();
//	cv::Canny(temp, edgemap, cannyThreshold2, cannyThreshold1);
	cv::Canny(grayImage_, edgemap, cannyThreshold2, cannyThreshold1);

	if (debug["showEdge"] == true)
	{
		cv::imshow("gray color edgemap", edgemap);
		cvMoveWindow("gray color edgemap", 0, 0);
	}

	if (rgbCanny)
	{
		cv::Mat redImage = cv::Mat(grayImage_.size(), CV_8UC1), blueImage = cv::Mat(grayImage_.size(), CV_8UC1), greenImage = cv::Mat(grayImage_.size(), CV_8UC1);

		// get single color image
		for (int y = 0; y < grayImage_.rows; y++)
			for (int x = 0; x < grayImage_.cols; x++)
			{
				redImage.at<unsigned char> (y, x) = originalImage_.at<bgr> (y, x).r;
				blueImage.at<unsigned char> (y, x) = originalImage_.at<bgr> (y, x).b;
				greenImage.at<unsigned char> (y, x) = originalImage_.at<bgr> (y, x).g;
			}

		// compute canny edge for every color separately
		cv::Mat edgemapG, edgemapR, edgemapB;
		Canny(redImage, edgemapR, cannyThreshold2, cannyThreshold1);
		Canny(greenImage, edgemapG, cannyThreshold2, cannyThreshold1);
		Canny(blueImage, edgemapB, cannyThreshold2, cannyThreshold1);

		int offsetY[] =
		{ -1, -1, -1, 0, 0, 1, 1, 1 };
		int offsetX[] =
		{ -1, 0, 1, -1, 1, -1, 0, 1 };
		std::vector<cv::Point> newEdgePoints;

		for (int rgb = 0; rgb < 3; rgb++)
		{
			cv::Mat actualColorEdgemap;
			switch (rgb)
			{
			case 0:
				actualColorEdgemap = edgemapR;
				break;
			case 1:
				actualColorEdgemap = edgemapG;
				break;
			case 2:
				actualColorEdgemap = edgemapB;
				break;
			}

			for (int y = 0; y < actualColorEdgemap.rows; y++)
				for (int x = 0; x < actualColorEdgemap.cols; x++)
				{
					bool goodEdge = true;
					if (edgemap.at<unsigned char>(y, x) != 255 && actualColorEdgemap.at<unsigned char>(y, x) == 255) // if there is no edge at the gray edgemap but an edge at the single color edgemap..
					{
						// check all 8 neighbor pixels in gray edgemap for an edge pixel. If there is an edge -> reject.
						// (there shall be no edges wider than 1 pixel, so no edge is allowed among the neighbors)
						for (int n = 0; n < 8; n++)
							if (edgemap.at<unsigned char> (y + offsetY[n], x + offsetX[n]) == 255)
							{
								goodEdge = false;
								break;
							}

						if (goodEdge)
							newEdgePoints.push_back(cv::Point(x, y));
					}
				}

			// add edges found in color image to gray edgemap
			for (unsigned int i = 0; i < newEdgePoints.size(); i++)
				edgemap.at<unsigned char>(newEdgePoints[i].y, newEdgePoints[i].x) = 255;
			newEdgePoints.clear();
		}

		if (debug["showEdge"] == true)
		{
			cv::imshow("color edgemap", edgemap);
			cvMoveWindow("color edgemap", 0, 0);
		}

	}


	if (debug["showEdge"] == true)
		cv::waitKey(0);
	return edgemap;
}

void DetectText::updateStrokeWidth(cv::Mat& swtmap, std::vector<cv::Point>& startPoints, std::vector<cv::Point>& strokePoints, int searchDirection, Purpose purpose)
{
	std::vector<cv::Point> pointStack;
	std::vector<float> SwtValues;
	std::multimap<double, cv::Point> strokePointsOrderedByLength;

	int offsetX5[] = {0, -1, 1, 0, 0};
	int offsetY5[] = {0, 0, 0, -1, 1};
	int offsetX9[] = {0, -1, 1, -1, 0, 1, -1, 0, 1};
	int offsetY9[] = {0, 0, 0, -1, -1, -1, 1, 1, 1};
	double tanCompareGradientParameter = tan(compareGradientParameter_);

	// Loop through all edgepoints, compute stroke width
//	cv::Mat outputtemp = originalImage_.clone();
	for (std::vector<cv::Point>::iterator itr = startPoints.begin(); itr != startPoints.end(); ++itr)
	{
		for (int mode=0; mode<3; mode++)
		{
			pointStack.clear();
			SwtValues.clear();
			float step = 1;
			float ix = (*itr).x;
			float iy = (*itr).y;
			float currX = ix;
			float currY = iy;
			bool isStroke = false;
			float iTheta = theta_.at<float>(*itr);
			double ciTheta = cos(iTheta);
			double siTheta = sin(iTheta);
			if (mode==1)
			{
				ciTheta = cos(iTheta) - sin(iTheta);
				siTheta = cos(iTheta) + sin(iTheta);
			}
			else if (mode==2)
			{
				ciTheta = cos(iTheta) + sin(iTheta);
				siTheta = -cos(iTheta) + sin(iTheta);
			}
			double adx = std::abs(ciTheta);
			double ady = std::abs(siTheta);
			double sx = ciTheta > 0 ? searchDirection : -searchDirection;
			double sy = siTheta > 0 ? searchDirection : -searchDirection;
			double err = adx - ady, e2 = 0.;

			pointStack.push_back(cv::Point(currX, currY));
			SwtValues.push_back(swtmap.at<float>(currY, currX));

			while (step < maxStrokeWidth_)
			{
				//going one pixel in the direction of the gradient to check if next pixel is also an edge
//				float nextX = round(ix + ciTheta * searchDirection * step);
//				float nextY = round(iy + siTheta * searchDirection * step);
				// Bresenham
				double nextX = currX;
				double nextY = currY;
				e2 = 2. * err;
				if (e2 > -ady)
				{
					err -= ady;
					nextX += sx;
				}
				if (e2 < adx)
				{
					err += adx;
					nextY += sy;
				}


				if (nextX < 1 || nextY < 1 || nextX >= edgemap_.cols-1 || nextY >= edgemap_.rows-1)
					break;

				step++;

				if (currX == nextX && currY == nextY)
					continue;

				currX = nextX;
				currY = nextY;

				pointStack.push_back(cv::Point(currX, currY));
				SwtValues.push_back(swtmap.at<float>(currY, currX));

				// if the next neighbor of the start pixel on the search line is also an edge pixel, break to avoid connection of un-connected regions
				if (fabs(currX-ix)<2.f && fabs(currY-iy)<2.f && edgemap_.at<unsigned char>(currY, currX)==255)
					break;

				bool foundEdgePoint = false;
				// search in 5-neighborhood for suitable counter edge points
				// todo: only use 3 neighborhood perpendicular to search direction
				int edgepointX = 0;
				int edgepointY = 0;
				if (fabs(currX-ix)>=2.f || fabs(currY-iy)>=2.f)
				{
					for (int k=0; k<5; k++)
					{
						edgepointX = currX+offsetX5[k];
						edgepointY = currY+offsetY5[k];
						if (edgemap_.at<unsigned char>(edgepointY, edgepointX) == 255)
						//if (edgemap_.at<unsigned char>(currY, currX) == 255)
						{
							foundEdgePoint = true;
							break;
						}
					}
				}

				if (foundEdgePoint == true && edgepointX>0 && edgepointY>0 && edgepointX<edgemap_.cols-1 && edgepointY<edgemap_.rows-1)
				{
					for (int k=0; k<9; k++)
					{
					//	float jTheta = theta_.at<float>(currY+offsetY9[k], currX+offsetX9[k]);
						//if opposite point of stroke with roughly opposite gradient is found...
						//double dTheta = abs(iTheta - jTheta);std::min(dTheta, 3.14159265359-dTheta)
						//std::cout << iTheta << " " << jTheta << " " << fmod(jTheta+M_PI,2*M_PI) << " " << fabs(iTheta-fmod(jTheta+M_PI,2*M_PI)) << std::endl;
						//getchar();
						//if (fabs(iTheta-fmod(jTheta+M_PI,2*M_PI)) < compareGradientParameter_) // paper: abs(abs(iTheta - jTheta) - 3.14) < 3.14 / 6
	//					double t = tan(iTheta-jTheta);
	//					if (-1 / sqrt(3) < t && t < 1 / sqrt(3))
						double tn = dy_.at<float>(iy,ix)*dx_.at<float>(edgepointY+offsetY9[k],edgepointX+offsetX9[k]) - dx_.at<float>(iy,ix)*dy_.at<float>(edgepointY+offsetY9[k],edgepointX+offsetX9[k]);
						double td = dx_.at<float>(iy,ix)*dx_.at<float>(edgepointY+offsetY9[k],edgepointX+offsetX9[k]) + dy_.at<float>(iy,ix)*dy_.at<float>(edgepointY+offsetY9[k],edgepointX+offsetX9[k]);
//						if (tn*7 < -td*4/*tan(compareGradientParameter_)*/ && tn*7 > td*4/*tan(compareGradientParameter_)*/)
						if (tn < -td*tanCompareGradientParameter && tn > td*tanCompareGradientParameter)
						{
							isStroke = true;
	//						if (purpose == UPDATE)
	//							strokePoints.push_back(cv::Point(ix, iy));
							break;
						}
					}
				}
				if (foundEdgePoint == true)
					break;
			}

			// ... then calculate newSwtVal for all cv::Points between the two stroke points
			if (isStroke)
			{
	//			for (int i = 0; i < pointStack.size(); i++)
	//				cv::rectangle(outputtemp, cv::Rect(pointStack[i].x, pointStack[i].y, 1, 1), cv::Scalar(0, 255, 0), 1, 1, 0);

				float newSwtVal;
				if (purpose == UPDATE)// update swt based on dist between edges
				{
					newSwtVal = (sqrt((currY - iy) * (currY - iy) + (currX - ix) * (currX - ix)) + 0.5);
					strokePointsOrderedByLength.insert(std::pair<double, cv::Point>(newSwtVal, cv::Point(ix, iy)));
					for (size_t i = 0; i < pointStack.size(); i++)
						swtmap.at<float>(pointStack[i]) = std::min(swtmap.at<float>(pointStack[i]), newSwtVal);
				}
				else if (purpose == REFINE) // refine swt based on median
				{
					nth_element(SwtValues.begin(), SwtValues.begin() + SwtValues.size() / 2, SwtValues.end());
					newSwtVal = SwtValues[SwtValues.size() / 2];
					for (size_t i = 0; i < pointStack.size(); i++)
						swtmap.at<float>(pointStack[i]) = newSwtVal;
				}

				// set all cv::Points between to the newSwtVal except they are smaller because of another stroke
//				for (size_t i = 0; i < pointStack.size(); i++)
//					swtmap.at<float>(pointStack[i]) = std::min(swtmap.at<float>(pointStack[i]), newSwtVal);
			}
	//		else
	//		{
	//			for (int i = 0; i < pointStack.size(); i++)
	//				cv::rectangle(outputtemp, cv::Rect(pointStack[i].x, pointStack[i].y, 1, 1), cv::Scalar(0, 0, 255), 1, 1, 0);
	//		}
		}
	} // end loop through edge points
//	cv::imshow("active stroke", outputtemp);

	// write the edge points for the refinement step ordered from short to long strokes
	if (purpose == UPDATE)
	{
		for (std::multimap<double, cv::Point>::iterator it = strokePointsOrderedByLength.end(); it != strokePointsOrderedByLength.begin(); )//it++)
		{
			--it;
			strokePoints.push_back(it->second);
		}

		// set initial values back to 0
		for (int y = 0; y < swtmap.rows; y++)
			for (int x = 0; x < swtmap.cols; x++)
				if (swtmap.at<float> (y, x) == initialStrokeWidth_)
					swtmap.at<float> (y, x) = 0;						// todo: check whether this may cause problems when grouping
	}

}

void DetectText::closeOutline(cv::Mat& edgemap)
{
	cv::Mat temp = cv::Mat::zeros(edgemap.size(), edgemap.type());

	for (int y=0; y<edgemap.rows-1; y++)
	{
		for (int x=0; x<edgemap.cols-1; x++)
		{
			if (temp.at<unsigned char>(y, x)==0)
				temp.at<unsigned char>(y, x) = edgemap.at<unsigned char>(y, x);
			if (edgemap.at<unsigned char>(y, x)==255 && edgemap.at<unsigned char>(y+1, x+1)==255)
			{
				temp.at<unsigned char>(y, x+1) = 255;
				temp.at<unsigned char>(y+1, x) = 255;
			}
			if (edgemap.at<unsigned char>(y+1, x)==255 && edgemap.at<unsigned char>(y, x+1)==255)
			{
				temp.at<unsigned char>(y, x) = 255;
				temp.at<unsigned char>(y+1, x+1) = 255;
			}
		}
		int x = edgemap.cols-1;
		if (temp.at<unsigned char>(y, x)==0)
			temp.at<unsigned char>(y, x) = edgemap.at<unsigned char>(y, x);
	}
	int y = edgemap.rows-1;
	for (int x=0; x<edgemap.cols; x++)
		if (temp.at<unsigned char>(y, x)==0)
			temp.at<unsigned char>(y, x) = edgemap.at<unsigned char>(y, x);

	edgemap = temp;
}

int DetectText::connectComponentAnalysis(const cv::Mat& swtmap, cv::Mat& ccmap)
{
	// Check all 8 neighbor pixels of each pixel for similar stroke width and for similar color, then form components with enumerative labels

	int ccmapInitialVal = ccmap.at<float>(0, 0);
	int offsetX8[] = {-1, 1, -1, 0, 1, -1, 0, 1};
	//{ -2, -1,  0,  1,  2, -2, -1,  0,  1,  2, -2, -1,  1,  2, -2, -1,  0,  1,  2, -2, -1,  0,  1,  2 };
	int offsetY8[] = {0, 0, -1, -1, -1, 1, 1, 1};
	//{ -2, -2, -2, -2, -2, -1, -1, -1, -1, -1,  0,  0,  0,  0,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2 };
	int nNeighbors = 8;//24;//8;
	int label = 0;

	int vectorSize = ccmap.rows * ccmap.cols;

	std::vector<int> pStack(vectorSize * 2);
	int stackPointer;

	std::vector<int> pVector(vectorSize * 2);
	int vectorPointer;

	int currentPointX;
	int currentPointY;

	for (int y = 0; y < ccmap.rows; y++)
	{
		for (int x = 0; x < ccmap.cols; x++)
		{
			bool connected = false;
			if (ccmap.at<float>(y, x) == ccmapInitialVal) // if pixel wasn't processed up to now
			{
				vectorPointer = 0;
				stackPointer = 0;
				pStack[stackPointer] = x;
				pStack[stackPointer + 1] = y;

				double componentMeanIntensity = 0.;
				cv::Point3d componentMeanColor(0.,0.,0.);
				int addedPixels = 0;

				while (stackPointer >= 0)
				{
					currentPointX = pStack[stackPointer];
					currentPointY = pStack[stackPointer + 1];
					stackPointer -= 2;

					pVector[vectorPointer] = currentPointX;
					pVector[vectorPointer + 1] = currentPointY;
					vectorPointer += 2;

					//check which one of the neighbors have similar sw and then label the regions belonging together
					float meanStrokeWidth = swtmap.at<float>(currentPointY, currentPointX);
					int addedNeighbors = 1;
					for (int i = 0; i < nNeighbors; i++)
					{
						int ny = currentPointY + offsetY8[i];
						int nx = currentPointX + offsetX8[i];

						if (ny < 0 || nx < 0 || ny >= ccmap.rows || nx >= ccmap.cols)
							continue;

						// swtmap == 0 -> not part of any component
						if (swtmap.at<float>(ny, nx) == 0)
						{
							ccmap.at<float>(ny, nx) = -2;
							continue;
						}

						if (ccmap.at<float>(ny, nx) == ccmapInitialVal)
						{
							float sw1 = swtmap.at<float>(ny, nx);
							//float meanStrokeWidth = swtmap.at<float>(y, x);

							// compute mean color and intensity for pixel (nx,ny)
							double intensity = (double)grayImage_.at<unsigned char>(ny, nx);
							cv::Point3d color((double)originalImage_.at<bgr>(ny, nx).b, (double)originalImage_.at<bgr>(ny, nx).g, (double)originalImage_.at<bgr>(ny, nx).r);
							for (int i = 0; i < nNeighbors; i++) //check all neighbors of actual neighbor for their color
							{
								int my = ny + offsetY8[i];
								int mx = nx + offsetX8[i];
								if (my < 0 || mx < 0 || my >= ccmap.rows || mx >= ccmap.cols)
									continue;
								intensity += (double)grayImage_.at<unsigned char>(my, mx);
								color += cv::Point3d((double)originalImage_.at<bgr>(my, mx).b, (double)originalImage_.at<bgr>(my, mx).g, (double)originalImage_.at<bgr>(my, mx).r);
							}
							intensity /= (nNeighbors + 1.);
							color *= 1./(nNeighbors + 1.);
							if (componentMeanIntensity == 0.)
							{
								componentMeanIntensity = intensity;
								componentMeanColor = color;
							}

							// do the pixels have similar stroke width?
							if (std::max(sw1, meanStrokeWidth) <= swCompareParameter * std::min(sw1, meanStrokeWidth) ||		// todo: ratio between a mean value over the component and sw1 better?
									(std::max(sw1, meanStrokeWidth) <= 1.5*swCompareParameter * std::min(sw1, meanStrokeWidth) && (fabs(componentMeanIntensity-intensity) < colorCompareParameter) && (fabs(componentMeanColor.x-color.x) < colorCompareParameter) &&
									(fabs(componentMeanColor.y-color.y) < colorCompareParameter) && (fabs(componentMeanColor.z-color.z) < colorCompareParameter)))
							{
//								if (processing_method_ == ORIGINAL_EPSHTEIN)
//								{
									ccmap.at<float>(ny, nx) = label;
									stackPointer += 2;
									pStack[stackPointer] = nx;
									pStack[stackPointer + 1] = ny;
									connected = true;

									// update mean stroke width
									meanStrokeWidth = (meanStrokeWidth*addedNeighbors + sw1)/(float)(addedNeighbors+1);
									addedNeighbors++;

//									// update mean intensity and color
//									componentMeanIntensity = ((componentMeanIntensity*addedPixels) + intensity) / (addedPixels+1.);
//									componentMeanColor = ((componentMeanColor*addedPixels) + color) * (1./(addedPixels+1.));
//									addedPixels++;

//								}
//								else
//								{
//									float meanRed = originalImage_.at<bgr>(ny, nx).r;
//									float meanGreen = originalImage_.at<bgr>(ny, nx).g;
//									float meanBlue = originalImage_.at<bgr>(ny, nx).b;
//									float meanclr = 0;
//									for (int i = 0; i < nNeighbors; i++) //check all neighbors of actual neighbor for their color
//									{
//										int my = ny + offsetY8[i];
//										int mx = nx + offsetX8[i];
//										if (my < 0 || mx < 0 || my >= ccmap.rows || mx >= ccmap.cols)
//											continue;
//										bgr neighborClrs = originalImage_.at<bgr>(my, mx);
//										meanRed += neighborClrs.r;
//										meanGreen += neighborClrs.g;
//										meanBlue += neighborClrs.b;
//										meanclr += static_cast<float>(grayImage_.at<unsigned char>(my, mx));
//									}
//									meanRed /= (nNeighbors + 1);
//									meanGreen /= (nNeighbors + 1);
//									meanBlue /= (nNeighbors + 1);
//									meanclr /= nNeighbors;
//
//									// do the pixels have similiar color?
//									if (abs((int)originalImage_.at<bgr>(y, x).g - (int)meanGreen) < colorCompareParameter && abs((int)originalImage_.at<bgr> (y, x).r - (int)meanRed)
//											< colorCompareParameter && abs((int)originalImage_.at<bgr>(y, x).b - (int)meanBlue) < colorCompareParameter)
//									{
//										ccmap.at<float>(ny, nx) = label;
//										stackPointer += 2;
//										pStack[stackPointer] = nx;
//										pStack[stackPointer + 1] = ny;
//										connected = true;
//									}
//								}
							}
						}
					}// loop through neighbors
				}

				if (connected)
				{
					ccmap.at<float>(y, x) = label;
					int minY = ccmap.rows, minX = ccmap.cols, maxY = 0, maxX = 0;
					int width, height;
					for (int i = 0; i < vectorPointer; i += 2)
					{
						// ROI for each component
						minY = std::min(minY, pVector[i + 1]);
						minX = std::min(minX, pVector[i]);
						maxY = std::max(maxY, pVector[i + 1]);
						maxX = std::max(maxX, pVector[i]);
					}
					width = maxX - minX + 1;
					height = maxY - minY + 1;
					cv::Rect letterRoi(minX, minY, width, height);
					labeledRegions_.push_back(letterRoi);
					label++;
				}
				else
					// pixel had no neighbor with similar color and sw
					ccmap.at<float>(y, x) = -2;
			}
		}// loop through ccmap
	}
	return label;
}

void DetectText::identifyLetters(const cv::Mat& swtmap, const cv::Mat& ccmap)
{
	// todo: parameter
	if (processing_method_ == ORIGINAL_EPSHTEIN)
		minLetterHeight_ = 10;//8;
	else
		minLetterHeight_ = std::max(10, 3 + (originalImage_.rows)/480); //default: 10

	assert(static_cast<size_t>(nComponent_) == labeledRegions_.size());
	isLetterRegion_.clear();
	isLetterRegion_.resize(nComponent_, false);
	medianStrokeWidth_.clear();
	medianStrokeWidth_.resize(nComponent_, -1.f);

	meanRGB_ = std::vector< std::vector<float> >(nComponent_, std::vector<float>(4)); // Foreground (letter component pixels) mean color: r g b gray

	meanBgRGB_ = std::vector< std::vector<float> >(nComponent_, std::vector<float>(4)); // Background (remaining pixels) mean color: r g b gray

	nLetter_ = 0;

	// For every found component
	for (size_t i = 0; i < nComponent_; i++)
	{
		//std::vector<bool> innerComponents(nComponent_, false);
		isLetterRegion_[i] = false;
		float maxStrokeWidth = 0;
		double sumStrokeWidth = 0;
		float currentStrokeWidth;
		bool isLetter = true;

		cv::Rect itr = labeledRegions_[i];

		// rule #1: height of component [not used atm]
		// rotated text leads to problems. 90° rotated 'l' may only be 1 pixel high.
		// nevertheless it might be convenient to implement another rule like rule #1 to check for size
		if ((processing_method_==ORIGINAL_EPSHTEIN) && (itr.height > maxLetterHeight_ || itr.height < minLetterHeight_ || itr.area() < 75/*38/*50*/))
			continue;

		// rule #2: aspect ratio has to be within 0.1 and 10
		if (processing_method_ == ORIGINAL_EPSHTEIN)
		{
			double aspectRatio = (double)std::max(itr.height, itr.width)/(double)std::min(itr.height, itr.width);
			isLetter = isLetter && (aspectRatio <= 7.0);
		}

		float maxY = itr.y + itr.height;
		float minY = itr.y;
		float maxX = itr.x + itr.width;
		float minX = itr.x;

		double m10=0., m01=0., m20=0., m11=0., m02=0.;

		// compute mean and variance of stroke width
		std::vector<float> iComponentStrokeWidth;
		for (int y = minY; y < maxY; y++)
		{
			for (int x = minX; x < maxX; x++)
			{
				int component = static_cast<int>(ccmap.at<float>(y, x)); // ccmap-Label = -2 in case no Region; 0,1,2,3... for every region
				if (component == static_cast<int>(i))
				{
					m10 += x;
					m01 += y;
					m20 += x*x;
					m11 += x*y;
					m02 += y*y;
					currentStrokeWidth = swtmap.at<float>(y, x);
					iComponentStrokeWidth.push_back(currentStrokeWidth);
					maxStrokeWidth = std::max(maxStrokeWidth, currentStrokeWidth);
					sumStrokeWidth += currentStrokeWidth;
				}
//				else if (component >= 0)
//					innerComponents[component] = true;
			}
		}
		double pixelCount = static_cast<double>(iComponentStrokeWidth.size());

		double xc = m10/pixelCount;
		double yc = m01/pixelCount;
		double af = m20/pixelCount - xc*xc;
		double bf = 2*m11/pixelCount - xc*yc;
		double cf = m02/pixelCount - yc*yc;
		double delta = sqrt(bf * bf + (af - cf) * (af - cf));
		double momentsRatio = sqrt(std::max(af + cf + delta, af + cf - delta) / std::min(af + cf + delta, af + cf - delta));
		if (momentsRatio > 8.0)
			continue;

		// rule #2: remove components that are too small/thin		// todo: reactivate
		if (pixelCount < 0.1*itr.area())
			continue;

		double meanStrokeWidth = sumStrokeWidth / pixelCount;
		double varianceStrokeWidth = 0;
		for (size_t ii = 0; ii < iComponentStrokeWidth.size(); ii++)
			varianceStrokeWidth += (iComponentStrokeWidth[ii] - meanStrokeWidth) * (iComponentStrokeWidth[ii] - meanStrokeWidth);
		varianceStrokeWidth = varianceStrokeWidth / pixelCount;

		// rule #2: variance of stroke width of pixels in region that are part of component
		isLetter = isLetter && (std::sqrt(varianceStrokeWidth) <= varianceParameter*meanStrokeWidth);

		// rule #3: diagonal of rect must be smaller than x*medianStrokeWidth     // paper: medianStrokeWidth , original text_detect: maxStrokeWidth
		// std::sort(iComponentStrokeWidth.begin(), iComponentStrokeWidth.end());
		// unsigned int medianStrokeWidth = iComponentStrokeWidth[iComponentStrokeWidth.size() / 2];
		//isLetter = isLetter && (sqrt(((itr.width) * (itr.width) + (itr.height) * (itr.height))) < maxStrokeWidth * diagonalParameter);
		std::nth_element(iComponentStrokeWidth.begin(), iComponentStrokeWidth.begin()+iComponentStrokeWidth.size()/2, iComponentStrokeWidth.end());
		medianStrokeWidth_[i] = *(iComponentStrokeWidth.begin()+iComponentStrokeWidth.size()/2);
		isLetter = isLetter && (pixelCount > 0.2*itr.area() || sqrt((double)(itr.width)*(itr.width) + (itr.height)*(itr.height)) > medianStrokeWidth_[i] * diagonalParameter);		// todo: reactivate

		// rule #4: pixelCount has to be bigger than maxStrokeWidth * x:
		if (processing_method_==BORMANN)
			isLetter = isLetter && (pixelCount > maxStrokeWidth * pixelCountParameter);

		// rule #5: width has to be smaller than x * height (x>1)
		//  isLetter = isLetter && (itr.width < heightParameter * itr.height);

		// rule #6: number of inner components must be small
		// attention: does not make sense at this place! -> this rule is checked after this loop
		//isLetter = isLetter && (countInnerLetterCandidates(innerComponents) <= innerLetterCandidatesParameter);

		// rule #7: Ratio of background color / foreground color has to be big.
		meanRGB_[i] = getMeanIntensity(ccmap, itr, static_cast<int>(i), false);
		meanBgRGB_[i] = getMeanIntensity(ccmap, itr, static_cast<int>(i), true);
		if (processing_method_==BORMANN && isLetter)
		{
			if (itr.area() > 200) // too small areas have bigger color difference
				if ((std::abs(meanRGB_[i][0] - meanBgRGB_[i][0]) < clrComponentParameter) ||
						(std::abs(meanRGB_[i][1]-meanBgRGB_[i][1]) < clrComponentParameter) || (std::abs(meanRGB_[i][2]-meanBgRGB_[i][2]) < clrComponentParameter))
					if ((std::abs(meanRGB_[i][0]-meanBgRGB_[i][0])) + (std::abs(meanRGB_[i][1]-meanBgRGB_[i][1])) + (std::abs(meanRGB_[i][2]-meanBgRGB_[i][2])) < clrComponentParameter * 4)
						isLetter = false;
		}

		// rule #8 fg gray has to correspond to actual font color
		//    if (isLetter)
		//      if (firstPass_)
		//        if (meanRGB_[i][3] < 75 || meanBgRGB_[i][3] > 175)
		//          isLetter = false;
		//        else if (meanRGB_[i][3] > 175 || meanBgRGB_[i][3] < 75)
		//          isLetter = false;

		isLetterRegion_[i] = isLetter;
		if (isLetter)
			nLetter_++;
	}

	// rule #6: number of inner components must be small
	for (unsigned int i=0; i<nComponent_; i++)
	{
		if (isLetterRegion_[i] == false)
			continue;

		std::vector<bool> innerComponents(nComponent_, false);
		// option a: comparison at pixel level
		int minX = labeledRegions_[i].x;
		int maxX = labeledRegions_[i].x+labeledRegions_[i].width;
		int minY = labeledRegions_[i].y;
		int maxY = labeledRegions_[i].y+labeledRegions_[i].height;
		for (int y = minY; y < maxY; y++)
		{
			for (int x = minX; x < maxX; x++)
			{
				int component = static_cast<int>(ccmap.at<float>(y, x)); //ccmap-Label = -2 in case no Region; 0,1,2,3... for every region
				if (component != -2 && component != (int)(i) && isLetterRegion_[component]==true)
					innerComponents[component] = true;
			}
		}
//		// option b: comparison with bounding box intersection
//		for (unsigned int j=0; j<nComponent_; j++)
//		{
//			if (i==j || isLetterRegion_[j]==false)
//				continue;
//			if ((labeledRegions_[i] & labeledRegions_[j]).area() != 0)
//				innerComponents[j] = true;
//		}

		if (countInnerLetterCandidates(innerComponents) > innerLetterCandidatesParameter)
		{
			isLetterRegion_[i] = false;
			nLetter_--;
		}
	}

	// Show the components before and after the rules/criterions were applied (letter candidates and letters)
	if (debug["showLetterCandidates"])
	{
		cv::Mat output = originalImage_.clone();
		for (size_t i = 0; i < nComponent_; i++)
		{
			if (firstPass_)
				cv::rectangle(output, cv::Point(labeledRegions_[i].x, labeledRegions_[i].y),
						cv::Point(labeledRegions_[i].x + labeledRegions_[i].width, labeledRegions_[i].y + labeledRegions_[i].height),
						cv::Scalar((255), (255), (255)), 1);
			else
				cv::rectangle(output, cv::Point(labeledRegions_[i].x, labeledRegions_[i].y),
						cv::Point(labeledRegions_[i].x + labeledRegions_[i].width, labeledRegions_[i].y + labeledRegions_[i].height), cv::Scalar((0), (0), (0)), 1);
		}
		if (firstPass_)
		{
			cv::imshow("bright letter candidates", output);
			cvMoveWindow("bright letter candidates", 0, 0);
		}
		else
		{
			cv::imshow("dark letter candidates", output);
			cvMoveWindow("dark letter candidates", 0, 0);
		}
		cv::waitKey(0);
	}
	if (debug["showLetters"])
	{
		cv::Mat output = originalImage_.clone();
		for (size_t i = 0; i < nComponent_; i++)
		{
			if (firstPass_)
			{
				if (isLetterRegion_[i] == true)
				{
					cv::rectangle(output, cv::Point(labeledRegions_[i].x, labeledRegions_[i].y),
							cv::Point(labeledRegions_[i].x + labeledRegions_[i].width, labeledRegions_[i].y + labeledRegions_[i].height),
							cv::Scalar((255), (255), (255)), 1);
					for (int y = labeledRegions_[i].y; y < labeledRegions_[i].y + labeledRegions_[i].height; y++)
						for (int x = labeledRegions_[i].x; x < labeledRegions_[i].x + labeledRegions_[i].width; x++)
						{
							float felement = static_cast<float> (i);
							if (felement == ccmap.at<float> (y, x))
								cv::rectangle(output, cv::Point(x, y), cv::Point(x, y), cv::Scalar(255, 255, 255), 1);
						}

				}
			}
			else
			{
				if (isLetterRegion_[i] == true)
				{
					cv::rectangle(output, cv::Point(labeledRegions_[i].x, labeledRegions_[i].y),
							cv::Point(labeledRegions_[i].x + labeledRegions_[i].width, labeledRegions_[i].y + labeledRegions_[i].height),
							cv::Scalar((0), (0), (0)), 1);
					for (int y = labeledRegions_[i].y; y < labeledRegions_[i].y + labeledRegions_[i].height; y++)
						for (int x = labeledRegions_[i].x; x < labeledRegions_[i].x + labeledRegions_[i].width; x++)
						{
							float felement = static_cast<float> (i);
							if (felement == ccmap.at<float> (y, x))
								cv::rectangle(output, cv::Point(x, y), cv::Point(x, y), cv::Scalar(0, 0, 0), 1);
						}
				}
			}
		}
		if (firstPass_)
		{
			cv::imshow("bright letters", output);
			cvMoveWindow("bright letters", 0, 0);
		}
		else
		{
			cv::imshow("dark letters", output);
			cvMoveWindow("dark letters", 0, 0);
		}
		cv::waitKey(0);
	}
}

inline int DetectText::countInnerLetterCandidates(std::vector<bool> & array)
{
	int count = 0;
	for (size_t i = 0; i < array.size(); i++)
		if (array[i] == true)
			count++;
	return count;
}

std::vector<float> DetectText::getMeanIntensity(const cv::Mat& ccmap, const cv::Rect& rect, int element, bool background)
{
	// get r g b and gray value of pixels in rect distinguished with ccmap label (which one is bg/foreground)

	std::vector<float> elementMeanRGB(4);

	bgr clr;
	double rSum = 0, bSum = 0, gSum = 0, graySum = 0, count = 0;
	float felement = static_cast<float>(element);

	for (int y = rect.y; y < rect.y + rect.height; y++)
		for (int x = rect.x; x < rect.x + rect.width; x++)
		{
			if (background) // get color of all pixels that dont belong to actual stroke in this label (the background pixels)
			{
				if (ccmap.at<float>(y, x) != felement)
				{
					graySum += static_cast<float> (grayImage_.at<unsigned char>(y, x));
					count++;
					clr = originalImage_.at<bgr>(y, x);
					rSum += (float) clr.r;
					bSum += (float) clr.b;
					gSum += (float) clr.g;
				}
			}
			else // foreground pixel color, means letter color
			{
				if (ccmap.at<float>(y, x) == felement)
				{
					graySum += static_cast<float>(grayImage_.at<unsigned char>(y, x));
					count+=1.;
					clr = originalImage_.at<bgr>(y, x);
					rSum += (float) clr.r;
					bSum += (float) clr.b;
					gSum += (float) clr.g;
				}
			}
		}

	elementMeanRGB[0] = rSum / count;
	elementMeanRGB[1] = gSum / count;
	elementMeanRGB[2] = bSum / count;
	elementMeanRGB[3] = graySum / count;

	return elementMeanRGB;
}

void DetectText::groupLetters(const cv::Mat& swtmap, const cv::Mat& ccmap)
{
	// group 2 letterboxes together if they fit

	//std::vector<float> medianSw(nComponent_);

	double largeLetterCountFactor = 1.0;
	if (nLetter_ > 200)
		largeLetterCountFactor = 0.4;	//0.4

	// for all possible letter candidate rects
	for (size_t i = 0; i < nComponent_; i++)
	{
		if (isLetterRegion_[i]==false)
			continue;

		cv::Rect iRect = labeledRegions_[i];

		for (size_t j = i + 1; j < nComponent_; j++)
		{
			if (isLetterRegion_[j]==false)
				continue;

			cv::Rect jRect = labeledRegions_[j];

			// rule 1: distance between components
			double dx = (iRect.x+iRect.width/2 - jRect.x-jRect.width/2);
			double dy = (iRect.y+iRect.height/2 - jRect.y-jRect.height/2);
			double distance = sqrt(dx*dx + dy*dy);

			double iDiagonal = sqrt(iRect.height * iRect.height + iRect.width * iRect.width);
			double jDiagonal = sqrt(jRect.height * jRect.height + jRect.width * jRect.width);

			// rule 1a: distance of two letters must be small enough
			if (processing_method_==ORIGINAL_EPSHTEIN)
			{
//				if (distance > std::max(iRect.width, jRect.width) * distanceRatioParameter * largeLetterCountFactor)
//					continue;
				if (std::abs(dx) > std::max(iRect.width, jRect.width) * distanceRatioParameter) // * largeLetterCountFactor)
					continue;
			}
			else
			{
				if (distance > std::min(iDiagonal, jDiagonal) * distanceRatioParameter * largeLetterCountFactor)
					continue;
			}

			// rule 1b: height ratio between two letters must be small enough
			if (processing_method_==ORIGINAL_EPSHTEIN)
				if ((double)std::max(iRect.height, jRect.height) > /*1.7*/2.2 * (double)std::min(iRect.height, jRect.height))
					continue;

			// rule 1c: vertical overlap should be large
			if (processing_method_==ORIGINAL_EPSHTEIN)
			{
				int verticalOverlap = std::min(iRect.y + iRect.height, jRect.y + jRect.height) - std::max(iRect.y, jRect.y);
				if (verticalOverlap * 1.3 < std::min(iRect.height, jRect.height))
					continue;
			}

			//medianSw[i] = getMedianStrokeWidth(ccmap, swtmap, iRect, static_cast<int>(i));
			//medianSw[j] = getMedianStrokeWidth(ccmap, swtmap, jRect, static_cast<int>(j));

			int negativeScore = 0; //high score is bad

			// rule 2: median of stroke width ratio
			if (std::max(medianStrokeWidth_[i], medianStrokeWidth_[j]) > medianSwParameter * std::min(medianStrokeWidth_[i], medianStrokeWidth_[j]) &&
					(std::abs(meanRGB_[i][3] - meanRGB_[j][3]) < 0.4*grayClrParameter && std::max(medianStrokeWidth_[i], medianStrokeWidth_[j]) > 2.5 * std::min(medianStrokeWidth_[i], medianStrokeWidth_[j])))
				negativeScore++;

//			// rule 3: diagonal ratio
//			if (std::max(iDiagonal, jDiagonal) > diagonalRatioParamter * std::min(iDiagonal, jDiagonal))
//				negativeScore++;

			// rule 4: average gray color of letters
			if (std::abs(meanRGB_[i][3] - meanRGB_[j][3]) > grayClrParameter)
				negativeScore++;

//			if ((iRect & jRect).area() > 0.25 * std::min(iRect.area(), jRect.area()))
//				negativeScore++;

			if (processing_method_==BORMANN)
			{
//				// rule 3: diagonal ratio
//				if ((std::max(iDiagonal, jDiagonal) / std::min(iDiagonal, jDiagonal)) > diagonalRatioParamter)
//					negativeScore++;

//				// rule 4: average gray color of letters
//				if (std::abs(meanRGB_[i][3] - meanRGB_[j][3]) > grayClrParameter)
//					negativeScore++;

				// rule 5: rgb of letters
				// foreground color difference between letters
				if (std::abs(meanRGB_[i][0] - meanRGB_[j][0]) > clrSingleParameter || std::abs(meanRGB_[i][1] - meanRGB_[j][1]) > clrSingleParameter
						|| std::abs(meanRGB_[i][2] - meanRGB_[j][2]) > clrSingleParameter)
					negativeScore += 2;
			}

			// background color difference between letters
			// if (std::abs(meanBgRGB_[i][0] - meanBgRGB_[j][0]) > clrSingleParameter || std::abs(meanBgRGB_[i][1]
			//     - meanBgRGB_[j][1]) > clrSingleParameter || std::abs(meanBgRGB_[i][2] - meanBgRGB_[j][2])
			//     > clrSingleParameter)
			//   score++;
			// fgDifferenceSum = std::abs(meanRGB_[i][0] - meanRGB_[j][0]) + std::abs(meanRGB_[i][1] - meanRGB_[j][1])
			//      + std::abs(meanRGB_[i][2] - meanRGB_[j][2]);
			//  bgDifferenceSum = std::abs(meanBgRGB_[i][0] - meanBgRGB_[j][0]) + std::abs(meanBgRGB_[i][1] - meanBgRGB_[j][1])
			//      + std::abs(meanBgRGB_[i][2] - meanBgRGB_[j][2]);
			// if ((fgDifferenceSum > clrSumParameter && bgDifferenceSum > clrSumParameter) || fgDifferenceSum > 2
			//     * clrSumParameter || fgDifferenceSum > 2 * clrSumParameter)
			//   score++;

			// rule #7: Areas of components have to be ~similiar
			if (processing_method_==BORMANN)
			{
				if ((std::max(iRect.area(), jRect.area()) / (float) std::min(iRect.area(), jRect.area())) > areaParameter)
					if (std::max(iRect.height, jRect.height) / (float) std::min(iRect.height, jRect.height) > areaParameter * 0.8)
						negativeScore++; // even though components can be rotated, their height has to be at least in the same range, to check for height in groupLetters is more convenient than in identifyLetters
			}

			// rule #8: Number of foreground pixels / all pixels of rect
			//      int pixelCountJ = 0, pixelCountI = 0;
			//
			//      float felement = static_cast<float> (i);
			//      for (int y = iRect.y; y < iRect.y + iRect.height; y++)
			//        for (int x = iRect.x; x < iRect.x + iRect.width; x++)
			//          if (ccmap.at<float> (y, x) == felement)
			//            pixelCountI++;
			//
			//      felement = static_cast<float> (j);
			//
			//      for (int y = jRect.y; y < jRect.y + jRect.height; y++)
			//        for (int x = jRect.x; x < jRect.x + jRect.width; x++)
			//          if (ccmap.at<float> (y, x) == felement)
			//            pixelCountJ++;
			//
			//      if (pixelCountJ / (float)jRect.area() > (1 - pixelParameter) || pixelCountJ / (float)jRect.area()
			//          < pixelParameter || pixelCountI / (float)iRect.area() > (1 - pixelParameter) || pixelCountI
			//          / (float)iRect.area() < pixelParameter)
			//      {
			//        negativeScore++;
			//      }

			bool isGroup = true;
			if (processing_method_==ORIGINAL_EPSHTEIN)
			{
				if (negativeScore > 0)
					isGroup = false;
			}
			else
			{
				if (negativeScore > 1)
					isGroup = false;
			}


			if (isGroup==true)
				letterGroups_.push_back(Pair(i, j, dx, dy));

		}// end for loop j
	}// end for loop i
}

float DetectText::getMedianStrokeWidth(const cv::Mat& ccmap, const cv::Mat& swtmap, const cv::Rect& rect, int element)
{
	assert(element >= 0);
	assert(isLetterRegion_[element]);

	std::vector<float> SwtValues;

	float felement = static_cast<float> (element);
	for (int y = rect.y; y < rect.y + rect.height; y++)
		for (int x = rect.x; x < rect.x + rect.width; x++)
		{
			if (ccmap.at<float> (y, x) == felement)
			{
				SwtValues.push_back(swtmap.at<float> (y, x));
			}
		}

	std::nth_element(SwtValues.begin(), SwtValues.begin() + SwtValues.size() / 2, SwtValues.end());

	return SwtValues[SwtValues.size() / 2];
}

// merges letterGroups_ to chains of letters, finds bounding box of these chains and creates the textRegions data structure
void DetectText::chainPairs(std::vector<TextRegion>& textRegions)
{
	if (debug["showPairs"])
	{
		cv::Mat output = originalImage_.clone();

		for (unsigned int i = 0; i < letterGroups_.size(); i++)
		{
			cv::rectangle(
					output,
					cv::Point(labeledRegions_.at(letterGroups_[i].left).x, labeledRegions_.at(letterGroups_[i].left).y),
					cv::Point(labeledRegions_.at(letterGroups_[i].left).x + labeledRegions_.at(letterGroups_[i].left).width,
							labeledRegions_.at(letterGroups_[i].left).y + labeledRegions_.at(letterGroups_[i].left).height),
					cv::Scalar((25 * i + 100) % 255, (35 * (i + 1) + 100) % 255, (45 * (i + 2) + 100) % 255), 1);
			cv::rectangle(
					output,
					cv::Point(labeledRegions_.at(letterGroups_[i].right).x, labeledRegions_.at(letterGroups_[i].right).y),
					cv::Point(labeledRegions_.at(letterGroups_[i].right).x + labeledRegions_.at(letterGroups_[i].right).width,
							labeledRegions_.at(letterGroups_[i].right).y + labeledRegions_.at(letterGroups_[i].right).height),
					cv::Scalar((25 * i + 100) % 255, (35 * (i + 1) + 100) % 255, (45 * (i + 2) + 100) % 255), 1);

			if (firstPass_)
			{
				cv::imshow("bright pairs", output);
				cvMoveWindow("bright pairs", 0, 0);
			}
			else
			{
				cv::imshow("dark pairs", output);
				cvMoveWindow("dark pairs", 0, 0);
			}
			cv::waitKey(0);
		}
	}

	std::vector<std::vector<int> > chains;		// contains letters that belong to the same region, outer index=region index, inner index=letter index (refers to index of labeledRegions)
	mergePairs(letterGroups_, chains);		// put letters of same regions into one group, yields several of those groups

	//std::vector<cv::Rect> initialBoxes;
	chainToBox(chains, /*initialBoxes,*/ textRegions); //initialHorizontalBoxes contains rects for every chain component with more than two components(letters)

	if (debug["showChains"])
	{
		cv::Mat output = originalImage_.clone();
		std::cout << "Initial Boxes: " << std::endl;
		for (unsigned int ii = 0; ii < textRegions.size(); ii++)
		{
			std::cout << "   " << textRegions[ii].boundingBox.x << "\t" << textRegions[ii].boundingBox.y << "\t" << textRegions[ii].boundingBox.width << "\t" << textRegions[ii].boundingBox.height << "\t" << std::endl;
			for (unsigned int i = 0; i < textRegions[ii].letters.size(); i++)
			{
				cv::rectangle(output, textRegions[ii].letters[i].boundingBox, cv::Scalar(255, 255, 255), 1, 1, 0);
				cv::rectangle(output, textRegions[ii].letters[i].centerPoint, textRegions[ii].letters[i].centerPoint, cv::Scalar(255, 255, 255), 2, 1, 0);
			}
			cv::rectangle(output, cv::Point(textRegions[ii].boundingBox.x, textRegions[ii].boundingBox.y), cv::Point(textRegions[ii].boundingBox.x+textRegions[ii].boundingBox.width, textRegions[ii].boundingBox.y+textRegions[ii].boundingBox.height), cv::Scalar(0,255,0), 2, 1, 0);
			cv::imshow("chains", output);
			cvMoveWindow("chains", 0, 0);
			cv::waitKey(0);
		}
	}
}

void DetectText::chainToBox(std::vector<std::vector<int> >& chains, /*std::vector<cv::Rect>& boundingBox,*/ std::vector<TextRegion>& textRegions)
{
	for (size_t i = 0; i < chains.size(); i++)
	{
		if (chains[i].size() < 3) //Only words with more than 2 letters	// todo: param
			continue;

		int minX = grayImage_.cols, minY = grayImage_.rows, maxX = 0, maxY = 0;
		int letterAreaSum = 0;
		int padding = 0;		// todo: param

		TextRegion textRegion;
		for (size_t j = 0; j < chains[i].size(); j++)
		{
			cv::Rect itr = labeledRegions_[chains[i][j]];
			letterAreaSum += itr.width * itr.height;
			minX = std::min(minX, itr.x);
			minY = std::min(minY, itr.y);
			maxX = std::max(maxX, itr.x + itr.width);
			maxY = std::max(maxY, itr.y + itr.height);

			Letter letter;
			letter.boundingBox = itr;
			letter.diameter = sqrt(itr.width*itr.width + itr.height*itr.height);
			letter.centerPoint = cv::Point2d(itr.x + 0.5 * (double)itr.width, itr.y + 0.5 * (double)itr.height);
			if (firstPass_)
				letter.fontColor = BRIGHT;
			else
				letter.fontColor = DARK;
			textRegion.letters.push_back(letter);
		}

		// add padding around each box
		minX = std::max(0, minX - padding);
		minY = std::max(0, minY - padding);
		maxX = std::min(grayImage_.cols, maxX + padding);
		maxY = std::min(grayImage_.rows, maxY + padding);

		//boundingBox.push_back(cv::Rect(minX, minY, maxX - minX, maxY - minY));
		textRegion.boundingBox = cv::Rect(minX, minY, maxX - minX, maxY - minY);

		if (textRegion.boundingBox.width > 1.9 * textRegion.boundingBox.height)
			textRegions.push_back(textRegion);
	}
}

bool DetectText::sameTextline(const TextRegion& a, const TextRegion& b)
{
	int width = std::min(a.boundingBox.x + a.boundingBox.width, b.boundingBox.x + b.boundingBox.width) - std::max(a.boundingBox.x, b.boundingBox.x);
	int height = std::min(a.boundingBox.y + a.boundingBox.height, b.boundingBox.y + b.boundingBox.height) - std::max(a.boundingBox.y, b.boundingBox.y);
	/* overlapped 10% */
	return (width > 0 && height > 0 &&
			width * height > 0.1 * std::max(a.boundingBox.width * a.boundingBox.height, b.boundingBox.width * b.boundingBox.height) &&
			width * height > 0.8 * std::min(a.boundingBox.width * a.boundingBox.height, b.boundingBox.width * b.boundingBox.height));
}

bool DetectText::pairsInLine(const Pair& a, const Pair& b)
{
	if (a.left==b.left || a.right==b.right)		// todo: use arbitrary angles
	{
		int tn = a.dy * b.dx - a.dx * b.dy;
		int td = a.dx * b.dx + a.dy * b.dy;
		// share the same end, opposite direction
		if (tn * 7 < -td * 4 && tn * 7 > td * 4)
			return true;
	}
	else if (a.left==b.right || a.right==b.left)
	{
		int tn = a.dy * b.dx - a.dx * b.dy;
		int td = a.dx * b.dx + a.dy * b.dy;
		// share the other end, same direction
		if (tn * 7 < td * 4 && tn * 7 > -td * 4)
			return true;
	}
	return false;
}

void DetectText::mergePairs(const std::vector<Pair>& groups, std::vector< std::vector<int> >& chains)
{
	chains.clear();

	std::vector<TreeNode> nodes(groups.size());
	for (unsigned int i=0; i<groups.size(); i++)
	{
		nodes[i].parent = -1;
		nodes[i].rank = 0;
		nodes[i].element = i;
	}

	for (unsigned int i=0; i<groups.size(); i++)
	{
		int root = i;
		while (nodes[root].parent != -1)
			root = nodes[root].parent;
		for (unsigned int j=0; j<groups.size(); j++)
		{
			if (i!=j && pairsInLine(groups[nodes[i].element], groups[nodes[j].element])==true)
			{
				int root2 = j;
				while (nodes[root2].parent != -1)
					root2 = nodes[root2].parent;
				if (root != root2)
				{
					if(nodes[root].rank > nodes[root2].rank)
						nodes[root2].parent = root;
					else
					{
						nodes[root].parent = root2;
						nodes[root2].rank += (nodes[root].rank==nodes[root2].rank ? 1 : 0);
						root = root2;
					}

					// collapse a branch to direct children of root
					int node = j;
					while(nodes[node].parent != -1)
					{
						int temp = nodes[node].parent;
						nodes[node].parent = root;
						node = temp;
					}
					node = i;
					while(nodes[node].parent != -1)
					{
						int temp = nodes[node].parent;
						nodes[node].parent = root;
						node = temp;
					}
				}
			}
		}
	}

	// insert pairs into chains
	int classIndex = 0;
	for (unsigned int i=0; i<groups.size(); i++)
	{
		int root = i;
		while (nodes[root].parent != -1)
			root = nodes[root].parent;
		if (nodes[root].rank >= 0)
			nodes[root].rank = ~classIndex++;

		int insertIndex = ~nodes[root].rank;
		if (insertIndex+1 < (int)chains.size())
		{
			// add new letters to chain (do not add if the letter is already in the list)
			for (int letterNumber=0; letterNumber<2; letterNumber++)
			{
				int letterIndex = (letterNumber==0 ? groups[i].left : groups[i].right);
				bool inList = false;
				for (unsigned int j=0; (inList==false && j<chains[insertIndex].size()); j++)
					if (chains[insertIndex][j] == letterIndex)
						inList = true;
				if (inList == false)
					chains[insertIndex].push_back(letterIndex);
			}
		}
		else
		{
			// create new chain
			chains.push_back(std::vector<int>());
			chains[insertIndex].push_back(groups[i].left);
			chains[insertIndex].push_back(groups[i].right);
		}
	}

//	// ------------ old code (does not obey direction of merged pairs) --------------
//
//	/* groups looks like this:
//	 *  4 5
//	 *  12 14
//	 *  44 45
//	 *   ...
//	 */
//	std::vector<std::vector<int> > initialChains;
//	initialChains.resize(groups.size());
//	for (size_t i = 0; i < groups.size(); i++)
//	{
//		std::vector<int> temp;
//		temp.push_back(groups[i].left);
//		temp.push_back(groups[i].right);
//		initialChains[i] = temp;
//	}
//
//	/* initialChains looks like this:
//	 * [0]  [1]  [2]
//	 *  4    12   44   ...
//	 *  5    14   45
//	 */
//
//	while (mergePairs(initialChains, chains))
//	{
//		initialChains = chains;
//		chains.clear();
//	}
}

bool DetectText::mergePairs(const std::vector< std::vector<int> >& initialChains, std::vector< std::vector<int> >& chains)
{
	chains.clear();

	bool merged = false;
	std::vector<int> mergedToChainBitMap(initialChains.size(), -1);

	for (size_t i = 0; i < initialChains.size(); i++)
	{
		if (mergedToChainBitMap[i] != -1)
			continue;

		for (size_t j = i + 1; j < initialChains.size(); j++)
		{
			// match elements in chain i,j
			for (size_t ki = 0; ki < initialChains[i].size(); ki++)
			{
				for (size_t kj = 0; kj < initialChains[j].size(); kj++)
				{
					// found match
					if (initialChains[i][ki] == initialChains[j][kj]) // Does any other initialChains[x] contain a identical component?
					{
						merged = true;
						// j already merged with others
						if (mergedToChainBitMap[j] != -1)
						{
							merge(initialChains[i], chains[mergedToChainBitMap[j]]);	// add new letters to existing chain

							mergedToChainBitMap[i] = mergedToChainBitMap[j];
						}
						else // start a new chain
						{
							std::vector<int> newChain;
							merge(initialChains[i], newChain);
							merge(initialChains[j], newChain);
							chains.push_back(newChain);
							mergedToChainBitMap[i] = chains.size() - 1;
							mergedToChainBitMap[j] = chains.size() - 1;
						}
						break;
					}
				}
				if (mergedToChainBitMap[i] != -1 && mergedToChainBitMap[j] != -1)
					break;
			}
		}

		// comparing with all other chains, not found a match
		if (mergedToChainBitMap[i] == -1)
		{
			chains.push_back(initialChains[i]);
			mergedToChainBitMap[i] = chains.size() - 1;
		}

	}

	if (!merged)
	{
		chains = initialChains;
	}
	return merged;
}

void DetectText::merge(const std::vector<int>& token, std::vector<int>& chain)
{
	std::vector<int>::iterator it;
	for (size_t i = 0; i < token.size(); i++)
	{
		it = find(chain.begin(), chain.end(), token[i]);
		if (it == chain.end())
		{
			chain.push_back(token[i]);
		}
	}
}

void DetectText::filterBoundingBoxes(std::vector<cv::Rect>& boundingBoxes, cv::Mat& ccmap, int rejectRatio)
{
	std::vector<cv::Rect> qualifiedBoxes;
	std::vector<int> components;

	for (size_t i = 0; i < boundingBoxes.size(); i++)
	{
		int isLetterCount = 0;
		int letterArea = 0;
		int nonLetterArea = 0;
		cv::Rect rect = boundingBoxes[i];

		float width = static_cast<float>(rect.width);
		float height = static_cast<float>(rect.height);
		if (width < 20) // Only these rects/words with width > 20 -> maybe to be changed when rotated
			continue;
		if (std::max(width, height) > 20.0*std::min(width, height)) // maybe to be changed
			continue;

		for (int y = rect.y; y < rect.y + rect.height; y++)
		{
			for (int x = rect.x; x < rect.x + rect.width; x++)
			{
				int componetIndex = static_cast<int>(ccmap.at<float>(y, x));

				if (componetIndex < 0)
					continue;

				if (isLetterRegion_[componetIndex])
					letterArea++;
				else
					nonLetterArea++;

				if (find(components.begin(), components.end(), componetIndex) == components.end())
				{
					components.push_back(componetIndex);
					if (isLetterRegion_[componetIndex])
						isLetterCount++;
				}
			}
		}

		// accept patch with few noise inside
		if (letterArea > 3 * nonLetterArea || static_cast<int>(components.size()) < rejectRatio * isLetterCount)
			qualifiedBoxes.push_back(rect);

		components.clear();
	}
	boundingBoxes = qualifiedBoxes;

	if (debug["showChains"])
	{
		cv::Mat output = originalImage_.clone();
		if (firstPass_)
		{
			for (unsigned int i = 0; i < boundingBoxes.size(); i++)
				rectangle(output, cv::Point(boundingBoxes[i].x, boundingBoxes[i].y),
						cv::Point(boundingBoxes[i].x + boundingBoxes[i].width, boundingBoxes[i].y + boundingBoxes[i].height), cv::Scalar(255, 255, 255), 2);
			cv::imshow("bright chains", output);
			cvMoveWindow("bright chains", 0, 0);
		}
		else
		{
			for (unsigned int i = 0; i < boundingBoxes.size(); i++)
				rectangle(output, cv::Point(boundingBoxes[i].x, boundingBoxes[i].y),
						cv::Point(boundingBoxes[i].x + boundingBoxes[i].width, boundingBoxes[i].y + boundingBoxes[i].height), cv::Scalar(0, 0, 0), 2);
			cv::imshow("dark chains", output);
			cvMoveWindow("dark chains", 0, 0);
		}
		cv::waitKey(0);
	}
}

void DetectText::combineNeighborBoxes(std::vector<cv::Rect>& originalBoxes)
{
	// Combine boxes that belong together, based on color and same area

	std::vector<cv::Rect> boxes = originalBoxes;
	std::vector<cv::Rect> finalBoxes;

	cv::Mat output = originalImage_.clone();

	std::vector<std::pair<unsigned int, unsigned int> > neighbors;
	int areaTotal = 0;

	for (unsigned int i = 0; i < boxes.size(); i++)
	{
		long red = 0, blue = 0, green = 0;
		for (int y = boxes[i].y; y < boxes[i].y + boxes[i].height; y++)
		{
			for (int x = boxes[i].x; x < boxes[i].x + boxes[i].width; x++)
			{
				bgr clr = originalImage_.at<bgr> (y, x);
				red += clr.r;
				blue += clr.b;
				green += clr.g;
			}
		}
		red /= boxes[i].area();
		blue /= boxes[i].area();
		green /= boxes[i].area();

		long red2 = 0, blue2 = 0, green2 = 0;

		for (unsigned int j = i + 1; j < boxes.size(); j++)
		{
			output = originalImage_.clone();
			for (int y2 = boxes[j].y; y2 < boxes[j].y + boxes[j].height; y2++)
			{
				for (int x2 = boxes[j].x; x2 < boxes[j].x + boxes[j].width; x2++)
				{
					bgr clr = originalImage_.at<bgr> (y2, x2);
					red2 += clr.r;
					blue2 += clr.b;
					green2 += clr.g;
				}
			}
			red2 /= boxes[j].area();
			blue2 /= boxes[j].area();
			green2 /= boxes[j].area();

			std::cout << "red:" << red << ",green:" << green << ",blue:" << blue << std::endl;
			std::cout << "red2: " << red2 << ",green2:" << green2 << "blue2: " << blue2 << std::endl;
			cv::rectangle(output, boxes[i], cv::Scalar(0, 0, 0), 2, 1, 0);
			cv::rectangle(output, boxes[j], cv::Scalar(0, 0, 0), 2, 1, 0);
			cv::imshow("2 boxes", output);

			// rgb, height and y criterion
			if (std::abs(red - red2) < 75 && std::abs(green - green2) < 75 && std::abs(blue - blue2) < 75)
			{
				// do they share area?
				cv::Rect r1 = boxes[i];
				r1.x = std::max(r1.x - 5, 0); // tolerance 5 pixel each side -> add padding
				r1.y = std::max(r1.y - 5, 0);
				if (r1.x + r1.width + 10 <= originalImage_.cols)
					r1.width = r1.width + 10;
				if (r1.y + r1.height + 10 <= originalImage_.rows)
					r1.height = r1.height + 10;

				cv::Rect r2 = boxes[j];
				r2.x = std::max(r2.x - 5, 0);
				r2.y = std::max(r2.y - 5, 0);
				if (r2.x + r2.width + 10 <= originalImage_.cols)
					r2.width = r2.width + 10;
				if (r2.y + r2.height + 10 <= originalImage_.rows)
					r2.height = r2.height + 10;
				cv::rectangle(output, r1, cv::Scalar(255, 255, 255), 2, 1, 0);
				cv::rectangle(output, r2, cv::Scalar(255, 255, 255), 2, 1, 0);
				std::cout << "same area: " << (r1 & r2).area() << std::endl;
				areaTotal += (r1 & r2).area();
				if ((r1 & r2).area() > 0)
					neighbors.push_back(std::pair<int, int>(i, j));
			}
			cv::waitKey(0);
		}
	}

	std::vector<cv::Rect> dummyBoxes;

	// Fuse two boxes if they meet the condition
	while (areaTotal > 0)
	{
		areaTotal = 0;
		output = originalImage_.clone();
		for (unsigned int i = 0; i < neighbors.size(); i++)
		{
			std::cout << "neighbors:" << i << "[" << neighbors[i].first << "|" << neighbors[i].second << "]" << std::endl;
			if (debug["showNeighborMerging"] == true)
			{
				cv::rectangle(output, boxes[neighbors[i].first], cv::Scalar(255, 255, 255), 2, 1, 0);
				cv::rectangle(output, boxes[neighbors[i].second], cv::Scalar(255, 255, 255), 2, 1, 0);
			}

			int minX = std::min(boxes[neighbors[i].first].x, boxes[neighbors[i].second].x);
			int minY = std::min(boxes[neighbors[i].first].y, boxes[neighbors[i].second].y);
			int maxX = std::max(boxes[neighbors[i].first].x + boxes[neighbors[i].first].width, boxes[neighbors[i].second].x + boxes[neighbors[i].second].width);
			int maxY = std::max(boxes[neighbors[i].first].y + boxes[neighbors[i].first].height,
					boxes[neighbors[i].second].y + boxes[neighbors[i].second].height);
			dummyBoxes.push_back(cv::Rect(minX, minY, maxX - minX, maxY - minY));

			if (debug["showNeighborMerging"] == true)
			{
				cv::rectangle(output, cv::Rect(minX - 2, minY - 2, maxX - minX + 4, maxY - minY + 4), cv::Scalar(150, 150, 150), 2, 1, 0);
				cv::imshow("combineNeighborBoxes", output);
				cvMoveWindow("combineNeighborBoxes", 0, 0);
				cv::waitKey(0);
			}
		}
		neighbors.clear();
		std::cout << "dummyBoxes.size(): " << dummyBoxes.size() << std::endl;
		for (unsigned int i = 0; i < dummyBoxes.size(); i++)
		{
			for (unsigned int j = i + 1; j < dummyBoxes.size(); j++)
			{
				areaTotal += (dummyBoxes[i] & dummyBoxes[j]).area();
				if ((dummyBoxes[i] & dummyBoxes[j]).area() > 0)
				{
					bool alreadyIn = false;
					for (unsigned int k = 0; k < neighbors.size(); k++)
					{
						if (neighbors[k].first == i || neighbors[k].second == i || neighbors[k].first == j || neighbors[k].second == j)
						{
							alreadyIn = true;
							break;
						}
					}
					if (!alreadyIn)
						neighbors.push_back(std::pair<int, int>(i, j));
				}
			}
		}
		// add not merged ones
		for (unsigned int i = 0; i < dummyBoxes.size(); i++)
		{
			bool alreadyIn = false;
			for (unsigned int k = 0; k < neighbors.size(); k++)
			{
				if (neighbors[k].first == i || neighbors[k].second == i)
				{
					alreadyIn = true;
					break;
				}
			}
			if (!alreadyIn)
				finalBoxes.push_back(dummyBoxes[i]);
		}
		boxes.clear();
		boxes = dummyBoxes;
		dummyBoxes.clear();
	}

	originalBoxes.clear();
	originalBoxes = boxes;
	for (unsigned int i = 0; i < finalBoxes.size(); i++)
		originalBoxes.push_back(finalBoxes[i]);
}

// computes the signed distance of a point (point) to a line (represented by a line point and the line normal is Hessian Normal Form)
double DetectText::pointLineDistance2D(cv::Point2d linePoint, cv::Point2d lineNormal, cv::Point2d point)
{
	cv::Point2d diff = linePoint-point;
	return diff.x*lineNormal.x + diff.y*lineNormal.y;
}


void DetectText::breakLines(std::vector<TextRegion>& textRegions)
{
	// this function splits up bounding boxes of potentially multiline text into boxes around single line text

	if (debug["showBreakLines"])
		std::cout << "Breaking boxes into single-line boxes." << std::endl;

	//For every boundingBox:
	std::vector<TextRegion> splitUpTextRegions;
	for (unsigned int textRegionIndex = 0; textRegionIndex < textRegions.size(); textRegionIndex++)
	{
		//which Letters belong to the current boundingBox
		// save bounding boxes and mid points of the currently active letters
		std::vector<cv::Rect> currentLetterBoxes(textRegions[textRegionIndex].letters.size());
		std::vector<cv::Point2d> currentPoints(textRegions[textRegionIndex].letters.size());
		std::vector<double> currentDiameters(textRegions[textRegionIndex].letters.size());
//		for (unsigned int i = 0; i < letters.size(); i++)
//			if ((boxes[boxIndex] & letters[i]).area() / (letters[i].area()) == 1.0)
//				currentLetterBoxes.push_back(letters[i]);
		double averageLetterSize = 0;
		for (unsigned int i=0; i<textRegions[textRegionIndex].letters.size(); i++)
		{
			currentLetterBoxes[i] = textRegions[textRegionIndex].letters[i].boundingBox;
			currentPoints[i] = textRegions[textRegionIndex].letters[i].centerPoint;
			currentDiameters[i] = textRegions[textRegionIndex].letters[i].diameter;
			averageLetterSize += textRegions[textRegionIndex].letters[i].diameter;
		}
		averageLetterSize /= (double)textRegions[textRegionIndex].letters.size();

		// use RANSAC to fit letters to straight lines
		// -------------------------------------------
		double confidenceLevel = 0.995;
		double problemDimension = 2;

		// calculate percent outliers based on pca
		cv::Mat pcaData(currentPoints.size(), 2, CV_32FC1);
		for (unsigned int i = 0; i < currentPoints.size(); i++)
		{
			pcaData.at<float>(i, 0) = currentPoints[i].x;
			pcaData.at<float>(i, 1) = currentPoints[i].y;
		}
		cv::PCA pca(pcaData, cv::noArray(), CV_PCA_DATA_AS_ROW, 2);
		float eigenValQuotient = std::min(pca.eigenvalues.at<float>(0, 0), pca.eigenvalues.at<float>(1, 0)) / std::max(pca.eigenvalues.at<float>(0, 0), pca.eigenvalues.at<float>(1, 0));
		// with perfect points orientation (all on one line) eigenValQuotient ~> 0 -> then percent of false data is supposed to be low
		// points are spread around the whole space -> similiar eigenvalues -> percent outliers is high: maxE
		double outlierRatio = (maxE - minE) * eigenValQuotient + minE;
		// calculate number of iterations
		int maxRANSACIterations = 5 * std::ceil(std::log10(1 - confidenceLevel) / (double)(std::log10(1 - (std::pow((1 - outlierRatio), problemDimension)))));

		double inlierDistanceThreshold = averageLetterSize * inlierDistanceThresholdFactor_;
		if (debug["showBreakLines"])
			std::cout << "inlierDistanceThreshold=" << inlierDistanceThreshold << std::endl;

		unsigned int lastSetSizeCurrentPoints = 0;
		while (currentPoints.size()>2 && (currentPoints.size()!=lastSetSizeCurrentPoints))
		{
			lastSetSizeCurrentPoints = currentPoints.size();

			// 1. RANSAC for line fitting
			std::cout << "Number current points = " << currentPoints.size() << std::endl;
			double bestScore = 0.;
			cv::Point2d bestP1(0,0);
			cv::Point2d bestNormal(-1,-1);
			for (int ransacAttempt=0; ransacAttempt<maxRANSACIterations; ransacAttempt++)
			{
				std::vector<int> randomPointSet; // which 2 points are chosen (index)
				std::vector<cv::Point> supportingPoints; //which points support the model

				// fill randomPointSet with random points for model
				while (randomPointSet.size() < problemDimension)
				{
					int nn = (int)((std::rand() / (float) RAND_MAX) * currentPoints.size());
					if (std::find(randomPointSet.begin(), randomPointSet.end(), nn) == randomPointSet.end())
					{
						randomPointSet.push_back(nn);
					}
				}

				// compute line model
				cv::Point2d p1 = currentPoints[randomPointSet[0]];
				double normVal = sqrt((currentPoints[randomPointSet[1]].x-currentPoints[randomPointSet[0]].x)*(currentPoints[randomPointSet[1]].x-currentPoints[randomPointSet[0]].x)+(currentPoints[randomPointSet[1]].y-currentPoints[randomPointSet[0]].y)*(currentPoints[randomPointSet[1]].y-currentPoints[randomPointSet[0]].y));
				cv::Point2d normal((currentPoints[randomPointSet[1]].y-currentPoints[randomPointSet[0]].y)/normVal, -(currentPoints[randomPointSet[1]].x-currentPoints[randomPointSet[0]].x)/normVal);

				// compute distance related score for the current model
				double score = 0.;
				for (unsigned int i=0; i<currentPoints.size(); i++)
				{
					cv::Point2d diff = p1-currentPoints[i];
					double dist = fabs(diff.x*normal.x + diff.y*normal.y);		// todo: this score also considers the outliers in the ranking!
					score += exp(log(0.1) * dist/inlierDistanceThreshold);		// todo: what is the reason for this way of computing the score? (but works well)
				}

				// update best
				if (score > bestScore)
				{
					bestScore = score;
					bestP1 = p1;
					bestNormal = normal;
				}
			}
			if (debug["showBreakLines"])
				std::cout << "bestScore=" << bestScore << std::endl;

			// 2. collect all inliers
			std::vector<int> inlierSet;
			for (unsigned int i=0; i<currentPoints.size(); i++)
			{
				// a) point needs to be close enough to the line
//				cv::Point2d diff = bestP1-currentPoints[i];
//				double dist = fabs(diff.x*bestNormal.x + diff.y*bestNormal.y);
				double dist = 0.;
				if ((dist=fabs(pointLineDistance2D(bestP1, bestNormal, currentPoints[i]))) < inlierDistanceThreshold)
				{
					// b) line has to cross bounding box of letter
					// todo: test all 4 corners of each letter box whether there distances to the line have different signs (i.e. all same sign, box is completely on one side of the line)
					bool negativeSign = false, positiveSign = false;
					(pointLineDistance2D(bestP1, bestNormal, cv::Point2d(currentLetterBoxes[i].x, currentLetterBoxes[i].y)) < 0.) ? negativeSign = true : positiveSign = true;
					(pointLineDistance2D(bestP1, bestNormal, cv::Point2d(currentLetterBoxes[i].x+currentLetterBoxes[i].width, currentLetterBoxes[i].y)) < 0.) ? negativeSign = true : positiveSign = true;
					(pointLineDistance2D(bestP1, bestNormal, cv::Point2d(currentLetterBoxes[i].x, currentLetterBoxes[i].y+currentLetterBoxes[i].height)) < 0.) ? negativeSign = true : positiveSign = true;
					(pointLineDistance2D(bestP1, bestNormal, cv::Point2d(currentLetterBoxes[i].x+currentLetterBoxes[i].width, currentLetterBoxes[i].y+currentLetterBoxes[i].height)) < 0.) ? negativeSign = true : positiveSign = true;

					if (negativeSign==true && positiveSign==true)
					{
						inlierSet.push_back(i);
						if (debug["showBreakLines"])
							std::cout << "inlier found with dist=" << dist << std::endl;
					}
				}
			}

			// 3. break inlier set with respect to letter sizes using non-parametric mode seeking with mean shift
			//  prepare mean shift set
			std::vector<double> meanShiftSet(inlierSet.size());		// contains the letter sizes
			for (unsigned int i = 0; i < inlierSet.size(); i++)
				meanShiftSet[i] = currentDiameters[inlierSet[i]];
				//meanShiftSet[i] = sqrt(currentLetterBoxes[inlierSet[i]].width*currentLetterBoxes[inlierSet[i]].width + currentLetterBoxes[inlierSet[i]].height*currentLetterBoxes[inlierSet[i]].height);
			//  compute median of letter sizes
			double medianLetterSizeInlierSet = 0;
			std::multiset<double> orderedLetterSizes;
			for (unsigned int i = 0; i < meanShiftSet.size(); i++)
				orderedLetterSizes.insert(meanShiftSet[i]);
			std::multiset<double>::iterator it = orderedLetterSizes.begin();
			for (int i=0; i<(int)orderedLetterSizes.size()/2; i++, it++);
			medianLetterSizeInlierSet = *it;
			double meanShiftBandwidth = 2./medianLetterSizeInlierSet * 2./medianLetterSizeInlierSet;
			//	compute mean shift segmentation
			std::vector< std::vector<int> > inlierSubSets;
			std::vector<double> convergencePoints;
			MeanShiftSegmentation1D(meanShiftSet, convergencePoints, inlierSubSets, meanShiftBandwidth, 100);
			//	remap inlierSubSets to indices in inlierSet
			for (unsigned int i=0; i<inlierSubSets.size(); i++)
				for (unsigned int j=0; j<inlierSubSets[i].size(); j++)
					inlierSubSets[i][j] = inlierSet[inlierSubSets[i][j]];

			// 4. verify sub sets and store parameters
			for (int subSetIndex=0; subSetIndex<(int)inlierSubSets.size(); subSetIndex++)
			{
				// skip single letters and do not remove them from the set of current points, they may fit elsewhere
				if (inlierSubSets[subSetIndex].size() < 2)
				{
					//currentPoints.erase(currentPoints.begin()+inlierSubSets[subSetIndex][0]);
					//currentLetterBoxes.erase(currentLetterBoxes.begin()+inlierSubSets[subSetIndex][0]);
					continue;
				}

				std::vector<int>& currentInlierSet = inlierSubSets[subSetIndex];

				// check whether inliers have similar size -> filter clutter
				double medianLetterSizeInlierSubSet = 0;
				std::multiset<double> orderedLetterSizesSub;
				for (unsigned int i = 0; i < currentInlierSet.size(); i++)
					orderedLetterSizesSub.insert(currentDiameters[currentInlierSet[i]]);
					//orderedLetterSizesSub.insert(sqrt(currentLetterBoxes[currentInlierSet[i]].width*currentLetterBoxes[currentInlierSet[i]].width + currentLetterBoxes[currentInlierSet[i]].height*currentLetterBoxes[currentInlierSet[i]].height));
				std::multiset<double>::iterator it = orderedLetterSizesSub.begin();
				for (int i=0; i<(int)orderedLetterSizesSub.size()/2; i++, it++);
				medianLetterSizeInlierSubSet = *it;
				double stddevLetterSize = 0.;
				for (unsigned int i = 0; i < currentInlierSet.size(); i++)
				{
					double letterSize = currentDiameters[currentInlierSet[i]];
					//	double letterSize = sqrt(currentLetterBoxes[currentInlierSet[i]].width*currentLetterBoxes[currentInlierSet[i]].width + currentLetterBoxes[currentInlierSet[i]].height*currentLetterBoxes[currentInlierSet[i]].height);
					stddevLetterSize += (letterSize - medianLetterSizeInlierSubSet) * (letterSize - medianLetterSizeInlierSubSet);
				}
				stddevLetterSize = sqrt(stddevLetterSize/(double)currentInlierSet.size());
				// debug
				if (debug["showBreakLines"])
				{
					cv::Mat output = originalImage_.clone();
					for (unsigned int i=0; i<currentInlierSet.size(); i++)
						cv::rectangle(output, cv::Point(currentLetterBoxes[currentInlierSet[i]].x, currentLetterBoxes[currentInlierSet[i]].y), cv::Point(currentLetterBoxes[currentInlierSet[i]].x+currentLetterBoxes[currentInlierSet[i]].width, currentLetterBoxes[currentInlierSet[i]].y+currentLetterBoxes[currentInlierSet[i]].height), cv::Scalar(0,0,255), 2, 1, 0);
					cv::line(output, cv::Point(bestP1.x-(-1000*bestNormal.y), bestP1.y-1000*bestNormal.x), cv::Point(bestP1.x+(-1000*bestNormal.y), bestP1.y+1000*bestNormal.x), cv::Scalar(255,255,255), 2, 1, 0);
					cv::imshow("breaking lines", output);
					cvMoveWindow("breaking lines", 0, 0);
					std::cout << "medianLetterSizeInlierSubSet=" << medianLetterSizeInlierSubSet << "   stddevLetterSize=" << stddevLetterSize << std::endl;
					cv::waitKey(0);
				}
				if (stddevLetterSize < 0.35*medianLetterSizeInlierSubSet)	// todo: parameter
				{
					// determine better line approximation based on all inliers with linear regression if more than 2 inliers available
					if (currentInlierSet.size() > 2)
					{
		//				std::cout << "line equation before: p1=(" << bestP1.x << ", " << bestP1.y << ")  normal=(" << bestNormal.x << ", " << bestNormal.y << ")" << std::endl;
						cv::Mat A(currentInlierSet.size(), 3, CV_64FC1);
						for (unsigned int i=0; i<currentInlierSet.size(); i++)
						{
							// regression problem: [x y 1] * [a; b; c] = 0
							A.at<double>(i,0) = currentPoints[currentInlierSet[i]].x;
							A.at<double>(i,1) = currentPoints[currentInlierSet[i]].y;
							A.at<double>(i,2) = 1.;
						}
						cv::SVD svd(A);
		//				std::cout << "u,w,v: (" << svd.u.rows << ", " << svd.u.cols << "), (" << svd.w.rows << ", " << svd.w.cols << "), (" << svd.vt.rows << ", " << svd.vt.cols << ")\n";
		//				cv::Mat W = cv::Mat::zeros(A.cols,A.cols,CV_64FC1);
		//				for (int i=0; i<svd.w.rows; i++)
		//					W.at<double>(i,i) = svd.w.at<double>(i);
		//				cv::Mat B = svd.u * W * svd.vt;
		//				std::cout << "A\n";
		//				for (int i=0; i<4; i++)
		//					std::cout << A.at<double>(i, 0) << ", " << A.at<double>(i, 1) << ", " << A.at<double>(i, 2) << std::endl;
		//				std::cout << "B\n";
		//				for (int i=0; i<4; i++)
		//					std::cout << B.at<double>(i, 0) << ", " << B.at<double>(i, 1) << ", " << B.at<double>(i, 2) << std::endl;
		//				std::cout << "vt\n";
		//				for (int i=0; i<svd.vt.rows; i++)
		//					std::cout << svd.vt.at<double>(i, 0) << ", " << svd.vt.at<double>(i, 1) << ", " << svd.vt.at<double>(i, 2) << ", " << svd.vt.at<double>(i, 3) << std::endl;

						// compute normal and point from [a;b;c]
						if (svd.vt.at<double>(2, 1) != 0.)
						{
							// y = -a/b * x - c/b
							bestNormal.x = svd.vt.at<double>(2, 0) / svd.vt.at<double>(2, 1);
							bestNormal.y = 1.;
							double normMagnitude = sqrt(bestNormal.x*bestNormal.x + bestNormal.y*bestNormal.y);
							bestNormal.x /= normMagnitude;	// normalize normal
							bestNormal.y /= normMagnitude;

							bestP1.x = 0.;
							bestP1.y = -svd.vt.at<double>(2, 2) / svd.vt.at<double>(2, 1);
						}
						else
						{
							// x = -c/a
							bestNormal.x = 1.;
							bestNormal.y = 0.;
							bestP1.x = -svd.vt.at<double>(2, 2) / svd.vt.at<double>(2, 0);
							bestP1.y = 0;
						}

		//				std::cout << "line equation after: p1=(" << bestP1.x << ", " << bestP1.y << ")  normal=(" << bestNormal.x << ", " << bestNormal.y << ")\n\n" << std::endl;
					}

					// retrieve bounding box of inlier set
					TextRegion textRegion;
					textRegion.letters.resize(currentInlierSet.size());
					cv::Point minPoint(100000, 100000), maxPoint(0, 0);
					double qualityScore = 0.;
					for (unsigned int i=0; i<currentInlierSet.size(); i++)
					{
						const cv::Rect& letterBox = currentLetterBoxes[currentInlierSet[i]];
						if (minPoint.x > letterBox.x)
							minPoint.x = letterBox.x;
						if (minPoint.y > letterBox.y)
							minPoint.y = letterBox.y;
						if (maxPoint.x < letterBox.x+letterBox.width)
							maxPoint.x = letterBox.x+letterBox.width;
						if (maxPoint.y < letterBox.y+letterBox.height)
							maxPoint.y = letterBox.y+letterBox.height;

						// store letters
						textRegion.letters[i].boundingBox = letterBox;
						textRegion.letters[i].diameter = currentDiameters[currentInlierSet[i]];
						textRegion.letters[i].centerPoint = currentPoints[currentInlierSet[i]];
						textRegion.letters[i].fontColor = textRegions[textRegionIndex].letters[0].fontColor;

						// quality score
						qualityScore += std::max(0., std::min(1., (currentDiameters[currentInlierSet[i]]-4*pointLineDistance2D(cv::Point2d(bestP1.x, bestP1.y), cv::Point2d(bestNormal.x, bestNormal.y), cv::Point2d(letterBox.x+0.5*letterBox.width, letterBox.y+0.5*letterBox.height))/currentDiameters[currentInlierSet[i]])));
						//sumLetterSize += currentDiameters[currentInlierSet[i]];
					}
					//qualityScore = std::min(10., 10.*qualityScore/sumLetterSize);	// qualityScore and sumLetterSize need to be normalized by letter count but both cancel here
					qualityScore += 0.5*textRegion.letters.size();
					textRegion.boundingBox = cv::Rect(minPoint, maxPoint);
					textRegion.lineEquation = cv::RotatedRect(cv::Point2f(bestP1.x, bestP1.y), cv::Size2f(bestNormal.x, bestNormal.y), qualityScore);
					textRegion.qualityScore = textRegion.boundingBox.width;//qualityScore;
					splitUpTextRegions.push_back(textRegion);

					// debug
					if (debug["showBreakLines"])
					{
						cv::Mat output = originalImage_.clone();
						for (unsigned int i=0; i<currentInlierSet.size(); i++)
							cv::rectangle(output, cv::Point(currentLetterBoxes[currentInlierSet[i]].x, currentLetterBoxes[currentInlierSet[i]].y), cv::Point(currentLetterBoxes[currentInlierSet[i]].x+currentLetterBoxes[currentInlierSet[i]].width, currentLetterBoxes[currentInlierSet[i]].y+currentLetterBoxes[currentInlierSet[i]].height), cv::Scalar(0,0,255), 2, 1, 0);
						cv::rectangle(output, cv::Point(minPoint.x, minPoint.y), cv::Point(maxPoint.x, maxPoint.y), cv::Scalar(0,255,0), 2, 1, 0);
						cv::imshow("breaking lines", output);
						cvMoveWindow("breaking lines", 0, 0);
						cv::waitKey(0);
					}
				}
			}

			// remove inlier set from currentPoints and currentLetterBoxes
			std::vector<int> deleteList;
			for (unsigned int subSetIndex=0; subSetIndex<inlierSubSets.size(); subSetIndex++)
				for (unsigned int i=0; i<inlierSubSets[subSetIndex].size(); i++)
					deleteList.push_back(inlierSubSets[subSetIndex][i]);
			std::sort(deleteList.begin(), deleteList.end(), std::greater<int>());
			for (unsigned int i=0; i<deleteList.size(); i++)
			{
				currentPoints.erase(currentPoints.begin()+deleteList[i]);
				currentLetterBoxes.erase(currentLetterBoxes.begin()+deleteList[i]);
				currentDiameters.erase(currentDiameters.begin()+deleteList[i]);
			}
		}
	}

	// return the set of divided boxes
	textRegions = splitUpTextRegions;
}

bool DetectText::spatialOrderX(cv::Rect a, cv::Rect b)
{
	return a.x < b.x;
}

void DetectText::disposal()
{
	labeledRegions_.clear();
	isLetterRegion_.clear();
	meanRGB_.clear();
	meanBgRGB_.clear();
	letterGroups_.clear();
	//textRegions_.clear();
}

void DetectText::deleteDoubleBrokenWords(std::vector<cv::Rect>& boundingBoxes)
{
	unsigned int count = 0;

	// std::set<int, std::greater<int> > deleteList;
	for (unsigned int j = 0; j < (unsigned int) boundingBoxes.size(); j++)
		for (int k = j + 1; k < (int) boundingBoxes.size(); k++)
		{
			if ((((boundingBoxes[j] & boundingBoxes[k]).area()) / ((float) boundingBoxes[k].area())) > 0.9 || (((boundingBoxes[j] & boundingBoxes[k]).area())
					/ ((float) boundingBoxes[j].area())) > 0.9)
			{
				unsigned int minX, maxX, minY, maxY;
				minX = std::min(boundingBoxes[j].x, boundingBoxes[k].x);
				minY = std::min(boundingBoxes[j].y, boundingBoxes[k].y);
				maxX = std::max(boundingBoxes[j].x + boundingBoxes[j].width, boundingBoxes[k].x + boundingBoxes[k].width);
				maxY = std::max(boundingBoxes[j].y + boundingBoxes[j].height, boundingBoxes[k].y + boundingBoxes[k].height);

				boundingBoxes.erase(boundingBoxes.begin() + j);
				boundingBoxes.erase(boundingBoxes.begin() + k - 1);
				boundingBoxes.push_back(cv::Rect(minX, minY, maxX - minX, maxY - minY));

				j = -1;
				k = 0;
				count++;
				break;
			}
		}

	if (count > 0)
		std::cout << "Deleted " << count << " boundingBoxes that were complete inside of other box." << std::endl;
	//for (std::set<int, std::greater<int> >::iterator it = deleteList.begin(); it != deleteList.end(); it++)
	//  boundingBoxes.erase(boundingBoxes.begin() + *it);
}

void DetectText::ocrPreprocess(std::vector<cv::Mat> & images)
{
	for (size_t i = 0; i < images.size(); i++)
	{
		cv::Mat image = images[i].clone();

		// scale image
		cv::Mat scaledImage;
		cv::resize(image, scaledImage, cv::Size(images[i].cols * 2, images[i].rows * 2), 0, 0, cv::INTER_LINEAR);
		image = scaledImage;

		// sharpen image
		cv::Mat sharpenedImage = sharpenImage(image);
		image = sharpenedImage;

		// blur image
		//    cv::Mat blurredImage;
		//    cv::GaussianBlur(image, blurredImage, cv::Size(), 1, 1);
		//    image = blurredImage;

		// add colored border
		//    FontColor f = (i < fontColorIndex_) ? BRIGHT : DARK;
		//    addBorder(image, finalBoundingBoxes_[i], f);

		// binarize
		//    cv::Mat binarizedImage;
		//    if (f == BRIGHT)
		//      binarizedImage = colorBackgroundBinary(notTransformedBoundingBoxes_[i], f, ccmapBright_(finalBoundingBoxes_[i]));
		//    else
		//      binarizedImage = colorBackgroundBinary(notTransformedBoundingBoxes_[i], f, ccmapDark_(finalBoundingBoxes_[i]));

		images[i] = image;
	}
}

void DetectText::rotate()
{
	int showHistogram = 1;

	// Method 1: Gradients

	// Gradient Histogram

	bgr whiteClr;
	whiteClr.r = 255;
	whiteClr.g = 255;
	whiteClr.b = 255;

	cv::Mat canvas;
	canvas.create(125, 360, CV_8UC3);
	int maxValue = 0, maxAngle = 0, angles = 360;
	int hist[angles], newHistogram[angles];
	double scale;

	for (unsigned int i = 0; i < boundingBoxes_.size(); i++)
	{
		// Show boundingBoxes if necessary

		cv::Mat output = originalImage_.clone();
		cv::rectangle(output, cv::Point(boundingBoxes_.at(i).x, boundingBoxes_.at(i).y),
				cv::Point(boundingBoxes_.at(i).x + boundingBoxes_.at(i).width, boundingBoxes_.at(i).y + boundingBoxes_.at(i).height),
				cv::Scalar(250, 210, 150), 2);

		// Reset Values
		maxValue = 0;
		maxAngle = 0;

		for (int y = 0; y < canvas.rows; y++)
			for (int x = 0; x < canvas.cols; x++)
			{
				canvas.at<bgr> (y, x) = whiteClr;
			}

		for (int j = 0; j < angles - 1; j++)
			hist[j] = 0;

		int count = 0;
		// If there is an edge at (y,x),
		// get the angle[-180,180] at (y,x) and add 180 degrees. (because histogram range is [0,360])
		// Then add one point in the histogram at the found angle.
		for (int y = boundingBoxes_[i].y + (int) (0.0 * boundingBoxes_[i].height); y < boundingBoxes_[i].y + boundingBoxes_[i].height - (int) (0.0
				* boundingBoxes_[i].height); y++)
			for (int x = boundingBoxes_[i].x + (int) (0.0 * boundingBoxes_[i].width); x < boundingBoxes_[i].x + boundingBoxes_[i].width - (int) (0.0
					* boundingBoxes_[i].width); x++)
				if (edgemap_.at<unsigned char> (y, x) == 255)
				{
					if (boxFontColor_[i] == BRIGHT)
					{
						if (ccmapBright_.at<float> (y, x) != -2)
						{
							int angle = (int) ((180 / 3.141592) * theta_.at<float> (y, x)) + 180;
							hist[angle]++;
							count++;
							cv::rectangle(output, cv::Point(x, y), cv::Point(x, y), cv::Scalar(50, 110, 250), 2);
						}
					}
					else
					{
						if (ccmapDark_.at<float> (y, x) != -2)
						{
							int angle = (int) ((180 / 3.141592) * theta_.at<float> (y, x)) + 180;
							hist[angle]++;
							count++;
							cv::rectangle(output, cv::Point(x, y), cv::Point(x, y), cv::Scalar(50, 110, 250), 2);
						}
					}
				}

		cv::imshow("right rectangles", output);
		cv::waitKey(0);

		std::cout << "count: " << count << std::endl;
		if (count < 100)
			continue;

		// Smoothing the histogram
		double mask[3];
		mask[0] = 0.25;
		mask[1] = 0.5;
		mask[2] = 0.25;

		for (int bin = 1; bin < 359; bin++)
		{
			double smoothedValue = 0;

			for (int i = 0; i < 3; i++)
				smoothedValue += hist[bin - 1 + i] * mask[i];

			newHistogram[bin] = smoothedValue;
		}
		newHistogram[0] = hist[0] * (2 / 3) + hist[1] * (1 / 3);
		newHistogram[359] = hist[358] * (1 / 3) + hist[359] * (2 / 3);

		for (int bin = 1; bin < 360; bin++)
		{
			hist[bin] = newHistogram[bin];
		}

		// Get maxValue and max angle
		for (int j = 0; j < angles - 1; j++)
			maxValue = hist[j] > maxValue ? hist[j] : maxValue;

		for (int j = 0; j < angles - 1; j++)
			if (maxValue == hist[j])
				maxAngle = j;

		// Fit histogram to the canvas height
		scale = (double) canvas.rows / maxValue;

		//Draw histogram
		if (showHistogram)
		{
			for (int j = 0; j < angles - 1; j++)
			{
				cv::Point pt1(j, canvas.rows - (hist[j] * scale));
				cv::Point pt2(j, canvas.rows);
				cv::line(canvas, pt1, pt2, cv::Scalar(250, 210, 150), 1, 8, 0);
			}
			cv::putText(canvas, "0", cv::Point(180, 122), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0, 0), 1.5, 8, false);
			cv::putText(canvas, "-90", cv::Point(90, 122), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0, 0), 1.5, 8, false);
			cv::putText(canvas, "-180", cv::Point(0, 122), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0, 0), 1.5, 8, false);
			cv::putText(canvas, "90", cv::Point(260, 122), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0, 0), 1.5, 8, false);
			cv::putText(canvas, "180", cv::Point(335, 122), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0, 0), 1.5, 8, false);
			cv::imshow("Gradients", canvas);
			std::cout << "maxAngle:" << maxAngle << "(" << maxAngle - 180 << ")" << std::endl;
			cv::waitKey(0);
		}

		/*
		 // Find border color

		 cv::Mat smallImg = originalImage_(boundingBoxes_[i]);
		 std::vector<bgr> bgColor;

		 // Average of all rectangle border Pixels

		 for (int y = 0; y < smallImg.rows; y++)
		 {
		 for (int x = 0; x < smallImg.cols; x++)
		 bgColor.push_back(smallImg.at<bgr> (y, x));

		 if (y == 0)
		 y = smallImg.rows - 2;
		 }

		 for (int x = 0; x < smallImg.cols; x++)
		 {
		 for (int y = 0; y < smallImg.rows; y++)
		 bgColor.push_back(smallImg.at<bgr> (y, x));

		 if (x == 0)
		 x = smallImg.cols - 2;
		 }

		 unsigned int allGreen = 0, allRed = 0, allBlue = 0, meanGreen = 0, meanRed = 0, meanBlue = 0;
		 for (unsigned int bgrIndex = 0; bgrIndex < bgColor.size(); bgrIndex++)
		 {
		 allRed += bgColor[bgrIndex].r;
		 allGreen += bgColor[bgrIndex].g;
		 allBlue += bgColor[bgrIndex].b;
		 }

		 meanGreen = (unsigned int)(allGreen / (float)bgColor.size());
		 meanBlue = (unsigned int)(allBlue / (float)bgColor.size());
		 meanRed = (unsigned int)(allRed / (float)bgColor.size());

		 // Average of border pixels that belong to background

		 bgColor.clear();

		 for (int y = 0; y < smallImg.rows; y++)
		 {
		 for (int x = 0; x < smallImg.cols; x++)
		 {
		 if (boxFontColor_[i] == BRIGHT) //bright text -> background pixels are dark -> to get average border color only take dark pixels
		 {
		 if (smallImg.at<bgr> (y, x).r < meanRed * 1.25 && smallImg.at<bgr> (y, x).g < meanGreen * 1.25
		 && smallImg.at<bgr> (y, x).b < meanBlue * 1.25) // tolerance 25%
		 bgColor.push_back(smallImg.at<bgr> (y, x));
		 }
		 else // dark font -> pixel has to be brighter / bigger than meanColor of border
		 {
		 if (smallImg.at<bgr> (y, x).r > meanRed * 0.75 && smallImg.at<bgr> (y, x).g > meanGreen * 0.75
		 && smallImg.at<bgr> (y, x).b > meanBlue * 0.75)
		 bgColor.push_back(smallImg.at<bgr> (y, x));
		 }
		 }
		 if (y == 0)
		 y = smallImg.rows - 2;
		 }
		 for (int x = 0; x < smallImg.cols; x++)
		 {
		 for (int y = 0; y < smallImg.rows; y++)
		 {
		 if (boxFontColor_[i] == 1)
		 {
		 if (smallImg.at<bgr> (y, x).r < meanRed * 1.25 && smallImg.at<bgr> (y, x).g < meanGreen * 1.25
		 && smallImg.at<bgr> (y, x).b < meanBlue * 1.25)
		 bgColor.push_back(smallImg.at<bgr> (y, x));
		 }
		 else
		 {
		 if (smallImg.at<bgr> (y, x).r > meanRed * 0.75 && smallImg.at<bgr> (y, x).g > meanGreen * 0.75
		 && smallImg.at<bgr> (y, x).b > meanBlue * 0.75)
		 bgColor.push_back(smallImg.at<bgr> (y, x));
		 }
		 }
		 if (x == 0)
		 x = smallImg.cols - 2;
		 }

		 allRed = 0;
		 allGreen = 0;
		 allBlue = 0;
		 for (unsigned int bgrIndex = 0; bgrIndex < bgColor.size(); bgrIndex++)
		 {
		 allRed += bgColor[bgrIndex].r;
		 allGreen += bgColor[bgrIndex].g;
		 allBlue += bgColor[bgrIndex].b;
		 }
		 meanGreen = (unsigned int)(allGreen / (float)bgColor.size());
		 meanBlue = (unsigned int)(allBlue / (float)bgColor.size());
		 meanRed = (unsigned int)(allRed / (float)bgColor.size());

		 cv::Scalar average_bg(meanBlue, meanGreen, meanRed);

		 */
		cv::Mat smallImg = originalImage_(boundingBoxes_[i]);
		cv::Scalar average_bg; // = findBorderColor(smallImg, boxFontColor_[i]);

		// Do the Rotation

		cv::Mat rotatedImage;

		// copy smallImg inside bigger image to have enough space for rotation
		cv::Mat img1(originalImage_(boundingBoxes_[i]).rows * 10, originalImage_(boundingBoxes_[i]).cols * 10, originalImage_(boundingBoxes_[i]).type(),
				average_bg);
		cv::Mat tmp = img1(
				cv::Rect(originalImage_(boundingBoxes_[i]).cols * 4, originalImage_(boundingBoxes_[i]).rows * 4, originalImage_(boundingBoxes_[i]).cols,
						originalImage_(boundingBoxes_[i]).rows));
		originalImage_(boundingBoxes_[i]).copyTo(tmp);
		cv::Mat toBeRotated = img1.clone();

		cv::Point2f center;
		center.x = 0.5 * toBeRotated.cols;
		center.y = 0.5 * toBeRotated.rows;

		// get the rotation matrix
		cv::Mat mapMatrix = cv::getRotationMatrix2D(center, maxAngle - 180, 1.0);

		// rotate
		cv::warpAffine(toBeRotated, rotatedImage, mapMatrix, cv::Size(toBeRotated.cols, toBeRotated.rows), cv::INTER_CUBIC, cv::BORDER_CONSTANT,
				cv::Scalar(average_bg));

		cv::imshow("rotated", rotatedImage);
		cv::waitKey(0);

		// Method 2) binary rotation
		// -------------------------

		cv::Mat binary = originalImage_(boundingBoxes_[i]).clone();

		// how many font pixel in each row
		cv::Mat binaryCanvasPixelEachRow(125, binary.rows, CV_8UC1);
		// how many pixels till first font pixel from left appears
		cv::Mat binaryCanvasFirstPixelFromLeft(125, binary.rows, CV_8UC1);

		// reset all
		for (int y = 0; y < binaryCanvasPixelEachRow.rows; y++)
			for (int x = 0; x < binaryCanvasPixelEachRow.cols; x++)
			{
				binaryCanvasPixelEachRow.at<unsigned char> (y, x) = 255;
				binaryCanvasFirstPixelFromLeft.at<unsigned char> (y, x) = 255;
			}
		unsigned int allPixelsNumber = 0;
		int binaryMaxValue = 0; // how many pixels are in row with most pixels
		int binaryMaxRow = 0; // in which row are the most pixels
		int binaryBin = binary.rows;

		int binaryHistEachRow[binaryBin];
		int binaryHistFirstLeft[binaryBin];
		int binaryHistEachRowSmooth[binaryBin]; // for smoothing
		int binaryHistFirstLeftSmooth[binaryBin]; // for smoothing

		for (int binIndex = 0; binIndex < binaryBin; binIndex++)
		{
			binaryHistEachRow[binIndex] = 0;
			binaryHistFirstLeft[binIndex] = 0;
			binaryHistEachRowSmooth[binIndex] = 0;
			binaryHistFirstLeftSmooth[binIndex] = 0;
		}

		if (boxFontColor_[i] == BRIGHT)
		{
			binary = colorBackgroundBinary(binary, BRIGHT, ccmapBright_(boundingBoxes_[i])); // get binary image
			for (int y = 0; y < binary.rows; y++)
			{
				for (int x = 0; x < binary.cols; x++)
				{
					if (binary.at<bgr> (y, x).b == 255 && binary.at<bgr> (y, x).g == 255 && binary.at<bgr> (y, x).r == 255)
					{
						allPixelsNumber++;
						binaryHistEachRow[y]++; // count font pixels in each row
						if (binaryHistFirstLeft[y] == 0)
							binaryHistFirstLeft[y] = x; // remember x location when first font pixel from left appears
					}
					if (x == binary.cols - 1 && binaryHistFirstLeft[y] == 0)
						binaryHistFirstLeft[y] = x; // set x location to maximum (binary.cols-1) in case there was no pixel in this row at all
				}
			}
		}
		else // dark font
		{
			binary = colorBackgroundBinary(binary, DARK, ccmapDark_(boundingBoxes_[i]));
			for (int y = 0; y < binary.rows; y++)
			{
				for (int x = 0; x < binary.cols; x++)
				{
					if (binary.at<bgr> (y, x).b == 0 && binary.at<bgr> (y, x).g == 0 && binary.at<bgr> (y, x).r == 0)
					{
						allPixelsNumber++;
						binaryHistEachRow[y]++;
						if (binaryHistFirstLeft[y] == 0)
							binaryHistFirstLeft[y] = x;
					}
					if (x == binary.cols - 1 && binaryHistFirstLeft[y] == 0)
						binaryHistFirstLeft[y] = x;
				}
			}
		}

		// Smooth both histograms

		for (int bin = 1; bin < binaryBin; bin++)
		{
			double smoothedValue = 0, smoothedValue2 = 0;

			for (int i = 0; i < 3; i++)
			{
				smoothedValue += binaryHistEachRow[bin - 1 + i] * mask[i];
				smoothedValue2 += binaryHistFirstLeft[bin - 1 + i] * mask[i];
			}

			binaryHistEachRowSmooth[bin] = smoothedValue;
			binaryHistFirstLeftSmooth[bin] = smoothedValue2;
		}
		binaryHistEachRowSmooth[0] = binaryHistEachRow[0] * (float) (2 / 3) + binaryHistEachRow[1] * (float) (1 / 3);
		binaryHistEachRowSmooth[359] = binaryHistEachRow[358] * (float) (1 / 3) + binaryHistEachRow[359] * (float) (2 / 3);
		binaryHistFirstLeftSmooth[0] = binaryHistFirstLeft[0] * (float) (2 / 3) + binaryHistFirstLeft[1] * (float) (1 / 3);
		binaryHistFirstLeftSmooth[359] = binaryHistFirstLeft[358] * (float) (2 / 3) + binaryHistFirstLeft[359] * (float) (1 / 3);

		for (int bin = 1; bin < 360; bin++)
		{
			binaryHistEachRow[bin] = binaryHistEachRowSmooth[bin];
			binaryHistFirstLeft[bin] = binaryHistFirstLeftSmooth[bin];
		}

		cv::imwrite("/home/rmb-rh/bin0.png", binary);

		// Get maxValue and max angle
		for (int j = 0; j < binaryBin - 1; j++)
			binaryMaxValue = binaryHistEachRow[j] > binaryMaxValue ? binaryHistEachRow[j] : binaryMaxValue;

		for (int j = 0; j < binaryBin - 1; j++)
			if (binaryMaxValue == binaryHistEachRow[j])
				binaryMaxRow = j;

		// Fit histogram to the canvas height
		double binaryScale = (double) binaryCanvasPixelEachRow.rows / binaryMaxValue;

		//Draw histogram
		if (showHistogram)
		{
			for (int j = 0; j < binaryBin - 1; j++)
			{
				cv::Point pt1(j, binaryCanvasPixelEachRow.rows - (binaryHistEachRow[j] * binaryScale));
				cv::Point pt2(j, binaryCanvasPixelEachRow.rows);
				cv::line(binaryCanvasPixelEachRow, pt1, pt2, cv::Scalar(0), 1, 8, 0);
			}
			cv::imshow("binaryCanvas", binaryCanvasPixelEachRow);
			cv::imshow("bin0", binary);

		}

		binaryMaxValue = 0;
		for (int j = 0; j < binaryBin - 1; j++)
			binaryMaxValue = binaryHistFirstLeft[j] > binaryMaxValue ? binaryHistFirstLeft[j] : binaryMaxValue;

		binaryScale = (double) binaryCanvasFirstPixelFromLeft.rows / binaryMaxValue;

		if (showHistogram)
		{
			for (int j = 0; j < binaryBin - 1; j++)
			{
				cv::Point pt1(j, binaryCanvasFirstPixelFromLeft.rows - (binaryHistFirstLeft[j] * binaryScale));
				cv::Point pt2(j, binaryCanvasFirstPixelFromLeft.rows);
				cv::line(binaryCanvasFirstPixelFromLeft, pt1, pt2, cv::Scalar(0), 1, 8, 0);
			}
			cv::imshow("binaryCanvasFirstLeft", binaryCanvasFirstPixelFromLeft);
			cv::imwrite("/home/rmb-rh/binaryCanvas.png", binaryCanvasPixelEachRow);
			cv::imwrite("/home/rmb-rh/binaryCanvasFromLeft.png", binaryCanvasFirstPixelFromLeft);
			cv::waitKey(0);

			//   cv::imshow("bin1", binaryCanvasFirstPixelFromLeft);
			//  cv::imwrite("/home/rmb-rh/binaryCanvas.png", binaryCanvas);
		}

	}
	// Linear Regression
	//    std::vector<cv::Point> currentPoints;
	//    float averageX = 0, averageY = 0, sx = 0, sy = 0, sxy = 0, rxy;
	//
	//    for (unsigned int i = 1; i < currentPoints.size(); i++)
	//    {
	//      currentPoints[i].y = 0;
	//    }
	//    for (unsigned int i = 1; i < currentPoints.size(); i++)
	//    {
	//      averageX += currentPoints[i].x;
	//      averageY += currentPoints[i].y;
	//    }
	//    averageX /= (float)currentPoints.size();
	//    averageY /= (float)currentPoints.size();
	//    for (unsigned int i = 1; i < currentPoints.size(); i++)
	//    {
	//      sx += (currentPoints[i].x - averageX) * (currentPoints[i].x - averageX);
	//      sy += (currentPoints[i].y - averageY) * (currentPoints[i].y - averageY);
	//      sxy += (currentPoints[i].y - averageY) * (currentPoints[i].x - averageX);
	//    }
	//    sx = std::sqrt(sx);
	//    sy = std::sqrt(sy);
	//    sxy /= (float)currentPoints.size();
	//
	//    rxy = sxy / (float)(sx * sy);
	//
	//    float byx = rxy * (sy / (float)sx);
	//    float ayx = averageY - byx * averageX;
	//
	//    std::cout << "GLEICHUNG: " << ayx << " + " << byx << "x" << std::endl;
	//    std::cout << "BESTIMMTHEITSMAß: " << rxy * rxy << std::endl;

	//------------------------------------


}

cv::Mat DetectText::colorBackgroundBinary(cv::Mat & m, FontColor f, cv::Mat cc)
{
	bool erodeDilate = false;
	cv::Mat clonedMat = m.clone();

	bgr white(255, 255, 255), black(0, 0, 0);

	for (int y = 0; y < cc.rows; y++)
	{
		for (int x = 0; x < cc.cols; x++)
		{
			int component = static_cast<int> (cc.at<float> (y, x));

			if (f == BRIGHT)
			{
				if (component == -2)
				{
					clonedMat.at<bgr> (y, x) = black;
					// (clonedMat.at<bgr> (y, x)).b /= 1.2;
					// (clonedMat.at<bgr> (y, x)).g /= 1.2;
					// (clonedMat.at<bgr> (y, x)).r /= 1.2;
				}
				else
				{
					clonedMat.at<bgr> (y, x) = white;

					//(clonedMat.at<bgr> (y, x)).b *= 1.2;
					//(clonedMat.at<bgr> (y, x)).g *= 1.2;
					//(clonedMat.at<bgr> (y, x)).r *= 1.2;
				}
			}
			else
			{
				if (component == -2)
				{
					clonedMat.at<bgr> (y, x) = white;
					//  (clonedMat.at<bgr> (y, x)).b *= 1.2;
					//  (clonedMat.at<bgr> (y, x)).g *= 1.2;
					//          (clonedMat.at<bgr> (y, x)).r *= 1.2;
				}
				else
				{
					clonedMat.at<bgr> (y, x) = black;
					//    (clonedMat.at<bgr> (y, x)).b /= 1.2;
					//   (clonedMat.at<bgr> (y, x)).g /= 1.2;
					//   (clonedMat.at<bgr> (y, x)).r /= 1.2;
				}
			}
		}
	}

	if (erodeDilate)
	{
		if (f == DARK)
		{
			cv::dilate(clonedMat, clonedMat, cv::Mat());
			//  cv::erode(clonedMat, clonedMat, cv::Mat());
		}
		else
		{
			cv::erode(clonedMat, clonedMat, cv::Mat());
			//  cv::dilate(clonedMat, clonedMat, cv::Mat());
		}
	}

	return clonedMat;
}

cv::Mat DetectText::sharpenImage(cv::Mat input)
{
	cv::Mat blurred;
	cv::GaussianBlur(input, blurred, cv::Size(), sigma_sharp, sigma_sharp);

	cv::Mat lowContrastMask = cv::abs(input - blurred) < threshold_sharp;
	cv::Mat sharpened = input * (1 + amount_sharp) + blurred * (-amount_sharp);
	input.copyTo(sharpened, lowContrastMask);
	return sharpened;
}

void DetectText::addBorder(cv::Mat & image, cv::Rect r, FontColor f)
{
	cv::Mat imageWithBorder;
	bgr clr = findBorderColor(r, f);

	cv::Mat img(image.rows + 10, image.cols + 10, image.type(), cv::Scalar(clr.r, clr.g, clr.b));
	cv::Mat tmp = img(cv::Rect(5, 5, image.cols, image.rows));
	image.copyTo(tmp);
	imageWithBorder = img.clone();

	image = imageWithBorder;
}

cv::Mat DetectText::binarizeViaContrast(cv::Mat input)
{
	cv::Mat grayImage(input.size(), CV_8UC1, cv::Scalar(0));
	cv::cvtColor(input, grayImage, CV_RGB2GRAY);
	cv::Mat output = grayImage.clone();

	unsigned int avgClr = 0;
	for (int y = 0; y < grayImage.rows; y++)
	{
		for (int x = 0; x < grayImage.cols; x++)
		{
			avgClr += grayImage.at<unsigned char> (y, x);
		}
	}

	avgClr /= (grayImage.rows * grayImage.cols);

	std::cout << "avgClr: " << avgClr << std::endl;

	for (int y = 0; y < grayImage.rows; y++)
	{
		for (int x = 0; x < grayImage.cols; x++)
		{
			if (grayImage.at<unsigned char> (y, x) >= avgClr)
				output.at<unsigned char> (y, x) = 255;
			else
				output.at<unsigned char> (y, x) = 0;
		}
	}

	//cv::imshow("Contrast maximum", output);
	//cv::waitKey(0);
	return output;
}

DetectText::bgr DetectText::findBorderColor(cv::Rect r, FontColor f)
{
	int method = 1;

	cv::Mat smallImg = originalImage_(r);
	std::vector<bgr> bgColor;

	unsigned int meanGreen = 0, meanRed = 0, meanBlue = 0;

	// mean color of bright border pixels in case font is dark and vice versa
	if (method == 1)
	{
		for (int y = 0; y < smallImg.rows; y++)
		{
			for (int x = 0; x < smallImg.cols; x++)
				bgColor.push_back(smallImg.at<bgr> (y, x));

			if (y == 0)
				y = smallImg.rows - 2;
		}

		for (int x = 0; x < smallImg.cols; x++)
		{
			for (int y = 0; y < smallImg.rows; y++)
				bgColor.push_back(smallImg.at<bgr> (y, x));

			if (x == 0)
				x = smallImg.cols - 2;
		}

		unsigned int allGreen = 0, allRed = 0, allBlue = 0;
		for (unsigned int bgrIndex = 0; bgrIndex < bgColor.size(); bgrIndex++)
		{
			allRed += bgColor[bgrIndex].r;
			allGreen += bgColor[bgrIndex].g;
			allBlue += bgColor[bgrIndex].b;
		}

		meanGreen = (unsigned int) (allGreen / (float) bgColor.size());
		meanBlue = (unsigned int) (allBlue / (float) bgColor.size());
		meanRed = (unsigned int) (allRed / (float) bgColor.size());

		// -> mean clr of all border pixels
		// now: only those pixels count that are below/above mean clr (depends on font):

		bgColor.clear();

		for (int y = 0; y < smallImg.rows; y++)
		{
			for (int x = 0; x < smallImg.cols; x++)
			{
				if (f == BRIGHT) //bright text -> background pixels are dark -> to get average border color only take dark pixels
				{
					if (smallImg.at<bgr> (y, x).r < meanRed * 1.25 && smallImg.at<bgr> (y, x).g < meanGreen * 1.25 && smallImg.at<bgr> (y, x).b < meanBlue
							* 1.25) // tolerance 25%
						bgColor.push_back(smallImg.at<bgr> (y, x));
				}
				else // dark font -> pixel has to be brighter / bigger than meanColor of border
				{
					if (smallImg.at<bgr> (y, x).r > meanRed * 0.75 && smallImg.at<bgr> (y, x).g > meanGreen * 0.75 && smallImg.at<bgr> (y, x).b > meanBlue
							* 0.75)
						bgColor.push_back(smallImg.at<bgr> (y, x));
				}
			}
			if (y == 0)
				y = smallImg.rows - 2;
		}

		for (int x = 0; x < smallImg.cols; x++)
		{
			for (int y = 0; y < smallImg.rows; y++)
			{
				if (f == 1)
				{
					if (smallImg.at<bgr> (y, x).r < meanRed * 1.25 && smallImg.at<bgr> (y, x).g < meanGreen * 1.25 && smallImg.at<bgr> (y, x).b < meanBlue
							* 1.25)
						bgColor.push_back(smallImg.at<bgr> (y, x));
				}
				else
				{
					if (smallImg.at<bgr> (y, x).r > meanRed * 0.75 && smallImg.at<bgr> (y, x).g > meanGreen * 0.75 && smallImg.at<bgr> (y, x).b > meanBlue
							* 0.75)
						bgColor.push_back(smallImg.at<bgr> (y, x));
				}
			}
			if (x == 0)
				x = smallImg.cols - 2;
		}

		allRed = 0;
		allGreen = 0;
		allBlue = 0;
		for (unsigned int bgrIndex = 0; bgrIndex < bgColor.size(); bgrIndex++)
		{
			allRed += bgColor[bgrIndex].r;
			allGreen += bgColor[bgrIndex].g;
			allBlue += bgColor[bgrIndex].b;
		}
		meanGreen = (unsigned int) (allGreen / (float) bgColor.size());
		meanBlue = (unsigned int) (allBlue / (float) bgColor.size());
		meanRed = (unsigned int) (allRed / (float) bgColor.size());
	}
	else // method 2: mean clr of all border pixels that belong to component on the ccmap
	{
		for (int y = 0; y < smallImg.rows; y++)
		{
			for (int x = 0; x < smallImg.cols; x++)
			{
				int component;
				if (f == BRIGHT)
				{
					component = static_cast<int> (ccmapBright_.at<float> (y + r.y, x + r.x));
					if (component == -2)
						bgColor.push_back(smallImg.at<bgr> (y, x));
				}
				else
				{
					component = static_cast<int> (ccmapDark_.at<float> (y + r.y, x + r.x));
					if (component == -2)
						bgColor.push_back(smallImg.at<bgr> (y, x));
				}
			}
			if (y == 0)
				y = smallImg.rows - 2;
		}

		for (int x = 0; x < smallImg.cols; x++)
		{
			for (int y = 0; y < smallImg.rows; y++)
			{
				int component;
				if (f == BRIGHT)
				{
					component = static_cast<int> (ccmapBright_.at<float> (y + r.y, x + r.x));
					if (component == -2)
						bgColor.push_back(smallImg.at<bgr> (y, x));
				}
				else
				{
					component = static_cast<int> (ccmapDark_.at<float> (y + r.y, x + r.x));
					if (component == -2)
						bgColor.push_back(smallImg.at<bgr> (y, x));
				}
			}
			if (x == 0)
				x = smallImg.cols - 2;
		}

		unsigned int allGreen = 0, allRed = 0, allBlue = 0;
		for (unsigned int bgrIndex = 0; bgrIndex < bgColor.size(); bgrIndex++)
		{
			allRed += bgColor[bgrIndex].r;
			allGreen += bgColor[bgrIndex].g;
			allBlue += bgColor[bgrIndex].b;
		}

		meanGreen = (unsigned int) (allGreen / (float) bgColor.size());
		meanBlue = (unsigned int) (allBlue / (float) bgColor.size());
		meanRed = (unsigned int) (allRed / (float) bgColor.size());
	}

	bgr avg_bg(meanRed, meanGreen, meanBlue);
	return avg_bg;
}

void DetectText::ocrRead(std::vector<cv::Mat> textImages)
{
	int imageVersions; // in case transformed images are used, there is a 180 degrees flipped version
	if (transformImages)
		imageVersions = 2;
	else
		imageVersions = 1;

	std::vector<float> score;
	std::vector<std::string> result;
	for (size_t i = 0; i < textImages.size(); i++)
	{
		//    cv::imshow("roar", textImages[i]);
		//    cv::waitKey(0);

		std::string res;
		score.push_back(ocrRead(textImages[i], res));
		result.push_back(res);

		if ((i + 1) % imageVersions != 0) // collect all different version results before comparing
			continue;

		int smallestElement = i;
		if (transformImages)
			if (score[i - 1] < score[i])
				smallestElement = i - 1;

		if (score[smallestElement] < 100) // if anything was found in the ocr software
		{
			finalTexts_.push_back(result[smallestElement]);
			finalScores_.push_back(score[smallestElement]);

			if (transformImages)
				finalBoxes_.push_back(finalRotatedBoundingBoxes_[(std::floor(smallestElement / (float) 2))]);
			else
				finalBoxes_.push_back(finalRotatedBoundingBoxes_[smallestElement]);

		}
		//      else if(debug["showAllBoxes"])
		//      {
		//        finalTexts_.push_back("---");
		//        finalBoxes_.push_back(finalBoundingBoxes_[(std::floor(smallestElement / (float)3))]);
		//        finalScores_.push_back(score[smallestElement]);
		//      }

		std::cout << "size of finalTexts_:" << finalTexts_.size() << std::endl << "-------------------" << std::endl;
		std::cout << "size of finalBoxes_:" << finalBoxes_.size() << std::endl << "-------------------" << std::endl;
	}
}

float DetectText::ocrRead(const cv::Mat& image, std::string& output)
{
	float score = 0;
	cv::imwrite("patch.tiff", image);

	int result;

	std::string cmd = ros::package::getPath("cob_tesseract") + "/bin/tesseract patch.tiff patch -psm 7 -l deu letters"; // before: psm -8
	result = system(cmd.c_str());

	assert(!result);
	std::ifstream fin("patch.txt");
	std::string str;

	int loopCount = 0;
	while (fin >> str)
	{
		std::cout << str << " ";
		std::string tempOutput;
		score += spellCheck(str, tempOutput, 2);
		std::cout << " -->  \"" << tempOutput.substr(0, tempOutput.length() - 1) << "\" , score: " << score << std::endl;
		output += tempOutput;
		loopCount++;
	}

	score /= loopCount;

	if (output.size() == 0)
		score = 100;

	result = system("$(rm patch.txt patch.tiff)");
	return score;
}

float DetectText::spellCheck(std::string& str, std::string& output, int method)
{
	// two option: 1 for aspell, 2 for correlation edit distance
	// return the score for the input
	std::string withoutStrangeMarks;
	float score = 0;

	// Trim Both leading and trailing spaces
	str = trim(str);

	// using tessdata/configs/letters.txt -> no 'strange' letters appear anymore
	//  for (size_t i = 0; i < str.length(); i++) // first replace all letters that are 'strange' with corresponding characters
	//  {
	//    if (std::isupper(str[i]) || std::islower(str[i]) || std::isdigit(str[i]))
	//      withoutStrangeMarks += str[i];
	//    else if (str[i] == '|' || str[i] == '/' || str[i] == '\\')
	//    {
	//      if ((i && std::isdigit(str[i - 1])) || ((i < str.length() - 1) && std::isdigit(str[i + 1]))) // is there a digit following or before the actual letter?
	//      {
	//        withoutStrangeMarks += '1'; //change to an '1'
	//        str[i] = '1';
	//      }
	//      else
	//      {
	//        withoutStrangeMarks += 'l'; //else think of the letter as a 'l'
	//        str[i] = 'l';
	//      }
	//    }
	//    else if (str[i] == '[')
	//    {
	//      withoutStrangeMarks += 'L';
	//    }
	//    else if (str[i] == ']')
	//    {
	//      withoutStrangeMarks += 'I';
	//    }
	//    else
	//    {
	//      str[i] = '-';
	//    }
	//  }

	withoutStrangeMarks = str;

	if (method == 1) // not so good
	{
		const std::string command("echo " + withoutStrangeMarks + " | aspell -a >> output");
		int r = system(command.c_str());
		std::fstream fin("output");
		std::string result;
		int count = 0;

		while (fin >> result)
		{
			if (count)
			{
				count++;
				if (count >= 5)
				{
					output += result + " ";
				}
				if (count == 10)
				{
					if ((output)[output.length() - 2] == ',')
						((output)[output.length() - 2] = ' ');
					break;
				}
			}
			if (result[0] == '&')
			{
				count++;
				output += "{";
			}
			else if (result[0] == '*')
			{
				output += " " + str;
				break;
			}
		}
		if (count)
			output += "}";
		r = system("rm output");
	}

	// dictionary search
	if (method == 2)
	{
		std::vector<Word> topk;
		std::string nearestWord;
		getTopkWords(withoutStrangeMarks, 3, topk); // find k dictionary words that fit best to string
		output = topk[0].word + " ";
		//    if (result_ == 1)
		//    {
		//      std::string topWord = topk[0].word;
		//      output = topk[0].word + " ";
		/*
		 if (topWord.length() < 3)
		 {
		 if (topk[0].score == 0)
		 score++;
		 else
		 output = "";
		 }
		 else if (topWord.length() < 6)
		 {
		 if (topk[0].score * 5 <= topWord.length())
		 score++;
		 else
		 output = "";
		 }
		 else
		 {
		 if (topk[0].score == 0)
		 score = topWord.length() * 2;
		 else if (topk[0].score <= topWord.length())
		 score = topWord.length();
		 }*/
		score++; //always give out everything
		// }
		//    else if (result_ == 2)
		//    {
		//      if (topk[0].score == 0)
		//      {
		//        output = topk[0].word + " ";
		//        score += topk[0].word.length() * 2;
		//      }
		//      else
		//      {
		//        output = "{" + withoutStrangeMarks + "->";
		//        // pick top 3 results
		////        for (int i = 0; i < 3; i++)
		////        {
		////          std::stringstream ss;
		////          ss << topk[i].score;
		////          std::string s = ss.str();
		////          output = output + topk[i].word + ":" + s + " ";
		////        }
		//        output += "} ";
		//      }
		//    }
		if (topk[0].word.length() < 3)
			score = 100;
		else
			score = topk[0].score / (float) topk[0].word.length();
	}
	return score;
}

std::string& DetectText::trim(std::string& str)
{
	// Trim Both leading and trailing spaces

	// Find the first character position after
	// excluding leading blank spaces
	size_t startpos = str.find_first_not_of(" \t");
	// Find the first character position from reverse af
	size_t endpos = str.find_last_not_of(" \t");
	// if all spaces or empty return an empty std::string
	if ((std::string::npos == startpos) || (std::string::npos == endpos))
		str = "";
	else
		str = str.substr(startpos, endpos - startpos + 1);
	return str;
}

void DetectText::getTopkWords(const std::string& str, const int k, std::vector<Word>& words) //k=3
{
	bool correlationUsage = true;
	float score, lowestScore = 100;
	words.clear();
	words.resize(k);
	for (size_t i = 0; i < wordList_.size(); i++)
	{
		if (correlationUsage)
			score = editDistanceFont(str, wordList_[i]); //compare every word in dictionary, score=0 -> perfect
		else
			score = editDistance(str, wordList_[i]); //compare every word in dictionary, score=0 -> perfect


		if (score < lowestScore)
		{
			Word w = Word(wordList_[i], score);
			lowestScore = insertToList(words, w);
		}
	}
}

int DetectText::editDistance(const std::string& s, const std::string& t)
{
	int n = s.length();
	int m = t.length();

	if (n == 0)
		return m;
	if (m == 0)
		return n;

	int **d = new int*[n + 1];
	for (int i = 0; i < n + 1; i++)
	{
		d[i] = new int[m + 1];
		memset(d[i], 0, (m + 1) * sizeof(int));
	}

	for (int i = 0; i < n + 1; i++)
		d[i][0] = i;
	for (int j = 0; j < m + 1; j++)
		d[0][j] = j;

	for (int i = 1; i < n + 1; i++)
	{
		char sc = s[i - 1];
		for (int j = 1; j < m + 1; j++)
		{
			int v = d[i - 1][j - 1];
			if (t[j - 1] != sc)
				v++;
			d[i][j] = std::min(std::min(d[i - 1][j] + 1, d[i][j - 1] + 1), v);
		}
	}
	return d[n][m];
}

float DetectText::editDistanceFont(const std::string& word, const std::string& dictionaryWord)
{
	// Example
	// word = 'Jodsalz', dictionaryWord = 'Abendhimmel'
	// d looks like this:
	// 0 1 2 3 4 5 6 7
	// 1 0 0 0 0 0 0 0
	// 2 0 0 0 0 0 0 0
	// 3 0 0 0 0 0 0 0
	// 4 0 0 0 0 0 0 0
	// 5 0 0 0 0 0 0 0
	// 6 0 0 0 0 0 0 0
	// 7 0 0 0 0 0 0 0
	// 8 0 0 0 0 0 0 0
	// 9 0 0 0 0 0 0 0
	//10 0 0 0 0 0 0 0
	//11 0 0 0 0 0 0 0
	//
	// All letters are compared with each other and a extra value is added the more
	// far away two letters are (regarding their position).
	//
	// e.g.:
	// correlate = correlation_.at<float> (getCorrelationIndex('J'), getCorrelationIndex('A');
	// correlate = 0.28456;
	// v = 0 + 1 - 0.28456 = 0.71544;
	// d[1][1] = min(1 + penalty, 1 + penalty, v);
	//
	// 0 1 2 3 4 5 6 7
	// 1 0.7 0 0 0 0 0 0
	// 2 0 0 0 0 0 0 0
	// …
	//
	// next letter combination: correlate = ... ('J'), getCorrelationIndex('b');
	// v = 1 + 1 - ...
	//     A
	//     |
	//     |
	//   because letters are not at same position ('J' is first, 'b' second) v grows bigger.
	//
	// with two identical words, the diagonal line of d is 0 (d[0][0]=0,d[1][1]=0,d[2][2]...). At the end the last element of d is taken as score result.


	float penalty = 0.7;
	int wordLength = word.length();
	int dictionaryWordLength = dictionaryWord.length();

	if (wordLength == 0)
		return dictionaryWordLength;
	if (dictionaryWordLength == 0)
		return wordLength;

	std::vector<std::vector<float> > d(wordLength + 1, std::vector<float>(dictionaryWordLength + 1));

	for (int i = 0; i < wordLength + 1; i++)
		for (int j = 0; j < dictionaryWordLength + 1; j++)
			d[i][j] = 0;

	for (int i = 0; i < wordLength + 1; i++)
		d[i][0] = i;

	for (int j = 0; j < dictionaryWordLength + 1; j++)
		d[0][j] = j;

	for (int i = 1; i < wordLength + 1; i++)
	{
		//char sc = word[i - 1];
		std::string sc = word.substr(i - 1, 1);
		for (int j = 1; j < dictionaryWordLength + 1; j++)
		{
			float v = d[i - 1][j - 1];
			//if ((dictionaryWord[j - 1] != sc))
			if ((dictionaryWord.substr(j - 1, 1).compare(sc) != 0))
			{
				float correlate;
				if (sc.compare("-") != 0)
				{
					int a = getCorrelationIndex(dictionaryWord.substr(j - 1, 1));//dictionaryWord[j - 1]);
					int b = getCorrelationIndex(sc);
					if (a == -1 || b == -1)
						correlate = 0;
					else
						correlate = correlation_.at<float> (a, b);
				}
				else
				{
					correlate = 1;
				}
				v = v + 1 - correlate;
			}
			d[i][j] = std::min(std::min(d[i - 1][j] + penalty, d[i][j - 1] + penalty), v);
		}
	}
	float result = d[wordLength][dictionaryWordLength];

	return result;
}

int DetectText::getCorrelationIndex(std::string letter)
{
	// get index in correlation matrix for given char
	if (letter[0] > 0)
		return (int) letter[0];
	else if (letter.compare("ö") == 0)
		return 246;
	else if (letter.compare("Ö") == 0)
		return 214;
	else if (letter.compare("ü") == 0)
		return 252;
	else if (letter.compare("Ü") == 0)
		return 220;
	else if (letter.compare("ä") == 0)
		return 228;
	else if (letter.compare("Ä") == 0)
		return 196;
	else if (letter.compare("ß") == 0)
		return 223;

	//  if (std::islower(letter))
	//  {
	//    return letter - 'a';
	//  }
	//  else if (std::isupper(letter))
	//  {
	//    return letter - 'A' + 26;
	//  }
	//  else if (std::isdigit(letter))
	//  {
	//    return letter - '0' + 52;
	//  }
	std::cout << "illegal letter: " << letter << std::endl;
	// assert(false);
	return -1;
}

float DetectText::insertToList(std::vector<Word>& words, Word& word) // for example word = ("able",3.7)

{
	// first search for the position
	size_t index = 0;

	for (size_t i = 0; i < words.size(); i++)
	{
		index = i;
		if (word.score < words[i].score) //in case score of actual dictionaryword is smaller than any entry in words: break, remember index
		{
			break;
		}
	}
	if (index != words.size())
	{
		for (size_t i = words.size() - 1; i > index; i--)
		{
			words[i] = words[i - 1];
		}
		words[index] = word;
	}
	return words[words.size() - 1].score; // return last score
}

void DetectText::showBoundingBoxes(std::vector<cv::RotatedRect>& boundingBoxes, std::vector<std::string>& text)
{
	for (size_t i = 0; i < boundingBoxes.size(); i++)
	{
		cv::Scalar scalar((25 * i + 100) % 255, (35 * (i + 1) + 100) % 255, (45 * (i + 2) + 100) % 255);
		cv::RotatedRect *rect = &boundingBoxes[i];
		cv::Point2f vertices[4];
		rect->points(vertices);
		for (int i = 0; i < 4; i++)
			cv::line(resultImage_, vertices[i], vertices[(i + 1) % 4], scalar);
		//    cv::rectangle(resultImage_, cv::Point(rect->x, rect->y), cv::Point(rect->x + rect->width, rect->y + rect->height),
		//                  scalar, 1);

	}
	overlayText(boundingBoxes, text);
}

void DetectText::overlayText(std::vector<cv::RotatedRect>& box, std::vector<std::string>& text)
{
	int textDisplayOffset = 1;
	// assert(box.size() == text.size());
	size_t lineWidth = 25;
	int indent = 50;
	int count = 0;
	for (size_t i = 0; i < box.size(); i++)
	{
		cv::Scalar color((25 * i + 100) % 255, (35 * (i + 1) + 100) % 255, (45 * (i + 2) + 100) % 255);
		if (count > 9)
			indent = 70;
		std::string output = text[i];
		if (output.compare("") == 0)
			continue;

		std::string s;
		std::stringstream out;
		out << count;
		count++;
		std::string prefix = "[";
		prefix = prefix + out.str() + "]";
		cv::putText(resultImage_, prefix, cv::Point(box[i].center.x + 0.5 * box[i].size.width, box[i].center.y - 0.5 * box[i].size.height),
				cv::FONT_HERSHEY_DUPLEX, 1, color, 1);
		cv::putText(resultImage_, prefix, cv::Point(grayImage_.cols, textDisplayOffset * 35), cv::FONT_HERSHEY_DUPLEX, 1, color, 1);
		while (output.length() > lineWidth)
		{
			cv::putText(resultImage_, output.substr(0, lineWidth), cv::Point(grayImage_.cols + indent, textDisplayOffset * 35), cv::FONT_HERSHEY_DUPLEX, 1,
					color, 1);
			output = output.substr(lineWidth);
			textDisplayOffset++;
		}
		cv::putText(resultImage_, output, cv::Point(grayImage_.cols + indent, textDisplayOffset * 35), cv::FONT_HERSHEY_DUPLEX, 1, color, 1);
		textDisplayOffset += 2;
	}
}

void DetectText::writeTxtsForEval()
{
	std::ofstream file1, file2, file3; // file3 = bezier
	std::string textname = filename_.substr(0, filename_.find_last_of("."));
	std::string textname2 = textname;
	std::string textname3 = textname;
	textname.append(".txt");
	textname2.append("t.txt");
	textname3.append("r.txt");
	file1.open(textname.c_str());
	file2.open(textname2.c_str());
	file3.open(textname3.c_str());

	for (unsigned int i = 0; i < finalBoundingBoxes_.size(); i++)
		file1 << finalBoundingBoxes_[i].x << "\n" << finalBoundingBoxes_[i].y << "\n" << finalBoundingBoxes_[i].width << "\n" << finalBoundingBoxes_[i].height
				<< "\n";

	file1.close();

	for (unsigned int i = 0; i < finalTexts_.size(); i++)
		file2 << finalTexts_[i] << "\n";

	file2.close();

	for (unsigned int i = 0; i < finalRotatedBoundingBoxes_.size(); i++)
		file3 << finalRotatedBoundingBoxes_[i].center.x << "\n" << finalRotatedBoundingBoxes_[i].center.y << "\n" << finalRotatedBoundingBoxes_[i].size.width
				<< "\n" << finalRotatedBoundingBoxes_[i].size.height << "\n" << finalRotatedBoundingBoxes_[i].angle << "\n";

	file3.close();
}

void DetectText::ransacPipeline(std::vector<TextRegion>& textRegions)
{
	std::vector<std::pair<std::vector<cv::Point>, std::vector<cv::Point> > > ransacSet;

	std::vector<cv::Rect> ransacBoxes;

	for (unsigned int textRegionIndex = 0; textRegionIndex < textRegions.size(); textRegionIndex++)
	{
		//    cv::Mat output = originalImage_.clone();

		cv::Rect r = textRegions[textRegionIndex].boundingBox;

		ransacSet = ransac(textRegions[textRegionIndex].letters);

		// transform text based on bezier line
		// calculate height and width of transformed image
		for (unsigned int i = 0; i < ransacSet.size(); i++)
		{
			cv::Mat model = createBezierCurve(ransacSet[i].second, debug["showBezier"]); // todo: parameter showBezier

			float biggestDistance = 0; // -> height of transformed image
			float minDistance = 1e5;
			float minDistanceLast = 1e5;
			float minT = 0, maxT = 1; // -> to calculate width of transformed image

			for (unsigned int j = 0; j < ransacSet[i].first.size(); j++)
			{
				for (unsigned int k = 0; k < textRegions[textRegionIndex].letters.size(); k++)
				{
					if ((ransacSet[i].first[j]).inside(textRegions[textRegionIndex].letters[k].boundingBox))
					{
						// todo: wouldn't it be sufficient to check the 4 corner points of the bounding box?
						// probably not because curve is bent

						// get max distance from curve
						// calculate distance of all border pixels of components to the curve
						for (int y = textRegions[textRegionIndex].letters[k].boundingBox.y, x = textRegions[textRegionIndex].letters[k].boundingBox.x;
								y < textRegions[textRegionIndex].letters[k].boundingBox.y + textRegions[textRegionIndex].letters[k].boundingBox.height; y++) // todo: y += height
						{
							std::pair<float, float> distanceT = getBezierDistance(model, cv::Point(x, y));
							if (biggestDistance < distanceT.first)
								biggestDistance = distanceT.first;

							// if actual component is first component of model, get nearest Point on border to calculate minimum t
							if (ransacSet[i].second[0] == ransacSet[i].first[j])
								if (distanceT.first < minDistance && distanceT.second < 0)
								{
									minDistance = distanceT.first;
									minT = distanceT.second;
								}

							// if actual component is last component of model, get nearest Point on border to calculate maximum t
							if (ransacSet[i].second[2] == ransacSet[i].first[j])
								if (distanceT.first < minDistanceLast && distanceT.second > 1)
								{
									minDistanceLast = distanceT.first;
									maxT = distanceT.second;
								}
						}

						for (int y = textRegions[textRegionIndex].letters[k].boundingBox.y, x = textRegions[textRegionIndex].letters[k].boundingBox.x
								+ textRegions[textRegionIndex].letters[k].boundingBox.width; y < textRegions[textRegionIndex].letters[k].boundingBox.y
								+ textRegions[textRegionIndex].letters[k].boundingBox.height; y++)
						{
							std::pair<float, float> distanceT = getBezierDistance(model, cv::Point(x, y));
							if (biggestDistance < distanceT.first)
								biggestDistance = distanceT.first;
							if (ransacSet[i].second[0] == ransacSet[i].first[j])
								if (distanceT.first < minDistance && distanceT.second < 0)
								{
									minDistance = distanceT.first;
									minT = distanceT.second;
								}

							if (ransacSet[i].second[2] == ransacSet[i].first[j])
								if (distanceT.first < minDistanceLast && distanceT.second > 1)
								{
									minDistanceLast = distanceT.first;
									maxT = distanceT.second;
								}
						}
						for (int x = textRegions[textRegionIndex].letters[k].boundingBox.x, y = textRegions[textRegionIndex].letters[k].boundingBox.y;
								x < textRegions[textRegionIndex].letters[k].boundingBox.x + textRegions[textRegionIndex].letters[k].boundingBox.width; x++)
						{
							std::pair<float, float> distanceT = getBezierDistance(model, cv::Point(x, y));
							if (biggestDistance < distanceT.first)
								biggestDistance = distanceT.first;
							if (ransacSet[i].second[0] == ransacSet[i].first[j])
								if (distanceT.first < minDistance && distanceT.second < 0)
								{
									minDistance = distanceT.first;
									minT = distanceT.second;
								}

							if (ransacSet[i].second[2] == ransacSet[i].first[j])
								if (distanceT.first < minDistanceLast && distanceT.second > 1)
								{
									minDistanceLast = distanceT.first;
									maxT = distanceT.second;
								}
						}
						for (int x = textRegions[textRegionIndex].letters[k].boundingBox.x, y = textRegions[textRegionIndex].letters[k].boundingBox.y
								+ textRegions[textRegionIndex].letters[k].boundingBox.height; x < textRegions[textRegionIndex].letters[k].boundingBox.x
								+ textRegions[textRegionIndex].letters[k].boundingBox.width; x++)
						{
							std::pair<float, float> distanceT = getBezierDistance(model, cv::Point(x, y));
							if (biggestDistance < distanceT.first)
								biggestDistance = distanceT.first;
							if (ransacSet[i].second[0] == ransacSet[i].first[j])
								if (distanceT.first < minDistance && distanceT.second < 0)
								{
									minDistance = distanceT.first;
									minT = distanceT.second;
								}

							if (ransacSet[i].second[2] == ransacSet[i].first[j])
								if (distanceT.first < minDistanceLast && distanceT.second > 1)
								{
									minDistanceLast = distanceT.first;
									maxT = distanceT.second;
								}
						}
					}
				}
			}

			float h = biggestDistance + 2; // 2*2 pixel padding
			float l = getBezierLength(model, minT < 0 ? minT * 1.1 : minT * 0.9, maxT * 1.1);

			cv::Mat rotatedBezier(h * 2, l, CV_8UC3, cv::Scalar(255, 255, 255));

			// make new bounding box
			int minX = 1e5, maxX = 0, minY = 1e5, maxY = 0;

			for (unsigned int letterIndex = 0; letterIndex < ransacSet[i].first.size(); letterIndex++)
			{
				for (unsigned int k = 0; k < textRegions[textRegionIndex].letters.size(); k++)
				{
					if ((ransacSet[i].first[letterIndex]).inside(textRegions[textRegionIndex].letters[k].boundingBox))
					{
						if (textRegions[textRegionIndex].letters[k].boundingBox.x < minX)
							minX = textRegions[textRegionIndex].letters[k].boundingBox.x;
						if (textRegions[textRegionIndex].letters[k].boundingBox.y < minY)
							minY = textRegions[textRegionIndex].letters[k].boundingBox.y;
						if (textRegions[textRegionIndex].letters[k].boundingBox.x + textRegions[textRegionIndex].letters[k].boundingBox.width > maxX)
							maxX = textRegions[textRegionIndex].letters[k].boundingBox.x + textRegions[textRegionIndex].letters[k].boundingBox.width;
						if (textRegions[textRegionIndex].letters[k].boundingBox.y + textRegions[textRegionIndex].letters[k].boundingBox.height > maxY)
							maxY = textRegions[textRegionIndex].letters[k].boundingBox.y + textRegions[textRegionIndex].letters[k].boundingBox.height;
					}
				}
			}

			cv::Rect newR(minX, minY, maxX - minX, maxY - minY);

			// form rotatedRect
			if (debug["showRansac"] == true)
			{
				std::cout << "point #1: " << model.at<float> (0, 2) << "|" << model.at<float> (1, 2) << std::endl;
				std::cout << "point #2: " << model.at<float> (0, 0) + model.at<float> (0, 1) + model.at<float> (0, 2) << "|" << model.at<float> (1, 0)
						+ model.at<float> (1, 1) + model.at<float> (1, 2) << std::endl;
				std::cout << "point #3: " << 0.25 * model.at<float> (0, 0) + 0.5 * model.at<float> (0, 1) + model.at<float> (0, 2) << "|" << 0.25 * model.at<
						float> (1, 0) + 0.5 * model.at<float> (1, 1) + model.at<float> (1, 2) << std::endl;
			}
			std::vector<cv::Point> pointvector;
			pointvector.push_back(cv::Point(model.at<float> (0, 2), model.at<float> (1, 2)));
			pointvector.push_back(
					cv::Point(model.at<float> (0, 0) + model.at<float> (0, 1) + model.at<float> (0, 2),
							model.at<float> (1, 0) + model.at<float> (1, 1) + model.at<float> (1, 2)));
			pointvector.push_back(
					cv::Point(0.25 * model.at<float> (0, 0) + 0.5 * model.at<float> (0, 1) + model.at<float> (0, 2),
							0.25 * model.at<float> (1, 0) + 0.5 * model.at<float> (1, 1) + model.at<float> (1, 2)));
			cv::RotatedRect rr = cv::minAreaRect(pointvector);

			cv::Mat img;
			cv::Point2f vertices[4];
			if (debug["showRansac"] == true)
			{
				img = originalImage_.clone();
				rr.points(vertices);
				for (int i = 0; i < 4; i++)
					cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 255, 255));
				cv::imshow("img", img);
			}

			h = biggestDistance;
			// change height and width of rotatedRect.
			// as the angle can be ambiguous, the size is changed based on what's bigger.
			if (rr.size.height > rr.size.width)
			{
				rr.size.height = 2 * h > l ? 2 * h : l;
				rr.size.width = 2 * h > l ? l : 2 * h;
			}
			else
			{
				rr.size.height = 2 * h > l ? l : 2 * h;
				rr.size.width = 2 * h > l ? 2 * h : l;
			}

			if (debug["showRansac"] == true)
			{
				rr.points(vertices);
				for (int i = 0; i < 4; i++)
					cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 255, 255));
				cv::imshow("img2", img);
				std::cout << "rr.angle:" << rr.angle << std::endl;
				cv::waitKey(0);
			}

			transformBezier(newR, model, rotatedBezier, minT < 0 ? minT * 1.1 : minT * 0.9, maxT * 1.1);

			transformedImage_.push_back(rotatedBezier);
			notTransformedImage_.push_back(originalImage_(newR));

			finalBoundingBoxes_.push_back(newR);
			finalRotatedBoundingBoxes_.push_back(rr);

		}
	}
}

std::vector< std::pair< std::vector<cv::Point>, std::vector<cv::Point> > > DetectText::ransac(std::vector<Letter> dataset)
{
	// calculates best quadratic Bézier Curves out of all points in dataset
	// returns 1 or more models, each returned vector-element includes all points that support the model and as second pair-element the 3 model-describing points

	// Algorithm Parameters
	unsigned int bezierDegree = 3; // => quadratic
	float e; // (number of outliers in data / number of points in data)
	float n; // number of iteration steps

	std::vector<std::pair<std::vector<cv::Point>, std::vector<cv::Point> > > ransacSubset; // to be returned

	cv::Mat output;

	// if box contains lines of text, a sub-dataset is build at the end of every iteration with the remaining points that were not used.
	// when sub-dataset doesn't contain more than 2 components anymore, the algorithm ends
	while (dataset.size() >= 3)
	{
		// calculate e (percent outliers) based on pca
		cv::Mat pcaData(dataset.size(), 2, CV_32FC1);
		for (unsigned int i = 0; i < dataset.size(); i++)
		{
			pcaData.at<float> (i, 0) = dataset[i].centerPoint.x;
			pcaData.at<float> (i, 1) = dataset[i].centerPoint.y;
		}
		cv::PCA pca(pcaData, cv::noArray(), CV_PCA_DATA_AS_ROW, 2);
		cv::Mat eigenVal = pca.eigenvalues;
		float eigenValQuotient = std::min(eigenVal.at<float> (0, 0), eigenVal.at<float> (1, 0))
				/ std::max(eigenVal.at<float> (0, 0), eigenVal.at<float> (1, 0));

		// with perfect points orientation (all on one line) eigenValQuotient ~> 0 -> then percent of false data is supposed to be low
		// points are spread around the whole space -> similiar eigenvalues -> percent outliers is high: maxE
		e = (maxE - minE) * eigenValQuotient + minE;

		// calculate number of iterations
		// todo: where does the 5 come from? to have more iterations -> better accuracy
		n = 5 * std::ceil(std::log10(1 - p) / (float) (std::log10(1 - (std::pow((1 - e), bezierDegree)))));

		if (dataset.size() == 3) // if there are only 3 components, don't calculate 200 identical models...
			n = 1;

		if (debug["showBezier"])
		{
			std::cout << "-----------------------" << std::endl << std::endl;
			for (unsigned int i = 0; i < dataset.size(); i++)
			{
				output = originalImage_.clone();
				cv::rectangle(output, cv::Rect(dataset[i].centerPoint.x, dataset[i].centerPoint.y, 1, 1), cv::Scalar(255, 255, 255), 2, 1, 0);
				//std::cout << "p" << i << ": " << dataset[i].centerPoint.x << "|" << dataset[i].centerPoint.y << std::endl;
			}
			std::cout << std::endl << n << " iterations.." << std::endl;
		}

		//std::vector<cv::Point> modelset(bezierDegree);
		// todo: change to cv::Point2d
		std::vector<cv::Point> finalModel; // 3 points describing best supported model
		std::vector<cv::Point> finalGoodPoints; // all points of best supported model after all iterations
		std::vector<float> finalTs; // t's, describing position on bezier model of all final points

		std::srand(time(NULL));
		int modelNr = -1;

		float highestScore = 0;

		for (unsigned int i = 0; i < n; i++) // Main Loop, n = number of models that will be created
		{
			std::vector<float> rectsArea;
			cv::Mat output2;

			std::vector<int> actualRandomSet; // which 3 points are chosen (index)
			std::vector<cv::Point> actualRandomPointSet; // which 3 points are chosen (point)
			std::vector<cv::Point> goodPoints; //which points support the model
			std::vector<float> ts; // all ts, t as in C(t)

			// fill actualRandomSet with random points for model
			//for (unsigned int j = 0; j < bezierDegree; j++) // todo: this for loop has no effect
			while (actualRandomSet.size() < 3)
			{
				int nn = (int) ((std::rand() / (float) RAND_MAX) * dataset.size());
				if (std::find(actualRandomSet.begin(), actualRandomSet.end(), nn) == actualRandomSet.end())
				{
					actualRandomSet.push_back(nn);
					actualRandomPointSet.push_back(dataset[nn].centerPoint);
				}
			}

			// calculate maxDistance for every pointset anew, maxDistance = max. distance between point and curve so that point still supports model
			for (unsigned int distanceParameterIndex = 0; distanceParameterIndex < actualRandomSet.size(); distanceParameterIndex++)
				rectsArea.push_back(std::sqrt(dataset[distanceParameterIndex].boundingBox.area()));

			float maxDistance = 0;
			for (unsigned int k = 0; k < rectsArea.size(); k++)
				maxDistance += rectsArea[k];
			maxDistance = distanceParameter * (maxDistance / actualRandomSet.size());

			// Reject Criterion #1: Reject model if letter sizes are too different (for the 3 randomly chosen letters)
			if (std::max(rectsArea[0], rectsArea[1]) / (std::min(rectsArea[0], rectsArea[1])) > 3) // todo: (i) make this a parameter, (ii) avoid division
				continue;
			if (std::max(rectsArea[0], rectsArea[2]) / (std::min(rectsArea[0], rectsArea[2])) > 3)
				continue;
			if (std::max(rectsArea[1], rectsArea[2]) / (std::min(rectsArea[1], rectsArea[2])) > 3)
				continue;

			// calculate bezier curve
			cv::Mat curve = createBezierCurve(actualRandomPointSet, debug["showBezier"]);
			if (curve.rows < 1)
			{
				std::cout << "bezier curve wasn't drawn correctly." << std::endl;
				return ransacSubset;
			}

			// Bending of curve -> angle difference
			//      float angleA = std::atan2(actualRandomPointSet[1].y - actualRandomPointSet[0].y, actualRandomPointSet[1].x
			//          - actualRandomPointSet[0].x) * (180.0 / 3.14159265);
			//
			//      float angleB = std::atan2(actualRandomPointSet[2].y - actualRandomPointSet[1].y, actualRandomPointSet[2].x
			//          - actualRandomPointSet[1].x) * (180.0 / 3.14159265);
			//
			//      float angleDifference = std::abs(angleA - angleB);
			//      if (angleDifference > 180)
			//        angleDifference = 360 - angleDifference;
			double dx1 = actualRandomPointSet[1].x - actualRandomPointSet[0].x;
			double dy1 = actualRandomPointSet[1].y - actualRandomPointSet[0].y;
			double dx2 = actualRandomPointSet[2].x - actualRandomPointSet[1].x;
			double dy2 = actualRandomPointSet[2].y - actualRandomPointSet[1].y;
			float angleDifference = (180.0 / 3.14159265) * std::abs(acos((dx1 * dx2 + dy1 * dy2) / (sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2)))));

			// draw chosen points
			if (debug["showBezier"])
			{
				output2 = output.clone();
				std::cout << std::endl << "Model #" << i << std::endl;
				std::cout << "Curve: " << std::endl;
				std::cout << "[ " << curve.at<float> (0, 0) << " " << curve.at<float> (0, 1) << " " << curve.at<float> (0, 2) << " ]" << std::endl;
				std::cout << "[ " << curve.at<float> (1, 0) << " " << curve.at<float> (1, 1) << " " << curve.at<float> (1, 2) << " ]" << std::endl;
				std::cout << "Points: " << std::endl;
				std::cout << actualRandomPointSet[0].x << "|" << actualRandomPointSet[0].y << ", " << actualRandomPointSet[1].x << "|"
						<< actualRandomPointSet[1].y << ", " << actualRandomPointSet[2].x << "|" << actualRandomPointSet[2].y << std::endl;
				//        std::cout << "angleA: " << angleA << ", angleB: " << angleB << std::endl;
				std::cout << "|angleA-angleB| = " << angleDifference << std::endl;
				for (unsigned int j = 0; j < actualRandomSet.size(); j++)
					cv::rectangle(output2, cv::Rect(dataset[actualRandomSet[j]].centerPoint.x, dataset[actualRandomSet[j]].centerPoint.y, 1, 1),
							cv::Scalar(255, 0, 0), 2, 1, 0);
				cv::imshow("ransac", output2);
				std::cout << "------------" << std::endl;
			}

			// Reject Criterion #2: Reject model if (rotated) heights are too different
			std::vector<unsigned int> heights;
			for (unsigned int actualSetIndex = 0; actualSetIndex < actualRandomSet.size(); actualSetIndex++)
			{
				std::pair<float, float> distanceTs = getBezierDistance(curve, dataset[actualSetIndex].centerPoint);
				// C'(t) = 2At+B
				int t = distanceTs.second;
				cv::Point2f normal(
						2 * cv::Point(curve.at<float> (0, 0), curve.at<float> (1, 0)).y * t + cv::Point(curve.at<float> (0, 1), curve.at<float> (1, 1)).y,
						-1
								* (2 * cv::Point(curve.at<float> (0, 0), curve.at<float> (1, 0)).x * t + cv::Point(curve.at<float> (0, 1),
										curve.at<float> (1, 1)).x));

				normal = normal * (float) (1 / (std::sqrt(normal.x * normal.x + normal.y * normal.y)));
				heights.push_back(calculateRealLetterHeight(dataset[actualSetIndex].centerPoint, dataset[actualSetIndex].boundingBox, normal));
			}
			float meanHeight = 0;
			float heightVariance = 0;

			for (unsigned int h = 0; h < 3; h++)
				meanHeight += heights[h];
			meanHeight /= 3;

			for (unsigned int h = 0; h < 3; h++)
				heightVariance += (meanHeight - heights[h]) * (meanHeight - heights[h]);
			heightVariance /= 3;

			if (heightVariance > 50)
				continue;

			// calculate distance between curve and each data point
			double distance = 0;

			for (unsigned int datasetIndex = 0; datasetIndex < dataset.size(); datasetIndex++)
			{
				std::pair<float, float> distanceTs = getBezierDistance(curve, dataset[datasetIndex].centerPoint);
				// todo: why count distance of all points, not just inliers?
				//        distance += distanceTs.first; // count the distance of every point to evaluate model based on summed distances
				if (distanceTs.first < maxDistance) // is the point near enough?
				{
					distance += distanceTs.first; // count the distance of every point to evaluate model based on summed distances
					goodPoints.push_back(dataset[datasetIndex].centerPoint);
					ts.push_back(distanceTs.second);
					if (debug["showBezier"])
					{
						cv::rectangle(output2, cv::Rect(dataset[datasetIndex].centerPoint.x, dataset[datasetIndex].centerPoint.y, 1, 1), cv::Scalar(0, 255, 0),
								2, 1, 0);
						if (distanceTs.first < 1e-4)
							std::cout << "distance: " << 0 << " [x]" << std::endl;
						else
							std::cout << "distance: " << distanceTs.first << " [x]" << std::endl;
						cv::imshow("ransac", output2);
						if (cv::waitKey(0) == 27) // ESC pressed
							debug["showBezier"] = false;
					}
				}
				else
				{
					if (debug["showBezier"])
					{
						cv::rectangle(output2, cv::Rect(dataset[datasetIndex].centerPoint.x, dataset[datasetIndex].centerPoint.y, 1, 1), cv::Scalar(0, 0, 255),
								2, 1, 0);
						std::cout << "distance: " << distanceTs.first << " [ ]" << std::endl;
						cv::imshow("ransac", output2);
						//cv::waitKey(0);
					}
				}
			}

			// remember model if its the best so far
			float score = 0;

			// 1) How many points support the model
			score += 100.f * (goodPoints.size() / dataset.size());

			// 2) How far is the distance of all these points to the model curve
			// 2) How far is the mean distance of all good points to the model curve
			score += 100.f / (distance / (double) goodPoints.size() + 1);

			// 3) How strong is the bending of the curve
			//      score += 300 / (angleDifference + 1);		// todo: avoid division
			score += 100.f * ((180.f - angleDifference) / 180.f);

			if (score > highestScore && angleDifference < bendParameter && goodPoints.size() >= 3)
			{
				finalGoodPoints = goodPoints;
				finalModel = actualRandomPointSet;
				finalTs = ts;
				highestScore = score;
				modelNr = i;
			}
			if (debug["showBezier"])
			{
				std::cout << "Σdistance: " << distance << std::endl;
				std::cout << goodPoints.size() << "/" << dataset.size() << " points support model" << std::endl;
			}
		}

		// show final chosen model
		if (debug["showBezier"])
		{
			for (unsigned int i = 0; i < finalGoodPoints.size(); i++)
				cv::rectangle(output, cv::Rect(finalGoodPoints[i].x, finalGoodPoints[i].y, 1, 1), cv::Scalar(0, 0, 0), 2, 1, 0);
			for (unsigned int i = 0; i < finalModel.size(); i++)
				cv::rectangle(output, cv::Rect(finalModel[i].x, finalModel[i].y, 1, 1), cv::Scalar(255, 0, 0), 2, 1, 0);
			cv::imshow("ransac", output);
			cv::waitKey(0);
			std::cout << "Model chosen: " << modelNr << std::endl;
		}

		if (modelNr == -1) // no good model found at all
			return ransacSubset;

		// if there only 3 points, don't calculate quadratic bezier for final model, its bending will be too strong. use normal line instead..
		if (finalGoodPoints.size() == 3)
		{
			finalGoodPoints[1].x = (int) (finalGoodPoints[0].x + finalGoodPoints[2].x) / 2.0;
			finalGoodPoints[1].y = (int) (finalGoodPoints[0].y + finalGoodPoints[2].y) / 2.0;
		}

		// create model with all found finalGoodPoints
		float biggestT = 0, biggestTIndex = 0;
		float smallestT = 1e10, smallestTIndex = 1e10;
		for (unsigned int i = 0; i < finalTs.size(); i++)
		{
			if (biggestT < finalTs[i])
			{
				biggestT = finalTs[i];
				biggestTIndex = i;
			}
			if (smallestT > finalTs[i])
			{
				smallestT = finalTs[i];
				smallestTIndex = i;
			}
		}

		// [smallestT, biggestT] -> [0, 1]
		float alpha = 1 / (float) (biggestT - smallestT);
		float beta = -smallestT / (float) (biggestT - smallestT);

		cv::Mat A(finalTs.size() * 2, 6, CV_32FC1);
		cv::Mat X(6, 1, CV_32FC1);
		cv::Mat C(finalTs.size() * 2, 1, CV_32FC1);

		for (unsigned int i = 0; i < finalTs.size(); i++)
		{
			// alpha * finalT + beta = t
			float newT = alpha * finalTs[i] + beta, a, b, c;

			a = newT * newT - 2 * newT + 1;
			b = -2 * newT * newT + 2 * newT;
			c = newT * newT;

			A.at<float> (i, 0) = a;
			A.at<float> (i, 1) = b;
			A.at<float> (i, 2) = c;
			A.at<float> (i, 3) = 0;
			A.at<float> (i, 4) = 0;
			A.at<float> (i, 5) = 0;

			A.at<float> (i + finalTs.size(), 0) = 0;
			A.at<float> (i + finalTs.size(), 1) = 0;
			A.at<float> (i + finalTs.size(), 2) = 0;
			A.at<float> (i + finalTs.size(), 3) = a;
			A.at<float> (i + finalTs.size(), 4) = b;
			A.at<float> (i + finalTs.size(), 5) = c;

			C.at<float> (i, 0) = finalGoodPoints[i].x;
			C.at<float> (i + finalTs.size(), 0) = finalGoodPoints[i].y;
		}

		//    for (unsigned int i = 0; i < 2 * finalTs.size(); i++)
		//    {
		//      std::cout << "[ " << A.at<float> (i, 0) << " " << A.at<float> (i, 1) << " " << A.at<float> (i, 2) << " " << A.at<
		//          float> (i, 3) << " " << A.at<float> (i, 4) << " " << A.at<float> (i, 5) << " ]  [ x" << i << " ]  =  [ "
		//          << C.at<float> (i, 0) << " ]" << std::endl;
		//    }

		cv::solve(A, C, X, cv::DECOMP_SVD);

		// draw final curve...
		cv::vector<cv::Point> finalBezierCurve;
		finalBezierCurve.push_back(cv::Point(X.at<float> (0, 0), X.at<float> (3, 0)));
		finalBezierCurve.push_back(cv::Point(X.at<float> (1, 0), X.at<float> (4, 0)));
		finalBezierCurve.push_back(cv::Point(X.at<float> (2, 0), X.at<float> (5, 0)));
		cv::Mat finalCurve = createBezierCurve(finalBezierCurve, debug["showRansac"]);

		std::pair<std::vector<cv::Point>, std::vector<cv::Point> > pointsAndCurve;
		pointsAndCurve.first = finalGoodPoints;
		pointsAndCurve.second = finalBezierCurve;

		ransacSubset.push_back(pointsAndCurve);

		// build new sub dataset with remaining points and loop
		std::vector<Letter> subDataset;
		for (unsigned int i = 0; i < dataset.size(); i++)
		{
			if (std::find(finalGoodPoints.begin(), finalGoodPoints.end(), cv::Point(dataset[i].centerPoint.x, dataset[i].centerPoint.y)) == finalGoodPoints.end())
				subDataset.push_back(dataset[i]);
		}
		dataset = subDataset;
		// loop as long as dataset contains more than 2 components
	}
	return ransacSubset;
}

cv::Mat DetectText::createBezierCurve(std::vector<cv::Point> & points, bool showBezier)
{
	if (points.size() != 3)
	{
		std::cout << "3 Points required for quadratic curve." << std::endl;
		return cv::Mat();
	}

	cv::Mat output = originalImage_.clone();

	cv::Mat R = (cv::Mat_<float>(2, 3) << points[0].x, points[1].x, points[2].x, points[0].y, points[1].y, points[2].y); // todo: create R not before correct order is known

	// 1.) compute distances of all 3 points and position them into right order,
	// i.e. the points with largest distance go to position 1 and 3
	float distance12 = std::sqrt((points[0].y - points[1].y) * (points[0].y - points[1].y) + (points[0].x - points[1].x) * (points[0].x - points[1].x));
	float distance13 = std::sqrt((points[0].y - points[2].y) * (points[0].y - points[2].y) + (points[0].x - points[2].x) * (points[0].x - points[2].x));
	float distance23 = std::sqrt((points[1].y - points[2].y) * (points[1].y - points[2].y) + (points[1].x - points[2].x) * (points[1].x - points[2].x));

	float t0 = distance12 / (float) (distance12 + distance23); // relation of distances, i.e. percentage of t at the point in the middle

	if (distance12 > distance13 && distance12 > distance23)
	{
		float tempX, tempY;
		tempX = R.at<float> (0, 1);
		tempY = R.at<float> (1, 1);
		R.at<float> (0, 1) = R.at<float> (0, 2);
		R.at<float> (1, 1) = R.at<float> (1, 2);
		R.at<float> (0, 2) = tempX;
		R.at<float> (1, 2) = tempY;
		t0 = distance13 / (float) (distance13 + distance23);
		cv::Point temp = points[1];
		points[1] = points[2];
		points[2] = temp;
	}
	else if (distance23 > distance12 && distance23 > distance13)
	{
		float tempX, tempY;
		tempX = R.at<float> (0, 1);
		tempY = R.at<float> (1, 1);
		R.at<float> (0, 1) = R.at<float> (0, 0);
		R.at<float> (1, 1) = R.at<float> (1, 0);
		R.at<float> (0, 0) = tempX;
		R.at<float> (1, 0) = tempY;
		t0 = distance12 / (float) (distance13 + distance12);
		cv::Point temp = points[1];
		points[1] = points[0];
		points[0] = temp;
	}

	// why? to avoid head-over as first guess!
	if (points[0].x > points[2].x)
	{
		float tempX, tempY;
		tempX = R.at<float> (0, 0);
		tempY = R.at<float> (1, 0);
		R.at<float> (0, 0) = R.at<float> (0, 2);
		R.at<float> (1, 0) = R.at<float> (1, 2);
		R.at<float> (0, 2) = tempX;
		R.at<float> (1, 2) = tempY;
		cv::Point temp = points[0];
		points[0] = points[2];
		points[2] = temp;
		t0 = 1 - t0;
	}

	// 2.) compute the Bezier point
	R.at<float> (0, 1) = (R.at<float> (0, 1) + R.at<float> (0, 0) * (-1 + 2 * t0 - t0 * t0) - R.at<float> (0, 2) * t0 * t0) / (2 * t0 - 2 * t0 * t0);
	R.at<float> (1, 1) = (R.at<float> (1, 1) + R.at<float> (1, 0) * (-1 + 2 * t0 - t0 * t0) - R.at<float> (1, 2) * t0 * t0) / (2 * t0 - 2 * t0 * t0);

	// 3.) draw bezier curve

	//  cv::Mat C = (cv::Mat_<float>(2, 3) << (R.at<float> (0, 0) - 2 * R.at<float> (0, 1) + R.at<float> (0, 2)), (-2 * R.at<
	//      float> (0, 0) + 2 * R.at<float> (0, 1)), R.at<float> (0, 0), (R.at<float> (1, 0) - 2 * R.at<float> (1, 1) + R.at<
	//      float> (1, 2)), (-2 * R.at<float> (1, 0) + 2 * R.at<float> (1, 1)), R.at<float> (1, 0));
	cv::Mat C(2, 3, CV_32FC1);
	C.at<float> (0, 0) = R.at<float> (0, 0) - 2 * R.at<float> (0, 1) + R.at<float> (0, 2);
	C.at<float> (0, 1) = -2 * R.at<float> (0, 0) + 2 * R.at<float> (0, 1);
	C.at<float> (0, 2) = R.at<float> (0, 0);
	C.at<float> (1, 0) = R.at<float> (1, 0) - 2 * R.at<float> (1, 1) + R.at<float> (1, 2);
	C.at<float> (1, 1) = -2 * R.at<float> (1, 0) + 2 * R.at<float> (1, 1);
	C.at<float> (1, 2) = R.at<float> (1, 0);

	if (showBezier)
	{
		// draw bezier point (lies not on curve)
		cv::rectangle(output, cv::Point((int) std::floor(R.at<float> (0, 1) + 0.5), (int) std::floor(R.at<float> (1, 1) + 0.5)),
				cv::Point((int) std::floor(R.at<float> (0, 1) + 0.5), (int) std::floor(R.at<float> (1, 1) + 0.5)), cv::Scalar(0, 255, 0), 1, 1, 0);

		// draw line as connected points
		for (float t = 0; t < 1; t += 0.01)
		{
			float Cx = (R.at<float> (0, 0) - 2 * R.at<float> (0, 1) + R.at<float> (0, 2)) * t * t + (-2 * R.at<float> (0, 0) + 2 * R.at<float> (0, 1)) * t
					+ R.at<float> (0, 0);
			float Cy = (R.at<float> (1, 0) - 2 * R.at<float> (1, 1) + R.at<float> (1, 2)) * t * t + (-2 * R.at<float> (1, 0) + 2 * R.at<float> (1, 1)) * t
					+ R.at<float> (1, 0);
			cv::rectangle(output, cv::Point((int) std::floor(Cx + 0.5), (int) std::floor(Cy + 0.5)),
					cv::Point((int) std::floor(Cx + 0.5), (int) std::floor(Cy + 0.5)), cv::Scalar(150, 200, 50), 1, 1, 0);
		}

		// draw the 3 points the curve is made of
		cv::rectangle(output, points[0], points[0], cv::Scalar(255, 255, 255), 2, 1, 0);
		cv::rectangle(output, points[1], points[1], cv::Scalar(255, 255, 255), 2, 1, 0);
		cv::rectangle(output, points[2], points[2], cv::Scalar(255, 255, 255), 2, 1, 0);

		cv::imshow("bezier", output);
		cv::waitKey(0);
	}
	return C;
}

void postRansacCriterions()
{

}

void DetectText::Cramer()
{
	// f(x) = ax² + bx +c
	// p1.y = a*p1.x² + b*p1.x + c
	// p2.y = a*p2.x² + b*p2.x + c
	// p3.y = a*p2.x² + b*p3.x + c

	// Cramer rule to solve

	//    // Det
	//    cv::Mat det(3, 3, CV_32FC1);
	//    cv::Mat det2(3, 3, CV_32FC1);
	//    for (int j = 0; j < 3; j++)
	//    {
	//      det.at<float> (j, 0) = dataset[actualRandomSet[j]].x * dataset[actualRandomSet[j]].x;
	//      det.at<float> (j, 1) = dataset[actualRandomSet[j]].x;
	//      det.at<float> (j, 2) = 1;
	//      det2.at<float> (j, 0) = dataset[actualRandomSet[j]].y * dataset[actualRandomSet[j]].y;
	//      det2.at<float> (j, 1) = dataset[actualRandomSet[j]].y;
	//      det2.at<float> (j, 2) = 1;
	//    }
	//
	//    float detA, detB, detC, detAll, detA2, detB2, detC2, detAll2;
	//
	//    for (int y = 0; y < 3; y++)
	//    {
	//      for (int x = 0; x < 3; x++)
	//      {
	//        std::cout << det.at<float> (y, x) << " ";
	//      }
	//      std::cout << std::endl;
	//    }
	//    std::cout << "determinant: " << cv::determinant(det) << std::endl;
	//
	//    detAll = cv::determinant(det);
	//    detAll2 = cv::determinant(det2);
	//
	//    for (int j = 0; j < 3; j++)
	//    {
	//      det.at<float> (j, 0) = dataset[actualRandomSet[j]].y;
	//      det.at<float> (j, 1) = dataset[actualRandomSet[j]].x;
	//      det.at<float> (j, 2) = 1;
	//      det2.at<float> (j, 0) = dataset[actualRandomSet[j]].x;
	//      det2.at<float> (j, 1) = dataset[actualRandomSet[j]].y;
	//      det2.at<float> (j, 2) = 1;
	//    }
	//
	//    for (int y = 0; y < 3; y++)
	//    {
	//      for (int x = 0; x < 3; x++)
	//      {
	//        std::cout << det.at<float> (y, x) << " ";
	//      }
	//      std::cout << std::endl;
	//    }
	//    std::cout << "determinant: " << cv::determinant(det) << std::endl;
	//
	//    detA = cv::determinant(det);
	//    detA2 = cv::determinant(det2);
	//
	//    for (int j = 0; j < 3; j++)
	//    {
	//      det.at<float> (j, 0) = dataset[actualRandomSet[j]].x * dataset[actualRandomSet[j]].x;
	//      det.at<float> (j, 1) = dataset[actualRandomSet[j]].y;
	//      det.at<float> (j, 2) = 1;
	//      det2.at<float> (j, 0) = dataset[actualRandomSet[j]].y * dataset[actualRandomSet[j]].y;
	//      det2.at<float> (j, 1) = dataset[actualRandomSet[j]].x;
	//      det2.at<float> (j, 2) = 1;
	//    }
	//
	//    detB = cv::determinant(det);
	//    detB2 = cv::determinant(det2);
	//
	//    for (int j = 0; j < 3; j++)
	//    {
	//      det.at<float> (j, 0) = dataset[actualRandomSet[j]].x * dataset[actualRandomSet[j]].x;
	//      det.at<float> (j, 1) = dataset[actualRandomSet[j]].x;
	//      det.at<float> (j, 2) = dataset[actualRandomSet[j]].y;
	//      det2.at<float> (j, 0) = dataset[actualRandomSet[j]].y * dataset[actualRandomSet[j]].y;
	//      det2.at<float> (j, 1) = dataset[actualRandomSet[j]].y;
	//      det2.at<float> (j, 2) = dataset[actualRandomSet[j]].x;
	//    }
	//
	//    detC = cv::determinant(det);
	//    detC2 = cv::determinant(det2);
	//
	//    for (int x = 0; x < 400; x++)
	//    {
	//      int y = (int)(detA / (float)detAll * x * x + detB / (float)detAll * x + detC / (float)detAll);
	//      int y2 = (int)(detA2 / (float)detAll2 * x * x + detB2 / (float)detAll2 * x + detC2 / (float)detAll2);
	//      cv::rectangle(output2, cv::Rect(x, 400 - y, 1, 1), cv::Scalar(255, 0, 0), 1, 1, 0);
	//      cv::rectangle(output2, cv::Rect(y2, 400 - x, 1, 1), cv::Scalar(255, 0, 0), 1, 1, 0);
	//    }
}

std::pair<float, float> DetectText::getBezierDistance(cv::Mat curve, cv::Point Q)
{
	// compute distance from Q to curve and t when C(t) is nearest Point to Q on curve
	float a = 2 * (curve.at<float> (0, 0) * curve.at<float> (0, 0) + curve.at<float> (1, 0) * curve.at<float> (1, 0)); //coefficients
	float b = 3 * (curve.at<float> (0, 0) * curve.at<float> (0, 1) + curve.at<float> (1, 0) * curve.at<float> (1, 1));
	float c = 2 * (curve.at<float> (0, 0) * (curve.at<float> (0, 2) - Q.x) + curve.at<float> (1, 0) * (curve.at<float> (1, 2) - Q.y)) + curve.at<float> (0, 1)
			* curve.at<float> (0, 1) + curve.at<float> (1, 1) * curve.at<float> (1, 1);
	float d = curve.at<float> (0, 1) * (curve.at<float> (0, 2) - Q.x) + curve.at<float> (1, 1) * (curve.at<float> (1, 2) - Q.y);

	// distinguish between several cases
	float ts = 0;

	if (std::abs(a) == 0)
	{
		if (std::abs(b) == 0)
			if (c == 0)
			{
				std::cout << "Error: a=0, b=0, c=0" << std::endl;
				//return -1;
			}
			else
			// c*ts+d = 0
			{
				ts = -d / c;
			}
		else
		{
			float p2 = c / (2 * b);
			float q = d / b;
			if (p2 * p2 - q < 0)
			{
				std::cout << "Error: a=0, b~=0, p2^2-q < 0" << std::endl;
				// return -1;
			}
			float t1 = -p2 - std::sqrt(p2 * p2 - q);
			float t2 = -p2 + std::sqrt(p2 * p2 - q);

			float dist1 = ((curve.at<float> (0, 0) * t1 * t1 + curve.at<float> (0, 1) * t1 + curve.at<float> (0, 2) - Q.x) * (curve.at<float> (0, 0) * t1 * t1
					+ curve.at<float> (0, 1) * t1 + curve.at<float> (0, 2) - Q.x) + (curve.at<float> (1, 0) * t1 * t1 + curve.at<float> (1, 1) * t1 + curve.at<
					float> (1, 2) - Q.y) * (curve.at<float> (1, 0) * t1 * t1 + curve.at<float> (1, 1) * t1 + curve.at<float> (1, 2) - Q.y));
			float dist2 = ((curve.at<float> (0, 0) * t2 * t2 + curve.at<float> (0, 1) * t2 + curve.at<float> (0, 2) - Q.x) * (curve.at<float> (0, 0) * t2 * t2
					+ curve.at<float> (0, 1) * t2 + curve.at<float> (0, 2) - Q.x) + (curve.at<float> (1, 0) * t2 * t2 + curve.at<float> (1, 1) * t2 + curve.at<
					float> (1, 2) - Q.y) * (curve.at<float> (1, 0) * t2 * t2 + curve.at<float> (1, 1) * t2 + curve.at<float> (1, 2) - Q.y));
			if (dist1 < dist2)
				ts = t1;
			else
				ts = t2;
		}
	}
	else // a*ts³+b*ts²+c*ts+d=0
	{
		//  If discrimant > 0, then the equation has three distinct real roots.
		//  If discrimant = 0, then the equation has a multiple root and all its roots are real.
		//  If discrimant < 0, then the equation has one real root and two nonreal complex conjugate roots.

		float discriminant = 18 * a * b * c * d - 4 * b * b * b * d + b * b * c * c - 4 * a * c * c * c - 27 * a * a * d * d;
		if (discriminant < 0)
		{
			// analytic real root of cubic function
			float ts_p1, ts_p2;
			if (2 * b * b * b - 9 * a * b * c + 27 * a * a * d + a * sqrt(-27 * discriminant) < 0)
			{
				ts_p1 = std::pow(-0.5 * (2 * b * b * b - 9 * a * b * c + 27 * a * a * d + a * sqrt(-27 * discriminant)), (1.0 / 3.0));
				ts_p1 *= -1;
			}
			else
			{
				ts_p1 = std::pow(0.5 * (2 * b * b * b - 9 * a * b * c + 27 * a * a * d + a * sqrt(-27 * discriminant)), (1.0 / 3.0));
			}
			if (2 * b * b * b - 9 * a * b * c + 27 * a * a * d - a * sqrt(-27 * discriminant) < 0)
			{
				ts_p2 = std::pow(-0.5 * (2 * b * b * b - 9 * a * b * c + 27 * a * a * d - a * sqrt(-27 * discriminant)), (1.0 / 3.0));
				ts_p2 *= -1;
			}
			else
			{
				ts_p2 = std::pow(0.5 * (2 * b * b * b - 9 * a * b * c + 27 * a * a * d - a * sqrt(-27 * discriminant)), (1.0 / 3.0));
			}
			ts = (-b / (3 * a)) - (1 / (3 * a)) * ts_p1 - (1 / (3 * a)) * ts_p2;
		}
		else
		{
			// 2 or 3 solutions, use Newton root approximation and choose the closest point on C(t)
			// a) roots of 1st derivative of cubic function -> get estimates for initial values for Newton method
			float p2 = b / (3 * a);
			float q = c / (3 * a);
			std::vector<float> tis; // initial values for Newton method

			if (p2 * p2 - q < 0) // no real roots in derivative? -> one real root for cubic function
			{
				tis.push_back(0.5); // should probably never happen because discriminant would be < 0
			}
			else if (p2 * p2 - q == 0) // one double real root in derivative -> one real root for cubic function
			{
				tis.push_back(-p2 - 0.25);
				tis.push_back(-p2 + 0.25);
			}
			else
			{
				// two distinct real roots in derivative -> three distinct real roots for cubic function
				float t1 = -p2 - std::sqrt(p2 * p2 - q);
				float t2 = -p2 + std::sqrt(p2 * p2 - q); // t1 < t2
				tis.push_back(t1 - 0.25);
				tis.push_back((t1 + t2) / 2);
				tis.push_back(t2 + 0.25);
			}

			// b) Newton method for each initial value and comparison for
			float tss = 0;
			long mindist = 1e10;
			for (unsigned int ii = 0; ii < tis.size(); ii++)
			{
				tss = tis[ii];
				float i = 0;
				float f = a * tss * tss * tss + b * tss * tss + c * tss + d;
				while (i < 100 && std::abs(f) > 1e-5)
				{
					tss = tss - f / (3 * a * tss * tss + 2 * b * tss + c);
					f = a * tss * tss * tss + b * tss * tss + c * tss + d;
					i = i + 1;
				}

				// get closest distance if multiple real roots present
				float dist_tss = ((curve.at<float> (0, 0) * tss * tss + curve.at<float> (0, 1) * tss + curve.at<float> (0, 2) - Q.x) * (curve.at<float> (0, 0)
						* tss * tss + curve.at<float> (0, 1) * tss + curve.at<float> (0, 2) - Q.x) + (curve.at<float> (1, 0) * tss * tss + curve.at<float> (1,
						1) * tss + curve.at<float> (1, 2) - Q.y) * (curve.at<float> (1, 0) * tss * tss + curve.at<float> (1, 1) * tss + curve.at<float> (1, 2)
						- Q.y)); // length of line between C(tss) and Q

				if (mindist > dist_tss && std::abs(tss) < 10)
				{
					ts = tss;
					mindist = dist_tss;
				}
			}
		}
	}

	// closest point to Q on C(t)
	float Csx = curve.at<float> (0, 0) * ts * ts + curve.at<float> (0, 1) * ts + curve.at<float> (0, 2);
	float Csy = curve.at<float> (1, 0) * ts * ts + curve.at<float> (1, 1) * ts + curve.at<float> (1, 2);

	float shortestDistance = std::sqrt((Csx - Q.x) * (Csx - Q.x) + (Csy - Q.y) * (Csy - Q.y));

	return std::pair<float, float>(shortestDistance, ts);
}

float DetectText::getBezierLength(cv::Mat curve, float mint, float maxt)
{
	// compute length of model curve
	float length = 1; // starts at 1 'cause in for-loop below the distance when C changes is counted -> C(mint) is missed

	// C(t) = At²+Bt+D
	cv::Point previousC((int) std::floor(curve.at<float> (0, 2) + 0.5), (int) std::floor(curve.at<float> (1, 2) + 0.5));

	for (double t = mint; t < maxt; t += 0.0001)
	{
		cv::Point C((int) std::floor((curve.at<float> (0, 0) * t * t + curve.at<float> (0, 1) * t + curve.at<float> (0, 2)) + 0.5),
				(int) std::floor((curve.at<float> (1, 0) * t * t + curve.at<float> (1, 1) * t + curve.at<float> (1, 2)) + 0.5));
		if (previousC.x != C.x && previousC.y != C.y)
			length += 1.41421356;
		else if (previousC.x != C.x)
			length++;
		else if (previousC.y != C.y)
			length++;
		previousC = C;
	}
	return length;
}

void DetectText::transformBezier(cv::Rect newR, cv::Mat curve, cv::Mat & transformedImage, float mint, float maxt)
{
	int method = 1;

	cv::Mat output = transformedImage.clone();

	// get background color of input image
	bgr bg_clr = firstPass_ ? findBorderColor(newR, BRIGHT) : findBorderColor(newR, DARK);

	// Nearest neighbor
	if (method == 1)
	{
		// Point on curve at last iteration
		cv::Point previousC(-1, -1);

		// x on output image
		float x = 0;
		int previousX = -1;

		std::vector<int> skippedCols; // because length > Σ points of bézierline, there are columns without a corresponding bézier point, these entire columns are interpolated in the end

		// t along the bézier curve
		for (float t = mint; t <= maxt; t += 0.0001)
		{
			cv::Point C((int) std::floor((curve.at<float> (0, 0) * t * t + curve.at<float> (0, 1) * t + curve.at<float> (0, 2)) + 0.5),
					(int) std::floor((curve.at<float> (1, 0) * t * t + curve.at<float> (1, 1) * t + curve.at<float> (1, 2)) + 0.5));

			// if C(t) is a new point on the curve
			if (C.x != previousC.x || C.y != previousC.y)
			{
				// Unit vector
				cv::Point2f Nt(2 * curve.at<float> (1, 0) * t + curve.at<float> (1, 1), (-1) * (2 * curve.at<float> (0, 0) * t + curve.at<float> (0, 1)));
				float absNt = std::sqrt(Nt.x * Nt.x + Nt.y * Nt.y);
				Nt = Nt * (1 / absNt);

				int i = 0;

				// fill upper side of transformedImage
				for (int y = std::floor(transformedImage.rows / 2.0); y >= 0; y--)
				{
					// Point in direction of unit vector
					cv::Point2f cNormal(C.x + i * Nt.x, C.y + i * Nt.y);
					cv::Point cNormalInteger = cNormal;

					// new point in normal direction found..
					bgr clr;

					// if point is not outside image
					if (!(cNormalInteger.x < 0 || cNormalInteger.x >= originalImage_.cols || cNormalInteger.y < 0 || cNormalInteger.y >= originalImage_.rows))
						clr = originalImage_.at<bgr> (cNormalInteger.y, cNormalInteger.x);
					else
						clr = bg_clr;

					output.at<bgr> (y, x) = clr;

					i++;
				}

				// same routine in the other direction
				Nt = Nt * (-1);
				i = 0;

				// fill down side of transformedImage
				for (int y = std::floor(transformedImage.rows / 2.0); y < transformedImage.rows; y++)
				{
					cv::Point2f cNormal(C.x + i * Nt.x, C.y + i * Nt.y);
					cv::Point cNormalInteger = cNormal;

					// new point in normal direction found
					bgr clr;
					if (!(cNormalInteger.x < 0 || cNormalInteger.x >= originalImage_.cols || cNormalInteger.y < 0 || cNormalInteger.y >= originalImage_.rows))
						clr = originalImage_.at<bgr> (cNormalInteger.y, cNormalInteger.x);
					else
						clr = bg_clr;

					output.at<bgr> (y, x) = clr;

					if (std::floor(x) - std::floor(previousX) > 1)
						skippedCols.push_back(std::floor(x - 1));

					i++;
				}
				// next pixel of the resulting image in x-direction
				previousX = x;
				if (previousC.x != C.x && previousC.y != C.y)
					x += 1.41421356;
				else
					x++;
			}
			previousC = C;
		}

		for (unsigned int x = 0; x < skippedCols.size(); x++)
			for (int y = 0; y < output.rows; y++)
			{
				bgr clr(0, 0, 0);
				clr.r = clr.r + 0.5 * output.at<bgr> (y, skippedCols[x] - 1).r + 0.5 * output.at<bgr> (y, skippedCols[x] + 1).r;
				clr.g = clr.g + 0.5 * output.at<bgr> (y, skippedCols[x] - 1).g + 0.5 * output.at<bgr> (y, skippedCols[x] + 1).g;
				clr.b = clr.b + 0.5 * output.at<bgr> (y, skippedCols[x] - 1).b + 0.5 * output.at<bgr> (y, skippedCols[x] + 1).b;
				output.at<bgr> (y, skippedCols[x]) = clr;
			}
	}

	// Bilinear filtering
	if (method == 2)
	{
		cv::Point previousC(-1, -1);
		float x = 0;
		int previousX = -1;

		std::vector<int> skippedCols; // because length > Σ points of bézierline, there are columns without a corresponding bézier point, these entire columns are interpolated in the end

		for (float t = mint; t <= maxt; t += 0.0001)
		{
			cv::Point C((int) std::floor((curve.at<float> (0, 0) * t * t + curve.at<float> (0, 1) * t + curve.at<float> (0, 2)) + 0.5),
					(int) std::floor((curve.at<float> (1, 0) * t * t + curve.at<float> (1, 1) * t + curve.at<float> (1, 2)) + 0.5));
			cv::Point2f previousCNormal(C.x, C.y);

			// new point on bézier curve
			if (previousC != C)
			{
				cv::Point2f CNormal;
				cv::Point2f Nt(2 * curve.at<float> (1, 0) * t + curve.at<float> (1, 1), (-1) * (2 * curve.at<float> (0, 0) * t + curve.at<float> (0, 1)));
				float absNt = std::sqrt(Nt.x * Nt.x + Nt.y * Nt.y);

				// above middle line
				for (int y = std::floor(transformedImage.rows / 2.0); y >= 0; y--)
				{
					int count = std::floor(transformedImage.rows / 2) - y;
					CNormal.x = C.x + count * (Nt.x / absNt);
					CNormal.y = C.y + count * (Nt.y / absNt);

					bgr meanClr(0, 0, 0);

					// if point lies not outside of originalImage
					if (!(CNormal.x < 0 || std::floor(CNormal.x + 0.5) >= originalImage_.cols || CNormal.y < 0 || std::floor(CNormal.y + 0.5)
							>= originalImage_.rows))
					{
						cv::Point cNormal1, cNormal2, cNormal3, cNormal4;

						// check if calculated point lies not directly on pixel
						if (CNormal.x - (int) std::floor(CNormal.x + 0.5) != 0 || CNormal.y - (int) std::floor(CNormal.y + 0.5) != 0)
						{
							// calculate  mean color value of 4 nearest pixels, weighted with distance
							cNormal1.x = std::floor(CNormal.x);
							cNormal1.y = std::floor(CNormal.y);
							cNormal2.x = std::floor(CNormal.x);
							cNormal2.y = std::ceil(CNormal.y);
							cNormal3.x = std::ceil(CNormal.x);
							cNormal3.y = std::floor(CNormal.y);
							cNormal4.x = std::ceil(CNormal.x);
							cNormal4.y = std::ceil(CNormal.y);

							CNormal.x -= std::floor(CNormal.x);
							CNormal.y -= std::floor(CNormal.y);

							meanClr.r = originalImage_.at<bgr> (cNormal1).r * (1 - CNormal.x) * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal3).r
									* CNormal.x * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal2).r * (1 - CNormal.x) * CNormal.y
									+ originalImage_.at<bgr> (cNormal4).r * CNormal.x * CNormal.y;
							meanClr.g = originalImage_.at<bgr> (cNormal1).g * (1 - CNormal.x) * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal3).g
									* CNormal.x * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal2).g * (1 - CNormal.x) * CNormal.y
									+ originalImage_.at<bgr> (cNormal4).g * CNormal.x * CNormal.y;
							meanClr.b = originalImage_.at<bgr> (cNormal1).b * (1 - CNormal.x) * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal3).b
									* CNormal.x * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal2).b * (1 - CNormal.x) * CNormal.y
									+ originalImage_.at<bgr> (cNormal4).b * CNormal.x * CNormal.y;
						}
						else
							// CNormal(float point) lies directly on a pixel -> just take his color
							meanClr = originalImage_.at<bgr> (CNormal.y, CNormal.x);
					}
					else
						// if point that is going to be copied lies outside of original image
						meanClr = bg_clr;

					output.at<bgr> (y, x) = meanClr;
					count++;
					previousCNormal = CNormal;
				}

				// opposite direction
				Nt *= -1;

				// below middle line
				for (int y = std::floor(transformedImage.rows / 2); y < transformedImage.rows; y++)
				{
					int count = y - std::floor(transformedImage.rows / 2);
					CNormal.x = C.x + count * (Nt.x / absNt);
					CNormal.y = C.y + count * (Nt.y / absNt);

					bgr meanClr(0, 0, 0);

					if (!(CNormal.x < 0 || std::floor(CNormal.x + 0.5) >= originalImage_.cols || std::floor(CNormal.y + 0.5) < 0 || std::floor(CNormal.y + 0.5)
							>= originalImage_.rows))
					{
						// check if calculated point lies not directly on pixel

						cv::Point cNormal1, cNormal2, cNormal3, cNormal4;
						if (CNormal.x - (int) std::floor(CNormal.x + 0.5) != 0 || CNormal.y - (int) std::floor(CNormal.y + 0.5) != 0)
						{
							cNormal1.x = std::floor(CNormal.x);
							cNormal1.y = std::floor(CNormal.y);
							cNormal2.x = std::floor(CNormal.x);
							cNormal2.y = std::ceil(CNormal.y);
							cNormal3.x = std::ceil(CNormal.x);
							cNormal3.y = std::floor(CNormal.y);
							cNormal4.x = std::ceil(CNormal.x);
							cNormal4.y = std::ceil(CNormal.y);
							CNormal.x -= std::floor(CNormal.x);
							CNormal.y -= std::floor(CNormal.y);

							meanClr.r = originalImage_.at<bgr> (cNormal1).r * (1 - CNormal.x) * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal3).r
									* CNormal.x * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal2).r * (1 - CNormal.x) * CNormal.y
									+ originalImage_.at<bgr> (cNormal4).r * CNormal.x * CNormal.y;
							meanClr.g = originalImage_.at<bgr> (cNormal1).g * (1 - CNormal.x) * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal3).g
									* CNormal.x * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal2).g * (1 - CNormal.x) * CNormal.y
									+ originalImage_.at<bgr> (cNormal4).g * CNormal.x * CNormal.y;
							meanClr.b = originalImage_.at<bgr> (cNormal1).b * (1 - CNormal.x) * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal3).b
									* CNormal.x * (1 - CNormal.y) + originalImage_.at<bgr> (cNormal2).b * (1 - CNormal.x) * CNormal.y
									+ originalImage_.at<bgr> (cNormal4).b * CNormal.x * CNormal.y;

						}
						else
							meanClr = originalImage_.at<bgr> (CNormal.y, CNormal.x);
					}
					else
						meanClr = bg_clr;

					output.at<bgr> (y, x) = meanClr;

					if (std::floor(x) - std::floor(previousX) > 1)
						skippedCols.push_back(std::floor(x - 1));

					count++;
				}
				previousX = x;
				if (previousC.x != C.x && previousC.y != C.y)
					x += 1.41421356;
				else
					x++;
			}
			previousC = C;
		}

		for (unsigned int x = 0; x < skippedCols.size(); x++)
		{
			for (int y = 0; y < output.rows; y++)
			{
				bgr clr(0, 0, 0);
				clr.r = clr.r + 0.5 * output.at<bgr> (y, skippedCols[x] - 1).r + 0.5 * output.at<bgr> (y, skippedCols[x] + 1).r;
				clr.g = clr.g + 0.5 * output.at<bgr> (y, skippedCols[x] - 1).g + 0.5 * output.at<bgr> (y, skippedCols[x] + 1).g;
				clr.b = clr.b + 0.5 * output.at<bgr> (y, skippedCols[x] - 1).b + 0.5 * output.at<bgr> (y, skippedCols[x] + 1).b;
				output.at<bgr> (y, skippedCols[x]) = clr;
			}
		}
	}
	transformedImage = output;
}

unsigned int DetectText::calculateRealLetterHeight(cv::Point2f p, cv::Rect r, cv::Point2f normal)
{
	cv::Point2f nextPoint = p + normal;
	unsigned int height = 0;

	while (nextPoint.inside(r))
	{
		height++;
		nextPoint = nextPoint + normal;
	}

	height *= 2;
	return height;
}

void DetectText::setParams(ros::NodeHandle & nh)
{
	int proc_meth = 0;
	nh.getParam("processing_method", proc_meth);
	this->processing_method_ = (ProcessingMethod)proc_meth;
	nh.getParam("transformImages", this->transformImages);
	nh.getParam("smoothImage", this->smoothImage);
	nh.getParam("maxStrokeWidthParameter", this->maxStrokeWidthParameter);
	nh.getParam("useColorEdge", this->useColorEdge);
	nh.getParam("cannyThreshold1", this->cannyThreshold1);
	nh.getParam("cannyThreshold2", this->cannyThreshold2);
	nh.getParam("compareGradientParameter", this->compareGradientParameter_);
	nh.getParam("swCompareParameter", this->swCompareParameter);
	nh.getParam("colorCompareParameter", this->colorCompareParameter);
	nh.getParam("maxLetterHeight_", this->maxLetterHeight_);
	nh.getParam("varianceParameter", this->varianceParameter);
	nh.getParam("diagonalParameter", this->diagonalParameter);
	nh.getParam("pixelCountParameter", this->pixelCountParameter);
	nh.getParam("innerLetterCandidatesParameter", this->innerLetterCandidatesParameter);
	nh.getParam("clrComponentParameter", this->clrComponentParameter);
	nh.getParam("distanceRatioParameter", this->distanceRatioParameter);
	nh.getParam("medianSwParameter", this->medianSwParameter);
	nh.getParam("diagonalRatioParamter", this->diagonalRatioParamter);
	nh.getParam("grayClrParameter", this->grayClrParameter);
	nh.getParam("clrSingleParameter", this->clrSingleParameter);
	nh.getParam("areaParameter", this->areaParameter);
	nh.getParam("pixelParameter", this->pixelParameter);
	nh.getParam("inlierDistanceThresholdFactor", this->inlierDistanceThresholdFactor_);
	nh.getParam("p", this->p);
	nh.getParam("maxE", this->maxE);
	nh.getParam("minE", this->minE);
	nh.getParam("bendParameter", this->bendParameter);
	nh.getParam("distanceParameter", this->distanceParameter);
	nh.getParam("sigma_sharp", this->sigma_sharp);
	nh.getParam("threshold_sharp", this->threshold_sharp);
	nh.getParam("amount_sharp", this->amount_sharp);
	nh.getParam("result_", this->result_);
	nh.getParam("showEdge", this->debug["showEdge"]);
	nh.getParam("showSWT", this->debug["showSWT"]);
	nh.getParam("showLetterCandidates", this->debug["showLetterCandidates"]);
	nh.getParam("showLetters", this->debug["showLetters"]);
	nh.getParam("showPairs", this->debug["showPairs"]);
	nh.getParam("showChains", this->debug["showChains"]);
	nh.getParam("showBreakLines", this->debug["showBreakLines"]);
	nh.getParam("showWords", this->debug["showWords"]);
	nh.getParam("showCriterions", this->debug["showCriterions"]);
	nh.getParam("showBezier", this->debug["showBezier"]);
	nh.getParam("showRansac", this->debug["showRansac"]);
	nh.getParam("showNeighborMerging", this->debug["showNeighborMerging"]);
	nh.getParam("showResult", this->debug["showResult"]);

	std::cout << "processing_method:" << processing_method_ << std::endl;
	std::cout << "smoothImage:" << smoothImage << std::endl;
	std::cout << "maxStrokeWidthParameter:" << maxStrokeWidthParameter << std::endl;
	std::cout << "useColorEdge:" << useColorEdge << std::endl;
	std::cout << "cannyThreshold1:" << cannyThreshold1 << std::endl;
	std::cout << "cannyThreshold2:" << cannyThreshold2 << std::endl;
	std::cout << "compareGradientParameter:" << compareGradientParameter_ << std::endl;
	std::cout << "swCompareParameter:" << swCompareParameter << std::endl;
	std::cout << "colorCompareParameter:" << colorCompareParameter << std::endl;
	std::cout << "maxLetterHeight_:" << maxLetterHeight_ << std::endl;
	std::cout << "varianceParameter:" << varianceParameter << std::endl;
	std::cout << "diagonalParameter:" << diagonalParameter << std::endl;
	std::cout << "pixelCountParameter:" << pixelCountParameter << std::endl;
	std::cout << "innerLetterCandidatesParameter:" << innerLetterCandidatesParameter << std::endl;
	std::cout << "clrComponentParameter:" << clrComponentParameter << std::endl;
	std::cout << "distanceRatioParameter:" << distanceRatioParameter << std::endl;
	std::cout << "medianSwParameter:" << medianSwParameter << std::endl;
	std::cout << "diagonalRatioParamter:" << diagonalRatioParamter << std::endl;
	std::cout << "grayClrParameter:" << grayClrParameter << std::endl;
	std::cout << "clrSingleParameter:" << clrSingleParameter << std::endl;
	std::cout << "areaParameter:" << areaParameter << std::endl;
	std::cout << "pixelParameter:" << pixelParameter << std::endl;
	std::cout << "inlierDistanceThresholdFactor:" << inlierDistanceThresholdFactor_ << std::endl;
	std::cout << "p:" << p << std::endl;
	std::cout << "maxE:" << maxE << std::endl;
	std::cout << "minE:" << minE << std::endl;
	std::cout << "bendParameter:" << bendParameter << std::endl;
	std::cout << "distanceParameter:" << distanceParameter << std::endl;



}

// Methods not used at the moment:

bool DetectText::spatialOrder(cv::Rect a, cv::Rect b)
{
	return a.y < b.y;
}

bool DetectText::pairOrder(Pair i, Pair j)
{
	return i.left < j.left;
}

bool DetectText::pointYOrder(cv::Point a, cv::Point b)
{
	return a.y < b.y;
}

void DetectText::overlapBoundingBoxes(std::vector<cv::Rect>& boundingBoxes)
{
	std::vector<cv::Rect> bigBoxes;
	// Merging BoundingBoxes
	cv::Mat tempMap(grayImage_.size(), CV_32FC1, cv::Scalar(0));
	for (size_t i = 0; i < boundingBoxes.size(); i++)
	{
		cv::Rect *rect = &boundingBoxes[i];
		for (int y = rect->y; y < rect->y + rect->height; y++)
			for (int x = rect->x; x < rect->x + rect->width; x++)
			{
				tempMap.at<float> (y, x) = 50;
			}
	}

	for (size_t i = 0; i < boundingBoxes.size(); i++)
	{
		if (tempMap.at<float> (boundingBoxes[i].y + 1, boundingBoxes[i].x + 1) != 50)
			continue;

		cv::Rect rect;
		cv::floodFill(tempMap, cv::Point(boundingBoxes[i].x, boundingBoxes[i].y), i + 100, &rect);
		int padding = 0;

		// add padding around each box
		int minX = std::max(0, rect.x - padding);
		int minY = std::max(0, rect.y - padding);
		int maxX = std::min(grayImage_.cols, rect.x + rect.width + padding);
		int maxY = std::min(grayImage_.rows, rect.y + rect.height + padding);
		bigBoxes.push_back(cv::Rect(minX, minY, maxX - minX, maxY - minY));
	}

	boundingBoxes = bigBoxes;
}


void DetectText::DFT(cv::Mat& input)
{
	// Fourier Transform of image
	cv::Mat I;
	cv::cvtColor(input, I, CV_RGB2GRAY);

	cv::Mat padded; //expand input image to optimal size
	int m = cv::getOptimalDFTSize(I.rows);
	int n = cv::getOptimalDFTSize(I.cols); // on the border add zero values
	cv::copyMakeBorder(I, padded, 0, m - I.rows, 0, n - I.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

	cv::Mat planes[] =
	{ cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F) };
	cv::Mat complexI;
	cv::merge(planes, 2, complexI); // Add to the expanded another plane with zeros

	dft(complexI, complexI); // this way the result may fit in the source matrix

	// compute the magnitude and switch to logarithmic scale
	// => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
	split(complexI, planes); // planes[0] = Re(DFT(I), planes[1] = Im(DFT(I))
	magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude
	cv::Mat magI = planes[0];

	//magI += cv::Scalar::all(1); // switch to logarithmic scale
	//log(magI, magI);

	// crop the spectrum, if it has an odd number of rows or columns
	magI = magI(cv::Rect(0, 0, magI.cols & -2, magI.rows & -2));

	// rearrange the quadrants of Fourier image  so that the origin is at the image center
	int cx = magI.cols / 2;
	int cy = magI.rows / 2;

	cv::Mat q0(magI, cv::Rect(0, 0, cx, cy)); // Top-Left - Create a ROI per quadrant
	cv::Mat q1(magI, cv::Rect(cx, 0, cx, cy)); // Top-Right
	cv::Mat q2(magI, cv::Rect(0, cy, cx, cy)); // Bottom-Left
	cv::Mat q3(magI, cv::Rect(cx, cy, cx, cy)); // Bottom-Right

	cv::Mat tmp; // swap quadrants (Top-Left with Bottom-Right)
	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);

	q1.copyTo(tmp); // swap quadrant (Top-Right with Bottom-Left)
	q2.copyTo(q1);
	tmp.copyTo(q2);

	normalize(magI, magI, 0, 1, CV_MINMAX); // Transform the matrix with float values into a
	// viewable image form (float between values 0 and 1).

	// imshow("Input Image", I); // Show the result
	imshow("spectrum magnitude", magI * 10);
	for (int y = 0; y < magI.rows; y++)
	{
		for (int x = 0; x < magI.cols; x++)
		{
			// std::cout << (unsigned int) magI.at<unsigned char> (y,x) << " ";
		}
		//  std::cout << std::endl;
	}
	// waitKey();
}

cv::Mat DetectText::filterPatch(const cv::Mat& patch)
{
	cv::Mat result;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(patch.cols / 3, patch.rows / 3));
	cv::morphologyEx(patch, result, cv::MORPH_TOPHAT, element);
	return result;
}


void DetectText::breakLinesIntoWords(std::vector<TextRegion>& textRegions)
{
	// break text into separate lines without destroying letter boxes
	cv::Mat output;
	if (debug["showWords"])
		output = originalImage_.clone();		// todo: make this a debug parameter

	std::vector<TextRegion> wordRegions;

	std::string breakingWordsDisplayName = "breaking words";
	if (firstPass_)
		breakingWordsDisplayName = "breaking bright words";
	else
		breakingWordsDisplayName = "breaking dark words";

	// used later for a parametric function
	double p = 0.75;
	double a1=-1/(p*p), b1=2/p, c1=0;
	double a2=1/(-p*p+2*p-1), b2=-2*p/(-p*p+2*p-1), c2=(2*p-1)/(-p*p+2*p-1);

	//For every boundingBox:
	for (unsigned int textRegionIndex = 0; textRegionIndex < textRegions.size(); textRegionIndex++)
	{
		std::vector<Letter>& letters = textRegions[textRegionIndex].letters;

		//which Letters belong to the current boundingBox, average letter size, stddev letter size
		std::vector<int> currentLetters; // which ones of all found letters in the text region belong to the current boundingBox
		double averageLetterSize = 0.;
		for (unsigned int i = 0; i < letters.size(); i++)
		{
			currentLetters.push_back(i);
			averageLetterSize += letters[i].diameter;
//			if (letters[i].diameter != sqrt(letters[i].boundingBox.width*letters[i].boundingBox.width+letters[i].boundingBox.height*letters[i].boundingBox.height))
//				std::cout << "                            ERROR: Wrong diameter: letters[i].diameter=" << letters[i].diameter << "   computed=" << sqrt(letters[i].boundingBox.width*letters[i].boundingBox.width+letters[i].boundingBox.height*letters[i].boundingBox.height) << std::endl;
		}
		averageLetterSize /= (double)letters.size();
		double stddevLetterSize = 0.;
		for (unsigned int i = 0; i < letters.size(); i++)
			stddevLetterSize += (letters[i].diameter-averageLetterSize)*(letters[i].diameter-averageLetterSize);
		stddevLetterSize = sqrt(stddevLetterSize/(double)letters.size());

		// Check 1: the size variance of letters should not be too large
		double textBoxDiameter = sqrt(textRegions[textRegionIndex].boundingBox.width*textRegions[textRegionIndex].boundingBox.width + textRegions[textRegionIndex].boundingBox.height*textRegions[textRegionIndex].boundingBox.height);
		//std::cout << "Size variance check: averageLetterSize=" << averageLetterSize << "   stddevLetterSize=" << stddevLetterSize << "   textBoxDiameter=" << textBoxDiameter << std::endl;
//xx		if (stddevLetterSize > 0.33*averageLetterSize)
//xx			continue;

		// Check 2:
		//if (textRegions[textRegionIndex].boundingBox.width < 1.7 * textRegions[textRegionIndex].letters.size() * textRegions[textRegionIndex].boundingBox.height)

		// Determine distance to nearest neighbor letter
		std::vector<Pair> letterDistances;
		for (unsigned int i = 0; i < currentLetters.size(); i++)
		{
			int smallestDistanceEdge = 100000;	// inf
			int smallestDistanceEdgeIndex = 0;
			for (unsigned int j = 0; j < currentLetters.size(); j++)
			{
				float distanceEdge = 100000;

				if (letters[currentLetters[j]].boundingBox.x > letters[currentLetters[i]].boundingBox.x)
					distanceEdge = letters[currentLetters[j]].boundingBox.x - letters[currentLetters[i]].boundingBox.x - letters[currentLetters[i]].boundingBox.width;

				if (distanceEdge < smallestDistanceEdge)
				{
					smallestDistanceEdge = distanceEdge;
					smallestDistanceEdgeIndex = j;
				}
			}

			if (smallestDistanceEdge != 100000) //100000 distance means there is no other letter on the right, its the last letter of a line
				letterDistances.push_back(Pair(letters[currentLetters[smallestDistanceEdgeIndex]].boundingBox.x, smallestDistanceEdge, 0., 0.));

			if (debug["showWords"])
			{
				if (firstPass_ == true)
				{
					cv::rectangle(output, letters[currentLetters[i]].boundingBox, cv::Scalar(255, 255, 255), 1);
					cv::rectangle(output, letters[currentLetters[smallestDistanceEdgeIndex]].boundingBox, cv::Scalar(255, 255, 255), 1);
				}
				else
				{
					cv::rectangle(output, letters[currentLetters[i]].boundingBox, cv::Scalar(0, 0, 0), 1);
					cv::rectangle(output, letters[currentLetters[smallestDistanceEdgeIndex]].boundingBox, cv::Scalar(0, 0, 0), 1);
				}
				cv::rectangle(output, textRegions[textRegionIndex].boundingBox, cv::Scalar(155, 155, 155), 2);
				cv::imshow(breakingWordsDisplayName.c_str(), output);
				cvMoveWindow(breakingWordsDisplayName.c_str(), 0, 0);
			}
		}

		// sort letterDistances according to their order of occurence from left to right
		sort(letterDistances.begin(), letterDistances.end(), DetectText::pairOrder);
		if (debug["showWords"])
			for (unsigned int a = 0; a < letterDistances.size(); a++)
				std::cout << "X-Coord: " << letterDistances[a].left << "     Distance: " << letterDistances[a].right << std::endl;

		// cluster distances
//		// tolerate negative distances down to -2, compute mean distance
//		double meanDistanceEdge = 0.0;		// mean distance between the available boxes
//		for (unsigned int i = 0; i < letterDistances.size(); i++)
//		{
//			if (letterDistances[i].right == -1 || letterDistances[i].right == -2)
//				letterDistances[i].right = 0;
//			meanDistanceEdge += letterDistances[i].right;
//		}
//		meanDistanceEdge /= (double)letterDistances.size();
//
		// find smallest distance
		int smallestWordBreak = 1000000, biggestWordBreak = -1000000;
		for (unsigned int i = 0; i < letterDistances.size(); i++)
		{
			if (letterDistances[i].right < smallestWordBreak)
				smallestWordBreak = letterDistances[i].right;
			if (letterDistances[i].right > biggestWordBreak)
				biggestWordBreak = letterDistances[i].right;
		}

		// draw histogram
		if (false)
		{
			if (letterDistances.size() > 0)
			{
				std::vector<double> histogram(biggestWordBreak-smallestWordBreak+1, 0.);
				for (unsigned int i = 0; i < letterDistances.size(); i++)
					histogram[letterDistances[i].right-smallestWordBreak]++;
				int displayWidth = 400, displayHeight = 100;
				cv::Mat display = cv::Mat::zeros(displayHeight, displayWidth, CV_8UC3);
				for (unsigned int i=0; i<histogram.size(); i++)
				{
					std::cout << "   " << histogram[i] << std::endl;
					cv::rectangle(display, cv::Point(i*(double)displayWidth/(double)histogram.size(), displayHeight-5*histogram[i]), cv::Point((i+1)*(double)displayWidth/(double)histogram.size(), displayHeight), cv::Scalar(255,128,0), -1);
				}
				cv::imshow("histogram", display);
				cv::waitKey();
			}
		}

		// add some feasibility checks
		// todo
		// 1. the height variance of letters should not be too large


		// break line with respect to letter distances using non-parametric mode seeking with mean shift
		//  prepare mean shift set
		std::vector<double> meanShiftSet(letterDistances.size());		// contains the letter sizes
		for (unsigned int i = 0; i < letterDistances.size(); i++)
			meanShiftSet[i] = letterDistances[i].right;
		//  compute median of letter distances
		double medianLetterSize = 0;
		std::multiset<double> orderedLetterSizes;
		for (unsigned int i = 0; i < letters.size(); i++)
			orderedLetterSizes.insert(sqrt(letters[i].boundingBox.width*letters[i].boundingBox.width + letters[i].boundingBox.height*letters[i].boundingBox.height));
		std::multiset<double>::iterator it = orderedLetterSizes.begin();
		for (int i=0; i<(int)orderedLetterSizes.size()/2; i++, it++);
		medianLetterSize = *it;
		double meanShiftBandwidth = 0.5 * 35./medianLetterSize*35./medianLetterSize;
		//std::cout << "bandwidth = " << meanShiftBandwidth << std::endl;
		//	compute mean shift segmentation
		std::vector< std::vector<int> > convergenceSets;
		std::vector<double> convergencePoints;
		if (meanShiftSet.size() > 0)
			MeanShiftSegmentation1D(meanShiftSet, convergencePoints, convergenceSets, meanShiftBandwidth, 100);

		// compute threshold and break line into words
		bool breakBox = true; // shall box be broken into separate boxes?
		double thresholdDistance = -1e10;
		std::vector<double> bestThresholds;
		if (convergencePoints.size() < 2)
			breakBox = false;
		else
		{
			// create data structure of convergence points and number of points that converged to them
			std::map<double, unsigned int > convergencePointsAndSupporters;	// contains the convergence points (double) and the number of points that converged to that point (int)
			for (unsigned int i=0; i<convergencePoints.size(); i++)
				convergencePointsAndSupporters[convergencePoints[i]] = convergenceSets[i].size();

			// compute appropriate distance threshold for line breaks
			std::map<double, unsigned int>::iterator it=convergencePointsAndSupporters.end();
			it--;
			if (it->second > 0.5*letterDistances.size())
				breakBox = false;	// not too many word breaks possible
			else
			{
				// divide histogram into two subsets, letter distances and word distances, using Otsu algorithm
				double bestSigma = 0.;
				for (double t = smallestWordBreak-0.5; t<biggestWordBreak; t += 1.0)
				{
					double w1 = 0., w2 = 0.;	// weights
					double mu1 = 0., mu2 = 0.;	// means
					double normalizer = 0.;
					for (it = convergencePointsAndSupporters.begin(); it != convergencePointsAndSupporters.end(); it++)
					{
						if (it->first < t)
						{
							w1 += (double)it->second;
							mu1 += (double)it->second*it->first;
						}
						else
						{
							w2 += (double)it->second;
							mu2 += (double)it->second*it->first;
						}
						normalizer += (double)it->second;
					}
					if (w1==0 || w2==0)
						continue;
					mu1 /= w1;	// normalizing w1 afterwards avoids normalizing mu1 here explicitly (this way mu1 is normalized by division through unnormalized w1)
					mu2 /= w2;
					w1 /= normalizer;
					w2 /= normalizer;
					// sigma_b
					double sigma_b = (w1<=p ? a1*w1*w1 + b1*w1 + c1 : a2*w1*w1 + b2*w1 + c2)*/*w1*w2*/(mu1-mu2)*(mu1-mu2); // the term w1*w2 term is replaced as we do not aspire to reward similarly sized subsets but require set 2 to be smaller (as there are less word breaks than letters), the peak of the new weight reward function is p=(0..1) (see above), typically p=0.8 makes sense, i.e. every 5th transition is a word separation
					//std::cout << " t=" << t << "\t sigma_b=" << sigma_b << "\t w1=" << w1 << "\t w2=" << w2 << "\t mu1=" << mu1 << "\t mu2=" << mu2 << std::endl;
					if (sigma_b > bestSigma)
					{
						bestSigma = sigma_b;
						bestThresholds.clear();
						bestThresholds.push_back(t);
					}
					else if (sigma_b == bestSigma)
					{
						bestThresholds.push_back(t);
					}
				}
			}

			thresholdDistance = 0.;
			for (unsigned int i=0; i<bestThresholds.size(); i++)
				thresholdDistance += bestThresholds[i];
			thresholdDistance /= (double)bestThresholds.size();
			//std::cout << "thresholdDistance: " << thresholdDistance << std::endl;
		}

/*		if (debug["showWords"])
		{
			//std::cout << "line equation: p1=(" << textRegions[textRegionIndex].lineEquation.center.x << ", " << lineEquations[boxIndex].center.y << ")  normal=(" << lineEquations[boxIndex].size.width << ", " << lineEquations[boxIndex].size.height << ")" << std::endl;
			cv::line(output, cv::Point(textRegions[textRegionIndex].lineEquation.center.x, textRegions[textRegionIndex].lineEquation.center.y), cv::Point(textRegions[textRegionIndex].lineEquation.center.x+5000*textRegions[textRegionIndex].lineEquation.size.height, textRegions[textRegionIndex].lineEquation.center.y-5000*textRegions[textRegionIndex].lineEquation.size.width), cv::Scalar(255, 0, 0), 2);
			cv::line(output, cv::Point(textRegions[textRegionIndex].lineEquation.center.x, textRegions[textRegionIndex].lineEquation.center.y), cv::Point(textRegions[textRegionIndex].lineEquation.center.x-5000*textRegions[textRegionIndex].lineEquation.size.height, textRegions[textRegionIndex].lineEquation.center.y+5000*textRegions[textRegionIndex].lineEquation.size.width), cv::Scalar(255, 0, 0), 2);
		}

		if (crit == -1)	// no regular text region, throw away
		{
			if (debug["showWords"])
			{
				cv::rectangle(output, textRegions[textRegionIndex].boundingBox, cv::Scalar((0), (0), (255)), 2);
				cv::imshow(breakingWordsDisplayName.c_str(), output);
				cv::waitKey(0);
			}
			continue;
		}
		else if (crit == 1)		// one word, do not break
			breakBox = false;
*/

		// Breaking and saving broken words
		int start = textRegions[textRegionIndex].boundingBox.x;
		int wordBreakAt = start;
		int numberWordRegions = wordRegions.size();
		if (breakBox == true)
		{
			for (unsigned int r = 0; r < letterDistances.size(); r++)
			{
				bool letter = false;
				if (letterDistances[r].right < thresholdDistance)
					letter = true;

				// reaching a break between two words:
				if (letter == false)
				{
					wordBreakAt = letterDistances[r].left - letterDistances[r].right;// + shift;
					cv::Rect re(start, textRegions[textRegionIndex].boundingBox.y, abs(wordBreakAt - start), textRegions[textRegionIndex].boundingBox.height);
					start = letterDistances[r].left;
					TextRegion word;
					word.boundingBox = re;
					word.lineEquation = textRegions[textRegionIndex].lineEquation;
					word.qualityScore = textRegions[textRegionIndex].qualityScore;
					// word.letters = ; --> see below
					wordRegions.push_back(word);
				}
				// reaching the end
				if (r == letterDistances.size() - 1)
				{
					cv::Rect re(start, textRegions[textRegionIndex].boundingBox.y, abs((textRegions[textRegionIndex].boundingBox.x + textRegions[textRegionIndex].boundingBox.width) - start), textRegions[textRegionIndex].boundingBox.height);
					TextRegion word;
					word.boundingBox = re;
					word.lineEquation = textRegions[textRegionIndex].lineEquation;
					word.qualityScore = textRegions[textRegionIndex].qualityScore;
					// word.letters = ; --> see below
					wordRegions.push_back(word);
				}
			}
		}
		else
		{
			wordRegions.push_back(textRegions[textRegionIndex]);
		}

		// Make Boxes smaller to increase precision and recall
		unsigned int newNumberWordRegions = wordRegions.size();
		for (unsigned int makeSmallerIndex = numberWordRegions; makeSmallerIndex < newNumberWordRegions; makeSmallerIndex++)
		{
			unsigned int smallestX = 10000, smallestY = 10000, biggestX = 0, biggestY = 0;
			for (unsigned int letterIndex = 0; letterIndex < currentLetters.size(); letterIndex++)
				if ((wordRegions[makeSmallerIndex].boundingBox & letters[currentLetters[letterIndex]].boundingBox).area() / letters[currentLetters[letterIndex]].boundingBox.area() > 0.5)		// todo: might cause problems?
				{
					if ((unsigned int)letters[currentLetters[letterIndex]].boundingBox.x < smallestX)
						smallestX = letters[currentLetters[letterIndex]].boundingBox.x;
					if ((unsigned int)letters[currentLetters[letterIndex]].boundingBox.y < smallestY)
						smallestY = letters[currentLetters[letterIndex]].boundingBox.y;
					if ((unsigned int)letters[currentLetters[letterIndex]].boundingBox.x + letters[currentLetters[letterIndex]].boundingBox.width > biggestX)
						biggestX = letters[currentLetters[letterIndex]].boundingBox.x + letters[currentLetters[letterIndex]].boundingBox.width;
					if ((unsigned int)letters[currentLetters[letterIndex]].boundingBox.y + letters[currentLetters[letterIndex]].boundingBox.height > biggestY)
						biggestY = letters[currentLetters[letterIndex]].boundingBox.y + letters[currentLetters[letterIndex]].boundingBox.height;
					wordRegions[makeSmallerIndex].letters.push_back(letters[currentLetters[letterIndex]]);
				}
			if (smallestX == 10000)
				continue;

			cv::Rect smallerRe(smallestX, smallestY, biggestX - smallestX, biggestY - smallestY);

			if (debug["showWords"])
			{
				cv::rectangle(output, smallerRe, cv::Scalar((0), (255), (0)), 2);
				cv::imshow(breakingWordsDisplayName.c_str(), output);
				cv::waitKey(0);
			}
			wordRegions[makeSmallerIndex].boundingBox = smallerRe;
		}
	}

	textRegions = wordRegions;
}


//void DetectText::breakLinesIntoWords(std::vector<TextRegion>& textRegions, std::vector<double>& qualityScore)
//{
//	// break text into separate lines without destroying letter boxes
//	cv::Mat output;
//	if (debug["showWords"])
//		output = originalImage_.clone();		// todo: make this a debug parameter
//
//	qualityScore.clear();
//	qualityScore.resize(textRegions.size(), 0.);
//	std::vector<double> brokenWordsQualityScore;
//
//	std::vector<TextRegion> wordRegions;
//
//	std::string breakingWordsDisplayName = "breaking words";
//	if (firstPass_)
//		breakingWordsDisplayName = "breaking bright words";
//	else
//		breakingWordsDisplayName = "breaking dark words";
//
//	//For every boundingBox:
//	for (unsigned int textRegionIndex = 0; textRegionIndex < textRegions.size(); textRegionIndex++)
//	{
//		std::vector<Letter>& letters = textRegions[textRegionIndex].letters;
//
//		//which Letters belong to the current boundingBox
//		std::vector<int> innerLetters, currentLetters; // which ones of all found letters on the image belong to the current boundingBox
//		double averageLetterSize = 0.;
//		for (unsigned int i = 0; i < letters.size(); i++)
//		{
////			if ((textRegions[textRegionIndex].boundingBox & letters[i].boundingBox).area() == 1.0 * letters[i].boundingBox.area())	// provided by new data structure
////			{
//				currentLetters.push_back(i);	// todo: exact method was not better, is it now?
//				//innerLetters.push_back(i);
//				averageLetterSize += sqrt(letters[i].boundingBox.width*letters[i].boundingBox.width + letters[i].boundingBox.height*letters[i].boundingBox.height);
////			}
//		}
//		averageLetterSize /= (double)currentLetters.size();
////		averageLetterSize /= (double)innerLetters.size();
////		double inlierDistanceThreshold = averageLetterSize * (inlierDistanceThresholdFactor_+0.1);
////		for (unsigned int i = 0; i < innerLetters.size(); i++)
////		{
////			cv::Point2f centralBasePoint(letters[innerLetters[i]].x+0.5*letters[innerLetters[i]].width, letters[innerLetters[i]].y+letters[innerLetters[i]].height);
////			cv::Point2f diff = lineEquations[boxIndex].center - centralBasePoint;
////			float dist = fabs(diff.x*lineEquations[boxIndex].size.width + diff.y*lineEquations[boxIndex].size.height);
////			if (dist < inlierDistanceThreshold)
////				currentLetters.push_back(innerLetters[i]);
////		}
//
//
//		// Determine distance to nearest neighbor
//		std::vector<Pair> wordBreaks;
//		int smallestDistanceEdge = 100000; //inf
//		int smallestDistanceEdgeIndex = 0;
//		for (unsigned int i = 0; i < currentLetters.size(); i++)
//		{
//			smallestDistanceEdge = 100000;
//			for (unsigned int j = 0; j < currentLetters.size(); j++)
//			{
//				float distanceEdge = 100000;
//
//				if (letters[currentLetters[j]].boundingBox.x > letters[currentLetters[i]].boundingBox.x)
//					distanceEdge = letters[currentLetters[j]].boundingBox.x - letters[currentLetters[i]].boundingBox.x - letters[currentLetters[i]].boundingBox.width;
//
//				if (distanceEdge < smallestDistanceEdge)
//				{
//					smallestDistanceEdge = distanceEdge;
//					smallestDistanceEdgeIndex = j;
//				}
//			}
//
//			if (smallestDistanceEdge != 100000) //100000 distance means there is no other letter on the right, its the last letter of a line
//				wordBreaks.push_back(Pair(letters[currentLetters[smallestDistanceEdgeIndex]].boundingBox.x, smallestDistanceEdge));
//
//			if (debug["showWords"])
//			{
//				if (firstPass_ == true)
//				{
//					cv::rectangle(output, letters[currentLetters[i]].boundingBox, cv::Scalar((255), (255), (255)), 1);
//					cv::rectangle(output, letters[currentLetters[smallestDistanceEdgeIndex]].boundingBox, cv::Scalar((255), (255), (255)), 1);
//				}
//				else
//				{
//					cv::rectangle(output, letters[currentLetters[i]].boundingBox, cv::Scalar((0), (0), (0)), 1);
//					cv::rectangle(output, letters[currentLetters[smallestDistanceEdgeIndex]].boundingBox, cv::Scalar((0), (0), (0)), 1);
//				}
//				cv::rectangle(output, textRegions[textRegionIndex].boundingBox, cv::Scalar((155), (155), (155)), 2);
//				cv::imshow(breakingWordsDisplayName.c_str(), output);
//				cvMoveWindow(breakingWordsDisplayName.c_str(), 0, 0);
//			}
//		}
//
//		//Sort wordBreaks
//		sort(wordBreaks.begin(), wordBreaks.end(), DetectText::pairOrder);
//
//		//tolerate negative distances down to -2, compute mean distance
//		double meanDistanceEdge = 0.0;		// mean distance between the available boxes
//		for (unsigned int i = 0; i < wordBreaks.size(); i++)
//		{
//			if (wordBreaks[i].right == -1 || wordBreaks[i].right == -2)
//				wordBreaks[i].right = 0;
//			meanDistanceEdge += wordBreaks[i].right;
//		}
//		meanDistanceEdge /= (double)wordBreaks.size();
//
//		if (debug["showWords"])
//			for (unsigned int a = 0; a < wordBreaks.size(); a++)
//				std::cout << "X-Coord: " << wordBreaks[a].left << "     Distance: " << wordBreaks[a].right << std::endl;
//
//		// find smallest distance
//		int smallestWordBreak = 0;
//		for (unsigned int i = 0; i < wordBreaks.size(); i++)
//			if (wordBreaks[i].right < smallestWordBreak)
//				smallestWordBreak = wordBreaks[i].right;
//
//
//		//Histogram of distances between characters in every bounding box
//		int histWidth = 600;
//		int histHeight = wordBreaks.size() * 4 > 100 ? wordBreaks.size() : 100;
//		int max = 300;		// largest bin in histogram
//		int bin = 30;		// number of bins of the histogram
//
//		// Determine bin based on width of boundingBox
//		max = (int)std::max(50.0, 10*ceil((double)(meanDistanceEdge-smallestWordBreak)));
//		bin = std::max(6, (int)currentLetters.size()/3);
//		//max = (int)std::max(100.0, 4*ceil(boxes[boxIndex].width/(double)currentLetters.size()));		// todo: mittlere boxbreite und nicht die Anzahl verwenden
//		//bin = std::max(10, (int)currentLetters.size());
//
////		switch ((int)boxes[boxIndex].width / 100)				// todo: check
////		{
////		case 0:
////			bin = 60;	//60
////			break;
////		case 1:
////			bin = 60;	//60
////			break;
////		case 2:
////			bin = 30;	//30
////			break;
////		case 3:
////			bin = 30;	//30
////			break;
////		case 4:
////			bin = 30;	//15
////			break;
////		case 5:
////			bin = 30;	//15
////			break;
////		}
////		if ((int)boxes[boxIndex].width / 100 > 5)
////		{
////			if (currentLetters.size() < 6)
////				bin = 30;	//7
////			else
////				bin = 30;	//10
////		}
//
//		if (debug["showWords"])
//		{
//			std::cout << "bin: " << bin << std::endl;
//			std::cout << "max: " << max << std::endl;
//		}
//		int hist[bin];
//		int newHistogram[bin];
//
//		//reset histogram
//		for (unsigned int i = 0; i < (unsigned int)bin; i++)
//		{
//			hist[i] = 0;
//			newHistogram[i] = 0;
//		}
//
//		//shift into positive numbers:
//		int shift = (max / ((double)bin)) * std::ceil(smallestWordBreak / (-max / ((double)bin)));
//		// shift in multiples of (max/bin), e.g. (max/bin)=5: smallest=-2 => shift=5; smallest=-19 => shift=20
//
//		// push negative distances into positive space for histogram
//		for (unsigned int i = 0; i < wordBreaks.size(); i++)
//			wordBreaks[i].right += shift;
//
//		// fill histogram
//		for (unsigned int i = 0; i < wordBreaks.size(); i++)
//		{
//			int dis = wordBreaks[i].right;
//			if (dis < max)
//			{
//				dis = dis * bin / max;
//				hist[dis]++;
//			}
//			else
//				hist[bin - 1]++;
//		}
//
//		if (debug["showWords"])
//		{
//			std::cout << "hist: ";
//			for (int a = 0; a < bin; a++)
//				std::cout << hist[a] << " ";
//			std::cout << std::endl;
//			std::cout << std::endl;
//		}
//
//		/*
//		 //smoothing
//		 double mask[3];
//		 mask[0] = 0.25;
//		 mask[1] = 0.5;
//		 mask[2] = 0.25;
//
//		 for (unsigned int a = 1; a < bin - 1; a++)
//		 {
//		 double smoothedValue = 0;
//		 for (int i = 0; i < 3; i++)
//		 {
//		 smoothedValue += hist[a - 1 + i] * mask[i];
//		 }
//		 newHistogram[a] = smoothedValue;
//		 }
//
//		 newHistogram[0] = (int)(hist[0] * 2 / 3 + hist[1] * 1 / 3);
//		 std::cout << "hist[0]" << hist[0] << ",hist[1]" << hist[1] << std::endl;
//		 std::cout << "newHistogram[0]: " << newHistogram[0] << std::endl;
//		 newHistogram[bin - 1] = hist[bin - 2] * (1 / 3) + hist[bin - 1] * (2 / 3);
//
//		 for (int a = 0; a < bin; a++)
//		 {
//		 hist[a] = newHistogram[a];
//		 std::cout << "hist[" << a << "]: " << hist[a];
//		 }
//		 */
//
//		// Get maxValues of peaks
//		int maxPeakNumbers = 0, secondMaxPeakNumbers = 0, maxPeakDistance = 0, secondMaxPeakDistance = 0, scale = 0;
//
//		// find biggest most left peak in histogram
//		for (int j = bin - 1; j >= 0; j--)
//			if (hist[j] >= maxPeakNumbers)
//			{
//				maxPeakNumbers = hist[j];
//				maxPeakDistance = j;
//			}
//
//		// find second biggest most right peak in histogram
//		for (int j = 0; j < bin; j++)
//			if (j != maxPeakDistance)
//				if (hist[j] >= secondMaxPeakNumbers)
//				{
//					secondMaxPeakNumbers = hist[j];
//					secondMaxPeakDistance = j;
//				}
//
//		if (debug["showWords"])
//		{
//			cv::Mat canvas(histHeight, histWidth, CV_8UC3, cv::Scalar(255, 255, 255));
//
//			// Fit histogram to the canvas height
//			scale = maxPeakNumbers > canvas.rows ? (double)canvas.rows / maxPeakNumbers : 1.;
//
//			//Draw histogram
//			for (int j = 0; j < bin; j++)
//			{
//				for (int k = 0; k < histWidth / bin; k++)
//				{
//					cv::Point pt1((j * histWidth / bin) + k, canvas.rows - (hist[j] * scale * 10));
//					cv::Point pt2((j * histWidth / bin) + k, canvas.rows);
//					cv::line(canvas, pt1, pt2, cv::Scalar(250, 210, 25), 1, 8, 0);
//				}
//				cv::line(canvas, cv::Point(j * histWidth / bin, canvas.rows), cv::Point(j * histWidth / bin, 0), cv::Scalar(200, 200, 200), 1, 8, 0);
//				std::stringstream out;
//				out << j * (max / bin) - shift;
//				if (bin > 30) // showing only every second number when there are too many, just for look
//
//				{
//					if (j % 2 == 0)
//						cv::putText(canvas, out.str(), cv::Point((j * histWidth / bin) + 0.25 * (histWidth / bin), canvas.rows - 3), cv::FONT_HERSHEY_SIMPLEX, 0.25,
//								cv::Scalar(0, 0, 0, 0), 1, 8, false);
//				}
//				else
//					cv::putText(canvas, out.str(), cv::Point((j * histWidth / bin) + 0.25 * (histWidth / bin), canvas.rows - 3), cv::FONT_HERSHEY_SIMPLEX, 0.25,
//							cv::Scalar(0, 0, 0, 0), 1, 8, false);
//			}
//			cv::imshow("~", canvas);
//			cvMoveWindow("~", 800, 0);
//			cv::waitKey(0);
//			std::cout << "high of highest peak: " << maxPeakNumbers << ", distance at peak: " << maxPeakDistance << std::endl;
//			std::cout << "high of second-highest peak: " << secondMaxPeakNumbers << ", distance of peak: " << secondMaxPeakDistance << std::endl;
//			std::cout << std::endl;
//		}
//
//		// criteria for splitting into single boxes
//		// ----------------------------------------
//
//		// Ratio negative distances / all distances
//		float howManyNegative = 0.0;
//		for (int j = 0; j < shift * (bin / max); j++)
//			howManyNegative += hist[j];
//		float negPosRatio = howManyNegative / (float)(wordBreaks.size());
//
//		// Ratios of y and height between the letterboxes inside a boundingBox
//		float maxY = 0.0, minY = 1000.0, relativeY = 0.0;
//		double meanBaseline = 0.0, baselineStddev = 0;
//		std::vector<double> distanceToBaseline(currentLetters.size());
//		for (unsigned int j = 0; j < currentLetters.size(); j++)
//		{
//			if (letters[currentLetters[j]].boundingBox.y < minY)
//				minY = letters[currentLetters[j]].boundingBox.y;
//			if (letters[currentLetters[j]].boundingBox.y > maxY)
//				maxY = letters[currentLetters[j]].boundingBox.y;
//			cv::Point2f centralBasePoint(letters[currentLetters[j]].boundingBox.x+0.5*letters[currentLetters[j]].boundingBox.width, letters[currentLetters[j]].boundingBox.y+letters[currentLetters[j]].boundingBox.height);
//			cv::Point2f diff = textRegions[textRegionIndex].lineEquation.center - centralBasePoint;
//			distanceToBaseline[j] = fabs(diff.x*textRegions[textRegionIndex].lineEquation.size.width + diff.y*textRegions[textRegionIndex].lineEquation.size.height);
//			meanBaseline += distanceToBaseline[j];
//			//meanBaseline += letters[currentLetters[j]].height + letters[currentLetters[j]].y;
//		}
//		meanBaseline /= (float)currentLetters.size();
//
//		for (unsigned int j = 0; j < currentLetters.size(); j++)
//		{
//			baselineStddev += (distanceToBaseline[j]-meanBaseline) * (distanceToBaseline[j]-meanBaseline);
//			//baselineStddev += (letters[currentLetters[j]].height + letters[currentLetters[j]].y - meanBaseline) * (letters[currentLetters[j]].height + letters[currentLetters[j]].y - meanBaseline);
//		}
//		baselineStddev = std::sqrt(baselineStddev/(float)currentLetters.size());
//
//		// quality score for text line
//		// given to the whole text line
//		for (unsigned int j = 0; j < currentLetters.size(); j++)
//		{
//			double letterSize = sqrt(letters[currentLetters[j]].boundingBox.width*letters[currentLetters[j]].boundingBox.width + letters[currentLetters[j]].boundingBox.height*letters[currentLetters[j]].boundingBox.height);
//			if (std::abs(letterSize-averageLetterSize) < 0.2*averageLetterSize)
//				if (std::abs(distanceToBaseline[j]-meanBaseline) < 0.1*meanBaseline)
//					qualityScore[textRegionIndex] += 1.;
//		}
//		if (debug["showCriterions"])
//			std::cout << "Box quality score: " << qualityScore[textRegionIndex] << std::endl;
//
//		relativeY = (maxY - minY) / (float)textRegions[textRegionIndex].boundingBox.height;
//
//		unsigned int numberBinsNotZero = 0;
//
//		for (int j = 0; j < bin; j++)
//			if (hist[j] > 0)
//				numberBinsNotZero++;
//
//		float lettersArea = 0;
//		for (unsigned int j = 0; j < currentLetters.size(); j++)
//			lettersArea += letters[currentLetters[j]].boundingBox.area();
//
//		// is text probably rotated?
//		bool textIsRotated = false;
//		int letterSmallerThanLast = 0;
//		int letterBiggerThanLast = 0;
//
//		bool steadyStructure = true;
//
//		std::vector<cv::Rect> dummyRects;
//		for (unsigned int j = 0; j < currentLetters.size(); j++)
//			dummyRects.push_back(letters[currentLetters[j]].boundingBox);
//
//		sort(dummyRects.begin(), dummyRects.end(), DetectText::spatialOrderX);
//		for (unsigned int j = 1; j < dummyRects.size(); j++)
//		{
//			if (dummyRects[j].y > dummyRects[j - 1].y)
//				letterSmallerThanLast++;						// todo: why check y to verify size???
//
//			if (dummyRects[j].y < dummyRects[j - 1].y)
//				letterBiggerThanLast++;
//
//			if (dummyRects[j].y > dummyRects[j - 1].y + dummyRects[j - 1].height)
//				steadyStructure = false;
//		}
//		if (letterSmallerThanLast - letterBiggerThanLast >= std::ceil(currentLetters.size() * 0.5))		// todo: correct?
//			textIsRotated = true;
//		if (letterBiggerThanLast - letterSmallerThanLast >= std::ceil(currentLetters.size() * 0.5))
//			textIsRotated = true;
//		std::cout << "letterSmallerThanLast: " << letterSmallerThanLast << std::endl;
//		std::cout << "letterBiggerThanLast: " << letterBiggerThanLast << std::endl;
//		textIsRotated = false;		// todo: hack!
//
//		// same area
//		float sameArea = 1.0;
//		if (currentLetters.size() == 2)
//			sameArea = letters[currentLetters[0]].boundingBox.area() / (float)letters[currentLetters[1]].boundingBox.area();
//
//		//---------------------------------------------------------------------------------------
//		// Criterions
//		int crit = decideWhichBreaks(negPosRatio, max / bin, baselineStddev, howManyNegative, shift, maxPeakDistance, secondMaxPeakDistance, maxPeakNumbers, secondMaxPeakNumbers,
//				textRegions[textRegionIndex].boundingBox.width, textRegions[textRegionIndex].boundingBox.height, numberBinsNotZero, wordBreaks, textRegions[textRegionIndex].boundingBox,
//				textIsRotated, relativeY, steadyStructure, sameArea);
//
//		bool breakBox = true; // shall box be broken into separate boxes?
//
//		//!!!
//		//crit = 1;
//
//		if (debug["showWords"])
//		{
//			//std::cout << "line equation: p1=(" << textRegions[textRegionIndex].lineEquation.center.x << ", " << lineEquations[boxIndex].center.y << ")  normal=(" << lineEquations[boxIndex].size.width << ", " << lineEquations[boxIndex].size.height << ")" << std::endl;
//			cv::line(output, cv::Point(textRegions[textRegionIndex].lineEquation.center.x, textRegions[textRegionIndex].lineEquation.center.y), cv::Point(textRegions[textRegionIndex].lineEquation.center.x+5000*textRegions[textRegionIndex].lineEquation.size.height, textRegions[textRegionIndex].lineEquation.center.y-5000*textRegions[textRegionIndex].lineEquation.size.width), cv::Scalar(255, 0, 0), 2);
//			cv::line(output, cv::Point(textRegions[textRegionIndex].lineEquation.center.x, textRegions[textRegionIndex].lineEquation.center.y), cv::Point(textRegions[textRegionIndex].lineEquation.center.x-5000*textRegions[textRegionIndex].lineEquation.size.height, textRegions[textRegionIndex].lineEquation.center.y+5000*textRegions[textRegionIndex].lineEquation.size.width), cv::Scalar(255, 0, 0), 2);
//		}
//
//		if (crit == -1)
//		{
//			if (debug["showWords"])
//			{
//				cv::rectangle(output, textRegions[textRegionIndex].boundingBox, cv::Scalar((0), (0), (255)), 2);
//				cv::imshow(breakingWordsDisplayName.c_str(), output);
//				cv::waitKey(0);
//			}
//			continue;
//		}
//		else if (crit == 1)
//			breakBox = false;
//
//		int start = textRegions[textRegionIndex].boundingBox.x;
//		int wordBreakAt = start;
//
//		int numberWordRegions = wordRegions.size();
//
//		// Merging letterboxes that are on top of each other (e.g. 's' is separated into upper and lower semi circle after cc)
//		// In this case merging means deleting one of the letterboxes, as the words are going to be broken based on the x-coordinate
////		if (breakBox == true)		// todo: what is this good for? --> apparently has no effect
////		{
////			std::set< int, std::greater<int> > deleteList;
////			for (unsigned int r = 0; r < wordBreaks.size(); r++)
////				for (unsigned int s = r + 1; s < wordBreaks.size(); s++)
////					if (abs(wordBreaks[r].left - wordBreaks[s].left) < 5)
////						deleteList.insert((int)s);
////
////			for (std::set< int, std::greater<int> >::iterator it = deleteList.begin(); it != deleteList.end(); it++)
////				wordBreaks.erase(wordBreaks.begin() + *it);
////		}
//
//		// Breaking and saving broken words
//		if (breakBox == true)
//		{
//			for (unsigned int r = 0; r < wordBreaks.size(); r++)
//			{
//				bool letter = false;
//				if (maxPeakDistance < 0)
//				{
//					if (wordBreaks[r].right - shift < abs((((max / bin) * maxPeakDistance) - shift) + (max / bin)))
//						letter = true;
//				}
//				else if (wordBreaks[r].right - shift <= ((((max / bin) * maxPeakDistance) - shift) + (max / bin)))
//				{
//					letter = true;
//				}
//
//				// reaching a break between two words:
//				if (letter == false)
//				{
//					wordBreakAt = wordBreaks[r].left - wordBreaks[r].right + shift;
//					cv::Rect re(start, textRegions[textRegionIndex].boundingBox.y, abs(wordBreakAt - start), textRegions[textRegionIndex].boundingBox.height);
//					start = wordBreaks[r].left;
//					TextRegion word;
//					word.boundingBox = re;
//					word.lineEquation = textRegions[textRegionIndex].lineEquation;
//					// word.letters = ; --> see below
//					wordRegions.push_back(word);
//					brokenWordsQualityScore.push_back(qualityScore[textRegionIndex]);
//				}
//				// reaching the end
//				if (r == wordBreaks.size() - 1)
//				{
//					cv::Rect re(start, textRegions[textRegionIndex].boundingBox.y, abs((textRegions[textRegionIndex].boundingBox.x + textRegions[textRegionIndex].boundingBox.width) - start), textRegions[textRegionIndex].boundingBox.height);
//					TextRegion word;
//					word.boundingBox = re;
//					word.lineEquation = textRegions[textRegionIndex].lineEquation;
//					// word.letters = ; --> see below
//					wordRegions.push_back(word);
//					brokenWordsQualityScore.push_back(qualityScore[textRegionIndex]);
//				}
//			}
//		}
//		else
//		{
//			wordRegions.push_back(textRegions[textRegionIndex]);
//			brokenWordsQualityScore.push_back(qualityScore[textRegionIndex]);
//		}
//
//		// Make Boxes smaller to increase precision and recall
//		unsigned int newNumberWordRegions = wordRegions.size();
//		for (unsigned int makeSmallerIndex = numberWordRegions; makeSmallerIndex < newNumberWordRegions; makeSmallerIndex++)
//		{
//			unsigned int smallestX = 10000, smallestY = 10000, biggestX = 0, biggestY = 0;
//			for (unsigned int letterIndex = 0; letterIndex < currentLetters.size(); letterIndex++)
//				if ((wordRegions[makeSmallerIndex].boundingBox & letters[currentLetters[letterIndex]].boundingBox).area() / letters[currentLetters[letterIndex]].boundingBox.area() > 0.5)		// todo: might cause problems?
//				{
//					if ((unsigned int)letters[currentLetters[letterIndex]].boundingBox.x < smallestX)
//						smallestX = letters[currentLetters[letterIndex]].boundingBox.x;
//					if ((unsigned int)letters[currentLetters[letterIndex]].boundingBox.y < smallestY)
//						smallestY = letters[currentLetters[letterIndex]].boundingBox.y;
//					if ((unsigned int)letters[currentLetters[letterIndex]].boundingBox.x + letters[currentLetters[letterIndex]].boundingBox.width > biggestX)
//						biggestX = letters[currentLetters[letterIndex]].boundingBox.x + letters[currentLetters[letterIndex]].boundingBox.width;
//					if ((unsigned int)letters[currentLetters[letterIndex]].boundingBox.y + letters[currentLetters[letterIndex]].boundingBox.height > biggestY)
//						biggestY = letters[currentLetters[letterIndex]].boundingBox.y + letters[currentLetters[letterIndex]].boundingBox.height;
//					wordRegions[makeSmallerIndex].letters.push_back(letters[currentLetters[letterIndex]]);
//				}
//			if (smallestX == 10000)
//				continue;
//
//			cv::Rect smallerRe(smallestX, smallestY, biggestX - smallestX, biggestY - smallestY);
//
//			if (debug["showWords"])
//			{
//				cv::rectangle(output, smallerRe, cv::Scalar((0), (255), (0)), 2);
//				cv::imshow(breakingWordsDisplayName.c_str(), output);
//				cv::waitKey(0);
//			}
//			wordRegions[makeSmallerIndex].boundingBox = smallerRe;
//		}
//	}
//
//	textRegions = wordRegions;
//	qualityScore = brokenWordsQualityScore;
//}
//
//int DetectText::decideWhichBreaks(float negPosRatio, float max_Bin, float baselineStddev, unsigned int howManyNegative, unsigned int shift,
//		int maxPeakDistance, int secondMaxPeakDistance, int maxPeakNumbers, int secondMaxPeakNumbers, unsigned int boxWidth, unsigned int boxHeight,
//		unsigned int numberBinsNotZero, std::vector<Pair> wordBreaks, cv::Rect box, bool textIsRotated, float relativeY, bool steadyStructure, float sameArea)
//{
//	bool broke = true;
//	bool showCriterions = debug["showCriterions"];
//
//	if (wordBreaks.size()==0)
//		return -1;
//
//	//---------------------------------------------------------------------------------------
//	//criterion #1 more negative than positive distances -> probably no letters -> throw away
//	//---------------------------------------------------------------------------------------
//	if (false) // Not convenient for rotated text
//	{
//		if (negPosRatio >= 0.5 || negPosRatio * wordBreaks.size() > 8)
//		{
//			broke = false;
//			if (showCriterions)
//			{
//				std::cout << "Criterion #1" << std::endl;
//				std::cout << "More negative than positive distances: " << std::endl;
//				std::cout << "Negative Distances: " << howManyNegative << "  >  Positive Distances: " << wordBreaks.size() - howManyNegative << std::endl;
//				std::cout << "- Text will not be broken!" << std::endl;
//			}
//			if (wordBreaks.size() > 3)
//				if (negPosRatio > 1 / (1 + 0.75) || maxPeakDistance < (int) shift)
//					return -1;
//		}
//	}
//
//	//---------------------------------------------------------------------------------------
//	//criterion #1.5 negatives are too negative
//	//---------------------------------------------------------------------------------------
//	/*if (shift > 20)
//	 {
//	 std::cout << "Criterion #1.5" << std::endl;
//	 std::cout << "Negatives too negative: -" << shift << std::endl;
//	 std::cout << "- Text will not be saved!" << std::endl;
//	 cv::rectangle(output, boxes[j], cv::Scalar((0), (0), (255)), 2);
//	 cv::imshow("breaking words", output);
//	 cv::waitKey(0);
//	 continue;
//	 }*/
//
//	//---------------------------------------------------------------------------------------
//	//criterion #2 there need to be 2 peaks (and one blank bin between them)
//	//---------------------------------------------------------------------------------------
//	if (secondMaxPeakNumbers == 0)
//	{
//		broke = false;
//		if (showCriterions)
//		{
//			std::cout << "Criterion #2" << std::endl;
//			std::cout << "Only one peak in histogram." << std::endl;
//			std::cout << "- Text will not be broken" << std::endl;
//			std::cout << "wordBreaks.size: " << wordBreaks.size() << std::endl;
//		}
//		int allDistances = 0;
//		for (unsigned int i = 0; i < wordBreaks.size(); i++)
//		{
//			std::cout << "wordBreaks[" << i << "]: " << wordBreaks[i].right << std::endl;
//			allDistances += wordBreaks[i].right;
//		}
//		std::cout << "box.width: " << box.width << std::endl;
//		if (allDistances > 0.5*box.width)
//			return -1;
//	}
//
//	if (!textIsRotated)
//		//---------------------------------------------------------------------------------------
//		//criterion #3 highest peak has to have more elements than second highest peak
//		//---------------------------------------------------------------------------------------
//		if (secondMaxPeakNumbers != 0)
//			//if (std::abs(maxPeakNumbers - secondMaxPeakNumbers) <= 0)
//			if (maxPeakNumbers - secondMaxPeakNumbers< 0)
//			{
//				broke = false;
//				if (showCriterions)
//				{
//					std::cout << "Criterion #3" << std::endl;
//					std::cout << "Peaks have same number of elements." << std::endl;
//					std::cout << "- Text will not be broken." << std::endl;
//					std::cout << "- Text will not be saved!" << std::endl;
//				}
//				return -1;
//			}
//
//	//---------------------------------------------------------------------------------------
//	//criterion #4 highest peak has to be on the left in the histogram (means it has to contain smaller distances)
//	//---------------------------------------------------------------------------------------
//	if (maxPeakDistance > secondMaxPeakDistance/* && secondMaxValue > 1*/)
//	{
//		std::cout << "secondMaxPeakDistance: " << secondMaxPeakDistance << ", shift: " << shift << std::endl;
//		if ((unsigned int) secondMaxPeakDistance > shift)
//		{
//			broke = false;
//			if (showCriterions)
//			{
//				std::cout << "Criterion #4" << std::endl;
//				std::cout << "Most distances contain bigger distances than second most distances" << std::endl;
//				std::cout << "- Text will not be broken" << std::endl;
//			}
//			if (!textIsRotated || wordBreaks.size() > 3)
//			{
//				if (showCriterions)
//					std::cout << "- Text will not be saved!" << std::endl;
//				return -1;
//			}
//		}
//	}
//
//	//---------------------------------------------------------------------------------------
//	//criterion #5 width/height of whole box has to be small
//	//---------------------------------------------------------------------------------------
//	if (box.width / (float) box.height < 1.7)
//	{
//		if (wordBreaks.size() > 1)
//		{
//			if (showCriterions)
//			{
//				std::cout << "Criterion #5" << std::endl;
//				std::cout << "width/height is too small: " << box.width / (float) box.height << std::endl;
//				std::cout << "- Text will not be saved!" << std::endl;
//			}
//			return -1;
//		}
//	}
//
//	//---------------------------------------------------------------------------------------
//	//criterion #6 highest peak has to have more than one element -> no pseudo 2-letter-words
//	//---------------------------------------------------------------------------------------
//	if (maxPeakNumbers < 2)
//	{
//		std::cout << "sameArea: " << sameArea << std::endl;
//		std::cout << "relativeY: " << relativeY << std::endl;
//		// if the 2 boxes are almost identical the possibility is high that it is a 2-letter-word->then don't throw away
//		if ((sameArea > 1.2 || sameArea < 0.8))
//		{
//			if (showCriterions)
//			{
//				std::cout << "Criterion #6.1" << std::endl;
//				std::cout << "highest peak too small: " << maxPeakNumbers << std::endl;
//				std::cout << "- Text will not be saved!" << std::endl;
//			}
//			return -1;
//		}
//		else if (relativeY > 0.5)
//		{
//			if (showCriterions)
//			{
//				std::cout << "Criterion #6.2" << std::endl;
//				std::cout << "highest peak too small: " << maxPeakNumbers << std::endl;
//				std::cout << "- Text will not be saved!" << std::endl;
//			}
//			return -1;
//		}
//		else if (box.width / 3 < wordBreaks[0].right)
//		{
//			if (showCriterions)
//			{
//				std::cout << "Criterion #6.3" << std::endl;
//				std::cout << "highest peak too small: " << maxPeakNumbers << std::endl;
//				std::cout << "- Text will not be saved!" << std::endl;
//			}
//			return -1;
//		}
//	}
//
//	//---------------------------------------------------------------------------------------
//	//criterion #7 relative Height !< 0.5
//	//---------------------------------------------------------------------------------------
//	/*
//	 if (relativeHigh > 0.4)
//	 {
//	 std::cout << "Criterion #7" << std::endl;
//	 std::cout << "heighs of boxes are too different!" << std::endl;
//	 //---------------------------------------------------------------------------------------
//	 //criterion #7.5 peaks shall not be more than 5 bins separated
//	 // if (abs(maxDistance - secondMaxDistance) > 5)
//	 // {
//	 //   std::cout << "Criterion #2.5" << std::endl;
//	 //   std::cout << "Peaks are too far away from each other" << std::endl;
//	 //   std::cout << "- Text will not be broken" << std::endl;
//	 //   std::cout << "- Text will not be saved!" << std::endl;
//	 cv::rectangle(output, boxes[j], cv::Scalar((0), (0), (255)), 2);
//	 cv::imshow("breaking words", output);
//	 cv::waitKey(0);
//	 continue;
//	 // }
//	 //---------------------------------------------------------------------------------------
//	 }*/
//
//	//---------------------------------------------------------------------------------------
//	//criterion #8 not too many bins with values
//	//---------------------------------------------------------------------------------------
//	if (numberBinsNotZero > 5)
//	{
//		if (showCriterions)
//		{
//			std::cout << "Criterion #8" << std::endl;
//			std::cout << "Too many different bins with values: " << numberBinsNotZero << std::endl;
//			std::cout << "- Text will not be saved!" << std::endl;
//		}
//		return -1;
//	}
//
//	//---------------------------------------------------------------------------------------
//	//criterion #9 Color and variance
//	//---------------------------------------------------------------------------------------
//
//	//---------------------------------------------------------------------------------------
//	//criterion #10 not too big distance between highest and second highest peak
//	//---------------------------------------------------------------------------------------
//	if (secondMaxPeakNumbers > 0)
//		if (secondMaxPeakDistance - maxPeakDistance > 7)
//		{
//			if (showCriterions)
//			{
//				std::cout << "Criterion #10" << std::endl;
//				std::cout << "too big distance between highest and second highest peak: " << secondMaxPeakDistance - maxPeakDistance << std::endl;
//				std::cout << "- Text will not be saved!" << std::endl;
//			}
//			return -1;
//		}
//
//	//---------------------------------------------------------------------------------------
//	// Criterion #11 No single letters allowed
//	//---------------------------------------------------------------------------------------
//	// Criterion #11.1 First distance is not supposed to be a big distance (else first/last letterbox would stand alone)
//	if (textIsRotated)
//		std::cout << "TEXT IS ROTATED!" << std::endl;
//	else
//		std::cout << "TEXT IS NOT ROTATED!" << std::endl;
//	if (false)
//		//if (!textIsRotated)
//			if (wordBreaks[0].right - shift > 0)
//				if ((wordBreaks[0].right > secondMaxPeakDistance*max_Bin + 0.5*max_Bin) && (wordBreaks[0].right < secondMaxPeakDistance * max_Bin + 1.5*max_Bin))
//				{
//					std::cout << wordBreaks[0].right << ">" << secondMaxPeakDistance << "*" << max_Bin << " - " << shift << std::endl;
//					std::cout << "heightVariance: " << baselineStddev << std::endl;
//					std::cout << "relativeY: " << relativeY << std::endl;
//					if (baselineStddev > 1.41 || relativeY > 0.35) //additional criterion to allow some boxes with single letters
//					{
//						if (showCriterions)
//						{
//							std::cout << "Criterion #11.1.1" << std::endl;
//							std::cout << "big gap/distance shall not be first element of wordBreaks!" << std::endl;
//							std::cout << "- Text will not be saved!" << std::endl;
//						}
//						return -1;
//					}
//				}
//	// Criterion #11.2 Last distance is not supposed to be a big distance
//	if (false)
//		if (wordBreaks[wordBreaks.size()-1].right-shift > 0)
//			if ((wordBreaks[wordBreaks.size()-1].right >= secondMaxPeakDistance*max_Bin+0.5*max_Bin) && (wordBreaks[wordBreaks.size()-1].right < secondMaxPeakDistance * max_Bin + 1.5*max_Bin))
//				if (baselineStddev > 1.41)
//				{
//					if (showCriterions)
//					{
//						std::cout << "Criterion #11.1.2" << std::endl;
//						std::cout << "big gap/distance shall not be last element of wordBreaks!" << std::endl;
//						std::cout << "- Text will not be saved!" << std::endl;
//						std::cout << "wordBreaks[wordBreaks.size() - 1].right: " << wordBreaks[wordBreaks.size() - 1].right << std::endl;
//					}
//					return -1;
//				}
//
//	//---------------------------------------------------------------------------------------
//	//criterion #12 variance of letter heights < 4
//	//---------------------------------------------------------------------------------------
////	std::cout << "Criterion 12.2.2: " << baselineStddev << "  (threshold: " << (0.05/15.*wordBreaks.size()+0.25/3.)*box.height << ", i.e. " << (0.05/15.*wordBreaks.size()+0.25/3.) << "*box.height)" << std::endl;
//
//	if (baselineStddev > 0.05*box.height && wordBreaks.size() <= 2)
//	{
//		if (showCriterions)
//		{
//			std::cout << "Criterion #12.1" << std::endl;
//			std::cout << "Variance of heights too big: " << baselineStddev << "  (threshold: " << 0.05*box.height << ")" << std::endl;
//			std::cout << "- Text will not be saved!" << std::endl;
//		}
//		return -1;
//	}
//	else if (baselineStddev > 0.15*box.height)
//	{
//		if (showCriterions)
//		{
//			std::cout << "Criterion #12.2.1" << std::endl;
//			std::cout << "Variance of heights too big: " << baselineStddev << "  (threshold: " << 0.3*box.height << ")" << std::endl;
//			std::cout << "- Text will not be saved!" << std::endl;
//		}
//		return -1;
//	}
//	else if (baselineStddev > (0.05/15.*wordBreaks.size()+0.25/3.)*box.height)
//	//else if (baselineStddev > (0.013333333*wordBreaks.size()+0.033333333)*box.height)
//	{
//		if (showCriterions)
//		{
//			std::cout << "Criterion #12.2.2" << std::endl;
//			std::cout << "Variance of heights too big: " << baselineStddev << "  (threshold: " << (0.05/15.*wordBreaks.size()+0.25/3.)*box.height << ", i.e. " << (0.05/15.*wordBreaks.size()+0.25/3.) << "*box.height)" << std::endl;
//			std::cout << "- Text will not be saved!" << std::endl;
//		}
//		return -1;
//	}
////	else if (heightStddev > 0.1*box.height && wordBreaks.size() < 10)
////	{
////		if (showCriterions)
////		{
////			std::cout << "Criterion #12.2.1" << std::endl;
////			std::cout << "Variance of heights too big: " << heightStddev << std::endl;
////			std::cout << "- Text will not be saved!" << std::endl;
////		}
////		return -1;
////	}
////	else if (heightStddev > 0.2*box.height && wordBreaks.size() < 15)
////	{
////		if (showCriterions)
////		{
////			std::cout << "Criterion #12.2.2" << std::endl;
////			std::cout << "Variance of heights too big: " << heightStddev << std::endl;
////			std::cout << "- Text will not be saved!" << std::endl;
////		}
////		return -1;
////	}
////	else if (heightStddev > 0.25*box.height && wordBreaks.size() < 20)
////	{
////		if (showCriterions)
////		{
////			std::cout << "Criterion #12.2.3" << std::endl;
////			std::cout << "Variance of heights too big: " << heightStddev << std::endl;
////			std::cout << "- Text will not be saved!" << std::endl;
////		}
////		return -1;
////	}
////	else if (heightStddev > 0.3*box.height)
////	{
////		if (showCriterions)
////		{
////			std::cout << "Criterion #12.2.4" << std::endl;
////			std::cout << "Variance of heights too big: " << heightStddev << std::endl;
////			std::cout << "- Text will not be saved!" << std::endl;
////		}
////		return -1;
////	}
//
//	//---------------------------------------------------------------------------------------
//	//criterion #13 Steady Structure
//	//---------------------------------------------------------------------------------------
//	if (!steadyStructure)
//	{
//		if (showCriterions)
//		{
//			std::cout << "Criterion #13" << std::endl;
//			std::cout << "no steady structure" << std::endl;
//			// std::cout << "- Text will not be saved!" << std::endl;
//		}
//		//return -1;
//	}
//
//	//---------------------------------------------------------------------------------------
//	//criterion #14 How negative are the negative
//	//---------------------------------------------------------------------------------------
//	unsigned int absNegative = 0;
//	for (unsigned int i = 0; i < wordBreaks.size(); i++)
//	{
//		std::cout << "wordBreaks[i].right: " << wordBreaks[i].right << ", shift: " << shift << std::endl;
//		if ((unsigned int) wordBreaks[i].right < shift)
//			absNegative -= (wordBreaks[i].right - shift);
//	}
//	std::cout << "absNegative: " << absNegative << std::endl;
//	if (absNegative > wordBreaks.size() * 10)
//	{
//		if (showCriterions)
//		{
//			std::cout << "Criterion #14" << std::endl;
//			std::cout << "too negative: " << absNegative << std::endl;
//			std::cout << "- Text will not be saved!" << std::endl;
//		}
//		return -1;
//	}
//
//	if (broke == true)
//		return 2;
//	else
//		return 1;
//}

void DetectText::showEdgeMap()
{
	if (firstPass_)
	{
		cv::imwrite("edgemap.png", edgemap_);
	}
}
void DetectText::showCcmap(cv::Mat& ccmap)
{

	cv::Mat ccmapLetters = ccmap * (1.0 / static_cast<float> (nComponent_));
	for (size_t i = 0; i < nComponent_; ++i)
	{
		cv::Rect *itr = &labeledRegions_[i];
		rectangle(ccmapLetters, cv::Point(itr->x, itr->y), cv::Point(itr->x + itr->width, itr->y + itr->height), cv::Scalar(0.5));
	}
	if (firstPass_)
		imwrite("ccmap1.jpg", ccmapLetters * nComponent_);
	else
		imwrite("ccmap2.jpg", ccmapLetters * nComponent_);
}
void DetectText::showSwtmap(cv::Mat& swtmap)
{
	if (firstPass_)
		imwrite("swtmap1.jpg", swtmap * 10);
	else
		imwrite("swtmap2.jpg", swtmap * 10);
}
void DetectText::showLetterDetection()
{
	cv::Mat output = originalImage_.clone();
	cv::Scalar scalar;
	if (firstPass_)
		scalar = cv::Scalar(0, 255, 0);
	else
		scalar = cv::Scalar(0, 0, 255);

	for (size_t i = 0; i < nComponent_; ++i)
	{
		if (isLetterRegion_[i])
		{
			cv::Rect *itr = &labeledRegions_[i];
			rectangle(output, cv::Point(itr->x, itr->y), cv::Point(itr->x + itr->width, itr->y + itr->height), scalar, 2);
			std::stringstream ss;
			std::string s;
			ss << i;
			s = ss.str() + ".tiff";
			imwrite(s, originalImage_(*itr));
		}
	}
	if (firstPass_)
		imwrite(outputPrefix_ + "_letters1.jpg", output);
	else
		imwrite(outputPrefix_ + "_letters2.jpg", output);
}
void DetectText::showLetterGroup()
{
	cv::Mat output = originalImage_.clone();
	cv::Scalar scalar;
	if (firstPass_)
		scalar = cv::Scalar(0, 255, 0);
	else
		scalar = cv::Scalar(0, 0, 255);

	for (size_t i = 0; i < nComponent_; ++i)
	{
		//if (isGrouped_[i]) isGrouped_[i] doesnt exist anymore, letterGroups_ has to be searched for i
		// {
		cv::Rect *itr = &labeledRegions_[i];
		rectangle(output, cv::Point(itr->x, itr->y), cv::Point(itr->x + itr->width, itr->y + itr->height), scalar, 2);
		// }
	}
	if (firstPass_)
		imwrite(outputPrefix_ + "_group1.jpg", output);
	else
		imwrite(outputPrefix_ + "_group2.jpg", output);
}
void DetectText::testGetCorrelationIndex()
{
	assert(getCorrelationIndex("a") == 97);
	assert(getCorrelationIndex("c") == 99);
	assert(getCorrelationIndex("A") == 65);
	assert(getCorrelationIndex("0") == 48);
	assert(getCorrelationIndex("9") == 57);
	std::cout << "pass getCorrelationIndex test" << std::endl;
}
void DetectText::testEditDistance()
{
	std::string a("hello");
	std::string b("helo");
	assert(editDistance(a,b)==1);
	std::string c("hello");
	std::string d("xello");
	std::cout << "distance betweeen " << c << " & " << d << ": " << editDistance(c, d) << std::endl;
	std::cout << "distance with font betweeen " << c << " & " << d << ":" << editDistanceFont(c, d) << std::endl;
}
void DetectText::testInsertToList()
{
	std::vector<Word> list;
	list.resize(10);

	for (int i = 0; i < 10; i++)
	{
		float score = rand() % 50;
		Word w = Word("", score);
		insertToList(list, w);
		for (size_t i = 0; i < 10; i++)
		{
			std::cout << list[i].score << " <= ";
		}
		std::cout << std::endl;
	}

}
void DetectText::testMergePairs()
{
	int a[] =
	{ 1, 2, 3 };
	int b[] =
	{ 2, 3, 9 };
	int c[] =
	{ 7, 5 };
	int d[] =
	{ 2, 4, 6 };

	std::vector<std::vector<int> > initialChain;
	std::vector<std::vector<int> > outputChain;
	std::vector<int> va(a, a + 3);
	std::vector<int> vb(b, b + 3);
	std::vector<int> vc(c, c + 2);
	std::vector<int> vd(d, d + 3);
	initialChain.push_back(va);
	initialChain.push_back(vb);
	initialChain.push_back(vc);
	initialChain.push_back(vd);

	while (mergePairs(initialChain, outputChain))
	{
		initialChain = outputChain;
		outputChain.clear();
	}

	for (size_t i = 0; i < outputChain.size(); i++)
	{
		for (size_t j = 0; j < outputChain[i].size(); j++)
		{
			std::cout << outputChain[i][j] << " ";
		}
		std::cout << std::endl;
	}

}
void DetectText::testEdgePoints(std::vector<cv::Point>& edgepoints)
{
	cv::Mat temp(edgemap_.size(), CV_8UC1);
	std::vector<cv::Point>::iterator itr = edgepoints.begin();
	for (; itr != edgepoints.end(); ++itr)
	{
		temp.at<unsigned char>(*itr) = 255;
	}
	imshow("test edge", temp);
	cv::waitKey();
}
