/*
 * c_vision,
 *
 *
 * Copyright (C) 2013 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_vision.
 *
 * c_vision is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_vision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_vision.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <opencv2/imgproc/imgproc.hpp>

#include "CognitiveDetector.h"
#include "DBScan.h"
#include "LineFilter.h"

void CognitiveDetector::detect(cv::Mat& frame)
{
	cv::Mat equalizedFrame = preprocessing(frame);
	std::vector<cv::Vec4i> lines = lineDetector.detect(equalizedFrame);

	LineFilter filter;
	filter.filter(lines, roll);
	std::vector<cv::Vec4i> verticalLines = filter.getVerticalLines();
	std::vector<cv::Vec4i> horizontalLines = filter.getHorizontalLines();

	std::vector<cv::KeyPoint> keyPoints = featureDetector.detect(
			equalizedFrame);

	std::vector<ObjectCluster> clusters = clusterDetector.detect(keyPoints);

	//display results
	viewer.setRoll(roll);
	viewer.setKeyPoints(&keyPoints);
	viewer.setVerticalLines(&verticalLines);
	viewer.setHorizontalLines(&horizontalLines);
	viewer.setClusters(&clusters);
	viewer.display(frame);
}

/* Image Processing Methods */

cv::Mat CognitiveDetector::preprocessing(cv::Mat& input)
{
	cv::Mat greyFrame, equalizedFrame;

	cvtColor(input, greyFrame, CV_BGR2GRAY);
	equalizeHist(greyFrame, equalizedFrame);

	return equalizedFrame;
}

