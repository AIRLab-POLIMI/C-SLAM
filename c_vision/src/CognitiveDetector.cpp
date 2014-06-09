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
#include "LineFilter.h"

using namespace cv;
using namespace std;

CognitiveDetector::CognitiveDetector() :
			featureDetector(cornerP.threshold),
			clusterDetector(clusterP.maxDistance, clusterP.minPoints),
			lineDetector(cannyP.apertureSize, houghP.rho, houghP.teta,
						houghP.threshold, houghP.minLineLenght,
						houghP.maxLineGap),
			pitch(0), roll(0), yaw(0)
{
	rectangles = NULL;
	poles = NULL;
	clusters = NULL;
}

void CognitiveDetector::detect(cv::Mat& frame)
{
	//preprocessing
	Mat equalizedFrame;
	preprocessing(frame, equalizedFrame);

	//detect lines
	vector<Vec4i> lines = lineDetector.detect(equalizedFrame);

	//filter lines
	LineFilter filter;
	filter.filter(lines, roll);
	const vector<Vec4i>& verticalLines = filter.getVerticalLines();
	const vector<Vec4i>& horizontalLines = filter.getHorizontalLines();

	//detect keypoints
	const vector<KeyPoint>& keyPoints = featureDetector.detect(equalizedFrame);

	//detect features
	HighLevelDetector highLevelDetector;
	highLevelDetector.detect(verticalLines, horizontalLines);

	rectangles = highLevelDetector.getRectangles();
	poles = highLevelDetector.getPoles();
	clusters = clusterDetector.detect(keyPoints);
}

void CognitiveDetector::preprocessing(Mat& input, Mat& equalizedFrame)
{
	Mat greyFrame;

	cvtColor(input, greyFrame, CV_BGR2GRAY);
	equalizeHist(greyFrame, equalizedFrame);
}

void CognitiveDetector::deleteDetections()
{
	delete rectangles;
	delete poles;
	delete clusters;
}

