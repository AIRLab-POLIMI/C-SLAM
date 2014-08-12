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

using namespace cv;
using namespace std;

CognitiveDetector::CognitiveDetector(ParameterServer& parameters) :
			clusterDetector(parameters.getDBScanParams()),
			lineDetector(parameters.getCannyParams(),
						parameters.getHoughParams(),
						parameters.getLFiltrerParams())
{
	roll = 0;
	setToNull();
}

void CognitiveDetector::detect(cv::Mat& image)
{
	//preprocessing
	Mat equalizedFrame, grayFrame;
	preprocessing(image, equalizedFrame, grayFrame);

	detectRectanglesAndPoles(equalizedFrame);
	clusters = clusterDetector.detect(grayFrame);
}

void CognitiveDetector::detectRectangles(cv::Mat& image)
{
	Mat equalizedFrame, grayFrame;
	preprocessing(image, equalizedFrame, grayFrame);

	detectRectanglesAndPoles(equalizedFrame);
}

void CognitiveDetector::deleteDetections()
{
	if (rectangles && poles)
	{
		delete rectangles;
		delete poles;

		delete verticalLines;
		delete horizontalLines;
	}

	if (clusters)
		delete clusters;

	setToNull();
}

void CognitiveDetector::preprocessing(Mat& input, Mat& equalizedFrame,
			Mat& grayFrame)
{
	cvtColor(input, grayFrame, CV_BGR2GRAY);
	equalizeHist(grayFrame, equalizedFrame);
}

void CognitiveDetector::detectRectanglesAndPoles(Mat& equalizedFrame)
{
	//detect lines
	lineDetector.detect(equalizedFrame, roll);
	verticalLines = lineDetector.getVerticalLines();
	horizontalLines = lineDetector.getHorizontalLines();

	//detect features
	HighLevelDetector highLevelDetector;
	highLevelDetector.detect(*verticalLines, *horizontalLines);
	rectangles = highLevelDetector.getRectangles();
	poles = highLevelDetector.getPoles();
}

void CognitiveDetector::setToNull()
{
	rectangles = NULL;
	poles = NULL;
	clusters = NULL;
	horizontalLines = NULL;
	verticalLines = NULL;
}

