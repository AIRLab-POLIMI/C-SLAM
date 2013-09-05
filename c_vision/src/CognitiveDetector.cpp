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
#include "Callbacks.h"

void CognitiveDetector::detect(cv::Mat frame)
{
	cv::Mat lineFrame = lineDetector.detect(frame);
	featureDetector.detect(frame);
	std::vector<cv::KeyPoint> keyPoints = featureDetector.getKeyPoints();

	std::vector<cv::KeyPoint> complexObjects =
			featureDetector.getComplexObject();

	std::vector<Line> lines = findLines(keyPoints);

	//display results
	displayResults(keyPoints, complexObjects, lines, frame);
	drawAxis(frame);

	imshow(WINDOW, frame);
	imshow(LINE_WINDOW, lineFrame);
	cvWaitKey(60);
}

std::vector<Line> CognitiveDetector::findLines(
		std::vector<cv::KeyPoint> keyPoints)
{

	std::vector<Line> lines;
	std::vector<Line> tmp;

	lines = findLines(-roll, keyPoints);
	tmp = findLines(-(roll + 90), keyPoints);
	lines.insert(lines.end(), tmp.begin(), tmp.end());

	return lines;

}

std::vector<Line> CognitiveDetector::findLines(double roll,
		const std::vector<cv::KeyPoint>& keyPoints)
{
	if (fmod(roll, 180) != 0)
	{
		double m = tan(roll * M_PI / 180.0);
		lineFinder.detect(keyPoints, m);
	}
	else
	{
		lineFinder.detect(keyPoints);
	}

	return lineFinder.getLines();
}

void CognitiveDetector::drawAxis(cv::Mat& input)
{
	cv::Point center, y1, y2, z1, z2;
	center.x = input.cols / 2;
	center.y = input.rows / 2;

	double offset1 = 50 * sin(roll * M_PI / 180.0);
	double offset2 = 50 * cos(roll * M_PI / 180.0);

	y1.x = center.x + offset2;
	y1.y = center.y - offset1;
	y2.x = center.x - offset2;
	y2.y = center.y + offset1;

	z1.x = center.x - offset1;
	z1.y = center.y - offset2;
	z2.x = center.x + offset1;
	z2.y = center.y + offset2;

	cv::line(input, y1, y2, cv::Scalar(0, 0, 0));
	cv::line(input, z1, z2, cv::Scalar(0, 0, 0));
	cv::circle(input, center, 5, cv::Scalar(0, 0, 0));

}

void CognitiveDetector::displayResults(
		const std::vector<cv::KeyPoint>& keyPoints,
		const std::vector<cv::KeyPoint>& complexObjects,
		const std::vector<Line>& lines, cv::Mat& frame)
{
	//display results
	for (size_t i = 0; i < keyPoints.size(); ++i)
	{
		const cv::KeyPoint& kp = keyPoints[i];
		cv::circle(frame, kp.pt, 2, CV_RGB(255, 0, 0));
	}

	for (size_t i = 0; i < complexObjects.size(); ++i)
	{
		const cv::KeyPoint& kp = complexObjects[i];
		cv::circle(frame, kp.pt, 20, CV_RGB(0, 0, 255));
	}

	for (size_t i = 0; i < lines.size(); i++)
	{
		Line line = lines[i];
		cv::line(frame, line.start.pt, line.end.pt, cv::Scalar(0, 255, 0), 2,
				8);
	}
}

void CognitiveDetector::createTrackBars()
{
	//Controls for corner
	cv::createTrackbar("threshold", WINDOW, NULL, 100, thresholdCorner,
			(void*) &featureDetector);
	cv::setTrackbarPos("threshold", WINDOW, cornerP.threshold);
	cv::createTrackbar("windowSize", WINDOW, NULL, 100, windowSizeCorner,
			(void*) &featureDetector);
	cv::setTrackbarPos("windowSize", WINDOW, cornerP.windowSize);
	cv::createTrackbar("minSize", WINDOW, NULL, 350, minClusterSizeCorner,
			(void*) &featureDetector);
	cv::setTrackbarPos("minSize", WINDOW, cornerP.clusterMinSize);
	cv::createTrackbar("noiseBarrier", WINDOW, NULL, 350, noisebarrierCorner,
			(void*) &featureDetector);
	cv::setTrackbarPos("noiseBarrier", WINDOW, cornerP.noiseBarrier);
	cv::createTrackbar("objectWindow", WINDOW, NULL, 350, objectWindoCorner,
			(void*) &featureDetector);
	cv::setTrackbarPos("objectWindow", WINDOW, cornerP.objectWindow);
	cv::createTrackbar("minObjectSize", WINDOW, NULL, 350, objectMinSizeCorner,
			(void*) &featureDetector);
	cv::setTrackbarPos("minObjectSize", WINDOW, cornerP.objectMinSize);

	//controls for line
	cv::createTrackbar("minCanny", LINE_WINDOW, NULL, 300, minCanny,
			(void*) &lineDetector);
	cv::setTrackbarPos("minCanny", LINE_WINDOW, cannyP.minCanny);
	cv::createTrackbar("maxCanny", LINE_WINDOW, NULL, 500, maxCanny,
			(void*) &lineDetector);
	cv::setTrackbarPos("maxCanny", LINE_WINDOW, cannyP.maxCanny);
	cv::createTrackbar("pThreshold", LINE_WINDOW, NULL, 150, thresholdHoughP,
			(void*) &lineDetector);
	cv::setTrackbarPos("pThreshold", LINE_WINDOW, houghP.threshold);
	cv::createTrackbar("minLineLenght", LINE_WINDOW, NULL, 150,
			minLineLengthHoughP, (void*) &lineDetector);
	cv::setTrackbarPos("minLineLenght", LINE_WINDOW, houghP.minLineLenght);
	cv::createTrackbar("maxLineGap", LINE_WINDOW, NULL, 50, maxLineGapHoughP,
			(void*) &lineDetector);
	cv::setTrackbarPos("maxLineGap", LINE_WINDOW, houghP.maxLineGap);

}
