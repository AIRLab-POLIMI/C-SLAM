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

#include "CognitiveDetector.h"

#include "Callbacks.h"

#include <iostream>

void CognitiveDetector::detect(cv::Mat frame)
{
	cv::Mat cornerFrame;
	cornerFrame = cornerDetector.detect(frame);
	drawAxis(cornerFrame);

	imshow(WINDOW, cornerFrame);
	cvWaitKey(60);
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

void CognitiveDetector::createTrackBars()
{
	//Controls for corner
	cv::createTrackbar("threshold", WINDOW, NULL, 100, thresholdCorner,
			(void*) &cornerDetector);
	cv::setTrackbarPos("threshold", WINDOW, cornerP.threshold);
	cv::createTrackbar("windowSize", WINDOW, NULL, 100, windowSizeCorner,
			(void*) &cornerDetector);
	cv::setTrackbarPos("windowSize", WINDOW, cornerP.windowSize);
	cv::createTrackbar("minSize", WINDOW, NULL, 350, minClusterSizeCorner,
			(void*) &cornerDetector);
	cv::setTrackbarPos("minSize", WINDOW, cornerP.clusterMinSize);
	cv::createTrackbar("noiseBarrier", WINDOW, NULL, 350, noisebarrierCorner,
			(void*) &cornerDetector);
	cv::setTrackbarPos("noiseBarrier", WINDOW, cornerP.noiseBarrier);
	cv::createTrackbar("objectWindow", WINDOW, NULL, 350, objectWindoCorner,
			(void*) &cornerDetector);
	cv::setTrackbarPos("objectWindow", WINDOW, cornerP.objectWindow);
	cv::createTrackbar("minObjectSize", WINDOW, NULL, 350, objectMinSizeCorner,
			(void*) &cornerDetector);
	cv::setTrackbarPos("minObjectSize", WINDOW, cornerP.objectMinSize);

}
