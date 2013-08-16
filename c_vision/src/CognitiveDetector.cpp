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
	cv::Point center, zAxis;
	center.x = input.cols / 2;
	center.y = input.rows / 2;


	double xOffset = 100 * sin(roll * M_PI / 180.0);
	double yOffset = 100 * cos(roll * M_PI / 180.0);

	zAxis.x = center.x - xOffset;
	zAxis.y = center.y - yOffset;


	cv::line(input, center, zAxis, cv::Scalar(0, 0, 0));
	cv::circle(input, center, 20, cv::Scalar(0, 0, 0));

}
