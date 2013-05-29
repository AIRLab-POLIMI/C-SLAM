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
#include "LineDetector.h"

cv::Mat LineDetector::detect(cv::Mat& input)
{
	cv::Mat output;

	cvtColor(input, output, CV_GRAY2BGR);
	return output;
}

cv::Mat HoughLineDetector::detect(cv::Mat& input)
{
	cv::Mat output = this->LineDetector::detect(input);
	std::vector<cv::Vec2f> lines;
	HoughLines(input, lines, rho, theta, threshold, 0, 0);

	for (size_t i = 0; i < lines.size(); i++)
	{
		float rho = lines[i][0], theta = lines[i][1];
		cv::Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a * rho, y0 = b * rho;
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		line(output, pt1, pt2, cv::Scalar(0, 0, 255), 3, CV_AA);
	}

	return output;
}

cv::Mat ProbabilisticHoughLineDetector::detect(cv::Mat& input)
{
	cv::Mat output = this->LineDetector::detect(input);

	std::vector<cv::Vec4i> lines;

	HoughLinesP(input, lines, rho, theta, threshold, minLineLength, maxLineGap);

	for (size_t i = 0; i < lines.size(); i++)
	{
		cv::Vec4i l = lines[i];
		line(output, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
				cv::Scalar(0, 0, 255), 3, 8);
	}

	return output;
}

int ProbabilisticHoughLineDetector::getMaxLineGap()
{
	return maxLineGap;
}

void ProbabilisticHoughLineDetector::setMaxLineGap(int maxLineGap)
{
	this->maxLineGap = maxLineGap;
}

int ProbabilisticHoughLineDetector::getMinLineLength()
{
	return minLineLength;
}

void ProbabilisticHoughLineDetector::setMinLineLength(int minLineLength)
{
	this->minLineLength = minLineLength;
}

int LineDetector::getRho()
{
	return rho;
}

void LineDetector::setRho(int rho)
{
	this->rho = rho;
}

double LineDetector::getTheta()
{
	return theta;
}

void LineDetector::setTheta(double theta)
{
	this->theta = theta;
}

int LineDetector::getThreshold()
{
	return threshold;
}

void LineDetector::setThreshold(int threshold)
{
	this->threshold = threshold;
}
