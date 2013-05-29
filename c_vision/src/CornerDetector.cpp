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
#include <opencv2/features2d/features2d.hpp>
#include "CornerDetector.h"

cv::Mat CornerDetector::detect(cv::Mat& input)
{

	cv::Mat output;
	std::vector<cv::KeyPoint> keyPoints;

	cvtColor(input, output, CV_BGR2GRAY);

	FAST(output, keyPoints, threshold);

	cvtColor(output, output, CV_GRAY2BGR);

	for (size_t i = 0; i < keyPoints.size(); ++i)
	{
		const cv::KeyPoint& kp = keyPoints[i];
		circle(output, kp.pt, kp.size / 2, CV_RGB(255, 0, 0));
	}

	return output;
}

int CornerDetector::getThreshold() const
{
	return threshold;
}

void CornerDetector::setThreshold(int threshold)
{
	this->threshold = threshold;
}
