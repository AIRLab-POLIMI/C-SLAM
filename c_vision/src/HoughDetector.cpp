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

#include "HoughDetector.h"

#include <opencv2/imgproc/imgproc.hpp>

cv::Mat HoughDetector::detect(cv::Mat& input)
{

	cv::Mat canny, output;

	Canny(input, canny, threshold1, threshold2, apertureSize);

	std::vector<cv::Vec4i> lines;

	HoughLinesP(canny, lines, rho, theta, threshold, minLineLength, maxLineGap);


	output = input.clone();

	for (size_t i = 0; i < lines.size(); i++)
	{
		cv::Vec4i l = lines[i];
		line(output, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]),
				cv::Scalar(0, 0, 255), 3, 8);
	}

	return output;

}
