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

using namespace std;
using namespace cv;

vector<Vec4i> HoughDetector::detect(Mat& input)
{

	Mat canny, eroded;

	double high_thres = cv::threshold(input, canny, 0, 255,
			CV_THRESH_BINARY + CV_THRESH_OTSU);
	double low_thres = high_thres * 0.75;

	erode(input, eroded, Mat());
	Canny(eroded, canny, low_thres, high_thres, apertureSize);



	vector<Vec4i> lines;

	HoughLinesP(canny, lines, rho, theta, threshold, minLineLength, maxLineGap);

	//add the upper and lower line
	lines.push_back(Vec4i(0, 0, input.rows, 0));
	lines.push_back(Vec4i(0, input.cols, input.rows, input.cols));

	return lines;

}
