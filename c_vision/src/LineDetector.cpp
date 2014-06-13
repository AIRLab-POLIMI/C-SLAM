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

#include "LineDetector.h"

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

LineDetector::LineDetector(CannyParam& cannyP, HoughParam& houghP) :
			cannyP(cannyP), houghP(houghP)
{
	cv::namedWindow("Canny");
}

vector<Vec4i> LineDetector::detect(Mat& input)
{

	Mat canny, eroded;

	double high_thres = cv::threshold(input, canny, 0, 255,
				CV_THRESH_BINARY + CV_THRESH_OTSU);
	double low_thres = high_thres * cannyP.alpha;
	Canny(input, canny, low_thres, high_thres, cannyP.apertureSize, true);

	imshow("Canny", canny);

	vector<Vec4i> lines;

	HoughLinesP(canny, lines, houghP.rho, houghP.teta, houghP.threshold,
				houghP.minLineLenght, houghP.maxLineGap);

	return lines;

}
