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

#include "LineFilter.h"

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

LineDetector::LineDetector(CannyParam& cannyP, HoughParam& houghP) :
			cannyP(cannyP), houghP(houghP), viewer("Canny")
{
	horizontalLines = NULL;
	verticalLines = NULL;
}

void LineDetector::detect(Mat& input, double roll)
{
	Mat canny;

	double high_thres = threshold(input, canny, 0, 255,
				CV_THRESH_BINARY + CV_THRESH_OTSU);
	double low_thres = high_thres * cannyP.alpha;
	Canny(input, canny, low_thres, high_thres, cannyP.apertureSize, true);

	thinning(canny, 1);

	vector<Vec4i> lines;

	HoughLinesP(canny, lines, houghP.rho, houghP.teta, houghP.threshold,
				houghP.minLineLenght, houghP.maxLineGap);

	/* Filter Lines */
	LineFilter filter;
	filter.filter(lines, roll);
	verticalLines = filter.getVerticalLines();
	horizontalLines = filter.getHorizontalLines();

	Mat colored;
	cvtColor(canny, colored, CV_GRAY2BGR);
	viewer.setHorizontalLines(horizontalLines);
	viewer.setVerticalLines(verticalLines);
	viewer.display(colored);

}

void LineDetector::thinning(Mat& im, int maxIterations)
{
	int iterationCount = 0;
	im /= 255;

	Mat prev = Mat::zeros(im.size(), CV_8UC1);
	Mat diff;

	do
	{
		thinningIteration(im, 0);
		thinningIteration(im, 1);
		absdiff(im, prev, diff);
		im.copyTo(prev);
	} while (countNonZero(diff) > 0 && iterationCount++ < maxIterations);

	im *= 255;
}

void LineDetector::thinningIteration(Mat& im, int iter)
{
	Mat marker = Mat::zeros(im.size(), CV_8UC1);

	for (int i = 1; i < im.rows - 1; i++)
	{
		for (int j = 1; j < im.cols - 1; j++)
		{
			uchar p2 = im.at<uchar>(i - 1, j);
			uchar p3 = im.at<uchar>(i - 1, j + 1);
			uchar p4 = im.at<uchar>(i, j + 1);
			uchar p5 = im.at<uchar>(i + 1, j + 1);
			uchar p6 = im.at<uchar>(i + 1, j);
			uchar p7 = im.at<uchar>(i + 1, j - 1);
			uchar p8 = im.at<uchar>(i, j - 1);
			uchar p9 = im.at<uchar>(i - 1, j - 1);

			int A = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1)
						+ (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1)
						+ (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1)
						+ (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
			int B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
			int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
			int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

			if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
				marker.at<uchar>(i, j) = 1;
		}
	}

	im &= ~marker;
}

