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

#include <iostream>

using namespace std;
using namespace cv;

LineDetector::LineDetector(CannyParam& cannyP, HoughParam& houghP,
			LFilterParam& filterP) :
			cannyP(cannyP), houghP(houghP), filterP(filterP), viewer("Canny")
{
	horizontalLines = NULL;
	verticalLines = NULL;
}

void LineDetector::detect(Mat& input, double roll, const cv::Mat& mask)
{
	Mat tmp, blurred;

	double high_thres;
	double low_thres;

	//blur(input, blurred, Size(cannyP.blur, cannyP.blur));
	//medianBlur(input, blurred, 5);
	//GaussianBlur(input, blurred,  Size(cannyP.blur, cannyP.blur), 20);
	//bilateralFilter(input, blurred, 5, 100, 100);

	GaussianBlur(input, blurred, cv::Size(0, 0), 6);
	addWeighted(input, 1.5, blurred, -0.5, 0, tmp);
	bilateralFilter(tmp, blurred, 5, 150, 150);

	if (cannyP.automatic)
	{
		high_thres = 0.8*computeThreshold(blurred);
		low_thres = high_thres * cannyP.alpha;
	}
	else
	{
		high_thres = cannyP.high;
		low_thres = cannyP.low;
	}

	Canny(blurred, tmp, low_thres, high_thres, cannyP.apertureSize, true);

	if (mask.empty())
	{
		canny = tmp;

		//FIXME: delete this hack: it's a problem of current images
		maskImage(canny);
	}
	else
	{
		canny = Mat(tmp.rows, tmp.cols, CV_8UC1,Scalar(0));
		tmp.copyTo(canny, mask);
	}

	vector<Vec4i> lines;

	HoughLinesP(canny, lines, houghP.rho, houghP.teta, houghP.threshold,
				houghP.minLineLenght, houghP.maxLineGap);

	/* Filter Lines */
	LineFilter filter(filterP);
	filter.filter(lines, roll);
	verticalLines = filter.getVerticalLines();
	horizontalLines = filter.getHorizontalLines();

}

void LineDetector::display()
{
	Mat colored;
	cvtColor(canny, colored, CV_GRAY2BGR);
	viewer.setHorizontalLines(horizontalLines);
	viewer.setVerticalLines(verticalLines);
	viewer.display(colored);
}

int LineDetector::computeThreshold(cv::Mat& src)
{
	Mat tmp;

	Mat dx, dy, g2, g;
	Sobel(src, dx, CV_16S, 1, 0, cannyP.apertureSize, 1, 0, BORDER_REPLICATE);
	Sobel(src, dy, CV_16S, 0, 1, cannyP.apertureSize, 1, 0, BORDER_REPLICATE);

	g2 = dx.mul(dx) + dy.mul(dy);

	g2.convertTo(g2, CV_64F);
	sqrt(g2, g);
	g.convertTo(g, CV_8UC1);

	return threshold(g, tmp, 0, 255, CV_THRESH_BINARY + CV_THRESH_OTSU);

}

void LineDetector::maskImage(Mat& canny)
{
	//FIXME: delete this hack: it's a problem of current images
	for (int i = 273; i < 273 + 124; i++)
		for (int j = canny.rows - 10; j < canny.rows; j++)
		{
			canny.at<uchar>(j, i) = 0;
		}
}

