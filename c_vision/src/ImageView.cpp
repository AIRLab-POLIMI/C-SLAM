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

#include "ImageView.h"

using namespace std;
using namespace cv;

ImageView::ImageView(std::string viewName) :
			viewName(viewName)
{
	cv::namedWindow(viewName);
	keyPoints = NULL;
	clusters = NULL;
	verticalLines = NULL;
	horizontalLines = NULL;
	rectangles = NULL;
	poles = NULL;
	roll = 0;
}

void ImageView::display(Mat& frame)
{
	//displayClusterResults(*keyPoints, *clusters, frame);
	//displayLineResults(*verticalLines, frame);
	//displayLineResults(*horizontalLines, frame);
	if (clusters)
		displayClustersResults(frame);
	if (rectangles)
		displayRectanglesResults(frame);
	if (poles)
		displayPolesResults(frame);
	drawAxis(frame);

	imshow(viewName, frame);
	cvWaitKey(60);
}

ImageView::~ImageView()
{
	cv::destroyWindow(viewName);
}

void ImageView::drawAxis(Mat& input)
{
	Point center, y1, y2, z1, z2;
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

	line(input, y1, y2, Scalar(0, 0, 0));
	line(input, z1, z2, Scalar(0, 0, 0));
	circle(input, center, 5, Scalar(0, 0, 0));

}

void ImageView::displayKeypointsResults(const std::vector<KeyPoint>& keyPoints,
			Mat& frame)
{
	//display results
	for (size_t i = 0; i < keyPoints.size(); ++i)
	{
		const KeyPoint& kp = keyPoints[i];
		circle(frame, kp.pt, 2, Scalar(0, 0, 255));
	}

}

void ImageView::displayLineResults(vector<Vec4i>& lines, Mat& frame)
{

	//display lines
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255));
	}

}

void ImageView::displayRectanglesResults(Mat& frame)
{
	drawContours(frame, rectangles, Scalar(0, 255, 0));
}

void ImageView::displayPolesResults(Mat& frame)
{
	drawContours(frame, poles, Scalar(0, 0, 255));
}

void ImageView::displayClustersResults(Mat& frame)
{
	/*display clusters*/
	for (vector<Cluster>::iterator it = clusters->begin();
				it != clusters->end(); ++it)
	{
		it->draw(frame, Scalar(255, 0, 0));
	}
}
