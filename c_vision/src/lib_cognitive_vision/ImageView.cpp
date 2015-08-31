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

#include <sstream>

using namespace std;
using namespace cv;

ImageView::ImageView() :
			viewName("")
{
	keyPoints = NULL;
	points = NULL;
	clusters = NULL;
	verticalLines = NULL;
	horizontalLines = NULL;
	rectangles = NULL;
	poles = NULL;
	roll = 0;
}

ImageView::ImageView(std::string viewName) :
			viewName(viewName)
{
	namedWindow(viewName);
	keyPoints = NULL;
	points = NULL;
	clusters = NULL;
	verticalLines = NULL;
	horizontalLines = NULL;
	rectangles = NULL;
	poles = NULL;
	roll = 0;
}

ImageView::ImageView(unsigned int id)
{
	stringstream ss;
	ss << "Object " << id;
	viewName = ss.str();
	cv::namedWindow(viewName);
	keyPoints = NULL;
	points = NULL;
	clusters = NULL;
	verticalLines = NULL;
	horizontalLines = NULL;
	rectangles = NULL;
	poles = NULL;
	roll = 0;
}

void ImageView::display()
{
	if (verticalLines)
		displayLineResults(*verticalLines);
	if (horizontalLines)
		displayLineResults(*horizontalLines);
	if (clusters)
		displayClustersResults();
	if (rectangles)
		displayRectanglesResults();
	if (poles)
		displayPolesResults();
	if(points)
		displayPointsResults(*points);
	drawAxis();

	imshow(viewName, image);
	cvWaitKey(1);
}

ImageView::~ImageView()
{
	cv::destroyWindow(viewName);
}

void ImageView::drawAxis()
{
	Point center, y1, y2, z1, z2;
	center.x = image.cols / 2;
	center.y = image.rows / 2;

	double offset1 = 50 * sin(roll);
	double offset2 = 50 * cos(roll);

	y1.x = center.x + offset2;
	y1.y = center.y - offset1;
	y2.x = center.x - offset2;
	y2.y = center.y + offset1;

	z1.x = center.x - offset1;
	z1.y = center.y - offset2;
	z2.x = center.x + offset1;
	z2.y = center.y + offset2;

	line(image, y1, y2, Scalar(0, 0, 0));
	line(image, z1, z2, Scalar(0, 0, 0));
	circle(image, center, 5, Scalar(0, 0, 0));

}

void ImageView::displayKeypointsResults(const std::vector<KeyPoint>& keyPoints)
{
	//display results
	for (size_t i = 0; i < keyPoints.size(); ++i)
	{
		const KeyPoint& kp = keyPoints[i];
		circle(image, kp.pt, 2, Scalar(0, 0, 255));
	}

}

void ImageView::displayPointsResults(const std::vector<Point>& points)
{
	//display results
	for (size_t i = 0; i < points.size(); ++i)
	{
		const Point& pt = points[i];
		circle(image, pt, 2, Scalar(0, 0, 255));
	}

}

void ImageView::displayLineResults(vector<Vec4i>& lines)
{

	//display lines
	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i l = lines[i];
		line(image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255));
	}

}

void ImageView::writeFeaturesClassifications(Feature& feature)
{
	int baseline = 0;
	int currentHeightOffset = 0;
	for (ClassificationMap::const_iterator j = feature.beginClass();
				j != feature.endClass(); ++j)
	{
		stringstream ss;
		ss << j->first << "(" << j->second << ")";
		string text = ss.str();

		Point featureCenter = feature.getCenter();
		Size textSize = getTextSize(text, FONT_HERSHEY_SIMPLEX, 0.4, 1,
					&baseline);
		Point textOrigin(featureCenter.x - textSize.width / 2,
					featureCenter.y + currentHeightOffset + textSize.height);

		putText(image, text, textOrigin, FONT_HERSHEY_SIMPLEX, 0.4,
					Scalar(0, 255, 0));

		currentHeightOffset += 2 * baseline + 3;
	}

}

void ImageView::displayRectanglesResults()
{
	drawContours(image, rectangles, Scalar(0, 255, 0));

	for (vector<Rectangle>::iterator i = rectangles->begin();
				i != rectangles->end(); ++i)
	{
		writeFeaturesClassifications(*i);
	}

}

void ImageView::displayPolesResults()
{
	drawContours(image, poles, Scalar(0, 0, 255));
}

void ImageView::displayClustersResults()
{
	/*display clusters*/
	for (vector<Cluster>::iterator it = clusters->begin();
				it != clusters->end(); ++it)
	{
		it->draw(image, Scalar(255, 0, 0));
		writeFeaturesClassifications(*it);
	}
}
