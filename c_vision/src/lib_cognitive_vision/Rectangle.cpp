/*
 * c_vision,
 *
 *
 * Copyright (C) 2014 Davide Tateo
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

#include "Rectangle.h"

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Rectangle::Rectangle(Point x, Point y, Point z, Point w) :
			x(x), y(y), z(z), w(w)
{
}

vector<Point> Rectangle::getPointsVector()
{
	vector<Point> points;
	points.push_back(x);
	points.push_back(y);
	points.push_back(z);
	points.push_back(w);

	return points;
}

Point Rectangle::getCenter()
{
	return (x + y + z + w) * 0.25;
}

void Rectangle::setFeature(Mat& R)
{
	vector<Point> transformedPoints;
	transform(getPointsVector(), transformedPoints, R);
	Point& x = transformedPoints[0];
	Point& y = transformedPoints[1];
	Point& z = transformedPoints[2];
	Point& w = transformedPoints[3];

	int deltaX = (norm(x - y) + norm(w - z)) / 2;
	int deltaY = (norm(x - w) + norm(y - z)) / 2;
	if (deltaX != 0)
	{
		featureMap["xMax"] = z.x;
		featureMap["yMax"] = z.y;
		featureMap["xMin"] = x.x;
		featureMap["yMin"] = x.y;
		featureMap["formFactor"] = 1000 * deltaY / deltaX;
		featureMap["area"] = contourArea(getPointsVector());
	}
}
