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

Rectangle::Rectangle(Point x, Point y, Point z, Point w, Eigen::Matrix3d& omega) :
			x(x), y(y), z(z), w(w), omega(omega)
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

	featureMap["xMax"] = z.x;
	featureMap["yMax"] = z.y;
	featureMap["xMin"] = x.x;
	featureMap["yMin"] = x.y;
	featureMap["formFactor"] = 1000 * computeFormFactor();
	featureMap["area"] = contourArea(getPointsVector());

}

double Rectangle::computeFormFactor()
{
	//Get the points
	Eigen::Vector3d m1(x.x, x.y, 1);
	Eigen::Vector3d m2(y.x, y.y, 1);
	Eigen::Vector3d m3(z.x, z.y, 1);
	Eigen::Vector3d m4(w.x, w.y, 1);

	//compute normals
	double c2 = (m1.cross(m3).transpose() * m4)[0]
				/ (m2.cross(m3).transpose() * m4)[0];
	double c3 = (m1.cross(m3).transpose() * m2)[0]
				/ (m4.cross(m3).transpose() * m2)[0];

	Eigen::Vector3d n2 = c2 * m2 - m1;
	Eigen::Vector3d n3 = c3 * m4 - m1;

	//Compute frame transaltion
	double ff2 = (n2.transpose() * omega * n2)[0]
				/ (n3.transpose() * omega * n3)[0];

	return sqrt(ff2);
}
