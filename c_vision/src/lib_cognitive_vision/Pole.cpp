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

#include "Pole.h"

using namespace std;

using namespace cv;

Pole::Pole(Point a1, Point a2, Point b1, Point b2) :
			a1(a1), a2(a2), b1(b1), b2(b2)
{
}

vector<Point> Pole::getPointsVector()
{
	std::vector<Point> points;
	points.push_back(a1);
	points.push_back(a2);
	points.push_back(b2);
	points.push_back(b1);

	return points;
}

Point Pole::getCenter()
{
	return 0.25 * (a1 + a2 + b1 + b2);
}

void Pole::setFeature(Mat& R)
{
	int deltaX = max(abs(a1.y - b1.y), abs(a2.y - b2.y));
	int deltaY = max(abs(a1.y - b1.y), abs(a2.y - b2.y));
	if (deltaX != 0) //TODO is ok???
	{
		int FormFactor = 1000 * deltaY / deltaX;
		featureMap["height"] = deltaY;
		featureMap["formFactor"] = FormFactor;
	}
}

