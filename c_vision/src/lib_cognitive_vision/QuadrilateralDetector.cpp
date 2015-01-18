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

#include "QuadrilateralDetector.h"

using namespace cv;
using namespace std;

QuadrilateralDetector::QuadrilateralDetector()
{
	rectangles = new std::vector<Rectangle>();
	poles = new std::vector<Pole>();
}

void QuadrilateralDetector::detect(std::vector<cv::Vec4i>& verticalLines,
			std::vector<cv::Vec4i>& horizontalLines, bool skipCheck)
{
	for (size_t i = 0; i + 1 < verticalLines.size(); i++)
	{	//i+1 instead size-1 to avoid integer overflow
		Vec4i v1 = verticalLines[i];
		Vec4i v2 = verticalLines[i + 1];
		//find possible poles
		if (!findPoles(v1, v2) && hasSufficientVerticalOverlap(v1, v2))
		{
			//else find possible squares
			for (size_t j = 0; j + 1 < horizontalLines.size(); j++)
			{
				Vec4i h1 = horizontalLines[j];
				if (isNotExternal(v1, v2, h1))
					for (size_t k = j + 1; k < horizontalLines.size(); k++)
					{
						Vec4i h2 = horizontalLines[k];

						if (isNotExternal(v1, v2, h2))
						{
							Point x, y, z, w;

							x = findInterception(h1, v1);
							y = findInterception(h1, v2);
							z = findInterception(h2, v2);
							w = findInterception(h2, v1);

							Rectangle rectangle(x, y, z, w);
							rectangles->push_back(rectangle);
						}

					}
			}
		}

	}

}

Point QuadrilateralDetector::findInterception(Vec4i l1, Vec4i l2)
{
	Vec3d p0, p1, p2, p3;
	Vec3d hl, vl;
	Vec3d p;

	//get the homogeneus coordinates
	getPointsCoordinates(l1, p0, p1);
	getPointsCoordinates(l2, p2, p3);

	//get the lines and the interception point
	hl = p0.cross(p1);
	hl = hl / norm(hl);

	vl = p2.cross(p3);
	vl = vl / norm(vl);

	p = hl.cross(vl);
	p = p / p[2];

	return Point(p[0], p[1]);

}

bool QuadrilateralDetector::findPoles(Vec4i l1, Vec4i l2)
{
	double dx, dy;

	Point p0, p1, p2, p3;

	getPointsCoordinates(l1, p0, p1);
	getPointsCoordinates(l2, p2, p3);

	//calculate the average dx and dy
	dx = max(norm(p0 - p2), norm(p1 - p3));
	dy = max(norm(p0 - p1), norm(p2 - p3));

	if (dy / dx > polesFormFactor)
	{
		Pole pole(p0, p1, p2, p3);
		poles->push_back(pole);
		return true;
	}

	return false;

}

bool QuadrilateralDetector::hasSufficientVerticalOverlap(Vec4i& v1, Vec4i& v2)
{
	int min1 = min(v1[1], v1[3]);
	int max1 = max(v1[1], v1[3]);
	int l1 = max1 - min1;

	int min2 = min(v2[1], v2[3]);
	int max2 = max(v2[1], v2[3]);
	int l2 = max1 - min1;

	int overlap = max(0, min(max1, max2) - max(min1, min2));

	double percentualOverlap = (double) overlap / min(l1, l2);

	//TODO: compute real overlap % using angle
	return percentualOverlap > 0.7;

}

bool QuadrilateralDetector::isNotExternal(Vec4i& v1, Vec4i& v2, Vec4i& h)
{
	int minh = min(h[0], h[2]);
	int maxh = max(h[0], h[2]);

	bool isAtLeft = maxh < v1[0] && maxh < v1[2];
	bool isAtRight = minh > v2[0] && minh < v2[2];

	return !(isAtLeft || isAtRight);
}

inline void QuadrilateralDetector::getPointsCoordinates(Vec4i l, Point& i,
			Point& j)
{
	i = Point(l[0], l[1]);
	j = Point(l[2], l[3]);
}

inline void QuadrilateralDetector::getPointsCoordinates(Vec4i l, Vec3d& i,
			Vec3d& j)
{
	i = Vec3d(l[0], l[1], 1);
	j = Vec3d(l[2], l[3], 1);
}

