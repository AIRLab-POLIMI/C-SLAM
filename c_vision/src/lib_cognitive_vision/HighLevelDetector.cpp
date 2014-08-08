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

#include "HighLevelDetector.h"

using namespace cv;
using namespace std;

HighLevelDetector::HighLevelDetector()
{
	rectangles = new std::vector<Rectangle>();
	poles = new std::vector<Pole>();
}

void HighLevelDetector::detect(std::vector<cv::Vec4i>& verticalLines,
			std::vector<cv::Vec4i>& horizontalLines)
{
	for (size_t i = 0; i + 1 < verticalLines.size(); i++)
	{	//i+1 instead size-1 to avoid integer overflow
		Vec4i v1 = verticalLines[i];
		Vec4i v2 = verticalLines[i + 1];
		//find possible poles
		if (!findPoles(v1, v2))
		{
			//else find possible squares
			for (size_t j = 0; j + 1 < horizontalLines.size(); j++)
			{
				Vec4i h1 = horizontalLines[j];
				for (size_t k = j + 1; k < horizontalLines.size(); k++)
				{
					vector<double> a(4);
					vector<double> b(4);

					Vec4i h2 = horizontalLines[k];
					Point x, y, z, w;

					x = findInterception(h1, v1, a[0], b[0]);
					y = findInterception(h1, v2, a[1], b[1]);
					z = findInterception(h2, v2, a[2], b[2]);
					w = findInterception(h2, v1, a[3], b[3]);

					if (isQuadrilateral(a, b))
					{
						Rectangle rectangle(x, y, z, w);
						rectangle.setFeature();
						rectangles->push_back(rectangle);
					}

				}
			}
		}

	}

}

Point HighLevelDetector::findInterception(Vec4i l1, Vec4i l2, double& a,
			double& b)
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

	//Calculate the linear combination parameters
	a = (p[0] - p0[0]) / (p1[0] - p0[0]);
	b = (p[1] - p2[1]) / (p3[1] - p2[1]);

	return Point(p[0], p[1]);

}

bool HighLevelDetector::findPoles(Vec4i l1, Vec4i l2)
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
		pole.setFeature();
		poles->push_back(pole);
		return true;
	}

	return false;

}

inline void HighLevelDetector::getPointsCoordinates(Vec4i l, Point& i, Point& j)
{
	i = Point(l[0], l[1]);
	j = Point(l[2], l[3]);
}

inline void HighLevelDetector::getPointsCoordinates(Vec4i l, Vec3d& i, Vec3d& j)
{
	i = Vec3d(l[0], l[1], 1);
	j = Vec3d(l[2], l[3], 1);
}

bool HighLevelDetector::isQuadrilateral(vector<double> a, vector<double> b)
{
	int segmentCounter = 0;

	segmentCounter += lineBelongToQuadrilateral(a[0], a[1]);
	segmentCounter += lineBelongToQuadrilateral(a[3], a[2]);
	segmentCounter += lineBelongToQuadrilateral(b[0], b[1]);
	segmentCounter += lineBelongToQuadrilateral(b[3], b[2]);

	return segmentCounter == 4;
}

bool HighLevelDetector::lineBelongToQuadrilateral(double a1, double a2)
{
	const double low = -0.45;
	const double high = 1.45;
	return (a1 >= low) && (a1 <= high) && (a2 >= low) && (a2 <= high);
}

