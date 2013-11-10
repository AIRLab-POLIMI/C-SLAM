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

#include <iostream>

using namespace cv;
using namespace std;

void HighLevelDetector::detect(std::vector<cv::Vec4i> verticalLines,
		std::vector<cv::Vec4i> horizontalLines)
{
	if (!verticalLines.empty() && !horizontalLines.empty())
		for (size_t i = 0; i + 1 < verticalLines.size(); i++)
		{	//i+1 instead size-1 to avoid integer overflow
			Vec4i v1 = verticalLines[i];
			Vec4i v2 = verticalLines[i + 1];
			//find possible poles
			findPoles(v1, v2);
			for (size_t j = 0; j + 1 < horizontalLines.size(); j++)
			{
				Vec4i h1 = horizontalLines[j];
				for (size_t k = j + 1; k < horizontalLines.size(); k++)
				{
					vector<Point> rectangle;
					vector<double> a;
					vector<double> b;

					a.resize(4);
					b.resize(4);

					Vec4i h2 = horizontalLines[k];
					Point x, y, z, w;

					x = findInterceptions(h1, v1, a[0], b[0]);
					y = findInterceptions(h1, v2, a[1], b[1]);
					z = findInterceptions(h2, v2, a[2], b[2]);
					w = findInterceptions(h2, v1, a[3], b[3]);

					if (isQuadrilateral(a, b))
					{
						rectangle.push_back(x);
						rectangle.push_back(y);
						rectangle.push_back(z);
						rectangle.push_back(w);

						rectangles.push_back(rectangle);
					}

				}
			}

		}

}

Point HighLevelDetector::findInterceptions(Vec4i l1, Vec4i l2, double& a,
		double& b)
{
	int x1, x2, x3, x4;
	int y1, y2, y3, y4;

	getPointsCoordinates(l1, x1, y1, x2, y2);
	getPointsCoordinates(l2, x3, y3, x4, y4);

	double an = -(x2 * (y4 - y3) + x3 * (y2 - y4) + x4 * (y3 - y2));
	double ad = (x1 * (y4 - y3) + x2 * (y3 - y4) + x4 * (y2 - y1)
			+ x3 * (y1 - y2));

	a = an / ad;

	double bn = (x1 * (y4 - y2) + x2 * (y1 - y4) + x4 * (y2 - y1));
	double bd = (x1 * (y4 - y3) + x2 * (y3 - y4) + x4 * (y2 - y1)
			+ x3 * (y1 - y2));

	b = bn / bd;

	int x = a * x1 + (1 - a) * x2;
	int y = a * y1 + (1 - a) * y2;

	return Point(x, y);

}

void HighLevelDetector::findPoles(Vec4i l1, Vec4i l2)
{
	int x1, x2, x3, x4;
	int y1, y2, y3, y4;

	double dx, dy;

	getPointsCoordinates(l1, x1, y1, x2, y2);
	getPointsCoordinates(l2, x3, y3, x4, y4);

	//calculate the average dx and dy
	dx = abs(x1 + x2 - x3 - x4) / 2;
	dy = abs((y1 - y2) + (y3 - y4)) / 2;

	if (dy / dx > polesFormFactor)
	{
		vector<Point> pole;
		pole.push_back(Point(x1, y1));
		pole.push_back(Point(x2, y2));
		pole.push_back(Point(x4, y4));
		pole.push_back(Point(x3, y3));
		poles.push_back(pole);
	}

}

inline void HighLevelDetector::getPointsCoordinates(cv::Vec4i l, int& x1,
		int& y1, int& x2, int& y2)
{
	x1 = l[0];
	y1 = l[1];

	x2 = l[2];
	y2 = l[3];
}

bool HighLevelDetector::isQuadrilateral(vector<double> a, vector<double> b)
{
	int interceptionCounter = 0;
	int segmentCounter = 0;

	for (size_t i = 0; i < a.size() && i < b.size(); i++)
	{
		interceptionCounter += (-0.1 <= a[i]) && (a[i] <= 1.1);
		interceptionCounter += (-0.1 <= b[i]) && (b[i] <= 1.1);
	}

	segmentCounter += lineBelongToQuadrilateral(a[0], a[1]);
	segmentCounter += lineBelongToQuadrilateral(a[2], a[3]);
	segmentCounter += lineBelongToQuadrilateral(b[0], b[1]);
	segmentCounter += lineBelongToQuadrilateral(b[2], b[3]);

	return (interceptionCounter > 3) && (segmentCounter == 4);

}

bool HighLevelDetector::lineBelongToQuadrilateral(double a1, double a2)
{
	double a = a1 * a2;
	return (a < 0) || ((a1 > 0) && (a < 1));
}

