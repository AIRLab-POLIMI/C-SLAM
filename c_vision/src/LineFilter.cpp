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

#include "LineFilter.h"

#include <algorithm>
#include <iostream>

#include <angles/angles.h>
#include <ros/ros.h>

using namespace angles;

using namespace cv;
using namespace std;

bool isHighestLine(cv::Vec4i i, cv::Vec4i j);
bool isLeftmostLine(cv::Vec4i i, cv::Vec4i j);

void LineFilter::filter(vector<Vec4i>& lines, double roll)
{
	const double delta_max_vertical = from_degrees(5.0); //5 degrees of error
	const double delta_max_horizontal = from_degrees(10.0); //5 degrees of error

	const double horizontal_line = from_degrees(roll);
	const double vertical_line = from_degrees(roll + 90);


	for (size_t i = 0; i < lines.size(); i++)
	{
		Vec4i& line = lines[i];
		int dx = line[0] - line[2];
		int dy = line[1] - line[3];

		double alpha_line = atan2(dy, dx);

		//e' orizzontale
		if (sameSlope(alpha_line, horizontal_line, delta_max_horizontal))
		{
			ROS_DEBUG_STREAM(
						"linea: " << to_degrees(alpha_line) << " orizzontale: " << roll << " distanza: " << to_degrees(shortest_angular_distance(alpha_line, horizontal_line)));
			horizontalLines.push_back(lines[i]);
		}
		//e' verticale
		else if (sameSlope(alpha_line, vertical_line, delta_max_vertical))
		{
			ROS_DEBUG_STREAM(
						"linea: " << to_degrees(alpha_line) << " verticale: " << roll + 90 << " distanza: " << to_degrees(shortest_angular_distance(alpha_line, vertical_line)));
			verticalLines.push_back(lines[i]);
		}
	}

	sort(verticalLines.begin(), verticalLines.end(), isLeftmostLine);
	sort(horizontalLines.begin(), horizontalLines.end(), isHighestLine);

}

bool LineFilter::sameSlope(double line, double reference, double maxDelta)
{
	double delta = abs(shortest_angular_distance(line,reference));
	return delta < maxDelta ||  delta > from_degrees(180) - maxDelta;
}

bool isHighestLine(Vec4i i, Vec4i j)
{
	return (i[1] > j[1]) && (i[3] > j[3]);
}

bool isLeftmostLine(Vec4i i, Vec4i j)
{
	return (i[0] < j[0]) && (i[2] < j[2]);
}
