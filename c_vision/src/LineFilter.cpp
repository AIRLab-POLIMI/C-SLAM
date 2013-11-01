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

void LineFilter::filter(std::vector<cv::Vec4i> lines, double roll)
{
	const double delta_max_vertical = 5 * M_PI / 180; //5 degrees of error
	const double delta_max_horizontal = 5 * M_PI / 180; //5 degrees of error

	const double orizontal_line1 = roll * M_PI / 180.0;
	const double orizontal_line2 = orizontal_line1 + M_PI;
	const double vertical_line = orizontal_line1 + M_PI/2;


	for (size_t i = 0; i < lines.size(); i++)
	{
		cv::Vec4i& line = lines[i];
		int dx = line[0] - line[2];
		int dy = line[1] - line[3];

		double alpha_line = atan2(dy, dx);

		if (sameSlope(vertical_line, alpha_line, delta_max_vertical))
		{
			verticalLines.push_back(lines[i]);
		}
		else if (sameSlope(orizontal_line1, alpha_line, delta_max_horizontal) ||
				sameSlope(orizontal_line2, alpha_line, delta_max_horizontal))
		{
			horizontalLines.push_back(lines[i]);
		}
	}

}

bool LineFilter::sameSlope(double alpha, double beta, double maxDelta)
{
	double delta = abs(abs(alpha) - abs(beta));

	return delta < maxDelta;

}

bool isHighestLine(cv::Vec4i i, cv::Vec4i j)
{
	return true;
}

bool isLeftmostLine(cv::Vec4i i, cv::Vec4i j)
{
	return true;
}
