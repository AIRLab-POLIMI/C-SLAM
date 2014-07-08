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

#ifndef LINEFILTER_H_
#define LINEFILTER_H_

#include <opencv2/core/core.hpp>

#include "ParameterServer.h"

class LineFilter
{
public:
	LineFilter(LFilterParam& filterP);

	void filter(std::vector<cv::Vec4i>& lines, double roll);

	std::vector<cv::Vec4i>* getVerticalLines()
	{
		return verticalLines;
	}

	std::vector<cv::Vec4i>* getHorizontalLines()
	{
		return horizontalLines;
	}

private:
	bool sameSlope(double line, double reference, double maxDelta);

	static bool isHighestLine(cv::Vec4i i, cv::Vec4i j);
	static bool isLeftmostLine(cv::Vec4i i, cv::Vec4i j);

private:
	std::vector<cv::Vec4i>* verticalLines;
	std::vector<cv::Vec4i>* horizontalLines;

	LFilterParam& filterP;
};

#endif /* LINEFILTER_H_ */
