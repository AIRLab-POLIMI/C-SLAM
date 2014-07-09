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

#ifndef HIGHLEVELDETECTOR_H_
#define HIGHLEVELDETECTOR_H_

#include <vector>
#include <opencv2/core/core.hpp>

#include "Rectangle.h"
#include "Pole.h"

class HighLevelDetector
{
public:
	HighLevelDetector();

	void detect(std::vector<cv::Vec4i>& verticalLines,
				std::vector<cv::Vec4i>& horizontalLines);

	inline std::vector<Pole>* getPoles() const
	{
		return poles;
	}

	inline std::vector<Rectangle>* getRectangles() const
	{
		return rectangles;
	}

private:
	inline void getPointsCoordinates(cv::Vec4i l, int& x1, int& y1, int& x2,
				int& y2);
	cv::Point findInterception(cv::Vec4i l1, cv::Vec4i l2, double& a,
				double& b);
	bool findPoles(cv::Vec4i l1, cv::Vec4i l2);
	bool isQuadrilateral(std::vector<double> a, std::vector<double> b);
	bool lineBelongToQuadrilateral(double a1, double a2);
	void normalizeLines(int& x1, int& y1, int& x2, int& y2, int& x3, int& y3,
				int& x4, int& y4);

private:
	std::vector<Rectangle>* rectangles;
	std::vector<Pole>* poles;
	static const int polesFormFactor = 20;

};

#endif /* HIGHLEVELDETECTOR_H_ */
