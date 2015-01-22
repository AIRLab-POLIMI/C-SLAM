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

#ifndef QUADRILATERALDETECTOR_H_
#define QUADRILATERALDETECTOR_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ParameterServer.h"

#include "Rectangle.h"
#include "Pole.h"

class QuadrilateralDetector
{
public:
	QuadrilateralDetector(QDetectorParam& quadP);

	void detect(std::vector<cv::Vec4i>& verticalLines,
				std::vector<cv::Vec4i>& horizontalLines,
				bool skipCheck = false);

	inline std::vector<Pole>* getPoles() const
	{
		return poles;
	}

	inline std::vector<Rectangle>* getRectangles() const
	{
		return rectangles;
	}

private:
	inline void getPointsCoordinates(cv::Vec4i l, cv::Point& i, cv::Point& j);
	inline void getPointsCoordinates(cv::Vec4i l, cv::Vec3d& i, cv::Vec3d& j);
	cv::Point findInterception(cv::Vec4i l1, cv::Vec4i l2);
	int getMidLine(const cv::Vec4i& v1, const cv::Vec4i& v2);
	bool findPoles(cv::Vec4i l1, cv::Vec4i l2);
	//Quadrilater checks
	bool hasSufficientVerticalOverlap(cv::Vec4i& v1, cv::Vec4i& v2);
	bool hasSufficientHorizontallOverlap(cv::Vec4i& h1, cv::Vec4i& h2);
	bool isNotExternal(cv::Vec4i& v1, cv::Vec4i& v2, cv::Vec4i& h);
	bool isAboveMidline(const cv::Vec4i& h, int midLine);
	bool hasSufficientSupportV(cv::Vec4i& v, cv::Point a, cv::Point b);
	bool hasSufficientSupportH(cv::Vec4i& h, cv::Point a, cv::Point b);
	bool hasSufficientSupport(const cv::Point& x, const cv::Point& y,
				const cv::Point& z, const cv::Point& w, cv::Vec4i& h1,
				cv::Vec4i& h2, cv::Vec4i& v1, cv::Vec4i& v2);

private:
	std::vector<Rectangle>* rectangles;
	std::vector<Pole>* poles;

	//Params
	QDetectorParam& quadP;

};

#endif /* QUADRILATERALDETECTOR_H_ */
