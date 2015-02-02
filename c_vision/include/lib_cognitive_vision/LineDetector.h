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

#ifndef LINEDETECTOR_H_
#define LINEDETECTOR_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "ParameterServer.h"
#include "ImageView.h"

class LineDetector
{
public:
	LineDetector(CannyParam& cannyP, HoughParam& houghP, LFilterParam& filterP);
	void detect(cv::Mat& input, double roll, const cv::Mat& mask = cv::Mat(), bool showCanny = true);

	std::vector<cv::Vec4i>* getVerticalLines()
	{
		return verticalLines;
	}

	std::vector<cv::Vec4i>* getHorizontalLines()
	{
		return horizontalLines;
	}

private:
	int computeThreshold(cv::Mat& src);

private:
	CannyParam& cannyP;
	HoughParam& houghP;
	LFilterParam& filterP;

	std::vector<cv::Vec4i>* verticalLines;
	std::vector<cv::Vec4i>* horizontalLines;

	ImageView viewer;

	//FIXME temporary hack
	void hackFunction(cv::Mat& canny);
};

#endif /* LINEDETECTOR_H_ */
