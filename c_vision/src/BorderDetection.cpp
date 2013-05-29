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

#include <opencv2/imgproc/imgproc.hpp>
#include "BorderDetector.h"


cv::Mat BorderDetector::detect(cv::Mat& input)
{
	cv::Mat output;
	Canny(input, output, threshold1, threshold2, apertureSize);
	return output;
}

bool BorderDetector::isAdvancedMode()
{
	return advancedMode;
}

void BorderDetector::setAdvancedMode(bool advancedMode)
{
	this->advancedMode = advancedMode;
}

int BorderDetector::getApertureSize()
{
	return apertureSize;
}

void BorderDetector::setApertureSize(int apertureSize)
{
	this->apertureSize = apertureSize;
}

int BorderDetector::getThreshold1()
{
	return threshold1;
}

void BorderDetector::setThreshold1(int threshold1)
{
	this->threshold1 = threshold1;
}

int BorderDetector::getThreshold2()
{
	return threshold2;
}

void BorderDetector::setThreshold2(int threshold2)
{
	this->threshold2 = threshold2;
}
