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

#ifndef BORDERDETECTOR_H_
#define BORDERDETECTOR_H_

#include <opencv2/core/core.hpp>

class BorderDetector
{
public:
	BorderDetector(int threshold1, int threshold2, int apertureSize,
			bool advancedMode) :
			threshold1(threshold1), threshold2(threshold2), apertureSize(
					apertureSize), advancedMode(advancedMode)
	{
	}

	cv::Mat detect(cv::Mat& input);

	//Getters and setters
	bool isAdvancedMode();
	void setAdvancedMode(bool advancedMode);
	int getApertureSize();
	void setApertureSize(int apertureSize);
	int getThreshold1();
	void setThreshold1(int threshold1);
	int getThreshold2();
	void setThreshold2(int threshold2);

private:
	int threshold1;
	int threshold2;
	int apertureSize;
	bool advancedMode;
};

#endif /* BORDERDETECTOR_H_ */
