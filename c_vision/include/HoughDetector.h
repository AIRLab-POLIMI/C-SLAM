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

#ifndef HOUGHDETECTOR_H_
#define HOUGHDETECTOR_H_

#include <opencv2/core/core.hpp>

class HoughDetector
{
public:
	HoughDetector(int apertureSize, int rho, double theta, int threshold,
			int minLineLenght, int maxLineGap) :
			apertureSize(apertureSize), rho(rho), theta(theta), threshold(
					threshold), minLineLength(minLineLenght), maxLineGap(
					maxLineGap)
	{
	}

	std::vector<cv::Vec4i> detect(cv::Mat& input);

	int getApertureSize() const
	{
		return apertureSize;
	}

	void setApertureSize(int apertureSize)
	{
		this->apertureSize = apertureSize;
	}

	int getMaxLineGap() const
	{
		return maxLineGap;
	}

	void setMaxLineGap(int maxLineGap)
	{
		this->maxLineGap = maxLineGap;
	}

	int getMinLineLength() const
	{
		return minLineLength;
	}

	void setMinLineLength(int minLineLength)
	{
		this->minLineLength = minLineLength;
	}

	int getRho() const
	{
		return rho;
	}

	void setRho(int rho)
	{
		this->rho = rho;
	}

	double getTheta() const
	{
		return theta;
	}

	void setTheta(double theta)
	{
		this->theta = theta;
	}

	int getThreshold() const
	{
		return threshold;
	}

	void setThreshold(int threshold)
	{
		this->threshold = threshold;
	}

private:
	int apertureSize;
	int rho;
	double theta;
	int threshold;
	int minLineLength;
	int maxLineGap;

};

#endif /* HOUGHDETECTOR_H_ */
