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

/*!
 * Class used to Detect Lines
 *
 *
 */
class LineDetector
{

public:
	LineDetector(int rho, double theta, int threshold) :
			rho(rho), theta(theta), threshold(threshold)
	{
	}

	virtual cv::Mat detect(cv::Mat& input);

	//getters
	int getRho();
	void setRho(int rho);
	double getTheta();
	void setTheta(double theta);
	int getThreshold();
	void setThreshold(int threshold);

	virtual ~LineDetector()
	{
	}

protected:
	///Distance resolution of the accumulator in pixels
	int rho;
	///Angle resolution of the accumulator in radians
	double theta;
	///The accumulator threshold parameter. Only those lines are returned that get enough votes
	int threshold;
};

class HoughLineDetector: public LineDetector
{

public:
	HoughLineDetector(int rho, double theta, int threshold) :
			LineDetector(rho, theta, threshold)
	{
	}
	cv::Mat detect(cv::Mat& input);

	~HoughLineDetector()
	{
	}

};

class ProbabilisticHoughLineDetector: public LineDetector
{

public:
	ProbabilisticHoughLineDetector(int rho, double theta, int threshold,
			int minLineLenght, int maxLineGap) :
			LineDetector(rho, theta, threshold), minLineLength(minLineLenght), maxLineGap(
					maxLineGap)
	{
	}

	cv::Mat detect(cv::Mat& input);

	//getters
	int getMaxLineGap();
	void setMaxLineGap(int maxLineGap);
	int getMinLineLength();
	void setMinLineLength(int minLineLength);

	~ProbabilisticHoughLineDetector()
	{
	}

private:
	///The minimum line length. Line segments shorter than that will be rejected
	int minLineLength;
	///The maximum allowed gap between points on the same line to link them.
	int maxLineGap;
};

#endif /* LINEDETECTOR_H_ */
