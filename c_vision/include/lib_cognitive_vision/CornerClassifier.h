/*
 * c_vision,
 *
 *
 * Copyright (C) 2015 Davide Tateo
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

#ifndef INCLUDE_LIB_COGNITIVE_VISION_CORNERCLASSIFIER_H_
#define INCLUDE_LIB_COGNITIVE_VISION_CORNERCLASSIFIER_H_

#include <opencv2/opencv.hpp>

#include "ParameterServer.h"

/**
 * x--N--x
 * |  |  |
 * W--x--E
 * |  |  |
 * x--S--x
 */

enum CornerType
{
	// L corners
	NW = 0,
	NE,
	SW,
	SE,
	// I corners
	NS,
	WE,
	// T-corners
	NWE,
	SWE,
	NSW,
	NSE,
	// X corners
	NSWE
};


enum Bucket
{
	N=0, S=1, W=2, E=3
};

class CornerClassifier
{
public:
	CornerClassifier(CornerClassParam& params);
	bool isCompatibleCorner(const cv::Mat& canny, const cv::Point& point, CornerType type);
	void setRoll(double roll)
	{
		sinR = std::sin(roll);
		cosR = std::cos(roll);
	}
private:
	void computeHistogram(const cv::Mat& canny, const cv::Point& point,
				cv::Mat& hist);
	void addToBucket(cv::Point c, int x, int y, cv::Mat& hist);
	CornerType findNearestNeighbour(const cv::Mat& hist);

private:
	static std::vector<cv::Mat> initPrototypes();

private:
	CornerClassParam& params;
	double sinR;
	double cosR;

private:
	static const std::vector<cv::Mat> cornerPrototypes;
	static const std::size_t prototypeN;

};

#endif /* INCLUDE_LIB_COGNITIVE_VISION_CORNERCLASSIFIER_H_ */
