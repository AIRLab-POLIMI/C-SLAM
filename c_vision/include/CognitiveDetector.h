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

#ifndef COGNITIVEDETECTOR_H_
#define COGNITIVEDETECTOR_H_

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "DefaultParameters.h"
#include "FeatureDetector.h"
#include "LineDetector.h"
#include "HoughDetector.h"
#include "DBScan.h"

#include <iostream>

static const std::string CORNER_WINDOW = "Detected Image";
static const std::string LINE_WINDOW = "Line Image";

class CognitiveDetector
{
public:
	CognitiveDetector() :
			featureDetector(cornerP.threshold), clusterDetector(
					clusterP.maxDistance, clusterP.minPoints), lineDetector(
					cannyP.minCanny, cannyP.maxCanny, cannyP.apertureSize,
					houghP.rho, houghP.teta, houghP.threshold,
					houghP.minLineLenght, houghP.maxLineGap), pitch(0), roll(0), yaw(
					0)
	{
		cv::namedWindow(CORNER_WINDOW);
		cv::namedWindow(LINE_WINDOW);
		createTrackBars();
	}

	~CognitiveDetector()
	{
		cv::destroyWindow(CORNER_WINDOW);
		cv::destroyWindow(LINE_WINDOW);
	}

	void detect(cv::Mat image);

	inline void setPitch(double pitch)
	{
		this->pitch = pitch;
	}

	inline void setRoll(double roll)
	{
		this->roll = roll;
		std::cerr << "roll is: " << roll << std::endl;

	}

	inline void setYaw(double yaw)
	{
		this->yaw = yaw;
	}

private:
	void drawAxis(cv::Mat& input);
	void createTrackBars();
	void displayClusterResults(std::vector<cv::KeyPoint>& keyPoints,
			std::vector<ObjectCluster>& clusters, cv::Mat& frame);
	void displayLineResults(std::vector<cv::Vec4i> lines, cv::Mat& frame);

private:
	std::vector<cv::Vec4i> extractVerticalLines(std::vector<cv::Vec4i> lines);

private:
	FeatureDetector featureDetector;
	DBScan clusterDetector;
	HoughDetector lineDetector;

	double pitch;
	double roll;
	double yaw;

};

#endif /* COGNITIVEDETECTOR_H_ */
