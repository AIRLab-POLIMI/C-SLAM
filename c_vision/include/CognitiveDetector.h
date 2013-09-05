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

static const std::string WINDOW = "Detected Image";
static const std::string LINE_WINDOW = "Line Image";

class CognitiveDetector
{
public:
	CognitiveDetector() :
			featureDetector(cornerP.threshold, cornerP.windowSize,
					cornerP.clusterMinSize, cornerP.noiseBarrier,
					cornerP.objectWindow, cornerP.objectMinSize), lineFinder(
					lineP.maxDelta), lineDetector(cannyP.minCanny,
					cannyP.maxCanny, cannyP.apertureSize, houghP.rho, houghP.teta,
					houghP.threshold, houghP.minLineLenght,
					houghP.maxLineGap), pitch(0), roll(0), yaw(0)
	{
		cv::namedWindow(WINDOW);
		cv::namedWindow(LINE_WINDOW);
		createTrackBars();
	}

	~CognitiveDetector()
	{
		cv::destroyWindow(WINDOW);
	}

	void detect(cv::Mat image);

	inline void setPitch(double pitch)
	{
		this->pitch = pitch;
	}

	inline void setRoll(double roll)
	{
		this->roll = roll;
	}

	inline void setYaw(double yaw)
	{
		this->yaw = yaw;
	}

private:
	std::vector<Line> findLines(std::vector<cv::KeyPoint> keyPoints);
	std::vector<Line> findLines(double roll,
			const std::vector<cv::KeyPoint>& keyPoints);
	void drawAxis(cv::Mat& input);
	void createTrackBars();
	void displayResults(const std::vector<cv::KeyPoint>& keyPoints,
			const std::vector<cv::KeyPoint>& complexObjects,
			const std::vector<Line>& lines, cv::Mat& frame);
	void addOldKeyPoints(std::vector<cv::KeyPoint>& keyPoints);

private:
	FeatureDetector featureDetector;
	LineDetector lineFinder;
	HoughDetector lineDetector;

	double pitch;
	double roll;
	double yaw;

};

#endif /* COGNITIVEDETECTOR_H_ */
