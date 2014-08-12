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

#include <opencv2/core/core.hpp>

#include "ParameterServer.h"
#include "ClusterDetector.h"
#include "LineDetector.h"
#include "HighLevelDetector.h"

#include "ImageView.h"

class CognitiveDetector
{
public:
	CognitiveDetector(ParameterServer& parameters);

	void detect(cv::Mat& image);
	void detectRectangles(cv::Mat& image);

	inline void setRoll(double roll)
	{
		this->roll = roll;
	}

	std::vector<Cluster>* getClusters() const
	{
		return clusters;
	}

	std::vector<Pole>* getPoles() const
	{
		return poles;
	}

	std::vector<Rectangle>* getRectangles() const
	{
		return rectangles;
	}

	void deleteDetections();

private:
	void preprocessing(cv::Mat& input, cv::Mat& equalizedFrame, cv::Mat& grayFrame);
	void detectRectanglesAndPoles(cv::Mat& equalizedFrame);
	void setToNull();

private:
	//detectors
	ClusterDetector clusterDetector;
	LineDetector lineDetector;

	//envirorment data
	double roll;

	//last detections
	std::vector<Rectangle>* rectangles;
	std::vector<Pole>* poles;
	std::vector<Cluster>* clusters;

	std::vector<cv::Vec4i>* verticalLines;
	std::vector<cv::Vec4i>* horizontalLines;
};

#endif /* COGNITIVEDETECTOR_H_ */
