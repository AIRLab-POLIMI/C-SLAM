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
#include <opencv2/features2d/features2d.hpp>
#include "CornerDetector.h"
#include "ClusterFilter.h"

cv::Mat CornerDetector::detect(cv::Mat& input)
{

	cv::Mat output;
	std::vector<cv::KeyPoint> keyPoints;
	std::vector<cv::KeyPoint> bigKeypoints;
	std::vector<cv::KeyPoint> complexObjects;

	cvtColor(input, output, CV_BGR2GRAY);

	FAST(output, keyPoints, threshold);

	cvtColor(output, output, CV_GRAY2BGR);

	ClusterFilter filterObject(clusterWindow, clusterMinSize, noiseBarrier,
			input.cols, input.rows);
	ClusterFilter objectFinder(objectWindow, objectMinSize, noiseBarrier,
			input.cols, input.rows);

	keyPoints = filterObject.filter(keyPoints);
	bigKeypoints = filterObject.getComplexObjects();

	//finds complex objects
	complexObjects = objectFinder.filter(bigKeypoints);


	//display results
	for (size_t i = 0; i < keyPoints.size(); ++i)
	{
		const cv::KeyPoint& kp = keyPoints[i];
		circle(output, kp.pt, kp.size / 2, CV_RGB(255, 0, 0));
	}

	for (size_t i = 0; i < complexObjects.size(); ++i)
		{
			const cv::KeyPoint& kp = complexObjects[i];
			circle(output, kp.pt, 20, CV_RGB(0, 0, 255));
		}

	return output;
}

int CornerDetector::getThreshold() const
{
	return threshold;
}

void CornerDetector::setThreshold(int threshold)
{
	this->threshold = threshold;
}

int CornerDetector::getClusterMinSize() const
{
	return clusterMinSize;
}

void CornerDetector::setClusterMinSize(int clusterMinSize)
{
	this->clusterMinSize = clusterMinSize;
}

int CornerDetector::getClusterWindow() const
{
	return clusterWindow;
}

void CornerDetector::setClusterWindow(int clusterWindow)
{
	this->clusterWindow = clusterWindow;
}

int CornerDetector::getNoiseBarrier() const
{
	return noiseBarrier;
}

void CornerDetector::setNoiseBarrier(int noiseBarrier)
{
	this->noiseBarrier = noiseBarrier;
}

int CornerDetector::getObjectMinSize() const
{
	return objectMinSize;
}

void CornerDetector::setObjectMinSize(int objectMinSize)
{
	this->objectMinSize = objectMinSize;
}

int CornerDetector::getObjectWindow() const
{
	return objectWindow;
}

void CornerDetector::setObjectWindow(int objectWindow)
{
	this->objectWindow = objectWindow;
}

