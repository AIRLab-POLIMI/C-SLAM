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
#include "FeatureDetector.h"
#include "ClusterFilter.h"

void FeatureDetector::detect(cv::Mat& input)
{

	cv::Mat greyFrame, equalizedFrame;
	std::vector<cv::KeyPoint> bigKeypoints;

	cvtColor(input, greyFrame, CV_BGR2GRAY);

	equalizeHist(greyFrame, equalizedFrame);

	FAST(equalizedFrame, keyPoints, threshold);

	ClusterFilter filterObject(clusterWindow, clusterMinSize, noiseBarrier,
			input.cols, input.rows);
	ClusterFilter objectFinder(objectWindow, objectMinSize, noiseBarrier,
			input.cols, input.rows);

	filterObject.filter(keyPoints);
	keyPoints = filterObject.getFilteredKeyPoints();
	bigKeypoints = filterObject.getComplexObjects();

	//finds complex objects
	objectFinder.filter(bigKeypoints);
	complexObjects = objectFinder.getFilteredKeyPoints();

}
