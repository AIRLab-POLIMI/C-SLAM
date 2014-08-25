/*
 * c_vision,
 *
 *
 * Copyright (C) 2014 Davide Tateo
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

#ifndef MAPPINGTRACKER_H_
#define MAPPINGTRACKER_H_

#include "CMT.h"
#include "RobotPose.h"

#include "WorldMap.h"

#include <map>

class MappingTracker: public CMT
{
public:
	MappingTracker();
	virtual void initialize(const cv::Mat& im_gray0, InitializationData& data);
	void mapObject(cv::Mat& image, const cv::Mat_<double>& K, RobotPose& pose,
				WorldMap& map);
	void localizeFromObject(const cv::Mat_<double>& K, RobotPose& pose);

private:
	double matchKeyPoints(
				const std::vector<std::pair<cv::KeyPoint, int> >& matches,
				std::vector<cv::Point2f>& points1,
				std::vector<cv::Point2f>& points2, cv::Mat& image);

	void matchReconstructed(
				const std::vector<std::pair<cv::KeyPoint, int> >& matches,
				std::vector<cv::Point2f>& imagePoints,
				std::vector<cv::Point3f>& objectPoints);

	void reconstructPoints(WorldMap& map,
				const std::vector<std::pair<cv::KeyPoint, int> >& matches,
				const cv::Mat_<double>& K, RobotPose pose,
				const std::vector<cv::Point2f>& points1,
				const std::vector<cv::Point2f>& points2);

private:
	std::vector<cv::KeyPoint> mappedKeyPoints;
	std::map<int, cv::Point3d> reconstructedMap;
	bool objectMapped;
	double minDistance;

};

#endif /* MAPPINGTRACKER_H_ */
