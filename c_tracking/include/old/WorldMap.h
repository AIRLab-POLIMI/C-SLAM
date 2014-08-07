/*
 * c_tracking,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_tracking.
 *
 * c_tracking is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_tracking is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_tracking.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef WORLDMAP_H_
#define WORLDMAP_H_

#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class WorldMap
{
public:
	WorldMap(ros::NodeHandle& n);
	void addObject(std::vector<cv::Point3d>& objectPoints);

private:
	ros::Publisher mapPublisher;


};


#endif /* WORLDMAP_H_ */
