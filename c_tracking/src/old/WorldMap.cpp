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

#include "WorldMap.h"

#include <pcl_ros/point_cloud.h>


using namespace cv;
using namespace std;

WorldMap::WorldMap(ros::NodeHandle& n)
{
	mapPublisher = n.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("cloud_map", 3);
}

void WorldMap::addObject(vector<Point3d>& objectPoints)
{

	ROS_INFO_STREAM("mapped " << objectPoints.size() << " points");
	pcl::PointCloud<pcl::PointXYZRGB> objectCloud;

	objectCloud.header.frame_id = "map";

	for(int i = 0; i < objectPoints.size(); i++)
	{
		pcl::PointXYZRGB pclPoint;

		pclPoint.x = objectPoints[i].x;
		pclPoint.y = objectPoints[i].y;
		pclPoint.z = objectPoints[i].z;

		uint8_t r = 255, g = 0, b = 0;    // Example: Red color
		uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		pclPoint.rgb = *reinterpret_cast<float*>(&rgb);

		objectCloud.push_back(pclPoint);
	}

	mapPublisher.publish(objectCloud);

}
