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

#include "Cluster.h"

bool Cluster::isEmpty()
{
	return points.size() == 0;
}

bool Cluster::addToCluster(cv::KeyPoint* point, int windowSize)
{
	int y = point->pt.y;
	int x = point->pt.x;
	float size = point->size;

	if (points.size() == 0)
	{
		start = y;
		massCenter.pt.x = x * size;
		massCenter.pt.y = y * size;

		massCenter.size = size;
		points.push_back(point);
		return true;
	}
	else if (y < start + windowSize)
	{
		massCenter.pt.x += x * size;
		massCenter.pt.y += y * size;
		massCenter.size += size;
		points.push_back(point);
		return true;
	}
	else
	{
		return false;
	}
}

cv::KeyPoint Cluster::getMassCenter()
{

	massCenter.pt.x /= massCenter.size;
	massCenter.pt.y /= massCenter.size;

	return massCenter;
}
