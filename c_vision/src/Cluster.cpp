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

#include "Cluster.h"

using namespace cv;

Cluster::Cluster()
{
	massCenter.pt.x = 0;
	massCenter.pt.y = 0;
	massCenter.size = 0;
}

void Cluster::add(KeyPoint point)
{
	updateMassCenter(point);
	updateBoundingBox(point);

	keyPoints.push_back(point);
}

Point Cluster::getCenter()
{
	return massCenter.pt;
}

void Cluster::updateMassCenter(KeyPoint point)
{
	int y = point.pt.y;
	int x = point.pt.x;
	float size = point.size;

	massCenter.pt.x += x * size;
	massCenter.pt.y += y * size;

	massCenter.size += size;
}

void Cluster::updateBoundingBox(KeyPoint point)
{
	int y = point.pt.y;
	int x = point.pt.x;

	if (keyPoints.size() > 0)
	{

		start.y = (start.y < y) ? start.y : y;
		start.x = (start.x < x) ? start.x : x;

		end.y = (end.y > y) ? end.y : y;
		end.x = (end.x > x) ? end.x : x;
	}
	else
	{
		start.x = x;
		start.y = y;

		end.x = x;
		end.y = y;
	}

}

void Cluster::setFeature()
{
	massCenter.pt.x /= massCenter.size;
	massCenter.pt.y /= massCenter.size;

	featureMap["x"] =  massCenter.pt.x;
	featureMap["y"] = massCenter.pt.y;
	featureMap["size"] = massCenter.size;
}

void Cluster::draw(Mat& frame, Scalar color) const
{
	rectangle(frame, start, end, color, 2);
}
