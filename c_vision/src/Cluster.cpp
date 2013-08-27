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

bool Cluster::belongTo(cv::KeyPoint* point)
{
	return isEmpty() || distance(*point, massCenter) * 2 < windowSize;
}

void Cluster::add(cv::KeyPoint* point)
{
	int y = point->pt.y;
	int x = point->pt.x;
	float size = point->size;

	massCenter.pt.x = massCenter.pt.x * massCenter.size + x * size;
	massCenter.pt.y = massCenter.pt.y * massCenter.size + y * size;

	massCenter.size += size;

	massCenter.pt.x /= massCenter.size;
	massCenter.pt.y /= massCenter.size;

	points.push_back(point);
}

cv::KeyPoint Cluster::getMassCenter()
{
	return massCenter;
}

int Cluster::distance(cv::KeyPoint start, cv::KeyPoint end)
{
	int ystart = start.pt.y;
	int yend = end.pt.y;

	int distance = ystart - yend;
	distance = (distance < 0) ? -distance : distance;

	return distance;

}

std::vector<cv::KeyPoint> MetaCluster::getMassCenters()
{
	std::vector<cv::KeyPoint> massCenters;

	//get all sub clusters of meta-cluster
	while (!points.empty())
	{
		Cluster cluster(windowSize);

		while (!points.empty() && cluster.belongTo(points.back()))
		{
			cluster.add(points.back());
			points.pop_back();
		}

		massCenters.push_back(cluster.getMassCenter());
	}

	return massCenters;
}

cv::KeyPoint LineCluster::calculateEdge(cv::KeyPoint* point)
{
	cv::KeyPoint edge;
	int y = point->pt.y;
	int x = point->pt.x;
	edge.pt.y = y;
	edge.pt.x = m * x + q;

	return edge;
}

bool LineCluster::belongTo(cv::KeyPoint* point)
{
	int y = point->pt.y;
	int x = point->pt.x;

	double d2 = (double) (x - m*y - q)*(x - m*y - q) / (1 +m*m);

	return isEmpty() || d2 <= maxDelta*maxDelta ;
}

void LineCluster::add(cv::KeyPoint* point)
{
	int y = point->pt.y;
	int x = point->pt.x;
	int n = points.size();

	double qn = x-m*y;

	q = (n*q + qn) / n;

	points.push_back(point);
}

