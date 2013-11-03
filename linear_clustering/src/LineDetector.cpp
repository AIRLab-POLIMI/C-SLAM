/*
 * linear_clustering,
 *
 *
 * Copyright (C) 2013 Davide Tateo
 * Versione 1.0
 *
 * This file is part of linear_clustering.
 *
 * linear_clustering is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * linear_clustering is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with linear_clustering.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "LineDetector.h"

#include <iostream>

void LineDetector::detect(std::vector<cv::KeyPoint> points, double m)
{
	Comparator comparator(m);
	std::sort(points.begin(), points.end(), comparator);

	for (size_t i = 0; i < points.size(); i++)
	{
		LineCluster* cluster = new LineCluster(m, maxDelta);

		while (i < points.size() && cluster->belongTo(&points[i]))
		{
			cluster->add(&points[i]);
			i++;
		}

		if (cluster->isLine())
			clusters.push_back(cluster);
	}

}

void LineDetector::detect(std::vector<cv::KeyPoint> points)
{
	ComparatorY comparator;
	std::sort(points.begin(), points.end(), comparator);

	for (size_t i = 0; i < points.size(); i++)
	{
		HorizontalLineCluster* cluster = new HorizontalLineCluster(maxDelta);

		while (i < points.size() && cluster->belongTo(&points[i]))
		{
			cluster->add(&points[i]);
			i++;
		}

		if (cluster->isLine())
			clusters.push_back(cluster);
	}

}

std::vector<Line> LineDetector::getLines()
{
	std::vector<Line> lines;
	std::vector<AbstractLineCluster*>::iterator it;
	for (it = clusters.begin(); it != clusters.end(); it++)
	{
		AbstractLineCluster* cluster = *it;
		Line line;
		line.start = cluster->getLineStart();
		line.end = cluster->getLineEnd();
		lines.push_back(line);
	}

	clusters.erase(clusters.begin(), clusters.end());

	return lines;

}
