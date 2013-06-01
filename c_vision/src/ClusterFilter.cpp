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

#include "ClusterFilter.h"

#include <iostream>

std::vector<cv::KeyPoint> ClusterFilter::filter(std::vector<cv::KeyPoint> input)
{
	orderKeyPoints(input);
	std::vector<cv::KeyPoint> output;

	std::cerr << "il vettore contiene " << output.size() << std::endl;
	std::cerr << "la finestra è larga " << windowSize << std::endl;
	std::cerr << "in media ci sono " << output.size() / width << " punti per finestra " << std::endl;


	//main algorithm loop
	while (baseIndex < width)
	{
		std::vector<Cluster> clusters;
		int remaining = windowSize;

		//start clustering

		while (remaining > 0)
		{
			Cluster cluster;
			createClusters(cluster, remaining);
			clusters.push_back(cluster);
		}
		//save results
		savePoints(clusters, output);
		//reorder new points
		orderVectors(baseIndex + stepSize + 1, topIndex);

		//Update the indexes
		baseIndex += stepSize;
		topIndex = baseIndex + windowSize;
		if (topIndex > width)
			topIndex = width;

	}

	return output;
}

void ClusterFilter::orderKeyPoints(std::vector<cv::KeyPoint>& input)
{
	std::vector<cv::KeyPoint>::iterator it;

	//orders the x
	for (it = input.begin(); it != input.end(); ++it)
	{
		keyPoints[(int) it->pt.x].push_back(*it);
	}

	//orders the y
	orderVectors(0, width);
}

void ClusterFilter::orderVectors(int begin, int end)
{
	for (int i = begin; i < end; i++)
	{
		if (keyPoints[i].size() > 1)
		{
			std::vector<cv::KeyPoint>::iterator begin = keyPoints[i].begin();
			std::vector<cv::KeyPoint>::iterator end = keyPoints[i].end();
			std::sort(begin, end, comparator);
		}
	}
}

void ClusterFilter::savePoints(std::vector<Cluster>& clusters,
		std::vector<cv::KeyPoint>& output)
{
	std::vector<Cluster>::iterator it;
	for (it = clusters.begin(); it != clusters.end(); ++it)
	{
		cv::KeyPoint massCenter = it->getMassCenter();
		int x = massCenter.pt.x;
		//puts new points in the matrix
		//but only those who are needed for the next window
		if (x - baseIndex > stepSize)
		{
			keyPoints[x].push_back(massCenter);
		}
		//puts the new points in the output
		//but only the ones who are already done
		else
		{
			output.push_back(massCenter);
		}
	}
}

void ClusterFilter::createClusters(Cluster& cluster, int& remaining)
{
	std::cerr << "baseIndex = " << baseIndex << " topIndex = " << topIndex <<std::endl;
	for (int i = baseIndex; i < topIndex; i++)
	{
		bool sameCluster = true;
		//create a cluster
		//TODO: correggere caso particolare in cui la colonna i+1 abbia cluster separati prima della colonna i sulla y
		while (keyPoints[i].size() > 0 && sameCluster)
		{
			cv::KeyPoint point = keyPoints[i].back();
			sameCluster = cluster.addToCluster(&point, windowSize);

			//delete point from the matrix if is in the cluster
			if (sameCluster)
				keyPoints[i].pop_back();

			//update remaining if a column has no more points
			if (keyPoints[i].size() == 0)
				remaining--;
		}
	}
}

ClusterFilter::~ClusterFilter()
{
	/*for (int i = 0; i < width; i++)
		delete keyPoints[i];*/
	delete[] keyPoints;
}

