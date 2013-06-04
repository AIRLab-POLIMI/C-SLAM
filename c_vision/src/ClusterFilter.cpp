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
	if (windowSize == 0 || input.size() == 0)
		return input;

	orderKeyPoints(input);

	//main algorithm loop
	while (baseIndex < width)
	{
		//The vector of clusters
		std::vector<Cluster> clusters;

		//find clusters in the input
		findClusters(clusters);

		//save results of clustering stage
		savePoints(clusters);

		//reorder new points added to the clusters
		orderVectors(baseIndex + stepSize + 1, topIndex);

		//Update the indexes
		updateIndexes();
	}

	return output;
}

std::vector<cv::KeyPoint> ClusterFilter::getComplexObjects()
{
	return complexObjects;
}

void ClusterFilter::findClusters(std::vector<Cluster>& clusters)
{
	int remaining;

	//start clustering
	do
	{
		Cluster cluster;
		createClusters(cluster);
		if (!cluster.isEmpty())
			clusters.push_back(cluster);
		remaining = countRemaining();
	} while (remaining > 0);
}

void ClusterFilter::savePoints(std::vector<Cluster>& clusters)
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
		else if (massCenter.size < clusterMinSize)
		{
			output.push_back(massCenter);
		}
		//puts the biggest points in the pipeline for complex objects recognition
		else if (massCenter.size > noiseBarrier)
		{
			complexObjects.push_back(massCenter);
		}
	}
}

void ClusterFilter::createClusters(Cluster& cluster)
{
	//Get the ordered keyPoints of the current analyzed window
	//We use only a pointer to that data, for efficiency reasons.
	std::vector<std::vector<cv::KeyPoint>*> points = getOrderedKeyPoints();

	std::vector<std::vector<cv::KeyPoint>*>::iterator it;

	for (it = points.begin(); it != points.end(); ++it)
	{
		std::vector<cv::KeyPoint>& currentVector = **it;
		bool sameCluster = true;
		//create a cluster
		while (currentVector.size() > 0 && sameCluster)
		{
			cv::KeyPoint point = currentVector.back();
			//check if the point belongs to cluster
			sameCluster = cluster.pointsBelongsTo(&point, windowSize);
			//if so...
			if (sameCluster)
			{
				//add the point to the cluster
				cluster.addToCluster(&point, windowSize);
				//delete point from the matrix if is in the cluster
				currentVector.pop_back();
			}
		}
	}
}

std::vector<std::vector<cv::KeyPoint>*> ClusterFilter::getOrderedKeyPoints()
{
	std::vector<std::vector<cv::KeyPoint>*> points;
	for (int i = baseIndex; i < topIndex; i++)
	{
		if (keyPoints[i].size() > 0)
			points.push_back(&keyPoints[i]);
	}

	std::vector<std::vector<cv::KeyPoint>*>::iterator begin = points.begin();
	std::vector<std::vector<cv::KeyPoint>*>::iterator end = points.end();
	std::sort(begin, end, comparator);

	std::sort(begin, end, comparator);

	return points;
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

int ClusterFilter::countRemaining()
{
	int remaining = 0;

	for (int i = baseIndex; i < topIndex; i++)
	{
		if (keyPoints[i].size() != 0)
			remaining++;
	}

	return remaining;
}

void ClusterFilter::updateIndexes()
{
	//Update the indexes
	baseIndex += stepSize;
	topIndex = baseIndex + windowSize;
	if (topIndex > width)
		topIndex = width;
}

ClusterFilter::~ClusterFilter()
{
	delete[] keyPoints;
}

