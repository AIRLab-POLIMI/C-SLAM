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

#ifndef CLUSTERFILTER_H_
#define CLUSTERFILTER_H_

#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include <cassert>

#include "Cluster.h"

class ClusterFilter
{
public:
	ClusterFilter(int windowSize, int clusterMinSize, int noiseBarrier,
			int width, int height) :
			windowSize(windowSize), clusterMinSize(clusterMinSize), noiseBarrier(
					noiseBarrier), width(width), height(height)
	{
		//the stepsize is the half of the windowSize
		// add 1 if the windowsize is not even
		stepSize = windowSize / 2 + windowSize % 2;

		//initialize window
		baseIndex = 0;
		topIndex = windowSize;

		//the matrix used within the algoritm
		keyPoints = new std::vector<cv::KeyPoint>[width];
		assert(windowSize < width);
	}

	std::vector<cv::KeyPoint> filter(std::vector<cv::KeyPoint> input);
	std::vector<cv::KeyPoint> getComplexObjects();

	~ClusterFilter();

private:
	void findClusters(std::vector<Cluster>& clusters);
	void savePoints(std::vector<Cluster>& clusters);
	void orderKeyPoints(std::vector<cv::KeyPoint>& input);
	void orderVectors(int begin, int end);
	void createClusters(Cluster& cluster);
	int countRemaining();
	std::vector<std::vector<cv::KeyPoint>*> getOrderedKeyPoints();
	void updateIndexes();

private:
	int windowSize;
	int clusterMinSize;
	int noiseBarrier;

	int width;
	int height;

	int stepSize;
	int baseIndex;
	int topIndex;

	std::vector<cv::KeyPoint>* keyPoints;
	std::vector<cv::KeyPoint> output;
	std::vector<cv::KeyPoint> complexObjects;

private:
	class Comparator
	{
	public:
		bool operator()(cv::KeyPoint i, cv::KeyPoint j)
		{
			return i.pt.y < j.pt.y;
		}

		bool operator()(std::vector<cv::KeyPoint>* i,
				std::vector<cv::KeyPoint>* j)
		{
			std::vector<cv::KeyPoint>& v1 = *i;
			std::vector<cv::KeyPoint>& v2 = *j;
			return (*this)(v1[0], v2[0]);
		}

	} comparator;
};

#endif /* CLUSTERFILTER_H_ */
