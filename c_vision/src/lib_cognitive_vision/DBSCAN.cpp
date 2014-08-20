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

#include "DBSCAN.h"
#include <set>

using namespace std;
using namespace cv;

DBSCAN::DBSCAN(double& eps, int& minPts) :
			maxDistance(eps), minPoints(minPts)
{
}

std::vector<Cluster>* DBSCAN::detect(const vector<KeyPoint>& keypoints)
{
	vector<Cluster>* clustersPointer = new vector<Cluster>();
	vector<Cluster>& clusters = *clustersPointer;
	set<int> clustered;
	set<int> visited;

	int keyPointsNumber = keypoints.size();

	for (size_t i = 0; i < keyPointsNumber; i++)
	{
		//consider unvisited points
		if (!visited.count(i))
		{
			visited.insert(i);
			vector<int> neighbor = listNeighbors(keypoints, keypoints[i]);

			//create new cluster
			if (neighbor.size() >= minPoints)
			{
				Cluster currentCluster;
				KeyPoint pt = keypoints[i];
				currentCluster.add(pt);
				clustered.insert(i);

				//expand cluster
				for (size_t j = 0; j < neighbor.size(); j++)
				{
					//search all reachable points
					if (!visited.count(neighbor[j]))
					{
						visited.insert(neighbor[j]);
						const vector<int>& newNeighbors = listNeighbors(
									keypoints, keypoints[neighbor[j]]);
						if (newNeighbors.size() >= minPoints)
						{
							neighbor.insert(neighbor.end(),
										newNeighbors.begin(),
										newNeighbors.end());
						}
					}

					// add new points to cluster
					if (!clustered.count(neighbor[j]))
					{
						clustered.insert(j);
						KeyPoint p = keypoints[neighbor[j]];
						currentCluster.add(p);
					}
				}

				clusters.push_back(currentCluster);
			}

		}
	}

	return clustersPointer;
}

vector<int> DBSCAN::listNeighbors(const vector<KeyPoint>& keypoints,
			const KeyPoint& keypoint)
{
	float dist;
	vector<int> neighbors;
	for (int i = 0; i < keypoints.size(); i++)
	{
		int dx = keypoint.pt.x - keypoints[i].pt.x;
		int dy = keypoint.pt.y - keypoints[i].pt.y;
		dist = dx * dx + dy * dy;
		if (dist <= maxDistance * maxDistance && dist != 0.0f)
		{
			neighbors.push_back(i);
		}
	}
	return neighbors;
}

