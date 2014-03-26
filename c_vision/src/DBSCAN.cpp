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

void ObjectCluster::add(KeyPoint point)
{
	updateMassCenter(point);
	updateBoundingBox(point);

	keyPoints.push_back(point);
}

void ObjectCluster::updateMassCenter(KeyPoint point)
{
	int y = point.pt.y;
	int x = point.pt.x;
	float size = point.size;

	massCenter.pt.x += x * size;
	massCenter.pt.y += y * size;

	massCenter.size += size;
}

void ObjectCluster::updateBoundingBox(KeyPoint point)
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

vector<ObjectCluster> DBSCAN::detect(vector<KeyPoint> &keypoints)
{
	vector<ObjectCluster> clusters;
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
				ObjectCluster currentCluster;
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
						vector<int> newNeighbors = listNeighbors(keypoints,
									keypoints[neighbor[j]]);
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
	return clusters;
}

vector<int> DBSCAN::listNeighbors(vector<KeyPoint> &keypoints,
			KeyPoint& keypoint)
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

