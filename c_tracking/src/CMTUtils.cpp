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
 *
 *  based on https://github.com/delmottea/libCMT/
 */

#include "CMTUtils.h"

#include <algorithm>

using namespace std;
using namespace cv;

Point2f rotate(Point2f p, float rad)
{
	if (rad == 0)
		return p;
	float s = sin(rad);
	float c = cos(rad);
	return Point2f(c * p.x - s * p.y, s * p.x + c * p.y);
}

float findMinSymetric(const vector<vector<float> >& dist,
			const vector<bool>& used, int limit, int &i, int &j)
{
	float min = dist[0][0];
	i = 0;
	j = 0;
	for (int x = 0; x < limit; x++)
	{
		if (!used[x])
		{
			for (int y = x + 1; y < limit; y++)
				if (!used[y] && dist[x][y] <= min)
				{
					min = dist[x][y];
					i = x;
					j = y;
				}
		}
	}
	return min;
}

vector<Cluster> linkage(const vector<Point2f>& list)
{
	float inf = 10000000;
	0;
	vector<bool> used;
	for (int i = 0; i < 2 * list.size(); i++)
		used.push_back(false);
	vector<vector<float> > dist;
	for (int i = 0; i < list.size(); i++)
	{
		vector<float> line;
		for (int j = 0; j < list.size(); j++)
		{
			if (i == j)
				line.push_back(inf);
			else
			{
				Point2f p = list[i] - list[j];
				line.push_back(sqrt(p.dot(p)));
			}
		}
		for (int j = 0; j < list.size(); j++)
			line.push_back(inf);
		dist.push_back(line);
	}
	for (int i = 0; i < list.size(); i++)
	{
		vector<float> line;
		for (int j = 0; j < 2 * list.size(); j++)
			line.push_back(inf);
		dist.push_back(line);
	}
	vector<Cluster> clusters;
	while (clusters.size() < list.size() - 1)
	{
		int x, y;
		float min = findMinSymetric(dist, used, list.size() + clusters.size(),
					x, y);
		Cluster cluster;
		cluster.first = x;
		cluster.second = y;
		cluster.dist = min;
		cluster.num = (x < list.size() ? 1 : clusters[x - list.size()].num)
					+ (y < list.size() ? 1 : clusters[y - list.size()].num);
		used[x] = true;
		used[y] = true;
		int limit = list.size() + clusters.size();
		for (int i = 0; i < limit; i++)
		{
			if (!used[i])
				dist[i][limit] = dist[limit][i] = std::min(dist[i][x],
							dist[i][y]);
		}
		clusters.push_back(cluster);
	}
	return clusters;
}

void fcluster_rec(vector<int>& data, const vector<Cluster>& clusters,
			float threshold, const Cluster& currentCluster, int& binId)
{
	int startBin = binId;
	if (currentCluster.first >= data.size())
		fcluster_rec(data, clusters, threshold,
					clusters[currentCluster.first - data.size()], binId);
	else
		data[currentCluster.first] = binId;

	if (startBin == binId && currentCluster.dist >= threshold)
		binId++;
	startBin = binId;

	if (currentCluster.second >= data.size())
		fcluster_rec(data, clusters, threshold,
					clusters[currentCluster.second - data.size()], binId);
	else
		data[currentCluster.second] = binId;

	if (startBin == binId && currentCluster.dist >= threshold)
		binId++;
}

vector<int> binCount(const vector<int>& T)
{
	vector<int> result;
	for (int i = 0; i < T.size(); i++)
	{
		while (T[i] >= result.size())
			result.push_back(0);
		result[T[i]]++;
	}
	return result;
}

int argmax(const vector<int>& list)
{
	int max = list[0];
	int id = 0;
	for (int i = 1; i < list.size(); i++)
		if (list[i] > max)
		{
			max = list[i];
			id = i;
		}
	return id;
}

vector<int> fcluster(const vector<Cluster>& clusters, float threshold)
{
	vector<int> data;
	for (int i = 0; i < clusters.size() + 1; i++)
		data.push_back(0);
	int binId = 0;
	fcluster_rec(data, clusters, threshold, clusters[clusters.size() - 1],
				binId);
	return data;
}


vector<bool> in1d(vector<int>& a, vector<int>& b)
{
	sort(b.begin(), b.end());
	vector<bool> result;
	for (int i = 0; i < a.size(); i++)
	{
		bool found = binary_search(b.begin(), b.end(), a[i]);
		result.push_back(found);
	}

	return result;
}
