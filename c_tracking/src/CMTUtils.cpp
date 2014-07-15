/*
 * Copyright (c) 2014, delmottea
 * Copyright (c) 2014, Davide Tateo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the {organization} nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "CMTUtils.h"

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

//TODO fare in n*log(n)
vector<bool> in1d(const vector<int>& a, const vector<int>& b)
{
	vector<bool> result;
	for (int i = 0; i < a.size(); i++)
	{
		bool found = false;
		for (int j = 0; j < b.size(); j++)
			if (a[i] == b[j])
			{
				found = true;
				break;
			}
		result.push_back(found);
	}
	return result;
}
