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

#include <vector>
#include <opencv2/opencv.hpp>

typedef std::pair<int, int> PairInt;
typedef std::pair<float, int> PairFloat;

struct Cluster
{
	int first, second; //cluster id
	float dist;
	int num;
};


template<typename T>
bool comparatorPair(const std::pair<T, int>& l, const std::pair<T, int>& r)
{
	return l.first < r.first;
}

template<typename T>
bool comparatorPairDesc(const std::pair<T, int>& l, const std::pair<T, int>& r)
{
	return l.first > r.first;
}

template<typename T>
T median(std::vector<T> list)
{
	T val;
	std::nth_element(&list[0], &list[0] + list.size() / 2,
				&list[0] + list.size());
	val = list[list.size() / 2];
	if (list.size() % 2 == 0)
	{
		std::nth_element(&list[0], &list[0] + list.size() / 2 - 1,
					&list[0] + list.size());
		val = (val + list[list.size() / 2 - 1]) / 2;
	}
	return val;
}

cv::Point2f rotate(cv::Point2f p, float rad);
std::vector<Cluster> linkage(const std::vector<cv::Point2f>& list);
std::vector<int> binCount(const std::vector<int>& T);
int argmax(const std::vector<int>& list);
std::vector<int> fcluster(const std::vector<Cluster>& clusters, float threshold);
std::vector<bool> in1d(std::vector<int>& a, std::vector<int>& b);

