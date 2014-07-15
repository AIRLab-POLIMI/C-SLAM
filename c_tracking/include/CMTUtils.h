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
float findMinSymetric(const std::vector<std::vector<float> >& dist,
			const std::vector<bool>& used, int limit, int &i, int &j);
std::vector<Cluster> linkage(const std::vector<cv::Point2f>& list);
void fcluster_rec(std::vector<int>& data, const std::vector<Cluster>& clusters,
			float threshold, const Cluster& currentCluster, int& binId);
std::vector<int> binCount(const std::vector<int>& T);
int argmax(const std::vector<int>& list);
std::vector<int> fcluster(const std::vector<Cluster>& clusters, float threshold);
std::vector<bool> in1d(const std::vector<int>& a, const std::vector<int>& b);

