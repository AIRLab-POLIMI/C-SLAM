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

#ifndef CLUSTER_H_
#define CLUSTER_H_

#include <vector>
#include <opencv2/features2d/features2d.hpp>

class AbstractCluster
{
public:
	inline bool isEmpty()
	{
		return points.size() == 0;
	}
	virtual bool belongTo(cv::KeyPoint* point) = 0;
	virtual void add(cv::KeyPoint* point) = 0;
	virtual ~AbstractCluster()
	{
	}

protected:
	std::vector<cv::KeyPoint*> points;
};

class Cluster: public AbstractCluster
{
public:

	Cluster(int windowSize)
	{
		this->windowSize = windowSize;
		massCenter.pt.x = 0;
		massCenter.pt.y = 0;
		massCenter.size = 0;
	}

	bool belongTo(cv::KeyPoint* point);
	void add(cv::KeyPoint* point);
	cv::KeyPoint getMassCenter();

private:
	int distance(cv::KeyPoint start, cv::KeyPoint end);

protected:
	int windowSize;

private:
	cv::KeyPoint massCenter;

};

class MetaCluster: public Cluster
{
public:

	MetaCluster(int windowSize) :
			Cluster(windowSize)
	{
	}

	std::vector<cv::KeyPoint> getMassCenters();

};

class LineCluster: AbstractCluster
{
public:
	LineCluster(double m, int maxDelta) :
			m(m), q(0), maxDelta(maxDelta)
	{
	}

	bool belongTo(cv::KeyPoint* point);
	void add(cv::KeyPoint* point);

	inline cv::KeyPoint getLineStart()
	{
		return calculateEdge(points.front());
	}

	inline cv::KeyPoint getLineEnd()
	{
		return calculateEdge(points.back());
	}

private:
	cv::KeyPoint calculateEdge(cv::KeyPoint* point);

private:
	double m;
	double q;
	double maxDelta;
};

#endif /* CLUSTER_H_ */
