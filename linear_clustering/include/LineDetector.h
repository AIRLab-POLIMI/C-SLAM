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

#ifndef LINEDETECTOR_H_
#define LINEDETECTOR_H_

#include <vector>
#include <opencv2/features2d/features2d.hpp>
#include "Cluster.h"

struct Line
{
	cv::KeyPoint start, end;
};

class LineDetector
{

public:
	LineDetector(double maxDelta) :
			maxDelta(maxDelta)
	{
	}
	void detect(std::vector<cv::KeyPoint> input, double m);
	void detect(std::vector<cv::KeyPoint> points);
	std::vector<Line> getLines();

private:
	std::vector<AbstractLineCluster*> clusters;
	double maxDelta;

private:
	class Comparator
	{
	public:

		Comparator(double m) :
				m(m)
		{
		}

		bool operator()(cv::KeyPoint i, cv::KeyPoint j)
		{
			int yi = i.pt.y;
			int xi = i.pt.x;
			double qi = xi - m * yi;

			int yj = j.pt.y;
			int xj = j.pt.x;
			double qj = xj - m * yj;

			return qi < qj;
		}

	private:
		double m;
	};

	class ComparatorY
	{
	public:

		bool operator()(cv::KeyPoint i, cv::KeyPoint j)
		{
			return i.pt.y < j.pt.y;
		}
	};

};

#endif /* LINEDETECTOR_H_ */
