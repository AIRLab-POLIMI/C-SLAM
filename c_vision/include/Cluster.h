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

class Cluster
{
public:

	Cluster()
	{
		start = 0;
		massCenter.pt.x = 0;
		massCenter.pt.y = 0;
		massCenter.size = 0;
	}

	bool isEmpty();
	bool pointsBelongsTo(cv::KeyPoint* point, int windowSize);
	void addToCluster(cv::KeyPoint* point, int windowSize);
	cv::KeyPoint getMassCenter();

private:
	int start;
	std::vector<cv::KeyPoint*> points;
	cv::KeyPoint massCenter;

};

#endif /* CLUSTER_H_ */
