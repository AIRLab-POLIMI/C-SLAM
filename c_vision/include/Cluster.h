/*
 * c_vision,
 *
 *
 * Copyright (C) 2014 Davide Tateo
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

#include "Feature.h"

class Cluster: public Feature
{
public:
	Cluster();

	void add(cv::KeyPoint point);

	void draw(cv::Mat& frame, cv::Scalar color) const;

	inline cv::KeyPoint getMassCenter() const
	{
		return massCenter;
	}

	inline std::vector<cv::KeyPoint> getKeyPoints()
	{
		return keyPoints;
	}

	virtual void setFeature();

private:
	void updateMassCenter(cv::KeyPoint point);
	void updateBoundingBox(cv::KeyPoint point);

private:
	std::vector<cv::KeyPoint> keyPoints;
	cv::KeyPoint massCenter;

	cv::Point start;
	cv::Point end;

};

#endif /* CLUSTER_H_ */
