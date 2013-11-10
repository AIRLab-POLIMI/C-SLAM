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

#ifndef DBSCAN_H_
#define DBSCAN_H_

#include <vector>
#include <opencv2/features2d/features2d.hpp>


class ObjectCluster
{
public:
	ObjectCluster()
	{
		massCenter.pt.x = 0;
		massCenter.pt.y = 0;
		massCenter.size = 0;
	}

	void add(cv::KeyPoint point);

	inline void draw(cv::Mat& frame, cv::Scalar color)
	{
		cv::rectangle(frame, start, end, color, 2);
	}

	inline cv::KeyPoint getMassCenter()
	{
		massCenter.pt.x /= massCenter.size;
		massCenter.pt.y /= massCenter.size;

		return massCenter;
	}

	inline std::vector<cv::KeyPoint> getKeyPoints()
	{
		return keyPoints;
	}

private:
	void updateMassCenter(cv::KeyPoint point);
	void updateBoundingBox(cv::KeyPoint point);

private:
	std::vector<cv::KeyPoint> keyPoints;
	cv::KeyPoint massCenter;

	cv::Point start;
	cv::Point end;

};

class DBSCAN
{
public:
	DBSCAN(double eps, int minPts) :
			maxDistance(eps), minPoints(minPts)
	{
	}
	std::vector<ObjectCluster> detect(std::vector<cv::KeyPoint> &keypoints);

	inline double getMaxDistance() const
	{
		return maxDistance;
	}

	inline void setMaxDistance(double eps)
	{
		this->maxDistance = eps;
	}

	inline int getMinPoints() const
	{
		return minPoints;
	}

	inline void setMinPoints(int minPts)
	{
		this->minPoints = minPts;
	}

private:
	std::vector<int> listNeighbors(std::vector<cv::KeyPoint> &keypoints,
			cv::KeyPoint &keypoint);

private:
	double maxDistance;
	int minPoints;

};

#endif /* DBSCAN_H_ */
