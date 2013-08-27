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

#ifndef CORNERDETECTOR_H_
#define CORNERDETECTOR_H_

#include <opencv2/core/core.hpp>

class CornerDetector
{
public:
	CornerDetector(int threshold, int clusterWindow, int clusterMinSize,
			int noiseBarrier, int objectWindow, int objectMinSize) :
			threshold(threshold), clusterWindow(clusterWindow), clusterMinSize(
					clusterMinSize), noiseBarrier(noiseBarrier), objectWindow(
					objectWindow), objectMinSize(objectMinSize)
	{
	}

	cv::Mat detect(cv::Mat& input);

	//getters and setters
	inline int getThreshold() const
	{
		return threshold;
	}

	inline void setThreshold(int threshold)
	{
		this->threshold = threshold;
	}

	inline int getClusterMinSize() const
	{
		return clusterMinSize;
	}

	inline void setClusterMinSize(int clusterMinSize)
	{
		this->clusterMinSize = clusterMinSize;
	}

	inline int getClusterWindow() const
	{
		return clusterWindow;
	}

	inline void setClusterWindow(int clusterWindow)
	{
		this->clusterWindow = clusterWindow;
	}

	inline int getNoiseBarrier() const
	{
		return noiseBarrier;
	}

	inline void setNoiseBarrier(int noiseBarrier)
	{
		this->noiseBarrier = noiseBarrier;
	}

	inline int getObjectMinSize() const
	{
		return objectMinSize;
	}

	inline void setObjectMinSize(int objectMinSize)
	{
		this->objectMinSize = objectMinSize;
	}

	inline int getObjectWindow() const
	{
		return objectWindow;
	}

	inline void setObjectWindow(int objectWindow)
	{
		this->objectWindow = objectWindow;
	}

private:
	int threshold;
	int clusterWindow;
	int clusterMinSize;
	int noiseBarrier;
	int objectWindow;
	int objectMinSize;
};

#endif /* CORNERDETECTOR_H_ */
