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

#ifndef PARAMETERSERVER_H_
#define PARAMETERSERVER_H_

#include <ros/ros.h>
#include <opencv2/core/core.hpp>


struct ClusterParam
{
	int threshold;
	int minPoints;
	double maxDistance;
};

struct CannyParam
{
	double alpha;
	int apertureSize;
};

struct HoughParam
{
	int rho;
	double teta;
	int threshold;
	int minLineLenght;
	int maxLineGap;
};

struct ClassifierParam
{
	double threshold;
};

class ParameterServer
{

public:
	ParameterServer(ros::NodeHandle& n);
	void updateParameters();


	inline CannyParam& getCannyParams()
	{
		return canny;
	}

	inline HoughParam& getHoughParams()
	{
		return hough;
	}

	inline ClusterParam& getDBScanParams()
	{
		return cluster;
	}

	inline ClassifierParam& getClassifierParams()
	{
		return classifier;
	}

private:
	ros::NodeHandle& n;

	CannyParam canny;
	HoughParam hough;
	ClusterParam cluster;
	ClassifierParam classifier;

private:

	void getCanny();
	void getHough();
	void getClassifier();
	void getCluster();
};

#endif /* PARAMETERSERVER_H_ */
