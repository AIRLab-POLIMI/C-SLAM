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

#include <dynamic_reconfigure/server.h>
#include <c_vision/ParametersConfig.h>

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
	int low;
	int high;
	int blur;
};

struct HoughParam
{
	int rho;
	double teta;
	int threshold;
	int minLineLenght;
	int maxLineGap;
};

struct LFilterParam
{
	double maxDeltaVertical;
	double maxDeltaHorizontal;
};

struct ClassifierParam
{
	double threshold;
};

class ParameterServer
{

public:
	ParameterServer();

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

	inline LFilterParam& getLFiltrerParams()
	{
		return lineFilter;
	}

private:
	void update(c_vision::ParametersConfig &config, uint32_t level);

private:
	//Dynamic_reconfigure server
	dynamic_reconfigure::Server<c_vision::ParametersConfig> server;

	//Parameters
	CannyParam canny;
	HoughParam hough;
	ClusterParam cluster;
	ClassifierParam classifier;
	LFilterParam lineFilter;

private:

	void getCanny();
	void getHough();
	void getClassifier();
	void getCluster();
	void getLFilter();
};

#endif /* PARAMETERSERVER_H_ */
