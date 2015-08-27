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
#include <Eigen/Dense>

struct CannyParam
{
	bool automatic;
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

struct QDetectorParam
{
	double verticalOverlap;
	double horizontalOverlap;
	double polesFormFactor;
	double segmentSupport;

	//ROMANONI params
	int points;
	double threshold;
	double maxDistance;

	Eigen::Matrix3d K;
	Eigen::Matrix3d omega;
};

struct CornerClassParam
{
	int kernelSize;
	int bucketWidth;
};

struct ClusterParam
{
	int threshold;
	int minPoints;
	double maxDistance;
};

struct ClassifierParam
{
	double threshold;
};

struct DisplayParam
{
	int currentObject;
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

	inline LFilterParam& getLFiltrerParams()
	{
		return lineFilter;
	}

	inline QDetectorParam& getQuadDetectorParams()
	{
		return quadDetector;
	}

	inline ClusterParam& getDBScanParams()
	{
		return cluster;
	}

	inline ClassifierParam& getClassifierParams()
	{
		return classifier;
	}

	inline DisplayParam& getDisplayParams()
	{
		return display;
	}

	inline CornerClassParam& getCornerClassParams()
	{
		return cornerClass;
	}

	inline std::string getCameraSource()
	{
		return camera_source;
	}

	inline std::string getImuSource()
	{
		return imu_source;
	}

private:
	void update(c_vision::ParametersConfig &config, uint32_t level);

private:
	//Dynamic_reconfigure server
	dynamic_reconfigure::Server<c_vision::ParametersConfig> server;

	//Parameters
	CannyParam canny;
	HoughParam hough;
	LFilterParam lineFilter;
	QDetectorParam quadDetector;
	CornerClassParam cornerClass;
	ClusterParam cluster;
	ClassifierParam classifier;
	DisplayParam display;

	std::string camera_source;
	std::string imu_source;

};

#endif /* PARAMETERSERVER_H_ */
