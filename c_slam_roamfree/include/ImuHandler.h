/*
 * c_slam_roamfree,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_slam_roamfree.
 *
 * c_slam_roamfree is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_slam_roamfree is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_slam_roamfree.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef IMUHANDLER_H_
#define IMUHANDLER_H_

#include <ROAMestimation/ROAMestimation.h>
#include <ROAMimu/IMUIntegralHandler.h>

namespace roamfree_c_slam
{
class ImuHandler
{
public:
	ImuHandler(ROAMestimation::FactorGraphFilter* filter, bool isAccBiasFixed, bool isGyroBiasFixed);
	void addMeasurement(double za[3], double zw[3], double t);

	inline void setBias(const Eigen::VectorXd& accBias, const Eigen::VectorXd& gyroBias)
	{
		this->accBias = accBias;
		this->gyroBias = gyroBias;
	}

	inline void setSensorframe(const Eigen::VectorXd& T_OS_IMU, const Eigen::VectorXd& x0)
	{
		this->T_OS_IMU = T_OS_IMU;
		this->x0 = x0;
	}

private:
	void initialize(double t);

private:
	ROAMestimation::FactorGraphFilter* filter;
	ROAMimu::IMUIntegralHandler* imu;
	bool initialized;

	bool isAccBiasFixed;
	Eigen::VectorXd accBias;

	bool isGyroBiasFixed;
	Eigen::VectorXd gyroBias;

	Eigen::VectorXd T_OS_IMU;
	Eigen::VectorXd x0;


};

}

#endif /* IMUHANDLER_H_ */
