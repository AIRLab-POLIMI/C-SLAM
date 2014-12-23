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

#include "ImuHandler.h"

#include <ros/ros.h>

#include <iostream>

using namespace ROAMestimation;
namespace roamfree_c_slam
{

ImuHandler::ImuHandler(FactorGraphFilter* filter, bool isAccBiasFixed,
			bool isGyroBiasFixed) :
			filter(filter), isAccBiasFixed(isAccBiasFixed), accBias(3),
			isGyroBiasFixed(isGyroBiasFixed), gyroBias(3), T_OS_IMU(7), x0(7)
{

	//get parameters

	double imu_N, imu_dt;

	ros::NodeHandle _node("~");

	if (!_node.getParam("IMU_N_integration_steps", imu_N))
	{
		ROS_FATAL("parameter IMU_N_integration_steps undefined");
	}

	if (!_node.getParam("IMU_nominal_period", imu_dt))
	{
		ROS_FATAL("parameter IMU_nominal_period undefined");
	}

	initialized = false;

	//imu centric by default
	T_OS_IMU <<  0.0, 0.0, 0.0, 0.5, 0.5, -0.5, 0.5;
	x0 << 0.0, -1.0, 0.0, 0.5, -0.5, 0.5, -0.5;

	//zero bias by default
	accBias << 0.0, 0.0, 0.0;
	gyroBias << 0.0, 0.0, 0.0;

	//setup roamfree imu integral handler
	imu = new ROAMimu::IMUIntegralHandler(imu_N, imu_dt); // instantiate the new handler

	imu->getSensorNoises() = Eigen::Matrix<double, 6, 6>::Identity(); // init the sensor noises
}

void ImuHandler::addMeasurement(double za[3], double zw[3], double t)
{
	if (!initialized)
		initialize(t);

	imu->step(za, zw);

}

void ImuHandler::initialize(double t)
{
	imu->init(filter, "IMUintegral", T_OS_IMU, accBias, isAccBiasFixed,
				gyroBias, isGyroBiasFixed, x0, t);

	initialized = true;
}

}
