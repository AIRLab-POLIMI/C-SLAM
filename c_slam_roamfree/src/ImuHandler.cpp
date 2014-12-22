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

ImuHandler::ImuHandler(FactorGraphFilter* filter) :
		filter(filter) {

	// -- get parameters

	double imu_N, imu_dt;

	ros::NodeHandle _node("~");

	if (!_node.getParam("IMU_N_integration_steps", imu_N)) {
		ROS_FATAL("parameter IMU_N_integration_steps undefined");
	}

	if (!_node.getParam("IMU_nominal_period", imu_dt)) {
		ROS_FATAL("parameter IMU_nominal_period undefined");
	}

	initialized = false;

	imu = new ROAMimu::IMUIntegralHandler(imu_N, imu_dt); // instantiate the new handler

	imu->getSensorNoises() = Eigen::Matrix<double, 6, 6>::Identity(); // init the sensor noises
	//imu->setPredictorEnabled(false);
}

void ImuHandler::addMeasurement(double za[3], double zw[3], double t) {
	if (!initialized)
		initialize(t);

	if (imu->step(za, zw))
		computePose(t);

}

void ImuHandler::computePose(double t) {
	//Pose data
	double r = 1; // meters
	double alpha = 0.01; // radians / s^2
	double w0 = 0.0; //initial angular speed
	double theta0 = 0;
	double imuRate = 50;

	//Pose computation
	double thetaRobot = theta0 + w0 * t + 0.5 * alpha * std::pow(t, 2);
	double theta = thetaRobot - M_PI / 2;

	double x = r * cos(theta);
	double y = r * sin(theta);
	double z = 0;

	Eigen::Matrix3d R;

	R << cos(thetaRobot), -sin(thetaRobot), 0, //
	sin(thetaRobot), cos(thetaRobot), 0, //
	0, 0, 1;

	Eigen::Quaterniond q(R);

	Eigen::VectorXd pose(7);
	pose << x, y, z, q.w(), q.x(), q.y(), q.z();

	//filter->getNewestPose()->setEstimate(pose);
	//filter->getNewestPose()->setFixed(true);

}

void ImuHandler::initialize(double t) {
	// we have to initialize the IMUIntegralHandler

	// zero bias
	Eigen::VectorXd accBias(3);  // Accelerometer and Gyroscope biases
	accBias << 0.0, 0.0, 0.0;
	Eigen::VectorXd gyroBias(3);
	gyroBias << 0.0, 0.0, 0.0;
	//*/

	/* firefly.bag biases
	Eigen::VectorXd accBias(3);  // Accelerometer and Gyroscope biases
	accBias << 0.1939, 0.0921, -0.2989;
	Eigen::VectorXd gyroBias(3);
	gyroBias << -0.0199, 0.0077, -0.0099;
	//*/

	Eigen::VectorXd x0(7); // initial robot position
	Eigen::VectorXd T_OS_IMU(7); // Transformation between Odometer and robot frame

	/* imu centric
	 T_OS_IMU << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
	 x0 << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0;
	 //*/

	// camera centric
	T_OS_IMU << 0.0, 0.0, 0.0, 0.5, 0.5, -0.5, 0.5;
	x0 << 0.0, -1.0, 0.0, 0.5, -0.5, 0.5, -0.5;

	//*/

	imu->init(filter, "IMUintegral", T_OS_IMU, accBias, false, gyroBias, false,
			x0, t);

	initialized = true;
}
