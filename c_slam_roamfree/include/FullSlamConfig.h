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

#ifndef FULLSLAMCONFIG_H_
#define FULLSLAMCONFIG_H_

#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include <ros/ros.h>

namespace roamfree_c_slam
{
class FullSlamConfig
{
public:
	FullSlamConfig();

	//Ros stuff
	std::string trackedFrame;
	std::string imuTopic;

	//Initial Pose
	Eigen::VectorXd x0;

	//Sensor relative pose
	Eigen::VectorXd T_O_IMU;
	Eigen::VectorXd T_O_CAMERA;

	//Imu bias
	bool isAccBiasFixed;
	Eigen::VectorXd accBias;
	bool isGyroBiasFixed;
	Eigen::VectorXd gyroBias;

	//Imu data
	int imu_N;
	double imu_dt;


	//Camera calibration
	Eigen::VectorXd K;

	//Vision Stuff
	double initialScale;
	double minWindowLenghtSecond;
	int minActiveFeatures;

	//Optimization method
	bool useGaussNewton;
	int iterationN;

private:
	void initROS(const ros::NodeHandle& n);
	void initPose(const ros::NodeHandle& n);
	void initRoamfree(const ros::NodeHandle& n);
	void initIMU(const ros::NodeHandle& n);
	void initCamera(const ros::NodeHandle& n);
};

}


#endif /* FULLSLAMCONFIG_H_ */
