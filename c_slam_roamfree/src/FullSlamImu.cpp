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

#include "FullSlamImu.h"

#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>

using namespace ROAMestimation;

namespace roamfree_c_slam
{

FullSlamImu::FullSlamImu(std::string imuTopic) :
			filter(NULL), imuHandler(NULL)
{
	//setup roamfree
	initRoamfree();

	//setup the imu handler
	initIMU();

	//subscribe to sensor topics
	imu_sub = n.subscribe(imuTopic, 60000, &FullSlamImu::imuCb, this);

	//advertise markers topic
	markers_pub = n.advertise<visualization_msgs::Marker>(
					"/visualization/features", 1);

}

FullSlamImu::~FullSlamImu()
{
	if (filter)
		delete filter;

	if (imuHandler)
		delete imuHandler;

}

void FullSlamImu::publishPose()
{
	ROAMestimation::PoseVertexWrapper_Ptr cameraPose_ptr =
				filter->getNewestPose();
	const Eigen::VectorXd &camera = cameraPose_ptr->getEstimate();

	tf::Transform T_WR_tf(
				tf::Quaternion(camera(4), camera(5), camera(6), camera(3)),
				tf::Vector3(camera(0), camera(1), camera(2)));

	pose_tf_br.sendTransform(
				tf::StampedTransform(T_WR_tf,
							ros::Time(cameraPose_ptr->getTimestamp()), "world",
							"camera_link"));

}

void FullSlamImu::imuCb(const sensor_msgs::Imu& msg)
{
	//ROS_INFO("imu callback");
	double t = msg.header.stamp.toSec();

	// fill temporaries with measurements
	double za[] =
	{ msg.linear_acceleration.x, msg.linear_acceleration.y,
	msg.linear_acceleration.z };
	double zw[] =
	{ msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z };

	imuHandler->addMeasurement(za, zw, t);

}

void FullSlamImu::initRoamfree()
{
	filter = FactorGraphFilterFactory::getNewFactorGraphFilter();
	filter->setLowLevelLogging(true); // default log folder
	system("mkdir -p /tmp/roamfree/");
	system("rm -f /tmp/roamfree/*.log");
	filter->setDeadReckoning(false);
	filter->setSolverMethod(LevenbergMarquardt);
}

void FullSlamImu::initIMU()
{
	//setup the handlers
	imuHandler = new ImuHandler(filter, false, false);

	/* Firefly initial pose and sensor pose
	 Eigen::VectorXd T_OS_IMU(7), x0(7);
	 T_OS_IMU << 0.0, 0.0, 0.0, 0.5, 0.5, -0.5, 0.5;
	 x0 << 0.0, 0.0, 0.08, 0.5, -0.5, 0.5, -0.5;
	 imuHandler->setSensorframe(T_OS_IMU, x0);

	 //Firefly bias
	 Eigen::VectorXd accBias(3);
	 accBias << 0.1939, 0.0921, -0.2989;
	 Eigen::VectorXd gyroBias(3);
	 gyroBias << -0.0199, 0.0077, -0.0099;
	 imuHandler->setBias(accBias, gyroBias);
	 //*/
}

}
