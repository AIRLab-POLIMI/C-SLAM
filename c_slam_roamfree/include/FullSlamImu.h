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

#ifndef FULLSLAMIMU_H_
#define FULLSLAMIMU_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <string>

#include "ImuHandler.h"

namespace roamfree_c_slam
{

class FullSlamImu
{
public:
	FullSlamImu(std::string imuTopic);
	virtual ~FullSlamImu();
	virtual void run() = 0;


protected:
	void imuCb(const sensor_msgs::Imu &msg);
	void publishPose();

private:
	void initIMU();
	void initRoamfree();

protected:
	const int iterationN = 1;

protected:
	ros::NodeHandle n;
	ros::Publisher markers_pub;

	ROAMestimation::FactorGraphFilter* filter;
	ImuHandler* imuHandler;

private:
	ros::Subscriber imu_sub;

	tf::Transform T_OC_tf; // transformation from robot to camera;
	tf::TransformBroadcaster pose_tf_br;


};

}


#endif /* FULLSLAMIMU_H_ */
