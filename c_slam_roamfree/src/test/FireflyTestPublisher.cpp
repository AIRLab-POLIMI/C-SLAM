/*
 * c_slam_roamfree,
 *
 *
 * Copyright (C) 2015 Davide Tateo
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

#include "test/FireflyTestPublisher.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace roamfree_c_slam
{

FireflyTestPublisher::FireflyTestPublisher()
{
	gtSubscriber = n.subscribe("/firefly/ground_truth/pose", 1000,
				&FireflyTestPublisher::handleGTdata, this);
}

void FireflyTestPublisher::handleGTdata(
			const geometry_msgs::PoseWithCovarianceStamped& pose_msg)
{
	//Get the world-robot transform
	Eigen::Vector3d t_wr(pose_msg.pose.pose.position.x,
				pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z);
	Eigen::Quaterniond q_wr(pose_msg.pose.pose.orientation.w,
				pose_msg.pose.pose.orientation.x,
				pose_msg.pose.pose.orientation.y,
				pose_msg.pose.pose.orientation.z);

	Eigen::Matrix4d H_WR;

	H_WR.block<3, 1>(0, 3) = t_wr;
	H_WR.block<3, 3>(0, 0) = q_wr.toRotationMatrix();
	H_WR.row(3) << 0, 0, 0, 1;

	//Get the robot-camera transform
	Eigen::Vector3d t_rc(0.0, 0.0, 0.0);
	Eigen::Quaterniond q_rc(0.5, -0.5, 0.5, -0.5);

	Eigen::Matrix4d H_RC;

	H_RC.block<3, 1>(0, 3) = t_rc;
	H_RC.block<3, 3>(0, 0) = q_rc.toRotationMatrix();
	H_RC.row(3) << 0, 0, 0, 1;

	//Get the world-camera transform
	Eigen::Matrix4d H_WC = H_WR * H_RC;

	//Publish results
	publishTracks(H_WC, pose_msg.header.stamp.toSec());
	publishGroundTruth(H_WC);

	publishGroundTruthLandmarkName();
	publishGroundTruthLandmarkPoints();
	publishGroundTruthLandmarkPointsName();
}

}
