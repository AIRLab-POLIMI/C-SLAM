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

#include <vector>
#include <sstream>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <c_slam_msgs/TrackedObject.h>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include "TestPublisher.h"

using namespace std;

class FireflyTestPublisher: public TestPublisher
{
public:

	FireflyTestPublisher()
	{
		gtSubscriber = n.subscribe("/firefly/ground_truth/pose", 1000,
					&FireflyTestPublisher::handleGTdata, this);
	}

	void handleGTdata(const geometry_msgs::PoseWithCovarianceStamped& pose_msg)
	{
		//Get the world-robot transform
		Eigen::Vector3d t_wr(pose_msg.pose.pose.position.x,
					pose_msg.pose.pose.position.y,
					pose_msg.pose.pose.position.z);
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
		Eigen::Quaterniond q_rc( 0.5, -0.5, 0.5, -0.5);

		Eigen::Matrix4d H_RC;

		H_RC.block<3, 1>(0, 3) = t_rc;
		H_RC.block<3, 3>(0, 0) = q_rc.toRotationMatrix();
		H_RC.row(3) << 0, 0, 0, 1;

		//Get the world-camera transform
		Eigen::Matrix4d H_WC = H_WR * H_RC;

		//Publish results
		publishTracks(H_WC, pose_msg.header.stamp.toSec());
		publishGroundTruth(H_WC);
		publishGroundTruthLandmark();
	}

private:
	ros::Subscriber gtSubscriber;
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "circular_test");

	FireflyTestPublisher publisher;

	publisher.setTracksCarpet();

	ros::spin();

	return 0;
}
