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

#ifndef TESTPUBLISHER_H_
#define TESTPUBLISHER_H_

#include <vector>
#include <sstream>

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <c_slam_msgs/TrackedObject.h>

#include <tf/transform_broadcaster.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace roamfree_c_slam
{
class TestPublisher
{
public:

	TestPublisher();

	void publishTracks(const Eigen::Matrix4d& H_WC, double t);

	void publishGroundTruth(Eigen::Matrix4d& H);
	void publishGroundTruthLandmark();
	void publishGroundTruthLandmarkName();
	void publishGroundTruthLandmarkPoints();
	void publishGroundTruthLandmarkPointsName();

public:
	void setTracksCircle(double r);
	void setTracksSquare(double r);
	void setTracksCarpet();
	void setTrackRoom();

private:
	void createRotatedRectangle(const double theta, const double w, const double h,
				double x, double y, double z, std::vector<Eigen::Vector4d>& track);

private:
	bool pointVisible(Eigen::Vector4d& trackPoint, Eigen::Matrix4d H_CW);

	bool trackVisible(std::vector<Eigen::Vector4d>& track,
				const Eigen::Matrix4d& H_CW);

protected:
	ros::NodeHandle n;
	ros::Publisher trackPublisher;
	ros::Publisher markersPublisher;
	tf::TransformBroadcaster br;

	//tracks
	std::vector<std::vector<Eigen::Vector4d> > tracks;
	std::vector<Eigen::Vector3d> tracksCM;

	Eigen::Matrix3d K;
};

}

#endif /* TESTPUBLISHER_H_ */
