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

#include "TestPublisher.h"

#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

using namespace std;

TestPublisher::TestPublisher()
{
	trackPublisher = n.advertise<c_slam_msgs::TrackedObject>("/tracks", 6000);
	markersPublisher = n.advertise<visualization_msgs::Marker>(
				"/visualization/features", 1);

	K << 565.59102697808, 0.0, 337.839450567586, //
	0.0, 563.936510489792, 199.522081717361, //
	0.0, 0.0, 1.0;
}

void TestPublisher::publishGroundTruth(Eigen::Matrix4d& H)
{
	tf::Transform transform;

	Eigen::Vector3d t = H.block<3, 1>(0, 3);
	Eigen::Matrix3d R = H.block<3, 3>(0, 0);
	Eigen::Quaterniond q(R);

	tf::Vector3 t_tf;
	tf::Quaternion q_tf;

	tf::vectorEigenToTF(t, t_tf);
	tf::quaternionEigenToTF(q, q_tf);

	transform.setOrigin(t_tf);
	transform.setRotation(q_tf);
	br.sendTransform(
				tf::StampedTransform(transform, ros::Time::now(), "world",
							"camera_ground_truth"));

}

void TestPublisher::publishGroundTruthLandmark()
{
	visualization_msgs::Marker msg;

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "/world";
	msg.type = visualization_msgs::Marker::CUBE_LIST;
	//msg.lifetime = ros::Duration(0.2);
	msg.frame_locked = false;
	msg.ns = "roamfree_markers_gt";
	msg.id = 1;
	msg.action = visualization_msgs::Marker::ADD;

	msg.color.r = 0.0;
	msg.color.g = 1.0;
	msg.color.b = 0.0;
	msg.color.a = 1.0;

	msg.scale.x = 0.10;
	msg.scale.y = 0.10;
	msg.scale.z = 0.40;

	msg.pose.position.x = 0.0;
	msg.pose.position.y = 0.0;
	msg.pose.position.z = 0.0;

	msg.pose.orientation.w = 1.0;

	msg.points.resize(tracksCM.size());

	for (int i = 0; i < tracksCM.size(); i++)
	{
		msg.points[i].x = tracksCM[i](0);
		msg.points[i].y = tracksCM[i](1);
		msg.points[i].z = tracksCM[i](2);
	}

	markersPublisher.publish(msg);
}

void TestPublisher::publishGroundTruthLandmarkPoints()
{
	visualization_msgs::Marker msg;

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "/world";
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	//msg.lifetime = ros::Duration(0.2);
	msg.frame_locked = false;
	msg.ns = "roamfree_markers_gt";
	msg.id = 1;
	msg.action = visualization_msgs::Marker::ADD;

	msg.color.r = 0.0;
	msg.color.g = 1.0;
	msg.color.b = 0.0;
	msg.color.a = 1.0;

	msg.scale.x = 0.10;
	msg.scale.y = 0.10;
	msg.scale.z = 0.10;

	msg.pose.position.x = 0.0;
	msg.pose.position.y = 0.0;
	msg.pose.position.z = 0.0;

	msg.pose.orientation.w = 1.0;

	for (int i = 0; i < tracks.size(); i++)
	{
		for (int j = 0; j < tracks[i].size(); j++)
		{
			geometry_msgs::Point p;

			p.x = tracks[i][j](0);
			p.y = tracks[i][j](1);
			p.z = tracks[i][j](2);

			msg.points.push_back(p);
		}
	}

	markersPublisher.publish(msg);
}

void TestPublisher::publishTracks(const Eigen::Matrix4d& H_WC, double t)
{
	Eigen::Matrix4d H_CW = H_WC.inverse();
	H_CW /= H_CW(3, 3);

	for (int i = 0; i < tracks.size(); i++)
	{
		if (trackVisible(tracks[i], H_CW))
		{
			c_slam_msgs::TrackedObject msg;

			msg.id = i;
			msg.imageStamp.fromSec(t);

			for (int j = 0; j < tracks[i].size(); j++)
			{
				Eigen::Vector3d homogeneusPoint = K * H_CW.topRows(3)
							* tracks[i][j];

				homogeneusPoint /= homogeneusPoint(2);
				geometry_msgs::Point32 point;

				point.x = homogeneusPoint(0);
				point.y = homogeneusPoint(1);

				msg.polygon.points.push_back(point);
			}

			trackPublisher.publish(msg);
		}

	}
}

void TestPublisher::setTracksCircle(double r)
{
	const int numTracks = 8;
	const double w = 0.5;
	const double h = 0.25;

	tracks.resize(numTracks);

	double theta = 0;
	cout << "c_generated = [";
	for (int i = 0; i < numTracks; i++)
	{
		double x = r * cos(theta);
		double y = r * sin(theta);
		double z = 0;

		Eigen::Vector3d trackCM;
		trackCM << x, y, z;
		tracksCM.push_back(trackCM);

		createRotatedRectangle(theta, w, h, x, y, z, tracks[i]);

		theta += 2 * M_PI / numTracks;
	}
	cout << "]" << endl;
}

void TestPublisher::setTracksSquare(double r)
{

	const int numTracks = 16;
	const double w = 0.5;
	const double h = 0.25;

	double cm[][3] =
	{
	{ 2.0, -2.0, -0.3 },
	{ 2.0, -1.0, 0.3 },
	{ 2.0, 0.0, -0.3 },
	{ 2.0, 1.0, 0.3 },
	{ 2.0, 2.0, -0.3 },
	{ -2.0, -2.0, -0.3 },
	{ -2.0, -1.0, 0.3 },
	{ -2.0, 0.0, -0.3 },
	{ -2.0, 1.0, 0.3 },
	{ -2.0, 2.0, -0.3 },
	{ -1.0, 2.0, 0.3 },
	{ -0.0, 2.0, -0.3 },
	{ 1.0, 2.0, 0.3 },
	{ -1.0, -2.0, 0.3 },
	{ 0.0, -2.0, -0.3 },
	{ 1.0, -2.0, 0.3 } };

	tracks.resize(numTracks);

	for (int i = 0; i < numTracks; i++)
	{
		Eigen::Vector3d trackCM;
		trackCM << cm[i][0], cm[i][1], cm[i][2];
		tracksCM.push_back(trackCM);

		double x = cm[i][0];
		double y = cm[i][1];
		double z = cm[i][2];

		double theta = atan2(y, x);
		createRotatedRectangle(theta, w, h, x, y, z, tracks[i]);
	}
}

void TestPublisher::setTracksCarpet()
{
	// generate a carpet of tracks in the area [-2.5 12.5] X [-2.5 7.5]
	double x = -25;
	while (x <= 25.0)
	{
		double y = -25;

		while (y < 25.0)
		{
			Eigen::Vector4d track;
			track << x, y, 0.0, 1.0;

			tracks.resize(tracks.size() + 1);

			for (int k = 0; k < 4; k++)
			{  // four superimposed points
				tracks[tracks.size() - 1].push_back(track);
			}

			Eigen::Vector3d trackCM;
			trackCM = track.head(3);
			tracksCM.push_back(trackCM);

			y += 5.0;
		}

		x += 5.0;
	}

}

bool TestPublisher::pointVisible(Eigen::Vector4d& trackPoint,
			Eigen::Matrix4d H_CW)
{
	Eigen::Vector4d trackRC = H_CW * trackPoint;
	trackRC /= trackRC(3);

	Eigen::Vector3d projection = K * H_CW.topRows(3) * trackPoint;
	projection /= projection(2);

	return trackRC(2) > 0 && projection(0) >= 0 && projection(0) < 640
				&& projection(1) >= 0 && projection(1) < 360;
}

bool TestPublisher::trackVisible(vector<Eigen::Vector4d>& track,
			const Eigen::Matrix4d& H_CW)
{
	for (int i = 0; i < track.size(); i++)
	{
		if (pointVisible(track[i], H_CW))
		{
			return true;
		}
	}

	return false;

}

void TestPublisher::createRotatedRectangle(const double theta, const double w,
			const double h, double x, double y, double z,
			vector<Eigen::Vector4d>& track)
{
	for (int j = 0; j < 4; j++)
	{
		double thetaR = theta + M_PI / 2;

		double xpr = ((j == 1 || j == 2) ? -1 : 1) * w / 2;
		double ypr = 0;
		double zpr = ((j >= 2) ? -1 : 1) * h / 2;

		double xpr2 = xpr * cos(thetaR) - ypr * sin(thetaR);
		double ypr2 = xpr * sin(thetaR) + ypr * cos(thetaR);
		double zpr2 = zpr;

		double xp = x + xpr2;
		double yp = y + ypr2;
		double zp = z + zpr2;

		Eigen::Vector4d trackVertex;
		trackVertex << xp, yp, zp, 1;

		cout << trackVertex(0) << "," << trackVertex(1) << "," << trackVertex(2)
					<< ";";
		cout << endl;
		track.push_back(trackVertex);
	}
}
