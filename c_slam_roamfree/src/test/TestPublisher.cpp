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

#include "test/TestPublisher.h"
#include "test/RectangleGenerator.h"

#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

using namespace std;

namespace roamfree_c_slam
{

TestPublisher::TestPublisher()
{
	trackPublisher = n.advertise<c_slam_msgs::TrackedObject>("/tracks", 6000);
	rectanglePublisher = n.advertise<c_slam_msgs::NamedPolygon>("/objects", 6000);
	markersPublisher = n.advertise<visualization_msgs::Marker>(
				"/visualization/features", 1);

	K << 460.890258789062, 0.0, 343.462058902507,
	/* */0.0, 529.505432128906, 201.160554159542,
	/* */0.0, 0.0, 1.0;
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
	msg.ns = "c_slam_gt";
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

void TestPublisher::publishGroundTruthLandmarkName()
{
	int id = 0;
	for (int i = 0; i < tracksCM.size(); i++)
	{

		visualization_msgs::Marker msg;

		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "/world";
		msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		//msg.lifetime = ros::Duration(0.2);
		msg.frame_locked = false;
		msg.ns = "c_slam_gt_name";
		msg.id = id++;
		msg.action = visualization_msgs::Marker::ADD;

		msg.color.r = 1.0;
		msg.color.g = 1.0;
		msg.color.b = 0.0;
		msg.color.a = 1.0;

		msg.scale.z = 0.10;

		msg.pose.position.x = tracksCM[i](0);
		msg.pose.position.y = tracksCM[i](1);
		msg.pose.position.z = tracksCM[i](2);

		msg.text = "Landmark_" + std::to_string(i);

		markersPublisher.publish(msg);

	}
}

void TestPublisher::publishGroundTruthLandmarkPoints()
{
	visualization_msgs::Marker msg;

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "/world";
	msg.type = visualization_msgs::Marker::SPHERE_LIST;
	//msg.lifetime = ros::Duration(0.2);
	msg.frame_locked = false;
	msg.ns = "c_slam_gt";
	msg.id = 2;
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

void TestPublisher::publishGroundTruthLandmarkPointsName()
{
	int id = 0;
	for (int i = 0; i < tracks.size(); i++)
	{
		for (int j = 0; j < tracks[i].size(); j++)
		{
			visualization_msgs::Marker msg;

			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "/world";
			msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			//msg.lifetime = ros::Duration(0.2);
			msg.frame_locked = false;
			msg.ns = "c_slam_gt_points_name";
			msg.id = id++;
			msg.action = visualization_msgs::Marker::ADD;

			msg.color.r = 1.0;
			msg.color.g = 1.0;
			msg.color.b = 0.0;
			msg.color.a = 1.0;

			msg.scale.z = 0.10;

			msg.pose.position.x = tracks[i][j](0);
			msg.pose.position.y = tracks[i][j](1);
			msg.pose.position.z = tracks[i][j](2) + 0.1;

			msg.text = "M" + std::to_string(j + 1);

			markersPublisher.publish(msg);
		}
	}

}

void TestPublisher::publishTracks(const Eigen::Matrix4d& H_WC, double t)
{
	Eigen::Matrix4d H_CW = H_WC.inverse();
	H_CW /= H_CW(3, 3);

	for (int i = 0; i < tracks.size(); i++)
	{
		bool isRectangleVisible = rectangleVisible(tracks[i], H_CW);

		if (isRectangleVisible || trackVisible(tracks[i], H_CW))
		{
			c_slam_msgs::TrackedObject msg;
			c_slam_msgs::NamedPolygon msgRect;

			msg.id = i;
			msg.imageStamp.fromSec(t);
			msgRect.id = i;
			msgRect.header.stamp.fromSec(t);

			for (int j = 0; j < tracks[i].size(); j++)
			{
				Eigen::Vector3d homogeneusPoint = K * H_CW.topRows(3)
							* tracks[i][j];

				homogeneusPoint /= homogeneusPoint(2);
				geometry_msgs::Point32 point;

				point.x = homogeneusPoint(0);
				point.y = homogeneusPoint(1);

				msg.polygon.points.push_back(point);
				msgRect.polygon.points.push_back(point);


			}

			trackPublisher.publish(msg);

			if(isRectangleVisible)
				rectanglePublisher.publish(msgRect);
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
	for (int i = 0; i < numTracks; i++)
	{
		double x = r * cos(theta);
		double y = r * sin(theta);
		double z = 0;

		RectangleGenerator gen(w, h);
		gen.setPosition(x, y, z);
		gen.setRPY(0, 0, theta + M_PI / 2);
		gen.generateTrack(tracks, tracksCM);

		theta += 2 * M_PI / numTracks;
	}
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

	for (int i = 0; i < numTracks; i++)
	{
		double x = cm[i][0];
		double y = cm[i][1];
		double z = cm[i][2];

		double theta = atan2(y, x);

		RectangleGenerator gen(w, h);
		gen.setPosition(x, y, z);
		gen.setRPY(0, 0, theta + M_PI / 2);
		gen.generateTrack(tracks, tracksCM);
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
			RectangleGenerator gen(0, 0);
			gen.setPosition(x, y, 0);
			gen.setRPY(0, M_PI / 2, 0);
			gen.generateTrack(tracks, tracksCM);

			y += 5.0;
		}

		x += 5.0;
	}

}

void TestPublisher::setTrackRoom()
{
	//Doors
	RectangleGenerator door1(0.8, 2.2);
	door1.setPosition(8.6, 0, 1.1);
	door1.setRPY(0, 0, M_PI / 2);
	door1.generateTrack(tracks, tracksCM);

	RectangleGenerator door2(0.8, 2.2);
	door2.setPosition(0, -4, 1.1);
	door2.setRPY(0, 0, 0);
	door2.generateTrack(tracks, tracksCM);

	RectangleGenerator door3(0.8, 2.2);
	door3.setPosition(-3, 8, 1.1);
	door3.setRPY(0, 0, - 3* M_PI / 4);
	door3.generateTrack(tracks, tracksCM);

	//Whiteboards
	RectangleGenerator whiteboard1(1.2, 0.8);
	whiteboard1.setPosition(8.6, -1.8, 1.4);
	whiteboard1.setRPY(0, 0, M_PI / 2);
	whiteboard1.generateTrack(tracks, tracksCM);

	RectangleGenerator whiteboard2(1.3, 0.75);
	whiteboard2.setPosition(3.6, 12.3, 1.4);
	whiteboard2.setRPY(0, 0, -M_PI);
	whiteboard2.generateTrack(tracks, tracksCM);

	//CabinetDoors
	RectangleGenerator cabinetDoor1(1, 2.0);
	cabinetDoor1.setPosition(8.0, 1.5, 1);
	cabinetDoor1.setRPY(0, 0, M_PI / 2);
	cabinetDoor1.generateTrack(tracks, tracksCM);

	RectangleGenerator cabinetDoor2(1, 2.0);
	cabinetDoor2.setPosition(8.0, 2.55, 1);
	cabinetDoor2.setRPY(0, 0, M_PI / 2);
	cabinetDoor2.generateTrack(tracks, tracksCM);

	RectangleGenerator cabinetDoor3(1, 2.0);
	cabinetDoor3.setPosition(8.0, 3.7, 1);
	cabinetDoor3.setRPY(0, 0, M_PI / 2);
	cabinetDoor3.generateTrack(tracks, tracksCM);

	RectangleGenerator cabinetDoor4(1, 2.0);
	cabinetDoor4.setPosition(8.0, 4.75, 1);
	cabinetDoor4.setRPY(0, 0, M_PI / 2);
	cabinetDoor4.generateTrack(tracks, tracksCM);

	RectangleGenerator cabinetDoor5(1, 2.0);
	cabinetDoor5.setPosition(6, 12.3, 1);
	cabinetDoor5.setRPY(0, 0, -M_PI);
	cabinetDoor5.generateTrack(tracks, tracksCM);

	RectangleGenerator cabinetDoor6(1, 2.0);
	cabinetDoor6.setPosition(7.05, 12.3, 1);
	cabinetDoor6.setRPY(0, 0, -M_PI);
	cabinetDoor6.generateTrack(tracks, tracksCM);

	//Cabinet
	RectangleGenerator cabinet1(2.5, 1.9);
	cabinet1.setPosition(8.0, 8.63, 1);
	cabinet1.setRPY(0, 0, M_PI / 2);
	cabinet1.generateTrack(tracks, tracksCM);

	RectangleGenerator cabinet2(2.6, 1.96);
	cabinet2.setPosition(6.83, -3.6, 1);
	cabinet2.setRPY(0, 0, 0);
	cabinet2.generateTrack(tracks, tracksCM);

	//Windows
	RectangleGenerator window1(0.6, 1.2);
	window1.setPosition(4, -4, 1.4);
	window1.setRPY(0, 0, 0);
	window1.generateTrack(tracks, tracksCM);

	RectangleGenerator window2(0.6, 1.2);
	window2.setPosition(-4, -4, 1.4);
	window2.setRPY(0, 0, 0);
	window2.generateTrack(tracks, tracksCM);

	RectangleGenerator window3(0.6, 1.2);
	window3.setPosition(-5, 0, 1.4);
	window3.setRPY(0, 0, -M_PI / 2);
	window3.generateTrack(tracks, tracksCM);

	RectangleGenerator window4(0.6, 1.2);
	window4.setPosition(-5, 2.5, 1.4);
	window4.setRPY(0, 0, -M_PI / 2);
	window4.generateTrack(tracks, tracksCM);

	RectangleGenerator window5(0.6, 1.2);
	window5.setPosition(-5, 5, 1.4);
	window5.setRPY(0, 0, -M_PI / 2);
	window5.generateTrack(tracks, tracksCM);

	//Pictures
	RectangleGenerator picture1(0.3, 0.75);
	picture1.setPosition(-3 + 3 / 2 * sqrt(2.0), 8 + 3 / 2 * sqrt(2.0), 1.1);
	picture1.setRPY(0, M_PI / 12, -3 * M_PI / 4);
	picture1.generateTrack(tracks, tracksCM);

	RectangleGenerator picture2(0.3, 0.75);
	picture2.setPosition(-3 + 3.5 / 2 * sqrt(2.0), 8 + 3.5 / 2 * sqrt(2.0),
				1.1);
	picture2.setRPY(0, M_PI / 13, -3 * M_PI / 4);
	picture2.generateTrack(tracks, tracksCM);

	//fan coils
	RectangleGenerator fanCoil1(1.5, 0.4);
	fanCoil1.setPosition(2.38, -3.8, 0.2);
	fanCoil1.setRPY(0, 0, 0);
	fanCoil1.generateTrack(tracks, tracksCM);

	RectangleGenerator fanCoil2(1.5, 0.4);
	fanCoil2.setPosition(-2.38, -3.8, 0.2);
	fanCoil2.setRPY(0, 0, 0);
	fanCoil2.generateTrack(tracks, tracksCM);

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

bool TestPublisher::rectangleVisible(vector<Eigen::Vector4d>& track,
			const Eigen::Matrix4d& H_CW)
{
	for (int i = 0; i < track.size(); i++)
	{
		if (!pointVisible(track[i], H_CW))
		{
			return false;
		}
	}

	return true;

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

		track.push_back(trackVertex);
	}
}

}
