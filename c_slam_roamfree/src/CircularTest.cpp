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

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <c_slam_msgs/TrackedObject.h>

#include <Eigen/Dense>

using namespace std;

class RosPublisher
{
public:
	RosPublisher()
	{
		imuPublisher = n.advertise<sensor_msgs::Imu>("/ardrone/imu", 1024);
		trackPublisher = n.advertise<c_slam_msgs::TrackedObject>("/tracks",
					1024);
		K << 565.59102697808, 0.0, 337.839450567586, //
		0.0, 563.936510489792, 199.522081717361, //
		0.0, 0.0, 1.0;
	}

	void publishIMU(vector<double>& za, vector<double>& zw, double t)
	{
		sensor_msgs::Imu msg;

		msg.angular_velocity.x = zw[0];
		msg.angular_velocity.y = zw[1];
		msg.angular_velocity.z = zw[2];

		msg.linear_acceleration.x = za[0];
		msg.linear_acceleration.y = za[1];
		msg.linear_acceleration.z = za[2];

		msg.header.stamp.fromSec(t);

		imuPublisher.publish(msg);
	}

	void publishTracks(vector<vector<Eigen::Vector4d> > tracks,
				Eigen::Matrix4d H, double t)
	{
		for (int i = 0; i < tracks.size(); i++)
		{
			int inImage = 0;
			for (int j = 0; j < tracks[i].size(); j++)
			{
				if(trackVisible(tracks[i][j], H))
				{

				}


			}

			if(inImage)
			{
				Eigen::Vector3d p1 = K * H.topRows(3) * tracks[i][0];
				Eigen::Vector3d p2 = K * H.topRows(3) * tracks[i][1];
				Eigen::Vector3d p3 = K * H.topRows(3) * tracks[i][2];
				Eigen::Vector3d p4 = K * H.topRows(3) * tracks[i][3];

				c_slam_msgs::TrackedObject msg;

				msg.id = i;
				msg.imageStamp.fromSec(t);


				msg.polygon.points[0].x = p1(0);
				msg.polygon.points[0].y = p1(1);
			}

		}
	}

private:
	bool trackVisible(Eigen::Vector4d& trackPoint, Eigen::Matrix4d H)
	{
		Eigen::Vector4d trackRC = H * trackPoint;

		Eigen::Vector3d projection = K * H.topRows(3) * trackPoint;

		return trackRC(3) > 0 && projection(0) >= 0 && projection(0) <= 320
					&& projection(1) >= 0 && projection(2) <= 240;
	}

private:
	ros::NodeHandle n;
	ros::Publisher imuPublisher;
	ros::Publisher trackPublisher;

	Eigen::Matrix3d K;
};

void setTracks(vector<vector<Eigen::Vector4d> >& tracks, double r)
{
	int numTracks = 8;

	const double w = 0.2;
	const double h = 0.1;

	tracks.resize(numTracks);

	double theta = 0;

	for (int i = 0; i < numTracks; i++)
	{
		double x = r * cos(theta);
		double y = r * sin(theta);
		double z = 0;
		for (int j = 1; j < 4; j++)
		{
			double thetaR = theta + M_PI / 2;

			double xpr = ((j == 2 || j == 3) ? -1 : 1) * w / 2;
			double ypr = 0;
			double zpr = ((j > 2) ? -1 : 1) * h / 2;

			double xpr2 = xpr * cos(thetaR) - ypr * sin(thetaR);
			double ypr2 = xpr * sin(thetaR) + ypr * cos(thetaR);
			double zpr2 = zpr;

			double xp = x + xpr2;
			double yp = y + ypr2;
			double zp = z + zpr2;

			Eigen::Vector4d track;
			track << xp, zp, yp, 1;
			tracks[i].push_back(track);
		}

		theta += 2 * M_PI / numTracks;
	}
}

void computeHomography(Eigen::Matrix4d& H, double t, double theta0, double w0,
			double alpha, double r)
{
	double thetaRobot = theta0 + w0 * t + 0.5 * alpha * std::pow(t, 2);
	double theta = thetaRobot - M_PI / 2;

	double x = r * cos(theta);
	double y = r * sin(theta);
	double z = 0;

	H << cos(theta), 0, sin(theta), x, //
	0, 0, 0, z, //
	-sin(theta), 0, cos(theta), y, //
	0, 0, 0, 1; //

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "circular_test");

	RosPublisher publisher;

	//IMU data
	double r = 1.0; // meters
	double alpha = 0.1; // radians / s^2
	double w0 = 0.0; //initial angular speed
	double theta0 = 0; //-M_PI / 2.0;
	double t = 0.0;
	double imuRate = 50;

	//Tracks data
	vector<vector<Eigen::Vector4d> > tracks;
	setTracks(tracks, 1.5);

	ROS_INFO("Simulation started");

	ros::Rate rate(imuRate);
	for (int i = 0; i < 10000; i++)
	{
		double w = w0 + alpha * t;

		vector<double> za =
		{ alpha * r, std::pow(w, 2) * r, 9.80566 };
		vector<double> zw =
		{ 0.0, 0.0, w };

		Eigen::Matrix4d H;
		computeHomography(H, t, theta0, w0, alpha, r);

		publisher.publishIMU(za, zw, t);

		t += 1.0 / imuRate;

		rate.sleep();
	}

	ROS_INFO("Simulation completed");

	return 0;
}
