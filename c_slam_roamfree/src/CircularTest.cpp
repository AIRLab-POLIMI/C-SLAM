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

using namespace std;

class RosPublisher
{
public:
	RosPublisher()
	{
		imuPublisher = n.advertise<sensor_msgs::Imu>("/ardrone/imu", 6000);
		trackPublisher = n.advertise<c_slam_msgs::TrackedObject>("/tracks",
					6000);
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
				const Eigen::Matrix4d& H_WC, double t)
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

	void publishGroundTruth(Eigen::Matrix4d& H)
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
								"ardrone_ground_truth"));

	}

	void publishGroundTruthLandmark(vector<Eigen::Vector3d>& tracksCM)
	{
		for (int i = 0; i < tracksCM.size(); i++)
		{
			stringstream ss;
			ss << "Track_" << i;
			tf::Transform trasform;
			tf::Vector3 t_m_tf;

			tf::vectorEigenToTF(tracksCM[i], t_m_tf);

			trasform.setOrigin(t_m_tf);
			trasform.setRotation(tf::Quaternion::getIdentity());
			br.sendTransform(
						tf::StampedTransform(trasform, ros::Time::now(),
									"world", ss.str()));
		}
	}

private:
	bool pointVisible(Eigen::Vector4d& trackPoint, Eigen::Matrix4d H_CW)
	{
		Eigen::Vector4d trackRC = H_CW * trackPoint;
		trackRC /= trackRC(3);

		Eigen::Vector3d projection = K * H_CW.topRows(3) * trackPoint;
		projection /= projection(2);

		return trackRC(2) > 0 && projection(0) >= 0 && projection(0) < 640
					&& projection(1) >= 0 && projection(1) < 360;
	}

	bool trackVisible(vector<Eigen::Vector4d>& track,
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

private:
	ros::NodeHandle n;
	ros::Publisher imuPublisher;
	ros::Publisher trackPublisher;
	tf::TransformBroadcaster br;

	Eigen::Matrix3d K;
};

/*
void setTracks(vector<vector<Eigen::Vector4d> >& tracks,
			vector<Eigen::Vector3d>& tracksCM, double r)
{
	int numTracks = 8;

	const double w = 0.2;
	const double h = 0.1;

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

			Eigen::Vector4d track;
			track << xp, yp, zp, 1;

			cout << track(0) << "," << track(1) << "," << track(2) << ";";
			cout << endl;
			tracks[i].push_back(track);
		}

		theta += 2 * M_PI / numTracks;
	}
	cout << "]" << endl;
}
*/

void setTracks(vector<vector<Eigen::Vector4d> >& tracks,
			vector<Eigen::Vector3d>& tracksCM, double r) {

	const int numTracks = 16;

	double cm [][3] = {
			{2.0,-2.0,-0.3},
			{2.0,-1.0, 0.3},
			{2.0, 0.0,-0.3},
			{2.0, 1.0, 0.3},
			{2.0, 2.0,-0.3},

			{-2.0,-2.0,-0.3},
			{-2.0,-1.0, 0.3},
			{-2.0, 0.0,-0.3},
			{-2.0, 1.0, 0.3},
			{-2.0, 2.0,-0.3},

			{-1.0, 2.0, 0.3},
			{-0.0, 2.0,-0.3},
			{ 1.0, 2.0, 0.3},

			{-1.0,-2.0, 0.3},
			{ 0.0,-2.0,-0.3},
			{ 1.0,-2.0, 0.3}
	};

	tracks.resize(numTracks);

	for (int k = 0; k < numTracks; k++) {
		Eigen::Vector3d trackCM;
		trackCM << cm[k][0], cm[k][1], cm[k][2];
		tracksCM.push_back(trackCM);

		for (int j = 0; j < 4; j++)
		{
			Eigen::Vector4d track;
			track << cm[k][0], cm[k][1], cm[k][2], 1;

			cout << track(0) << "," << track(1) << "," << track(2) << ";";
			cout << endl;
			tracks[k].push_back(track);
		}
	}
}


void computeCameraPose(Eigen::Matrix4d& H, double t, double theta0, double w0,
			double alpha, double r)
{
	double thetaRobot = theta0 + w0 * t + 0.5 * alpha * std::pow(t, 2);
	double theta = thetaRobot - M_PI / 2;

	double x = r * cos(theta);
	double y = r * sin(theta);
	double z = 0;

	Eigen::Matrix4d H_WR;

	H_WR << cos(thetaRobot), -sin(thetaRobot), 0, x, //
	sin(thetaRobot), cos(thetaRobot), 0, y, //
	0, 0, 1, z, //
	0, 0, 0, 1; //

	Eigen::Matrix4d H_RC;

	H_RC << 0, 0, 1, 0, //
	-1, 0, 0, 0, //
	0, -1, 0, 0, //
	0, 0, 0, 1;

	H =  H_WR * H_RC;
	H = H / H(3, 3);

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "circular_test");

	RosPublisher publisher;

	//IMU data
	double r = 1; // meters
	double alpha = 0.01; // radians / s^2
	double w0 = 0.0; //initial angular speed
	double thetaRobot0 = 0; //-M_PI / 2.0;
	double t = 0.0;
	double imuRate = 50;

	//Tracks data
	vector<vector<Eigen::Vector4d> > tracks;
	vector<Eigen::Vector3d> tracksCM;
	setTracks(tracks, tracksCM, 2.5);

	ROS_INFO("Waiting other nodes to start...");

	ros::Duration(2.0).sleep();

	ROS_INFO("Simulation started");

	ros::Rate rate(imuRate);
	for (int i = 0; i < 10000 && ros::ok(); i++)
	{
		double w = w0 + alpha * t;

		vector<double> za =
		{ alpha * r, std::pow(w, 2) * r, 9.80665 };
		vector<double> zw =
		{ 0.0, 0.0, w };

		Eigen::Matrix4d H_WC;
		computeCameraPose(H_WC, t, thetaRobot0, w0, alpha, r);

		publisher.publishIMU(za, zw, t);
		if (i % 10 == 0)
		{
			publisher.publishTracks(tracks, H_WC, t);
			publisher.publishGroundTruth(H_WC);
			publisher.publishGroundTruthLandmark(tracksCM);
		}

		t += 1.0 / imuRate;

		rate.sleep();
	}

	ROS_INFO("Simulation completed");

	return 0;
}
