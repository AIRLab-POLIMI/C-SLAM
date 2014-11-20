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

using namespace std;

class RosPublisher
{
public:

	RosPublisher()
	{
		setTracks();

		trackPublisher = n.advertise<c_slam_msgs::TrackedObject>("/tracks",
					6000);

		gtSubscriber = n.subscribe("/firefly/ground_truth/pose", 1000,
					&RosPublisher::publishTracks, this);

		K << 565.59102697808, 0.0, 337.839450567586, //
		0.0, 563.936510489792, 199.522081717361, //
		0.0, 0.0, 1.0;
	}

	void publishTracks(const geometry_msgs::PoseWithCovarianceStamped& pose_msg)
	{
		geometry_msgs::Quaternion q = pose_msg.pose.pose.orientation;
		geometry_msgs::Point p = pose_msg.pose.pose.position;
		tf::Quaternion q_tf;
		tf::quaternionMsgToTF(q, q_tf);
		tf::Matrix3x3 R(q_tf);

		//TODO compute pose

		Eigen::Matrix4d H_CW, H_WC, H_RC, H_WR;

		H_WR << R[0][0], R[0][1], R[0][0], p.x, //
		R[1][0], R[1][1], R[1][1], p.y, //
		R[2][0], R[2][1], R[2][2], p.z, //
		0, 0, 0, 1;

		H_RC << 0, 0, 1, 0, //
		-1, 0, 0, 0, //
		0, -1, 0, 0, //
		0, 0, 0, 1;

		H_WC = H_WR * H_RC;
		H_WC = H_WC / H_WC(3, 3);

		H_CW = H_WC.inverse();
		H_CW /= H_CW(3, 3);

		for (int i = 0; i < tracks.size(); i++)
		{
			if (trackVisible(tracks[i], H_CW))
			{
				c_slam_msgs::TrackedObject msg;

				msg.id = i;
				msg.imageStamp = pose_msg.header.stamp;

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

	void publishGroundTruthLandmark()
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

	void setTracks()
	{

		const int numTracks = 16;

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

		for (int k = 0; k < numTracks; k++)
		{
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

private:
	ros::NodeHandle n;
	ros::Publisher trackPublisher;
	ros::Subscriber gtSubscriber;
	tf::TransformBroadcaster br;

	//tracks
	vector<vector<Eigen::Vector4d> > tracks;
	vector<Eigen::Vector3d> tracksCM;

	Eigen::Matrix3d K;
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "circular_test");

	RosPublisher publisher;

	ros::Rate rate(30);
	while (ros::ok())
	{
		publisher.publishGroundTruthLandmark();
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
