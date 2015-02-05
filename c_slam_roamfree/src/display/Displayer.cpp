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

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include <vector>
#include <sstream>

#include <c_slam_msgs/TrackedObject.h>

using namespace std;

cv::Mat image(360, 640, CV_8UC3);
ros::Time last_ts;

void tracksCb(const c_slam_msgs::TrackedObject& msg)
{

	if (msg.imageStamp > last_ts) {
		image.setTo(cv::Scalar(0, 0, 0));
		last_ts = msg.imageStamp;
	}

	vector<geometry_msgs::Point32> points = msg.polygon.points;
	vector<cv::Point> polygon;
	for (int i = 0; i < points.size(); i++)
	{
		geometry_msgs::Point32& p = points[i];

		cv::Point vertex(p.x, p.y);
		polygon.push_back(vertex);
		cv::circle(image, vertex, 2, cv::Scalar(0, 0, 255));
	}
	vector<vector<cv::Point> > countours;
	countours.push_back(polygon);
	cv::drawContours(image, countours, -1, cv::Scalar(255, 0, 0));

	double x =
				0.25
							* (msg.polygon.points[0].x + msg.polygon.points[1].x
										+ msg.polygon.points[2].x
										+ msg.polygon.points[3].x);
	double y =
				0.25
							* (msg.polygon.points[0].y + msg.polygon.points[1].y
										+ msg.polygon.points[2].y
										+ msg.polygon.points[3].y);

	cv::Point featureCenter(x, y);
	stringstream ss;
	ss << "Landmark_" << msg.id;

	int baseline = 0;
	cv::Size textSize = cv::getTextSize(ss.str(), cv::FONT_HERSHEY_SIMPLEX, 0.4,
				1, &baseline);
	cv::Point textOrigin(featureCenter.x - textSize.width / 2,
				featureCenter.y + textSize.height - 40);

	cv::putText(image, ss.str(), textOrigin, cv::FONT_HERSHEY_SIMPLEX, 0.4,
				cv::Scalar(0, 255, 0));

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "displayer");

	ros::NodeHandle n;

	ros::Subscriber trackSub = n.subscribe("/tracks", 1024, tracksCb);
	cv::namedWindow("Tracks");

	ROS_INFO("Displayer started");

	ros::Rate rate(100);
	while (ros::ok())
	{
		//image.setTo(cv::Scalar(0, 0, 0));
		ros::spinOnce();

		cv::imshow("Tracks", image);
		cv::waitKey(1);

		rate.sleep();
	}

	ROS_INFO("Displayer node shut down");

	return 0;
}
