/*
 * c_tracking,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_tracking.
 *
 * c_tracking is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_tracking is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_tracking.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <c_tracking/NamedPolygon.h>

using namespace std;
using namespace cv;

namespace enc = sensor_msgs::image_encodings;

bool input = false;
vector<Point2f> polygon;
Mat frame;
string src_window = "Track Selector";
ros::Publisher publisher;

void sendPolygonMessage()
{
	c_tracking::NamedPolygon message;
	for(int i = 0; i < polygon.size(); i++)
	{
		geometry_msgs::Point32 point;
		point.x = polygon[i].x;
		point.y = polygon[i].y;
		point.z = 0;

		message.polygon.points.push_back(point);
	}

	publisher.publish(message);

}

void mouseHandler(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDOWN)
	{
		/* left button clicked. ROI selection begins */
		polygon.push_back(cv::Point2f(x, y));
		input = true;

		cv::Mat img2 = frame.clone();

		for (int j = 0; j + 1 < polygon.size(); j++)
			cv::line(img2, polygon[j], polygon[j + 1],
						cv::Scalar(255, 255, 255));

		cv::imshow(src_window, img2);

	}

	if (event == CV_EVENT_RBUTTONDOWN)
	{
		cv::Mat img2 = frame.clone();
		cv::imshow(src_window, img2);

		cv::Mat gray;
		cvtColor(frame, gray, CV_BGR2GRAY);

		sendPolygonMessage();
		polygon.clear();

		input = false;
	}
}

void handleImage(const sensor_msgs::ImageConstPtr& msg)
{

	if(!input)
	{
		try
		{
			cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
			frame = cv_ptr->image;
			imshow(src_window, frame);

		} catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}

}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Object_selector");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);

	image_transport::Subscriber imageSubscriber = it.subscribe("/ardrone/image_rect_color", 1, &handleImage);
	publisher = n.advertise<c_tracking::NamedPolygon>("to_track", 1000);

	namedWindow(src_window);
	setMouseCallback(src_window, mouseHandler);

	ROS_INFO("Manual object selector started");

	while(ros::ok())
	{
		waitKey(1);
		ros::spinOnce();
	}

}
