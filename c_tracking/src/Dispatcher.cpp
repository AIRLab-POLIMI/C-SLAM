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

#include "Dispatcher.h"

#include <string>

namespace enc = sensor_msgs::image_encodings;


int drag = 0, select_flag = 0;

cv::Point point1, point2;
bool callback = false;
std::string src_window = "frame";
cv::Mat frame;

void mouseHandler(int event, int x, int y, int flags, void* param)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag && !select_flag)
    {
        /* left button clicked. ROI selection begins */
        point1 = cv::Point(x, y);
        drag = 1;
    }

    if (event == CV_EVENT_MOUSEMOVE && drag && !select_flag)
    {
        /* mouse dragged. ROI being selected */
        cv::Mat img1 = frame.clone();
        point2 = cv::Point(x, y);
        cv::rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
        cv::imshow(src_window, img1);
    }

    if (event == CV_EVENT_LBUTTONUP && drag && !select_flag)
    {
        cv::Mat img2 = frame.clone();
        point2 = cv::Point(x, y);
        drag = 0;
        select_flag = 1;
        cv::imshow(src_window, img2);
        callback = true;
    }
}

Dispatcher::Dispatcher(ros::NodeHandle& n) :
		n(n), it(n)
{
	navdataSubscriber = n.subscribe("/ardrone/navdata", 1,
			&Dispatcher::handleNavdata, this);
	imageSubscriber = it.subscribe("/ardrone/image_rect_color", 1,
			&Dispatcher::handleImage, this);
	rotX = rotY = rotZ = 0;

}

void Dispatcher::handleNavdata(const ardrone_autonomy::Navdata& navdata)
{
	/*
	 *	X axis outgoing from the drone camera, opposite convention wrt ardrone autonomy driver
	 *	Z axis upside
	 *	Y axis to the left (TODO check)
	 */
	rotX = -navdata.rotX;
	rotY = navdata.rotY;
	rotZ = navdata.rotZ;
}

void Dispatcher::handleImage(const sensor_msgs::ImageConstPtr& msg)
{

	cv_bridge::CvImagePtr cv_ptr, cv_ptr_color;
	cv::Mat color;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
		cv_ptr_color = cv_bridge::toCvCopy(msg, enc::BGR8);
		frame = cv_ptr_color->image;

	} catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	for (int i = 0; i < tracks.size(); i++)
	{
		CMT& cmt = tracks[i];
		cmt.processFrame(cv_ptr->image);
		cv::line(frame, cmt.topLeft, cmt.topRight,
				cv::Scalar(255, 255, 255));
		cv::line(frame, cmt.topRight, cmt.bottomRight,
				cv::Scalar(255, 255, 255));
		cv::line(frame, cmt.bottomRight, cmt.bottomLeft,
				cv::Scalar(255, 255, 255));
		cv::line(frame, cmt.bottomLeft, cmt.topLeft,
				cv::Scalar(255, 255, 255));
	}

	cv::imshow(src_window, frame);
	cv::waitKey(1);

}
