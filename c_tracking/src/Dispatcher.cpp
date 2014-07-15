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
#include <angles/angles.h>

namespace enc = sensor_msgs::image_encodings;

std::vector<cv::Point2f> polygon;
std::string src_window = "frame";
cv::Mat frame;

bool input = false;

void mouseHandler(int event, int x, int y, int flags, void* param)
{
	Dispatcher& D = *(Dispatcher*) param;

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

		//setup initialization data
		InitializationData data;
		//TODO
		data.polygon = polygon;
		polygon.clear();
		D.featureExtractor.discriminateKeyPoints(gray, data);

		//setup tracker
		CMT cmt;
		cmt.initialize(gray, data);
		D.tracks.push_back(cmt);

		input = false;
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

	cv::namedWindow(src_window, CV_WINDOW_AUTOSIZE);
	cv::setMouseCallback(src_window, mouseHandler, this);

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
	double currentRot = rotX;
	cv_bridge::CvImagePtr cv_ptr, cv_ptr_color;
	cv::Mat color;

	if (input)
	{
		cv::waitKey(50);
		return;
	}

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

	//extract features
	featureExtractor.detect(cv_ptr->image);
	std::vector<cv::KeyPoint>& keypoints = featureExtractor.getKeypoints();
	cv::Mat& features = featureExtractor.getFeatures();

	std::vector<std::vector<cv::Point2f> > contours;

	//track
	for (int i = 0; i < tracks.size(); i++)
	{
		CMT& cmt = tracks[i];
		cmt.processFrame(cv_ptr->image, keypoints, features);
		const std::vector<cv::Point2f>& polygon = cmt.getTrackedPolygon();
		contours.push_back(polygon);

		for (int j = 0; j < cmt.trackedKeypoints.size(); j++)
			cv::circle(frame, cmt.trackedKeypoints[j].first.pt, 3,
						cv::Scalar(255, 255, 255));
	}

	cv::drawContours(frame, contours, -1, cv::Scalar(255, 255, 255));

	cv::imshow(src_window, frame);
	cv::waitKey(1);

}
