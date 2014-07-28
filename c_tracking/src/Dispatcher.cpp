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
#include <stdexcept>
#include <angles/angles.h>

namespace enc = sensor_msgs::image_encodings;

Dispatcher::Dispatcher(ros::NodeHandle& n) :
			n(n), it(n)
{
	navdataSubscriber = n.subscribe("/ardrone/navdata", 1,
				&Dispatcher::handleNavdata, this);
	toTrackSubscriber = n.subscribe("/to_track", 1,
				&Dispatcher::handleObjectTrackRequest, this);
	imageSubscriber = it.subscribeCamera("/ardrone/image_rect_color", 1,
				&Dispatcher::handleImage, this);
	rotX = rotY = rotZ = 0;
	src_window = "Cognitive Tracking";

	cv::namedWindow(src_window, CV_WINDOW_AUTOSIZE);

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

void Dispatcher::handleImage(const sensor_msgs::ImageConstPtr& msg,
			const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	cameraModel.fromCameraInfo(info_msg);

	double currentRot = rotX;
	cv_bridge::CvImagePtr cv_ptr_color;
	cv::Mat coloredImage;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
		cv_ptr_color = cv_bridge::toCvCopy(msg, enc::BGR8);
		coloredImage = cv_ptr_color->image;

	} catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//extract features
	featureExtractor.detect(cv_ptr->image);
	std::vector<cv::KeyPoint>& keypoints = featureExtractor.getKeypoints();
	cv::Mat& features = featureExtractor.getFeatures();

	//track
	for (int i = 0; i < tracks.size(); i++)
	{
		MappingTracker& track = tracks[i];
		track.processFrame(cv_ptr->image, keypoints, features);
		cameraModel.fullIntrinsicMatrix();
		track.mapObject(coloredImage, cameraModel.fullIntrinsicMatrix());
		const std::vector<cv::Point2f>& polygon = track.getTrackedPolygon();
		drawPolygon(coloredImage, polygon, cv::Scalar(255, 255, 255));

		const std::vector<std::pair<cv::KeyPoint, int> >& trackedKeypoints =
					track.getTrackedKeypoints();
		const std::vector<std::pair<cv::KeyPoint, int> >& activeKeypoints =
					track.getActiveKeypoints();

		drawKeypoints(coloredImage, trackedKeypoints,
					cv::Scalar(255, 255, 255));
		drawKeypoints(coloredImage, activeKeypoints, cv::Scalar(255, 0, 0));
	}

	cv::imshow(src_window, coloredImage);
	cv::waitKey(1);

}

void Dispatcher::handleObjectTrackRequest(
			const c_tracking::NamedPolygon& polygonMessage)
{
	try
	{
		InitializationData data;
		getPolygon(polygonMessage, data.polygon);
		featureExtractor.discriminateKeyPoints(cv_ptr->image, data);

		//setup tracker
		MappingTracker cmt;
		cmt.initialize(cv_ptr->image, data);
		tracks.push_back(cmt);

	} catch (std::runtime_error& e)
	{
		ROS_WARN("No Keypoints in the selected object, abort tracking");
	}
}

void Dispatcher::getPolygon(const c_tracking::NamedPolygon& polygonMessage,
			std::vector<cv::Point2f>& polygon)
{
	const geometry_msgs::Polygon& p = polygonMessage.polygon;
	for (int i = 0; i < p.points.size(); i++)
	{
		geometry_msgs::Point32 point = p.points[i];
		polygon.push_back(cv::Point2f(point.x, point.y));
	}
}

void Dispatcher::drawPolygon(cv::Mat& frame,
			const std::vector<cv::Point2f>& polygon, cv::Scalar colour)
{
	for (int i = 0; i + 1 < polygon.size(); i++)
		cv::line(frame, polygon[i], polygon[i + 1], cv::Scalar(255, 255, 255));

	if (polygon.size() > 2)
	{
		cv::line(frame, polygon[0], polygon[polygon.size() - 1],
					cv::Scalar(255, 255, 255));
	}
}

void Dispatcher::drawKeypoints(cv::Mat& frame,
			const std::vector<std::pair<cv::KeyPoint, int> >& keypoints,
			cv::Scalar color)
{
	for (int j = 0; j < keypoints.size(); j++)
		cv::circle(frame, keypoints[j].first.pt, 3, color);
}

