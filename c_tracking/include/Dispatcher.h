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

#ifndef DISPATCHER_H_
#define DISPATCHER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ardrone_autonomy/Navdata.h>

#include <c_tracking/NamedPolygon.h>

#include "RobotPose.h"
#include "MappingTracker.h"
#include "CMTFeatureExtractor.h"

class Dispatcher
{
public:
	Dispatcher(ros::NodeHandle& n);
	void handleNavdata(const ardrone_autonomy::Navdata& navdata);
	void handleImage(const sensor_msgs::ImageConstPtr& msg,
				const sensor_msgs::CameraInfoConstPtr& info_msg);
	void handleObjectTrackRequest(
				const c_tracking::NamedPolygon& polygonMessage);

private:
	void getPolygon(const c_tracking::NamedPolygon& polygonMessage,
				std::vector<cv::Point2f>& polygon);
	void drawPolygon(cv::Mat& frame, const std::vector<cv::Point2f>& contour,
				cv::Scalar colour);
	void drawKeypoints(cv::Mat& frame,
				const std::vector<std::pair<cv::KeyPoint, int> >& keypoints,
				cv::Scalar color);

private:
	//Ros management
	ros::NodeHandle& n;
	image_transport::ImageTransport it;
	ros::Subscriber navdataSubscriber;
	ros::Subscriber toTrackSubscriber;
	image_transport::CameraSubscriber imageSubscriber;

	//camera model data
	image_geometry::PinholeCameraModel cameraModel;

	//Last image pointer
	cv_bridge::CvImagePtr cv_ptr;

	//Tracks
	CMTFeatureExtractor featureExtractor;
	std::vector<MappingTracker> tracks;

	//robot Pose
	RobotPose pose;

	//Odometry Data
	double rotX, rotY, rotZ;

	//display
	std::string src_window;
};

#endif /* DISPATCHER_H_ */
