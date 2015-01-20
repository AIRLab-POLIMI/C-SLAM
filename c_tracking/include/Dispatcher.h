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
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <c_slam_msgs/NamedPolygon.h>
#include <c_slam_msgs/TrackedObject.h>

#include "ParameterServer.h"

#include "Track.h"
#include "CMTFeatureExtractor.h"

class Dispatcher
{
public:
	Dispatcher(ros::NodeHandle& n);
	void handleImage(const sensor_msgs::ImageConstPtr& msg);
	void handleObjectTrackRequest(
				const c_slam_msgs::NamedPolygon& polygonMessage);

private:
	void publishTrack(const uint64_t& id,
				const std::vector<cv::Point2f>& polygon, const cv::Rect& roi,
				const ros::Time& stamp);
	void getPolygon(const c_slam_msgs::NamedPolygon& polygonMessage,
				std::vector<cv::Point2f>& polygon, cv::Point2f& massCenter);
	cv::Rect findRoi(const std::vector<cv::Point2f>& polygon, cv::Mat& image);

private:
	bool isSameObject(Track& track, std::vector<cv::Point2f>& polygon,
				cv::Point2f& massCenter);
	bool isTrackedObject(std::vector<cv::Point2f>& polygon,
				cv::Point2f& massCenter);
	void drawResults(cv::Mat& coloredImage, const cv::Rect& roi, Track& track);
	void drawPolygon(cv::Mat& frame, const std::vector<cv::Point2f>& contour,
				cv::Scalar colour);
	void drawKeypoints(cv::Mat& frame,
				const std::vector<std::pair<cv::KeyPoint, int> >& keypoints,
				cv::Scalar color);

private:
	//Ros management
	image_transport::ImageTransport it;
	image_transport::Subscriber imageSubscriber;
	ros::Subscriber toTrackSubscriber;
	ros::Publisher trackPublisher;

	//Parameters
	ParameterServer parameterServer;

	//Last image pointer
	cv_bridge::CvImagePtr cv_ptr;

	//Tracks
	CMTFeatureExtractor featureExtractor;
	std::vector<Track> tracks;

	//display
	std::string src_window;

	uint64_t nextId;
};

#endif /* DISPATCHER_H_ */
