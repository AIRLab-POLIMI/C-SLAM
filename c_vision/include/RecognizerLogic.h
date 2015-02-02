/*
 * c_vision,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_vision.
 *
 * c_vision is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_vision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_vision.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RECOGNIZERLOGIC_H_
#define RECOGNIZERLOGIC_H_

#include "BaseLogic.h"

#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/cache.h>
#include <image_geometry/pinhole_camera_model.h>
#include <c_slam_msgs/TrackedObject.h>

#include "ParameterServer.h"
#include "AdvancedDetector.h"

#include "ObjectClassificator.h"

#include <map>

class RecognizerLogic: public BaseLogic
{
public:
	RecognizerLogic(ros::NodeHandle n, ParameterServer& parameters);
	void handleCamera(const sensor_msgs::ImageConstPtr& msg,
				const sensor_msgs::CameraInfoConstPtr& info_msg);
	void handleTrack(const c_slam_msgs::TrackedObject& track);

private:
	void detect(cv::Mat& image, cv::Mat& mask, bool showCanny);
	void classify(image_geometry::PinholeCameraModel& cameraModel, cv::Rect& roi);
	void display(cv::Mat& image, std::size_t id);

	void rectify(ObjectClassificator& classificator, image_geometry::PinholeCameraModel& cameraModel, cv::Rect& roi);

private:
	void getImageData(const c_slam_msgs::TrackedObject& track,
				cv_bridge::CvImagePtr& cv_ptr,
				cv_bridge::CvImagePtr& cv_ptr_color,
				image_geometry::PinholeCameraModel& cameraModel);
	void getRoi(const c_slam_msgs::TrackedObject& track, cv::Mat& input,
				cv::Rect& roi, cv::Mat& image, cv::Mat& mask);

private:
	//Ros management
	image_transport::CameraSubscriber cameraSubscriber;

	message_filters::Cache<sensor_msgs::CameraInfo> infoCache;
	message_filters::Cache<sensor_msgs::Image> imageCache;

	ros::Subscriber trackSubscriber;

	//Object detection
	AdvancedDetector detector;


	//Visualization
	DisplayParam& dispP;
	std::map<std::size_t, ImageView> viewers;
};

#endif /* RECOGNIZERLOGIC_H_ */
