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

#ifndef BASELOGIC_H_
#define BASELOGIC_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <c_fuzzy/Classification.h>
#include <tf/transform_listener.h>
#include <opencv2/core/core.hpp>

#include "ParameterServer.h"

class BaseLogic
{
public:
	BaseLogic(ros::NodeHandle& n, ParameterServer& parameters);
	void handleImu(const sensor_msgs::Imu& imu);

protected:
	void connectToClassificationServer();
	void callClassificationService(c_fuzzy::Classification& serviceCall);
	void sendFeatures(
				const std::vector<std::pair<std::vector<cv::Point>, std::string> >& features, const cv::Point& offset = cv::Point());

protected:
	//Ros management
	ros::NodeHandle& n;
	image_transport::ImageTransport it;

	ros::Subscriber imuSubscriber;
	image_transport::Subscriber imageSubscriber;
	tf::TransformListener tfListener;
	ros::ServiceClient classificationService;

	ros::Publisher publisher;

	//envirorment data
	std::string camera_frame_id;
	double roll;
	cv::Mat R;


	//parameters
	ClassifierParam& classifierParam;
};

#endif /* BASELOGIC_H_ */
