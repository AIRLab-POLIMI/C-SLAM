/*
 * c_vision,
 *
 *
 * Copyright (C) 2013 Davide Tateo
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

#ifndef DISPATCHER_H_
#define DISPATCHER_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ardrone_autonomy/Navdata.h>
#include <c_fuzzy/Classification.h>
#include "CognitiveDetector.h"

class Dispatcher
{
public:
	Dispatcher(ros::NodeHandle& n);
	void handleNavdata(const ardrone_autonomy::Navdata& navdata);
	void handleImage(const sensor_msgs::ImageConstPtr& msg);
	void classify(c_fuzzy::Classification& serviceCall);

private:
	void connectToClassificationServer();

private:
	//Ros management
	ros::NodeHandle& n;
	image_transport::ImageTransport it;
	ros::Subscriber navdataSubscriber;
	image_transport::Subscriber imageSubscriber;
	ros::ServiceClient classificationService;


	//Data needed to detect objects
	double rotX, rotY, rotZ;
	CognitiveDetector detector;
	double classifierThreshold;
};

#endif /* DISPATCHER_H_ */
