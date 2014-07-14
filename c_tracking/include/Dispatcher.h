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
#include <ardrone_autonomy/Navdata.h>
#include "CMT.h"

class Dispatcher
{
public:
	Dispatcher(ros::NodeHandle& n);
	void handleNavdata(const ardrone_autonomy::Navdata& navdata);
	void handleImage(const sensor_msgs::ImageConstPtr& msg);
	void handleObject();

private:
	void trackImage();

private:
	//Ros management
	ros::NodeHandle& n;
	image_transport::ImageTransport it;
	ros::Subscriber navdataSubscriber;
	image_transport::Subscriber imageSubscriber;

	//Tracks
public: //TODO levami
	CMTFeatureExtractor featureExtractor;
	std::vector<CMT> tracks;

	//Odometry Data
	double rotX, rotY, rotZ;
};


#endif /* DISPATCHER_H_ */
