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

#include <iostream>

#include "Dispatcher.h"

namespace enc = sensor_msgs::image_encodings;

Dispatcher::Dispatcher(ros::NodeHandle& n) :
			it(n), rotX(0), rotY(0), rotZ(0)
{
	navdataSubscriber = n.subscribe("/ardrone/navdata", 1,
				&Dispatcher::handleNavdata, this);
	imageSubscriber = it.subscribe("/ardrone/image_rect_color", 1,
				&Dispatcher::handleImage, this);
	classificationService = n.serviceClient<c_fuzzy::Classification>(
				"classification", true);

}

void Dispatcher::handleNavdata(const ardrone_autonomy::Navdata& navdata)
{
	rotX = navdata.rotX;
	rotY = navdata.rotY;
	rotZ = navdata.rotZ;
}

void Dispatcher::handleImage(const sensor_msgs::ImageConstPtr& msg)
{

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	detector.setRoll(rotX);
	detector.setPitch(rotY);
	detector.setYaw(rotZ);

	detector.detect(cv_ptr->image);
}

void Dispatcher::classify(c_fuzzy::Classification& serviceCall)
{
	if (classificationService.isValid())
	{
		classificationService.call(serviceCall);
	}
}
