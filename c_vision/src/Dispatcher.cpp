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
#include <sstream>

#include "Dispatcher.h"
#include "ObjectClassificator.h"

namespace enc = sensor_msgs::image_encodings;

Dispatcher::Dispatcher(ros::NodeHandle& n, ParameterServer& parameterServer) :
			n(n), it(n), detector(parameterServer), viewer("Detected Image"),
			classifierParam(parameterServer.getClassifierParams())
{
	navdataSubscriber = n.subscribe("/ardrone/navdata", 1,
				&Dispatcher::handleNavdata, this);
	imageSubscriber = it.subscribe("/ardrone/image_rect_color", 1,
				&Dispatcher::handleImage, this);
	connectToClassificationServer();

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

	detect(cv_ptr);
	classify();
	display(cv_ptr);

}

void Dispatcher::detect(const cv_bridge::CvImagePtr& cv_ptr)
{
	detector.setRoll(rotX);
	detector.setPitch(rotY);
	detector.setYaw(rotZ);
	detector.detect(cv_ptr->image);
}

void Dispatcher::classify()
{
	c_fuzzy::Classification serviceCall;

	ObjectClassificator classificator(serviceCall, classifierParam);
	classificator.processFeatures(detector.getRectangles());
	//TODO no cluster detection...
	//classificator.processFeatures(detector.getPoles());
	//classificator.processFeatures(detector.getClusters());

	if (classificationService.isValid())
	{
		classificationService.call(serviceCall);
	}
	else
	{
		ROS_ERROR("Service down, waiting reconnection...");
		classificationService.waitForExistence();
		connectToClassificationServer();
	}

	classificator.labelFeatures();

}

void Dispatcher::display(const cv_bridge::CvImagePtr& cv_ptr)
{
	viewer.setClusters(detector.getClusters());
	viewer.setRectangles(detector.getRectangles());
	viewer.setPoles(detector.getPoles());
	viewer.setRoll(rotX);
	viewer.display(cv_ptr->image);

	detector.deleteDetections();
}

void Dispatcher::connectToClassificationServer()
{
	classificationService = n.serviceClient<c_fuzzy::Classification>(
				"classification", true);
}

