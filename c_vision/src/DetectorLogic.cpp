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

#include "DetectorLogic.h"
#include "ObjectClassificator.h"

#include <c_tracking/NamedPolygon.h>

namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace cv;

DetectorLogic::DetectorLogic(ros::NodeHandle& n, ParameterServer& parameters) :
			BaseLogic(n, parameters), detector(parameters),
			viewer("Detected Image")
{
	rotX = 0;
	imageSubscriber = it.subscribe("/ardrone/image_rect_color", 1,
				&DetectorLogic::handleImage, this);
	detectionPublisher = n.advertise<c_tracking::NamedPolygon>("to_track", 10);
}

void DetectorLogic::handleImage(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);

		detect(cv_ptr);
		classify();
		display(cv_ptr);

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
	}
}

void DetectorLogic::detect(const cv_bridge::CvImagePtr& cv_ptr)
{
	detector.setRoll(rotX);
	detector.detect(cv_ptr->image);
}

void DetectorLogic::classify()
{
	c_fuzzy::Classification serviceCall;

	ObjectClassificator classificator(serviceCall, classifierParam);
	classificator.processFeatures(detector.getRectangles());

	callClassificationService(serviceCall);

	classificator.labelFeatures();

	const vector<pair<vector<Point>, string> >& features =
				classificator.getGoodFeatures();

	sendFeatures(features);

}

void DetectorLogic::display(const cv_bridge::CvImagePtr& cv_ptr)
{
	viewer.setRectangles(detector.getRectangles());
	viewer.setPoles(detector.getPoles());
	viewer.setRoll(rotX);
	viewer.display(cv_ptr->image);

	detector.deleteDetections();
}

void DetectorLogic::sendFeatures(
			const vector<pair<vector<Point>, string> >& features)
{
	for (vector<pair<vector<Point>, string> >::const_iterator i =
				features.begin(); i != features.end(); ++i)
	{
		const vector<Point>& polygon = i->first;
		const string& name = i->second;

		c_tracking::NamedPolygon message;

		message.polygonLabel = name;

		for (int i = 0; i < polygon.size(); i++)
		{
			geometry_msgs::Point32 point;
			point.x = polygon[i].x;
			point.y = polygon[i].y;
			point.z = 0;

			message.polygon.points.push_back(point);
		}

		detectionPublisher.publish(message);
	}
}

