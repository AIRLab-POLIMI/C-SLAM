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

#include "SLAMLogic.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

using namespace ros;
using namespace sensor_msgs;
using namespace image_geometry;

namespace enc = sensor_msgs::image_encodings;

SLAMLogic::SLAMLogic(NodeHandle n, ParameterServer& parameters) :
			it(n), infoCache(15), imageCache(15)
{
	cameraSubscriber = it.subscribeCamera("/ardrone/image_rect_color", 1,
				&SLAMLogic::handleCamera, this);
	trackSubscriber = n.subscribe("tracks", 100, &SLAMLogic::handleTrack, this);
	cv::namedWindow("test", CV_WINDOW_AUTOSIZE);
}

void SLAMLogic::handleCamera(const ImageConstPtr& msg,
			const CameraInfoConstPtr& info_msg)
{
	imageCache.add(msg);
	infoCache.add(info_msg);
}

void SLAMLogic::handleTrack(const c_tracking::TrackedObject& track)
{
	cv::Mat coloredImage;
	cv_bridge::CvImagePtr cv_ptr, cv_ptr_color;
	PinholeCameraModel cameraModel;

	Time t = track.imageStamp + Duration(0, 100);
	const ImageConstPtr& img = imageCache.getElemBeforeTime(t);
	const CameraInfoConstPtr& info_msg = infoCache.getElemBeforeTime(t);

	if (img == NULL || info_msg == NULL)
	{
		ROS_WARN("No valid image/info pairs in cache");
		return;
	}

	cameraModel.fromCameraInfo(info_msg);

	try
	{
		cv_ptr = cv_bridge::toCvCopy(img, enc::MONO8);
		cv_ptr_color = cv_bridge::toCvCopy(img, enc::BGR8);
		coloredImage = cv_ptr_color->image;

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv::imshow("test", coloredImage);
	cv::waitKey(1);

}
