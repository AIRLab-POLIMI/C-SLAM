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

#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

using namespace ros;
using namespace sensor_msgs;
using namespace image_geometry;
using namespace cv_bridge;
using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

SLAMLogic::SLAMLogic(NodeHandle n, ParameterServer& parameters) :
			it(n), infoCache(15), imageCache(15)
{
	cameraSubscriber = it.subscribeCamera("/ardrone/image_rect_color", 1,
				&SLAMLogic::handleCamera, this);
	trackSubscriber = n.subscribe("tracks", 100, &SLAMLogic::handleTrack, this);
	namedWindow("test", CV_WINDOW_AUTOSIZE);
}

void SLAMLogic::handleCamera(const ImageConstPtr& msg,
			const CameraInfoConstPtr& info_msg)
{
	imageCache.add(msg);
	infoCache.add(info_msg);
}

void SLAMLogic::handleTrack(const c_tracking::TrackedObject& track)
{
	Mat coloredImage, objectImage;
	Rect roi;
	CvImagePtr cv_ptr, cv_ptr_color;
	PinholeCameraModel cameraModel;

	try
	{
		getImageData(track, cv_ptr, cv_ptr_color, cameraModel);
		coloredImage = cv_ptr_color->image;
		getRoi(track, coloredImage, roi, objectImage);
		imshow("test", objectImage);
		waitKey(1);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	catch (runtime_error& e)
	{
		ROS_WARN(e.what());
	}
}

void SLAMLogic::getImageData(const c_tracking::TrackedObject& track,
			CvImagePtr& cv_ptr, CvImagePtr& cv_ptr_color,
			PinholeCameraModel& cameraModel)
{
	Time t = track.imageStamp + Duration(0, 100);
	const ImageConstPtr& img = imageCache.getElemBeforeTime(t);
	const CameraInfoConstPtr& info_msg = infoCache.getElemBeforeTime(t);

	if (img == NULL || info_msg == NULL)
		throw runtime_error("No valid image/info pairs in cache");

	cameraModel.fromCameraInfo(info_msg);

	cv_ptr = toCvCopy(img, enc::MONO8);
	cv_ptr_color = toCvCopy(img, enc::BGR8);
}

void SLAMLogic::getRoi(const c_tracking::TrackedObject& track, Mat& input,
			Rect& roi, Mat& image)
{
	vector<Point> polygon;

	roi.x = track.roi.x_offset;
	roi.y = track.roi.y_offset;
	roi.width = track.roi.width;
	roi.height = track.roi.height;

	for (int i = 0; i < track.polygon.points.size(); i++)
	{
		geometry_msgs::Point32 p = track.polygon.points[i];
		Point point(p.x - roi.x, p.y - roi.y);
		polygon.push_back(point);
	}

	//get the roi
	Mat tmp;
	tmp = input(roi);

	//create the mask and apply it
	Mat mask(tmp.size(), CV_8UC1);
	mask.setTo(0);
	fillConvexPoly(mask, polygon, 255, 8, 0);
	tmp.copyTo(image, mask);

}
