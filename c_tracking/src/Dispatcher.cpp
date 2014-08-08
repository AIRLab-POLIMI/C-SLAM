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

#include "Dispatcher.h"

#include <string>
#include <stdexcept>
#include <angles/angles.h>

namespace enc = sensor_msgs::image_encodings;
using namespace std;
using namespace cv;

Dispatcher::Dispatcher(ros::NodeHandle& n) :
			n(n), it(n)
{
	toTrackSubscriber = n.subscribe("to_track", 1,
				&Dispatcher::handleObjectTrackRequest, this);
	trackPublisher = n.advertise<c_tracking::TrackedObject>("tracks", 1000);
	imageSubscriber = it.subscribeCamera("/ardrone/image_rect_color", 1,
				&Dispatcher::handleImage, this);

	src_window = "Cognitive Tracking";

	namedWindow(src_window, CV_WINDOW_AUTOSIZE);
}

void Dispatcher::handleImage(const sensor_msgs::ImageConstPtr& msg,
			const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	cameraModel.fromCameraInfo(info_msg);

	cv_bridge::CvImagePtr cv_ptr_color;
	Mat coloredImage;

	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, enc::MONO8);
		cv_ptr_color = cv_bridge::toCvCopy(msg, enc::BGR8);
		coloredImage = cv_ptr_color->image;

	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	//extract features
	featureExtractor.detect(cv_ptr->image);
	vector<KeyPoint>& keypoints = featureExtractor.getKeypoints();
	Mat& features = featureExtractor.getFeatures();

	//track
	for (int i = 0; i < tracks.size(); i++)
	{
		Track& track = tracks[i];
		Mat& grayImage = cv_ptr->image;
		track.processFrame(grayImage, keypoints, features);

		if (track.found())
		{
			//extract roi
			const vector<Point2f>& polygon = track.getTrackedPolygon();
			const Rect& roi = findRoi(polygon, grayImage);

			//send results
			publishTrack(polygon, roi);

			//draw results
			drawResults(coloredImage, roi, track);
		}
	}

	imshow(src_window, coloredImage);
	waitKey(1);

}

void Dispatcher::handleObjectTrackRequest(
			const c_tracking::NamedPolygon& polygonMessage)
{
	//if an image does not exist, return
	if (cv_ptr.use_count() == 0)
		return;

	try
	{
		InitializationData data;
		Point2f massCenter;
		getPolygon(polygonMessage, data.polygon, massCenter);

		if (isTrackedObject(data.polygon, massCenter))
			return;

		//setup tracker
		featureExtractor.discriminateKeyPoints(cv_ptr->image, data);
		Track track;
		track.initialize(cv_ptr->image, data);
		track.setLabel(polygonMessage.polygonLabel);
		tracks.push_back(track);

	}
	catch (runtime_error& e)
	{
		ROS_WARN("No Keypoints in the selected object, abort tracking");
	}
}

bool Dispatcher::isTrackedObject(vector<Point2f>& polygon, Point2f& massCenter)
{
	for (vector<Track>::iterator it = tracks.begin(); it != tracks.end(); ++it)
	{
		Track& track = *it;
		if (isSameObject(track, polygon, massCenter))
		{
			return true;
		}
	}

	return false;
}

bool Dispatcher::isSameObject(Track& track, vector<Point2f>& polygon,
			Point2f& massCenter)
{
	if (!track.found())
		return false;

	const vector<Point2f>& trackedPolygon = track.getTrackedPolygon();
	const Point2f& trackedCenter = track.getObjectCenter();

	if (trackedPolygon.size() < 3)
		return false;

	bool insideTrack = pointPolygonTest(trackedPolygon, massCenter, false) >= 0;
	bool insideObject = pointPolygonTest(polygon, trackedCenter, false) >= 0;

	return insideTrack || insideObject;
}

void Dispatcher::getPolygon(const c_tracking::NamedPolygon& polygonMessage,
			vector<Point2f>& polygon, Point2f& massCenter)
{
	const geometry_msgs::Polygon& p = polygonMessage.polygon;

	for (int i = 0; i < p.points.size(); i++)
	{
		geometry_msgs::Point32 point = p.points[i];
		Point2f cp(point.x, point.y);
		polygon.push_back(cp);
		massCenter += cp;
	}

	massCenter *= 1.0 / polygon.size();

	//scale polygon
	for (int i = 0; i < polygon.size(); i++)
	{
		Point2f& cp = polygon[i];
		cp = (1.1 * (cp - massCenter)) + massCenter;
	}
}

void Dispatcher::publishTrack(const std::vector<cv::Point2f>& polygon,
			const cv::Rect& roi)
{
	c_tracking::TrackedObject message;
	for (int i = 0; i < polygon.size(); i++)
	{
		geometry_msgs::Point32 point;
		point.x = polygon[i].x;
		point.y = polygon[i].y;
		point.z = 0;
		message.polygon.points.push_back(point);
	}

	message.roi.x_offset = roi.x;
	message.roi.y_offset = roi.y;
	message.roi.width = roi.width;
	message.roi.height = roi.height;

	trackPublisher.publish(message);
}

Rect Dispatcher::findRoi(const vector<Point2f>& polygon, Mat& image)
{

	const Point2f& p = polygon[1];
	float xMax = 0;
	float yMax = 0;
	float xMin = image.cols;
	float yMin = image.rows;

	for (int i = 0; i < polygon.size(); i++)
	{
		xMax = max(xMax, polygon[i].x);
		yMax = max(yMax, polygon[i].y);
		xMin = min(xMin, polygon[i].x);
		yMin = min(yMin, polygon[i].y);
	}

	float x = max(xMin, 0.0f);
	float y = max(yMin, 0.0f);
	float X = min(xMax, (float) image.cols);
	float Y = min(yMax, (float) image.rows);
	float w = max(0.0f, X - x);
	float h = max(0.0f, Y - y);

	return Rect(x, y, w, h);
}

void Dispatcher::drawResults(Mat& coloredImage, const Rect& roi, Track& track)
{
	const vector<pair<KeyPoint, int> >& trackedKeypoints =
				track.getTrackedKeypoints();
	const vector<pair<KeyPoint, int> >& activeKeypoints =
				track.getActiveKeypoints();
	const vector<Point2f>& polygon = track.getTrackedPolygon();

	rectangle(coloredImage, roi, Scalar(255, 255, 255));
	drawPolygon(coloredImage, polygon, Scalar(255, 255, 255));
	drawKeypoints(coloredImage, trackedKeypoints, Scalar(255, 255, 255));
	drawKeypoints(coloredImage, activeKeypoints, Scalar(255, 0, 0));
	putText(coloredImage, track.getLabel(), track.getObjectCenter(),
				FONT_HERSHEY_SIMPLEX, 0.4, Scalar(255, 255, 255));
}

void Dispatcher::drawPolygon(Mat& frame, const vector<Point2f>& polygon,
			Scalar colour)
{
	for (int i = 0; i + 1 < polygon.size(); i++)
		line(frame, polygon[i], polygon[i + 1], Scalar(255, 255, 255));

	if (polygon.size() > 2)
	{
		line(frame, polygon[0], polygon[polygon.size() - 1],
					Scalar(255, 255, 255));
	}
}

void Dispatcher::drawKeypoints(Mat& frame,
			const vector<pair<KeyPoint, int> >& keypoints, Scalar color)
{
	for (int j = 0; j < keypoints.size(); j++)
		circle(frame, keypoints[j].first.pt, 3, color);
}
