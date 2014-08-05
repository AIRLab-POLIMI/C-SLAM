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
			n(n), it(n), map(n)
{
	navdataSubscriber = n.subscribe("/ardrone/navdata", 1,
				&Dispatcher::handleNavdata, this);
	toTrackSubscriber = n.subscribe("to_track", 1,
				&Dispatcher::handleObjectTrackRequest, this);
	trackPublisher = n.advertise<c_tracking::TrackedObject>("tracks", 1000);
	imageSubscriber = it.subscribeCamera("/ardrone/image_rect_color", 1,
				&Dispatcher::handleImage, this);
	rotX = rotY = rotZ = 0;
	src_window = "Cognitive Tracking";

	namedWindow(src_window, CV_WINDOW_AUTOSIZE);
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

void Dispatcher::handleImage(const sensor_msgs::ImageConstPtr& msg,
			const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	cameraModel.fromCameraInfo(info_msg);

	double currentRot = rotX;
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
		CMT& track = tracks[i];
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

	pose.updateRobotPose(cameraModel.tfFrame());

	imshow(src_window, coloredImage);
	waitKey(1);

}

void Dispatcher::handleObjectTrackRequest(
			const c_tracking::NamedPolygon& polygonMessage)
{
	try
	{
		InitializationData data;
		getPolygon(polygonMessage, data.polygon);
		featureExtractor.discriminateKeyPoints(cv_ptr->image, data);

		//setup tracker
		CMT cmt;
		cmt.initialize(cv_ptr->image, data);
		tracks.push_back(cmt);

	}
	catch (runtime_error& e)
	{
		ROS_WARN("No Keypoints in the selected object, abort tracking");
	}
}

void Dispatcher::getPolygon(const c_tracking::NamedPolygon& polygonMessage,
			vector<Point2f>& polygon)
{
	const geometry_msgs::Polygon& p = polygonMessage.polygon;
	for (int i = 0; i < p.points.size(); i++)
	{
		geometry_msgs::Point32 point = p.points[i];
		polygon.push_back(Point2f(point.x, point.y));
	}
}

void Dispatcher::publishTrack(const std::vector<cv::Point2f>& polygon, const cv::Rect& roi)
{
	c_tracking::TrackedObject message;
	for(int i = 0; i < polygon.size(); i++)
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

void Dispatcher::drawResults(Mat& coloredImage, const Rect& roi, CMT& track)
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

/*void Dispatcher::computeVertices(Mat& objectImage)
 {
 if (objectImage.rows * objectImage.cols > 0)
 {
 Mat canny;

 vector<vector<Point> > contours;
 vector<Vec4i> hierarchy;

 double high_thres = 0.25*threshold(objectImage, canny, 0, 255,
 CV_THRESH_BINARY + CV_THRESH_OTSU);
 double low_thres = high_thres * 0.5;

 Canny(objectImage, canny, low_thres, high_thres, 3, true);

 Mat colored;
 cvtColor(canny, colored, CV_GRAY2BGR);

 vector<Vec4i> lines;

 HoughLinesP(canny, lines, 1, CV_PI / 180, 20, 40, 10);

 for (size_t i = 0; i < lines.size(); i++)
 {
 line(colored, Point(lines[i][0], lines[i][1]),
 Point(lines[i][2], lines[i][3]),
 Scalar(0, 0, 255), 3, 8);
 }

 for (int i = 0; i < contours.size(); i++)
 {
 drawContours(colored, contours, i, Scalar(255, 0, 0), 2, 8,
 hierarchy, 0, Point());
 }
 }
 }*/
