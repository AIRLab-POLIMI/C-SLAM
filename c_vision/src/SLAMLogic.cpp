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

#include "MetricRectification.h"

using namespace ros;
using namespace sensor_msgs;
using namespace image_geometry;
using namespace cv_bridge;
using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

SLAMLogic::SLAMLogic(NodeHandle n, ParameterServer& parameters) :
			BaseLogic(n, parameters), infoCache(15), imageCache(15),
			detector(parameters), viewer("ROI")
{
	cameraSubscriber = it.subscribeCamera("/ardrone/image_rect_color", 1,
				&SLAMLogic::handleCamera, this);
	trackSubscriber = n.subscribe("tracks", 10, &SLAMLogic::handleTrack, this);

	namedWindow("rectified");
}

void SLAMLogic::handleCamera(const ImageConstPtr& msg,
			const CameraInfoConstPtr& info_msg)
{
	camera_frame_id = msg->header.frame_id;
	imageCache.add(msg);
	infoCache.add(info_msg);
}

void SLAMLogic::handleTrack(const c_tracking::TrackedObject& track)
{
	Mat objectImage, mask;
	Rect roi;
	CvImagePtr cv_ptr, cv_ptr_color;
	PinholeCameraModel cameraModel;

	try
	{
		getImageData(track, cv_ptr, cv_ptr_color, cameraModel);
		getRoi(track, cv_ptr_color->image, roi, objectImage, mask);
		detect(objectImage, mask);
		classify(cameraModel);
		display(objectImage);
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

void SLAMLogic::detect(Mat& image, Mat& mask)
{
	Mat greyFrame;

	detector.setRoll(roll);
	detector.detect(image, mask);
}

void SLAMLogic::classify(PinholeCameraModel& cameraModel)
{
	c_fuzzy::Classification serviceCall;
	ObjectClassificator classificator(serviceCall, classifierParam);
	classificator.processFeatures(detector.getRectangles(), R);
	classificator.processFeatures(detector.getPoles(), R);
	classificator.processFeatures(detector.getClusters(), R);

	callClassificationService(serviceCall);

	classificator.labelFeatures();

	rectify(classificator, cameraModel);

}

void SLAMLogic::rectify(ObjectClassificator& classificator,
			PinholeCameraModel& cameraModel)
{
	const vector<pair<vector<Point>, string> >& features =
				classificator.getGoodFeatures();

	vector<vector<Point> > rectifiedvector;

	for (vector<pair<vector<Point>, string> >::const_iterator it =
				features.begin(); it != features.end(); ++it)
	{

		if (it->second != "Handle")
		{
			const vector<Point>& quadrilateral = it->first;
			const Point& x = quadrilateral[0];
			const Point& y = quadrilateral[1];
			const Point& z = quadrilateral[2];
			const Point& w = quadrilateral[3];

			Vec3d h1, h2, v1, v2;
			metric_rectification::findLine(x, y, h1);
			metric_rectification::findLine(z, w, h2);
			metric_rectification::findLine(x, w, v1);
			metric_rectification::findLine(y, z, v2);

			Vec3d van1 = h1.cross(h2);
			van1 = van1 / norm(van1);
			Vec3d van2 = v1.cross(v2);
			van2 = van2 / norm(van2);

			Mat H = metric_rectification::metricRectify(
						cameraModel.fullIntrinsicMatrix(), van1, van2);

			vector<Point2f> old;

			for (int i = 0; i < it->first.size(); i++)
			{
				old.push_back(it->first[i]);
			}

			vector<Point2f> rectified;

			perspectiveTransform(old, rectified, H);

			Point2f originP = rectified[0];
			Point2f verticalP = rectified[3];
			Vec3d origin(originP.x, originP.y, 1);
			Vec3d vertical(verticalP.x, verticalP.y, 1);

			Mat H2 = metric_rectification::getScaleTranslationAndRotation(
						origin, vertical, 100);

			vector<Point2f> rotatedAndScaled;

			perspectiveTransform(rectified, rotatedAndScaled, H2);

			vector<Point> newVec;

			for (int i = 0; i < rotatedAndScaled.size(); i++)
			{
				newVec.push_back(rotatedAndScaled[i]);
			}

			rectifiedvector.push_back(newVec);
		}
	}

	Mat rectifiedFrame(150, 600, CV_8UC3);

	rectifiedFrame.setTo(Scalar(0, 0, 0));

	drawContours(rectifiedFrame, rectifiedvector, -1, Scalar(0, 255, 0));

	imshow("rectified", rectifiedFrame);
}

void SLAMLogic::display(Mat& image)
{
	viewer.setRectangles(detector.getRectangles());
	viewer.setPoles(detector.getPoles());
	viewer.setClusters(detector.getClusters());
	viewer.setRoll(roll);
	viewer.display(image);

	detector.deleteDetections();
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
			Rect& roi, Mat& image, Mat& mask)
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
	image = input(roi);

	//create the mask
	mask = Mat(image.size(), CV_8UC1);
	mask.setTo(0);
	fillConvexPoly(mask, polygon, 255, 8, 0);
}
