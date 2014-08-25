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

#include "MappingTracker.h"

#include "opencv3.h"

using namespace std;
using namespace cv;

MappingTracker::MappingTracker()
{
	objectMapped = false;

	minDistance = 120.0;
}

void MappingTracker::initialize(const Mat& im_gray0, InitializationData& data)
{
	mappedKeyPoints = data.selected_keypoints;
	CMT::initialize(im_gray0, data);
}

void MappingTracker::mapObject(Mat& image, const Mat_<double>& K,
			RobotPose& pose, WorldMap& map)
{

	const vector<pair<KeyPoint, int> >& matches = this->getActiveKeypoints();

	size_t size = matches.size();

	if (objectMapped && size > 4)
	{
		localizeFromObject(K, pose);
	}
	else
	{

		if (size > 8)
		{
			vector<Point2f> points1;
			vector<Point2f> points2;

			double averageDistance = matchKeyPoints(matches, points1, points2,
						image);
			if (averageDistance > minDistance)
			{
				reconstructPoints(map, matches, K, pose, points1, points2);
				objectMapped = true;
			}

		}

	}
}

void MappingTracker::localizeFromObject(const Mat_<double>& K, RobotPose& pose)
{
	if (found())
	{
		const vector<pair<KeyPoint, int> >& matches =
					this->getActiveKeypoints();
		vector<Point3f> objectPoints;
		vector<Point2f> imagePoints;

		matchReconstructed(matches, imagePoints, objectPoints);

		cv::Mat rvec, t;
		solvePnPRansac(objectPoints, imagePoints, K, Mat(), rvec, t);

		cv::Mat R;
		cv::Rodrigues(rvec, R); // R is 3x3

		R = R.t();  // rotation of inverse
		t = -R * t; // translation of inverse

		pose.addObjectPose(R, t);
	}
}

void MappingTracker::matchReconstructed(
			const vector<pair<KeyPoint, int> >& matches,
			vector<Point2f>& imagePoints, vector<Point3f>& objectPoints)
{
	for (int i = 0; i < matches.size(); i++)
	{
		int index = matches[i].second - 1;
		if (reconstructedMap.count(index))
		{
			imagePoints.push_back(matches[i].first.pt);
			objectPoints.push_back(reconstructedMap[index]);
		}
	}
}

double MappingTracker::matchKeyPoints(
			const vector<pair<KeyPoint, int> >& matches,
			vector<Point2f>& points1, vector<Point2f>& points2, Mat& image)
{
	double averageDistance = 0;

	size_t size = matches.size();
	for (int i = 0; i < size; i++)
	{
		int matchIndex = matches[i].second - 1; //class 0 is background
		const Point2f& matchPoint = matches[i].first.pt;
		const Point2f& basePoint = mappedKeyPoints[matchIndex].pt;
		points1.push_back(basePoint);
		points2.push_back(matchPoint);
		line(image, matchPoint, basePoint, Scalar(0, 0, 255));
		averageDistance += norm(matchPoint - basePoint);
	}

	return averageDistance / size;

}

void MappingTracker::reconstructPoints(WorldMap& map,
			const vector<pair<KeyPoint, int> >& matches, const Mat_<double>& K,
			RobotPose pose, const vector<Point2f>& points1,
			const vector<Point2f>& points2)
{
	vector<Point2f> normalizedPoints1;
	vector<Point2f> normalizedPoints2;
	Mat outlierMask;

	//compute essential matrix
	Mat F = findFundamentalMat(points1, points2, CV_FM_LMEDS, 1., 0.99,
				outlierMask);
	Mat E = K.t() * F * K;

	//compute normalized points
	undistortPoints(points1, normalizedPoints1, K, Mat());
	undistortPoints(points2, normalizedPoints2, K, Mat());
	//find relative pose
	Mat R;
	Mat t;
	cv3::recoverPose(E, normalizedPoints1, normalizedPoints2, R, t, 1.0,
				Point2d(0, 0), outlierMask);

	//compute two camera matrices
	Mat P0, P1;
	pose.computeCameraMatrices(P0, P1, R, t);

	//Find the 3d shape of object keypoints
	Mat_<float> points3D;
	triangulatePoints(P0, P1, normalizedPoints1, normalizedPoints2, points3D);
	points3D.row(0) /= points3D.row(3);
	points3D.row(1) /= points3D.row(3);
	points3D.row(2) /= points3D.row(3);
	points3D.row(3) /= points3D.row(3);

	vector<Point3d> pointCloud;

	for (int i = 0; i < matches.size(); i++)
	{
		if (outlierMask.at<int>(i, 0))
		{
			int index = matches[i].second - 1;
			Point3d p(points3D(0, i), points3D(1, i), points3D(2, i));
			reconstructedMap[index] = p;
			pointCloud.push_back(p);
		}
	}

	map.addObject(pointCloud);

}

