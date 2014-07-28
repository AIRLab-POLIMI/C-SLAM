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
 *
 *  based on https://github.com/delmottea/libCMT/
 */

#include "CMT.h"

#include <stdexcept>
#include <angles/angles.h>

#include "CMTUtils.h"

using namespace std;
using namespace cv;

CMT::CMT()
{
	descriptorLength = 512;
	thrOutlier = 20;
	thrConf = 0.75;
	thrRatio = 0.8;
	minimumKeypontsFraction = 3;

	estimateScale = true;
	estimateRotation = true;

	initialKeypointsNumber = 0;

	//initialize matcher
	std::string matcherType = "BruteForce-Hamming";
	descriptorMatcher = DescriptorMatcher::create(matcherType);
}

void CMT::initialize(Mat im_gray0, InitializationData& data)
{
	//get the keypoints
	vector<KeyPoint>& selected_keypoints = data.selected_keypoints;
	vector<KeyPoint>& background_keypoints = data.background_keypoints;

	//get the features
	selectedFeatures = data.selected_features;
	Mat& background_features = data.background_features;

	//Assign each keypoint a class starting from 1 (background is 0)
	selectedClasses = vector<int>();
	for (int i = 1; i <= selected_keypoints.size(); i++)
		selectedClasses.push_back(i);

	//Stack background features and selected features into database
	featuresDatabase = Mat(0,
				max(background_features.cols, selectedFeatures.cols),
				background_features.type());
	featuresDatabase.push_back(background_features);
	featuresDatabase.push_back(selectedFeatures);

	//Same for classes
	classesDatabase.resize(background_keypoints.size(), 0);
	classesDatabase.insert(classesDatabase.end(), selectedClasses.begin(),
				selectedClasses.end());

	//Get all distances between selected keypoints in squareform and get all angles between selected keypoints
	for (int i = 0; i < selected_keypoints.size(); i++)
	{
		vector<float> lineSquare;
		vector<float> lineAngle;
		for (int j = 0; j < selected_keypoints.size(); j++)
		{
			Point2f delta = selected_keypoints[j].pt - selected_keypoints[i].pt;
			lineSquare.push_back(norm(delta));
			lineAngle.push_back(atan2(delta.y, delta.x));
		}
		squareForm.push_back(lineSquare);
		angles.push_back(lineAngle);
	}

	//Find the center of selected keypoints
	Point2f center(0, 0);
	for (int i = 0; i < selected_keypoints.size(); i++)
		center += selected_keypoints[i].pt;
	center *= (1.0 / selected_keypoints.size());

	//Remember the rectangle coordinates relative to the center
	vector<Point2f>& polygon = data.polygon;

	for (int i = 0; i < polygon.size(); i++)
		relativePolygon.push_back(polygon[i] - center);

	//Calculate springs of each keypoint
	springs = vector<Point2f>();
	for (int i = 0; i < selected_keypoints.size(); i++)
		springs.push_back(selected_keypoints[i].pt - center);

	//Set start image for tracking
	im_prev = im_gray0.clone();

	//Make keypoints 'active' keypoints
	for (int i = 0; i < selected_keypoints.size(); i++)
		activeKeypoints.push_back(
					make_pair(selected_keypoints[i], selectedClasses[i]));

	//Remember number of initial keypoints
	initialKeypointsNumber = selected_keypoints.size();
}

void CMT::processFrame(Mat im_gray, vector<KeyPoint>& keypoints, Mat& features)
{
	track(im_gray);


	//estimate center, sclae and rotation
	Point2f center;
	float scaleEstimate;
	float rotationEstimate;

	estimate(center, scaleEstimate, rotationEstimate);

	//Create list of active keypoints
	activeKeypoints.clear();

	//For each keypoint and its descriptor
	for (int i = 0; i < keypoints.size(); i++)
	{
		KeyPoint keypoint = keypoints[i];

		//First: Match over whole image
		//Compute distances to all descriptors
		vector<DMatch> matches;
		descriptorMatcher->match(featuresDatabase, features.row(i), matches);

		//Convert distances to confidences, do not weight
		vector<float> combined;
		for (int j = 0; j < matches.size(); j++)
			combined.push_back(1 - matches[j].distance / descriptorLength);

		vector<int>& classes = classesDatabase;

		//Sort in descending order
		vector<pair<float, int> > sorted_conf;
		for (int j = 0; j < combined.size(); j++)
			sorted_conf.push_back(make_pair(combined[j], j));
		sort(&sorted_conf[0], &sorted_conf[0] + sorted_conf.size(),
					comparatorPairDesc<float>);

		//Get best and second best index
		int bestInd = sorted_conf[0].second;
		int secondBestInd = sorted_conf[1].second;

		//Compute distance ratio according to Lowe
		float ratio = (1 - combined[bestInd]) / (1 - combined[secondBestInd]);

		//Extract class of best match
		int keypoint_class = classes[bestInd];

		//If distance ratio is ok and absolute distance is ok and keypoint class is not background
		if (ratio < thrRatio && combined[bestInd] > thrConf
					&& keypoint_class != 0)
			activeKeypoints.push_back(make_pair(keypoint, keypoint_class));

		//In a second step, try to match difficult keypoints
		//If structural constraints are applicable
		if (!(isnan(center.x) | isnan(center.y)))
		{
			//Compute distances to initial descriptors
			vector<DMatch> matches;
			descriptorMatcher->match(selectedFeatures, features.row(i),
						matches);

			//Convert distances to confidences
			vector<float> confidences;
			for (int i = 0; i < matches.size(); i++)
				confidences.push_back(
							1 - matches[i].distance / descriptorLength);

			//Compute the keypoint location relative to the object center
			Point2f relative_location = keypoint.pt - center;

			//Compute the distances to all springs
			vector<float> displacements;
			for (int i = 0; i < springs.size(); i++)
			{
				Point2f p = (scaleEstimate
							* rotate(springs[i], -rotationEstimate)
							- relative_location);
				displacements.push_back(sqrt(p.dot(p)));
			}

			//For each spring, calculate weight
			vector<float> combined;
			for (int i = 0; i < confidences.size(); i++)
				combined.push_back(
							(displacements[i] < thrOutlier) * confidences[i]);

			vector<int>& classes = selectedClasses;

			//Sort in descending order
			vector<pair<float, int> > sorted_conf;
			for (int i = 0; i < combined.size(); i++)
				sorted_conf.push_back(make_pair(combined[i], i));
			sort(&sorted_conf[0], &sorted_conf[0] + sorted_conf.size(),
						comparatorPairDesc<float>);

			//Get best and second best index
			int bestInd = sorted_conf[0].second;
			int secondBestInd = sorted_conf[1].second;

			//Compute distance ratio according to Lowe
			float ratio = (1 - combined[bestInd])
						/ (1 - combined[secondBestInd]);

			//Extract class of best match
			int keypoint_class = classes[bestInd];

			//If distance ratio is ok and absolute distance is ok and keypoint class is not background
			if (ratio < thrRatio && combined[bestInd] > thrConf
						&& keypoint_class != 0)
			{
				for (int i = activeKeypoints.size() - 1; i >= 0; i--)
					if (activeKeypoints[i].second == keypoint_class)
						activeKeypoints.erase(activeKeypoints.begin() + i);
				activeKeypoints.push_back(make_pair(keypoint, keypoint_class));
			}
		}
	}

	//If some keypoints have been tracked
	if (trackedKeypoints.size() > 0)
	{
		//Extract the keypoint classes
		vector<int> tracked_classes;
		for (int i = 0; i < trackedKeypoints.size(); i++)
			tracked_classes.push_back(trackedKeypoints[i].second);
		//If there already are some active keypoints
		if (activeKeypoints.size() > 0)
		{
			//Add all tracked keypoints that have not been matched
			vector<int> associated_classes;
			for (int i = 0; i < activeKeypoints.size(); i++)
				associated_classes.push_back(activeKeypoints[i].second);
			const vector<bool>& notmissing = in1d(tracked_classes,
						associated_classes);
			for (int i = 0; i < trackedKeypoints.size(); i++)
				if (!notmissing[i])
					activeKeypoints.push_back(trackedKeypoints[i]);
		}
		else
			activeKeypoints = trackedKeypoints;
	}

	//Update object state estimate
	computeBoundingBox(im_gray, center, rotationEstimate, scaleEstimate);
}

CMT::~CMT()
{
}

void CMT::computeBoundingBox(const Mat& im_gray, const Point2f& center,
			float rotationEstimate, float scaleEstimate)
{
	//Update object state estimate
	vector<pair<KeyPoint, int> > activeKeypointsBefore = activeKeypoints;
	im_prev = im_gray;
	trackedPolygon.clear();
	if (!(isnan(center.x) | isnan(center.y))
				&& activeKeypoints.size()
							> initialKeypointsNumber / minimumKeypontsFraction)
	{
		for (int i = 0; i < relativePolygon.size(); i++)
		{
			Point2f point = center
						+ scaleEstimate
									* rotate(relativePolygon[i],
												rotationEstimate);
			trackedPolygon.push_back(point);
		}
	}
}

void CMT::estimate(Point2f& center, float& scaleEstimate, float& medRot)
{
	center = Point2f(NAN, NAN);
	scaleEstimate = NAN;
	medRot = NAN;

	vector<pair<KeyPoint, int> > keypoints;

	//At least 2 keypoints are needed for scale
	if (trackedKeypoints.size() > 1)
	{
		//sort
		vector<pair<int, int> > list;
		for (int i = 0; i < trackedKeypoints.size(); i++)
			list.push_back(make_pair(trackedKeypoints[i].second, i));
		sort(&list[0], &list[0] + list.size(), comparatorPair<int>);
		for (int i = 0; i < list.size(); i++)
			keypoints.push_back(trackedKeypoints[list[i].second]);

		vector<int> ind1;
		vector<int> ind2;
		for (int i = 0; i < list.size(); i++)
			for (int j = 0; j < list.size(); j++)
			{
				if (i != j && keypoints[i].second != keypoints[j].second)
				{
					ind1.push_back(i);
					ind2.push_back(j);
				}
			}

		if (ind1.size() > 0)
		{
			vector<int> class_ind1;
			vector<int> class_ind2;
			vector<KeyPoint> pts_ind1;
			vector<KeyPoint> pts_ind2;
			for (int i = 0; i < ind1.size(); i++)
			{
				class_ind1.push_back(keypoints[ind1[i]].second - 1);
				class_ind2.push_back(keypoints[ind2[i]].second - 1);
				pts_ind1.push_back(keypoints[ind1[i]].first);
				pts_ind2.push_back(keypoints[ind2[i]].first);
			}

			vector<float> scaleChange;
			vector<float> angleDiffs;
			for (int i = 0; i < pts_ind1.size(); i++)
			{
				Point2f p = pts_ind2[i].pt - pts_ind1[i].pt;
				//This distance might be 0 for some combinations,
				//as it can happen that there is more than one keypoint at a single location
				float dist = norm(p);
				float origDist = squareForm[class_ind1[i]][class_ind2[i]];
				scaleChange.push_back(dist / origDist);
				//Compute angle
				float angle = atan2(p.y, p.x);
				float origAngle = angles[class_ind1[i]][class_ind2[i]];
				float angleDiff = angles::shortest_angular_distance(origAngle,
							angle);
				angleDiffs.push_back(angleDiff);
			}
			scaleEstimate = median(scaleChange);
			if (!estimateScale)
				scaleEstimate = 1;
			medRot = median(angleDiffs);
			if (!estimateRotation)
				medRot = 0;

			std::vector<cv::Point2f> votes;
			for (int i = 0; i < keypoints.size(); i++)
				votes.push_back(
							keypoints[i].first.pt
										- scaleEstimate
													* rotate(
																springs[keypoints[i].second
																			- 1],
																medRot));
			//Compute linkage between pairwise distances
			vector<Cluster> linkageData = linkage(votes);

			//Perform hierarchical distance-based clustering
			vector<int> T = fcluster(linkageData, thrOutlier);
			//Count votes for each cluster
			vector<int> cnt = binCount(T);
			//Get largest class
			int Cmax = argmax(cnt);

			//Remember outliers
			outliers = vector<pair<KeyPoint, int> >();
			vector<pair<KeyPoint, int> > newKeypoints;
			vector<Point2f> newVotes;
			for (int i = 0; i < keypoints.size(); i++)
			{
				if (T[i] != Cmax)
					outliers.push_back(keypoints[i]);
				else
				{
					newKeypoints.push_back(keypoints[i]);
					newVotes.push_back(votes[i]);
				}
			}

			trackedKeypoints = newKeypoints;

			center = Point2f(0, 0);
			for (int i = 0; i < newVotes.size(); i++)
				center += newVotes[i];

			center *= 1.0 / newVotes.size();
		}
	}
}

void CMT::track(Mat im_gray, int THR_FB)
{

	trackedKeypoints.clear();

	//Status of tracked keypoint - True means successfully tracked
	vector<unsigned char> status;

	//If at least one keypoint is active
	if (activeKeypoints.size() > 0)
	{
		vector<Point2f> pts;
		vector<Point2f> pts_back;
		vector<Point2f> nextPts;
		vector<unsigned char> status_back;
		vector<float> err;
		vector<float> err_back;
		vector<float> fb_err;
		for (int i = 0; i < activeKeypoints.size(); i++)
			pts.push_back(
						Point2f(activeKeypoints[i].first.pt.x,
									activeKeypoints[i].first.pt.y));

		//Calculate forward optical flow for prev_location
		calcOpticalFlowPyrLK(im_prev, im_gray, pts, nextPts, status, err);
		//Calculate backward optical flow for prev_location
		calcOpticalFlowPyrLK(im_gray, im_prev, nextPts, pts_back, status_back,
					err_back);

		//Calculate forward-backward error
		for (int i = 0; i < pts.size(); i++)
		{
			fb_err.push_back(norm(pts_back[i] - pts[i]));
		}

		//Set status depending on fb_err and lk error
		for (int i = 0; i < status.size(); i++)
			status[i] = fb_err[i] <= THR_FB & status[i];

		for (int i = 0; i < pts.size(); i++)
		{
			pair<KeyPoint, int> p = activeKeypoints[i];
			if (status[i])
				p.first.pt = nextPts[i];
			trackedKeypoints.push_back(p);
		}
	}

}

