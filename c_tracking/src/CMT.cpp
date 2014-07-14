/*
 * Copyright (c) 2014, delmottea, Davide Tateo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the {organization} nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "CMT.h"

#include <cmath>
#include <angles/angles.h>

#include "CMTUtils.h"

using namespace std;
using namespace cv;

CMTFeatureExtractor::CMTFeatureExtractor()
{
	std::string detectorType = "Feature2D.BRISK";
	std::string descriptorType = "Feature2D.BRISK";

	//Initialise detector, descriptor, matcher
	detector = Algorithm::create<FeatureDetector>(detectorType.c_str());
	descriptorExtractor = Algorithm::create<DescriptorExtractor>(
				descriptorType.c_str());
}

void CMTFeatureExtractor::detect(Mat im_gray)
{
	detector->detect(im_gray, keypoints);
	descriptorExtractor->compute(im_gray, keypoints, features);
}

void CMTFeatureExtractor::discriminateKeyPoints(Mat im_gray,
			InitializationData& data)
{
	//Remember keypoints that are in the rectangle as selected keypoints
	inout_rect(keypoints, data.topleft, data.bottomright,
				data.selected_keypoints, data.background_keypoints);
	descriptorExtractor->compute(im_gray, data.selected_keypoints,
				data.selected_features);

	if (data.selected_keypoints.size() == 0)
	{
		//TODO eccezione
		return;
	}

	//Remember keypoints that are not in the rectangle as background keypoints
	descriptorExtractor->compute(im_gray, data.background_keypoints,
				data.background_features);
}

void CMTFeatureExtractor::inout_rect(const vector<KeyPoint>& keypoints,
			Point2f topleft, Point2f bottomright, vector<KeyPoint>& in,
			vector<KeyPoint>& out)
{
	for (int i = 0; i < keypoints.size(); i++)
	{
		if (keypoints[i].pt.x > topleft.x && keypoints[i].pt.y > topleft.y
					&& keypoints[i].pt.x < bottomright.x
					&& keypoints[i].pt.y < bottomright.y)
			in.push_back(keypoints[i]);
		else
			out.push_back(keypoints[i]);
	}
}

CMT::CMT()
{
	thrOutlier = 20;
	thrConf = 0.75;
	thrRatio = 0.8;
	descriptorLength = 512;
	estimateScale = true;
	estimateRotation = true;
	nbInitialKeypoints = 0;

	//initialize matcher
	std::string matcherType = "BruteForce-Hamming";
	descriptorMatcher = DescriptorMatcher::create(matcherType.c_str());
}

void CMT::initialize(Mat im_gray0, InitializationData& data)
{
	//get the keypoints
	vector<KeyPoint>& selected_keypoints = data.selected_keypoints;
	vector<KeyPoint>& background_keypoints = data.background_keypoints;

	//get the features
	selectedFeatures = data.selected_features;
	Mat background_features = data.background_features;

	//Assign each keypoint a class starting from 1, background is 0
	selectedClasses = vector<int>();
	for (int i = 1; i <= selected_keypoints.size(); i++)
		selectedClasses.push_back(i);
	vector<int> backgroundClasses;
	for (int i = 0; i < background_keypoints.size(); i++)
		backgroundClasses.push_back(0);

	//Stack background features and selected features into database
	featuresDatabase = Mat(background_features.rows + selectedFeatures.rows,
				max(background_features.cols, selectedFeatures.cols),
				background_features.type());
	if (background_features.cols > 0)
		background_features.copyTo(
					featuresDatabase(
								Rect(0, 0, background_features.cols,
											background_features.rows)));
	if (selectedFeatures.cols > 0)
		selectedFeatures.copyTo(
					featuresDatabase(
								Rect(0, background_features.rows,
											selectedFeatures.cols,
											selectedFeatures.rows)));

	//Same for classes
	classesDatabase = vector<int>();
	for (int i = 0; i < backgroundClasses.size(); i++)
		classesDatabase.push_back(backgroundClasses[i]);
	for (int i = 0; i < selectedClasses.size(); i++)
		classesDatabase.push_back(selectedClasses[i]);

	//Get all distances between selected keypoints in squareform and get all angles between selected keypoints
	squareForm = vector<vector<float> >();
	angles = vector<vector<float> >();
	for (int i = 0; i < selected_keypoints.size(); i++)
	{
		vector<float> lineSquare;
		vector<float> lineAngle;
		for (int j = 0; j < selected_keypoints.size(); j++)
		{
			float dx = selected_keypoints[j].pt.x - selected_keypoints[i].pt.x;
			float dy = selected_keypoints[j].pt.y - selected_keypoints[i].pt.y;
			lineSquare.push_back(sqrt(dx * dx + dy * dy));
			lineAngle.push_back(atan2(dy, dx));
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
	Point2f& topleft = data.topleft;
	Point2f& bottomright = data.bottomright;

	centerToTopLeft = topleft - center;
	centerToTopRight = Point2f(bottomright.x, topleft.y) - center;
	centerToBottomRight = bottomright - center;
	centerToBottomLeft = Point2f(topleft.x, bottomright.y) - center;

	//Calculate springs of each keypoint
	springs = vector<Point2f>();
	for (int i = 0; i < selected_keypoints.size(); i++)
		springs.push_back(selected_keypoints[i].pt - center);

	//Set start image for tracking
	im_prev = im_gray0.clone();

	//Make keypoints 'active' keypoints
	activeKeypoints = vector<pair<KeyPoint, int> >();
	for (int i = 0; i < selected_keypoints.size(); i++)
		activeKeypoints.push_back(
					make_pair(selected_keypoints[i], selectedClasses[i]));

	//Remember number of initial keypoints
	nbInitialKeypoints = selected_keypoints.size();
}

void CMT::processFrame(Mat im_gray, vector<KeyPoint>& keypoints, Mat& features)
{
	trackedKeypoints = vector<pair<KeyPoint, int> >();
	vector<unsigned char> status;
	track(im_gray, activeKeypoints, trackedKeypoints, status);

	Point2f center;
	float scaleEstimate;
	float rotationEstimate;
	vector<pair<KeyPoint, int> > trackedKeypoints2;
	estimate(trackedKeypoints, center, scaleEstimate, rotationEstimate,
				trackedKeypoints2);
	trackedKeypoints = trackedKeypoints2;

	//Create list of active keypoints
	activeKeypoints = vector<pair<KeyPoint, int> >();

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
		vector<PairFloat> sorted_conf;
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
			vector<PairFloat> sorted_conf;
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
			vector<bool> notmissing = in1d(tracked_classes, associated_classes);
			for (int i = 0; i < trackedKeypoints.size(); i++)
				if (!notmissing[i])
					activeKeypoints.push_back(trackedKeypoints[i]);
		}
		else
			activeKeypoints = trackedKeypoints;
	}

	//Update object state estimate
	vector<pair<KeyPoint, int> > activeKeypointsBefore = activeKeypoints;
	im_prev = im_gray;
	topLeft = Point2f(NAN, NAN);
	topRight = Point2f(NAN, NAN);
	bottomLeft = Point2f(NAN, NAN);
	bottomRight = Point2f(NAN, NAN);

	boundingbox = Rect_<float>(NAN, NAN, NAN, NAN);

	if (!(isnan(center.x) | isnan(center.y))
				&& activeKeypoints.size() > nbInitialKeypoints / 10)
	{

		topLeft = center
					+ scaleEstimate * rotate(centerToTopLeft, rotationEstimate);
		topRight = center
					+ scaleEstimate
								* rotate(centerToTopRight, rotationEstimate);
		bottomLeft = center
					+ scaleEstimate
								* rotate(centerToBottomLeft, rotationEstimate);
		bottomRight = center
					+ scaleEstimate
								* rotate(centerToBottomRight, rotationEstimate);

		float minx = min(min(topLeft.x, topRight.x),
					min(bottomRight.x, bottomLeft.x));
		float miny = min(min(topLeft.y, topRight.y),
					min(bottomRight.y, bottomLeft.y));
		float maxx = max(max(topLeft.x, topRight.x),
					max(bottomRight.x, bottomLeft.x));
		float maxy = max(max(topLeft.y, topRight.y),
					max(bottomRight.y, bottomLeft.y));

		boundingbox = Rect_<float>(minx, miny, maxx - minx, maxy - miny);
	}
}
void CMT::estimate(const vector<pair<KeyPoint, int> >& keypointsIN,
			Point2f& center, float& scaleEstimate, float& medRot,
			vector<pair<KeyPoint, int> >& keypoints)
{
	center = Point2f(NAN, NAN);
	scaleEstimate = NAN;
	medRot = NAN;

	//At least 2 keypoints are needed for scale
	if (keypointsIN.size() > 1)
	{
		//sort
		vector<PairInt> list;
		for (int i = 0; i < keypointsIN.size(); i++)
			list.push_back(make_pair(keypointsIN[i].second, i));
		sort(&list[0], &list[0] + list.size(), comparatorPair<int>);
		for (int i = 0; i < list.size(); i++)
			keypoints.push_back(keypointsIN[list[i].second]);

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
				float dist = sqrt(p.dot(p));
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
			votes = vector<Point2f>();
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
			keypoints = newKeypoints;

			center = Point2f(0, 0);
			for (int i = 0; i < newVotes.size(); i++)
				center += newVotes[i];
			center *= (1.0 / newVotes.size());
		}
	}
}

void CMT::track(Mat im_gray, const vector<pair<KeyPoint, int> >& keypointsIN,
			vector<pair<KeyPoint, int> >& keypointsTracked,
			vector<unsigned char>& status, int THR_FB)
{
	//Status of tracked keypoint - True means successfully tracked
	status = vector<unsigned char>();

	//If at least one keypoint is active
	if (keypointsIN.size() > 0)
	{
		vector<Point2f> pts;
		vector<Point2f> pts_back;
		vector<Point2f> nextPts;
		vector<unsigned char> status_back;
		vector<float> err;
		vector<float> err_back;
		vector<float> fb_err;
		for (int i = 0; i < keypointsIN.size(); i++)
			pts.push_back(
						Point2f(keypointsIN[i].first.pt.x,
									keypointsIN[i].first.pt.y));

		//Calculate forward optical flow for prev_location
		calcOpticalFlowPyrLK(im_prev, im_gray, pts, nextPts, status, err);
		//Calculate backward optical flow for prev_location
		calcOpticalFlowPyrLK(im_gray, im_prev, nextPts, pts_back, status_back,
					err_back);

		//Calculate forward-backward error
		for (int i = 0; i < pts.size(); i++)
		{
			Point2f v = pts_back[i] - pts[i];
			fb_err.push_back(sqrt(v.dot(v)));
		}

		//Set status depending on fb_err and lk error
		for (int i = 0; i < status.size(); i++)
			status[i] = fb_err[i] <= THR_FB & status[i];

		keypointsTracked = vector<pair<KeyPoint, int> >();
		for (int i = 0; i < pts.size(); i++)
		{
			pair<KeyPoint, int> p = keypointsIN[i];
			if (status[i])
				p.first.pt = nextPts[i];
			keypointsTracked.push_back(p);
		}
	}
	else
		keypointsTracked = vector<pair<KeyPoint, int> >();
}

