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

#ifndef CMT_H
#define CMT_H

#include <opencv2/opencv.hpp>
#include "InitializationData.h"

class CMT
{

public:
	CMT();
	virtual void initialize(const cv::Mat& im_gray0, InitializationData& data);
	virtual void processFrame(const cv::Mat& im_gray, std::vector<cv::KeyPoint>& keypoints,
				cv::Mat& features);

	inline const std::vector<cv::Point2f>& getTrackedPolygon() const
	{
		return trackedPolygon;
	}

	inline const std::vector<std::pair<cv::KeyPoint, int> >& getTrackedKeypoints() const
	{
		return trackedKeypoints;
	}

	inline const std::vector<std::pair<cv::KeyPoint, int> >& getActiveKeypoints() const
	{
		return activeKeypoints;
	}

	virtual ~CMT();

private:
	void track(const cv::Mat& im_gray, int THR_FB = 20);

	void estimate(cv::Point2f& center, float& scaleEstimate, float& medRot);

	void computeBoundingBox(const cv::Mat& im_gray, const cv::Point2f& center,
				float rotationEstimate, float scaleEstimate);

private:
	//algorithm
	cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;

	//Parameters
	int descriptorLength;
	int thrOutlier;
	float thrConf;
	float thrRatio;
	int minimumKeypontsFraction;

	bool estimateScale;
	bool estimateRotation;

	//Object model data
	cv::Mat selectedFeatures;
	std::vector<int> selectedClasses;
	cv::Mat featuresDatabase;
	std::vector<int> classesDatabase;
	std::vector<cv::Point2f> springs;

	//scale and rotation data
	std::vector<std::vector<float> > squareForm;
	std::vector<std::vector<float> > angles;

	//tracking data
	cv::Mat im_prev;
	std::vector<std::pair<cv::KeyPoint, int> > activeKeypoints;
	std::vector<std::pair<cv::KeyPoint, int> > trackedKeypoints;
	int initialKeypointsNumber;

	//Polygon coordinates
	std::vector<cv::Point2f> relativePolygon;
	std::vector<cv::Point2f> trackedPolygon;

	//Probably useless
	std::vector<std::pair<cv::KeyPoint, int> > outliers;
};

#endif // CMT_H
