/*
 * Copyright (c) 2014, delmottea
 * Copyright (c) 2014, Davide Tateo
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

#ifndef CMT_H
#define CMT_H

#include <opencv2/opencv.hpp>

struct InitializationData
{
	//Object keypoints and features
	std::vector<cv::KeyPoint> selected_keypoints;
	cv::Mat selected_features;

	//background keypoints and features
	std::vector<cv::KeyPoint> background_keypoints;
	cv::Mat background_features;

	//initial polygon
	std::vector<cv::Point2f> polygon;
};

class CMTFeatureExtractor
{
public:
	CMTFeatureExtractor();
	void detect(cv::Mat im_gray);
	void discriminateKeyPoints(cv::Mat im_gray, InitializationData& data);

	inline std::vector<cv::KeyPoint>& getKeypoints()
	{
		return keypoints;
	}

	inline cv::Mat& getFeatures()
	{
		return features;
	}

private:
	void insidePolygon(const std::vector<cv::KeyPoint>& keypoints,
				std::vector<cv::Point2f>& polygon,
				std::vector<cv::KeyPoint>& in, std::vector<cv::KeyPoint>& out);

private:
	//algorithms
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;

	//extracted data
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat features;

};

class CMT
{
public:

	int descriptorLength;
	int thrOutlier;
	float thrConf;
	float thrRatio;

	bool estimateScale;
	bool estimateRotation;

	cv::Mat selectedFeatures;
	std::vector<int> selectedClasses;
	cv::Mat featuresDatabase;
	std::vector<int> classesDatabase;

	std::vector<std::vector<float> > squareForm;
	std::vector<std::vector<float> > angles;

	std::vector<cv::Point2f> springs;

	cv::Mat im_prev;
	std::vector<std::pair<cv::KeyPoint, int> > activeKeypoints;
	std::vector<std::pair<cv::KeyPoint, int> > trackedKeypoints;

	int nbInitialKeypoints;

	std::vector<cv::Point2f> votes;

	std::vector<std::pair<cv::KeyPoint, int> > outliers;

public:
	CMT();
	void initialize(cv::Mat im_gray0, InitializationData& data);
	void processFrame(cv::Mat im_gray, std::vector<cv::KeyPoint>& keypoints,
				cv::Mat& features);

	inline const std::vector<cv::Point2f>& getTrackedPolygon() const
	{
		return trackedPolygon;
	}

private:
	void track(cv::Mat im_gray,
				const std::vector<std::pair<cv::KeyPoint, int> >& keypointsIN,
				std::vector<std::pair<cv::KeyPoint, int> >& keypointsTracked,
				std::vector<unsigned char>& status, int THR_FB = 20);

	void estimate(const std::vector<std::pair<cv::KeyPoint, int> >& keypointsIN,
				cv::Point2f& center, float& scaleEstimate, float& medRot,
				std::vector<std::pair<cv::KeyPoint, int> >& keypoints);

	void computeBoundingBox(const cv::Mat& im_gray, const cv::Point2f& center,
				float rotationEstimate, float scaleEstimate);

private:
	//algorithms
	cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
	static const int minimumFraction = 3; //10;

	//Polygon coordinates
	std::vector<cv::Point2f> relativePolygon;
	std::vector<cv::Point2f> trackedPolygon;
};

#endif // CMT_H
