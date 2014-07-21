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
 *   based on https://github.com/delmottea/libCMT/
 */

#ifndef CMTFEATUREEXTRACTOR_H_
#define CMTFEATUREEXTRACTOR_H_

#include <opencv2/opencv.hpp>
#include "InitializationData.h"

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


#endif /* CMTFEATUREEXTRACTOR_H_ */
