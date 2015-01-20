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

#include "CMTFeatureExtractor.h"

#include <stdexcept>

using namespace cv;

CMTFeatureExtractor::CMTFeatureExtractor(ExtractionParam& par) : par(par)
{
	std::string detectorType = "Feature2D.BRISK";
	std::string descriptorType = "Feature2D.BRISK";

	//Initialise detector, descriptor, matcher
	detector = Algorithm::create<FeatureDetector>(detectorType);
	descriptorExtractor = Algorithm::create<DescriptorExtractor>(
				descriptorType);
}

void CMTFeatureExtractor::detect(Mat im_gray)
{
	detector->set("thres", par.threshold);
	detector->detect(im_gray, keypoints);
	descriptorExtractor->compute(im_gray, keypoints, features);
}

void CMTFeatureExtractor::discriminateKeyPoints(Mat im_gray,
			InitializationData& data)
{
	//Remember keypoints that are in the rectangle as selected keypoints
	insidePolygon(keypoints, data.polygon, data.selected_keypoints,
				data.background_keypoints);
	descriptorExtractor->compute(im_gray, data.selected_keypoints,
				data.selected_features);

	if (data.selected_keypoints.size() == 0)
	{
		throw std::runtime_error("No keypoints in the selection");
	}

	//Remember keypoints that are not in the rectangle as background keypoints
	descriptorExtractor->compute(im_gray, data.background_keypoints,
				data.background_features);
}

void CMTFeatureExtractor::insidePolygon(const vector<KeyPoint>& keypoints,
			vector<Point2f>& polygon, vector<KeyPoint>& in,
			vector<KeyPoint>& out)
{
	for (int i = 0; i < keypoints.size(); i++)
	{
		if (pointPolygonTest(polygon, keypoints[i].pt, false) >= 0)
			in.push_back(keypoints[i]);
		else
			out.push_back(keypoints[i]);
	}
}
