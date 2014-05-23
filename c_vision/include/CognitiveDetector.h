/*
 * c_vision,
 *
 *
 * Copyright (C) 2013 Davide Tateo
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

#ifndef COGNITIVEDETECTOR_H_
#define COGNITIVEDETECTOR_H_

#include <opencv2/core/core.hpp>

#include "DefaultParameters.h"
#include "FeatureDetector.h"
#include "HoughDetector.h"
#include "HighLevelDetector.h"
#include "DBSCAN.h"
#include "ObjectClassificator.h"

#include "ImageView.h"

class CognitiveDetector
{
public:
	CognitiveDetector();

	void detect(cv::Mat& image, ObjectClassificator& classificator);

	inline void setPitch(double pitch)
	{
		this->pitch = pitch;
	}

	inline void setRoll(double roll)
	{
		this->roll = roll;
	}

	inline void setYaw(double yaw)
	{
		this->yaw = yaw;
	}

private:
	std::vector<std::vector<cv::Point> > detectRectangles(
				std::vector<cv::Vec4i> verticalLines,
				std::vector<cv::Vec4i> horizontalLines);
	cv::Point findInterceptions(cv::Vec4i l1, cv::Vec4i l2);
	cv::Mat preprocessing(cv::Mat& input);

	template<class T>
	void processFeatures(const std::vector<T>& features,
				ObjectClassificator& classificator)
	{
		typedef typename std::vector<T> FeatureVector;

		for (typename FeatureVector::const_iterator i = features.begin();
						i != features.end(); ++i)
			{
				const Feature& feature = *i;
				classificator.addFeature(feature);
			}
	}

private:
	FeatureDetector featureDetector;
	DBSCAN clusterDetector;
	HoughDetector lineDetector;

	double pitch;
	double roll;
	double yaw;

private:
	ImageView viewer;

};

#endif /* COGNITIVEDETECTOR_H_ */
