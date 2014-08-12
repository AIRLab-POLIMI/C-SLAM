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

#ifndef DETECTORLOGIC_H_
#define DETECTORLOGIC_H_

#include "BaseLogic.h"


#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "ParameterServer.h"
#include "CognitiveDetector.h"
#include "ImageView.h"

class DetectorLogic : public BaseLogic
{
public:
	DetectorLogic(ros::NodeHandle& n, ParameterServer& parameterServer);
	void handleImage(const sensor_msgs::ImageConstPtr& msg);

private:
	void detect(const cv_bridge::CvImagePtr& cv_ptr);
	void classify();
	void display(const cv_bridge::CvImagePtr& cv_ptr);
	void sendFeatures(
				const std::vector<std::pair<std::vector<cv::Point>, std::string> >& features);

private:
	//Ros management
	ros::Publisher detectionPublisher;

	//Data needed to detect objects
	CognitiveDetector detector;

	//Data needed to display results
	ImageView viewer;

	//parameters
	ClassifierParam& classifierParam;
};

#endif /* DETECTORLOGIC_H_ */
