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
 */

#ifndef MAPPINGTRACKER_H_
#define MAPPINGTRACKER_H_

#include "CMT.h"

class MappingTracker: public CMT
{
public:
	MappingTracker();
	virtual void initialize(const cv::Mat& im_gray0, InitializationData& data);
	void mapObject(cv::Mat& image, const cv::Mat_<double>& K);
	void decomposeEssentialMat(cv::InputArray _E, cv::OutputArray _R1,
				cv::OutputArray _R2, cv::OutputArray _t);

private:
	std::vector<cv::KeyPoint> mappedKeyPoints;
	bool objectMapped;
	double minDistance;

};

#endif /* MAPPINGTRACKER_H_ */
