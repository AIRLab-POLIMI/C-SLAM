/*
 * c_slam_roamfree,
 *
 *
 * Copyright (C) 2015 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_slam_roamfree.
 *
 * c_slam_roamfree is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_slam_roamfree is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_slam_roamfree.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_OBJECTOBSERVATIONDESCRIPTOR_H_
#define INCLUDE_OBJECTOBSERVATIONDESCRIPTOR_H_

#include <map>
#include <Eigen/Dense>

#include "ROAMestimation/ROAMestimation.h"

namespace ROAMvision
{

class ObjectObservationDescriptor
{

public:
	double t;
	ROAMestimation::PoseVertexWrapper_Ptr pose;
	std::vector<Eigen::Vector2d> z;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};


typedef std::map<double, ObjectObservationDescriptor> ObjectObservationMap;

}


#endif /* INCLUDE_OBJECTOBSERVATIONDESCRIPTOR_H_ */
