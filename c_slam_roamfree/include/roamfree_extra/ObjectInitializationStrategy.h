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

#ifndef INCLUDE_OBJECTINITIALIZATIONSTRATEGY_H_
#define INCLUDE_OBJECTINITIALIZATIONSTRATEGY_H_

#include "ObjectObservationDescriptor.h"

namespace ROAMvision
{
class ObjectInitializationStrategy
{

public:

	ObjectInitializationStrategy(const ObjectObservationMap &zHistory,
				const double *K);

	virtual ~ObjectInitializationStrategy();

	virtual bool initialize(Eigen::VectorXd &HP) = 0;

protected:
	const ObjectObservationMap &_zHistory;
	const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> _K;

};

}


#endif /* INCLUDE_OBJECTINITIALIZATIONSTRATEGY_H_ */
