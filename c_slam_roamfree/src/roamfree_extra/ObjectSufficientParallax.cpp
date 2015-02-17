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

#include "roamfree_extra/ObjectSufficientParallax.h"

#include <iostream> //todo why?

using namespace std;

namespace ROAMvision
{

ObjectSufficientParallax::ObjectSufficientParallax(double minParallax,
			const ObjectObservationMap& zHistory, const double *K) :
			ObjectInitializationStrategy(zHistory, K),
			_minParallax(minParallax), _traveled(0.0)
{
}

bool ObjectSufficientParallax::initialize()
{

	if (_zHistory.size() >= 2)
	{

		auto it = _zHistory.begin();
		const ObjectObservationDescriptor *last = &(it->second);

		// put the viewing ray in world frame
		Eigen::Vector3d rCnorm, rWnorm;

		const Eigen::VectorXd &T_WC_0 = last->pose->getEstimate();

		Eigen::Vector3d z0;
		z0 << last->z(0), last->z(1), 1.0;
		rCnorm << _K.inverse() * z0;
		rCnorm.normalize();

		Eigen::Quaterniond q_WC0(T_WC_0(3), T_WC_0(4), T_WC_0(5), T_WC_0(6));
		rWnorm = q_WC0._transformVector(rCnorm);

		// project the distance on a plane orthogonal to the viewing ray and take the maximum

		while (++it != _zHistory.end()
					&& it->second.pose->hasBeenEstimated() == true)
		{
			const ObjectObservationDescriptor *cur = &(it->second);

			Eigen::Vector3d distW = last->pose->getEstimate().head(3)
						- cur->pose->getEstimate().head(3);

			// project the distance
			double distOrth = (distW - distW.dot(rWnorm) * rWnorm).norm();

			_traveled = max(_traveled, distOrth);
		}

	}

	if (_traveled > _minParallax && _zHistory.size() >= 3)
	{
		cerr << fixed << "[ObjectSufficientParallax] traveled " << _traveled << " m" << endl;

		return true;
	}

	return false;
}

}
