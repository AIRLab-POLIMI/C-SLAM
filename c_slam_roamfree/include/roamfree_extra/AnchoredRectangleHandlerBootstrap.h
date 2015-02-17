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

#ifndef INCLUDE_ROAMFREE_EXTRA_ANCHOREDRECTANGLEHANDLERBOOTSTRAP_H_
#define INCLUDE_ROAMFREE_EXTRA_ANCHOREDRECTANGLEHANDLERBOOTSTRAP_H_

#include "AnchoredRectangleHandler.h"

#include <map>

namespace ROAMvision {

class AnchoredRectangleHandlerBootstrap: public AnchoredRectangleHandler {
public:
	AnchoredRectangleHandlerBootstrap(double initialDepth);
	virtual bool addFeatureObservation(long int id, double t,
			const Eigen::VectorXd &z, const Eigen::MatrixXd &cov);

	void fixImmutableFeaturePoses(const Eigen::VectorXd &pose,
			double percentageThreshold);

	inline bool bootstrapCompleted() const {
		return !_bootstrap;
	}

protected:
	bool initFeature(const std::string& sensor, const Eigen::VectorXd& z,
			ROAMestimation::PoseVertexWrapper_Ptr pv, long int id);

	void voteFixedPoseCandidates(
			std::map<double, unsigned int>& candidates,
			ObjectObservationMap& map);

	bool _bootstrap;
};

}

#endif /* INCLUDE_ROAMFREE_EXTRA_ANCHOREDRECTANGLEHANDLERBOOTSTRAP_H_ */
