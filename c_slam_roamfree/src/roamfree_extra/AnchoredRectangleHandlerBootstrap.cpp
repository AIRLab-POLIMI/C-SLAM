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

#include "roamfree_extra/AnchoredRectangleHandlerBootstrap.h"

namespace ROAMvision
{
AnchoredRectangleHandlerBootstrap::AnchoredRectangleHandlerBootstrap(
			double initialDepth) :
			AnchoredRectangleHandler(initialDepth)
{
	_bootstrap = true;
}

bool AnchoredRectangleHandlerBootstrap::addFeatureObservation(long int id,
			double t, const Eigen::VectorXd &z, const Eigen::MatrixXd &cov)
{

}

void AnchoredRectangleHandlerBootstrap::fixImmutableFeaturePoses(
			const Eigen::VectorXd &pose, double percentageThreshold)
{

}

bool AnchoredRectangleHandlerBootstrap::initFeature(const std::string& sensor,
			const Eigen::VectorXd& z, ROAMestimation::PoseVertexWrapper_Ptr pv,
			long int id)
{

}

void AnchoredRectangleHandlerBootstrap::voteFixedPoseCandidates(
			std::map<double, unsigned int>& candidates,
			ObjectObservationMap& map)
{

}

}
