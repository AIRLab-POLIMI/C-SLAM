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

#include "roamfree_extra/ObjectSufficientZChange.h"

#include <iostream> //TODO why?

using namespace ROAMestimation;
using namespace std;

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
	if (_bootstrap && AnchoredRectangleHandler::getNActiveFeatures() > 3)
	{
		_bootstrap = false;
	}

	return AnchoredRectangleHandler::addFeatureObservation(id, t, z, cov);
}

void AnchoredRectangleHandlerBootstrap::fixImmutableFeaturePoses(
			const Eigen::VectorXd &pose, double percentageThreshold)
{
	map<double, unsigned int> candidates;

	for (auto& feature : _objects)
	{
		auto& map = feature.second.zHistory;
		voteFixedPoseCandidates(candidates, map);
	}

	for (auto& candidate : candidates)
	{
		double percentage = static_cast<double>(candidate.second)
					/ static_cast<double>(_objects.size());

		if (percentage >= percentageThreshold)
		{
			auto pose_ptr = _filter->getNearestPoseByTimestamp(candidate.first);
			pose_ptr->setEstimate(pose);
			pose_ptr->setFixed(true);
		}
	}

}

bool AnchoredRectangleHandlerBootstrap::initFeature(const std::string& sensor,
			const Eigen::VectorXd& z, ROAMestimation::PoseVertexWrapper_Ptr pv,
			long int id)
{
	if (_bootstrap)
	{
		const Eigen::VectorXd &anchor_frame = pv->getEstimate();
		Eigen::VectorXd dim0(2), f0(7), foq0(4), fohp0(3);

		initRectangle(anchor_frame, _lambda, z, dim0, fohp0, foq0);

		_filter->addSensor(sensor, AnchoredRectangularObject, false, false);
		_filter->shareSensorFrame("Camera", sensor);
		_filter->shareParameter("Camera_CM", sensor + "_CM");

		_filter->addConstantParameter(Euclidean2D, sensor + "_Dim", 0.0, dim0,
					false);

		_filter->poseVertexAsParameter(pv, sensor + "_F");

		_filter->addConstantParameter(Quaternion, sensor + "_FOq", pv->getTimestamp(), foq0,
					false);

		_filter->addConstantParameter(Euclidean3D, sensor + "_FOhp", pv->getTimestamp(), fohp0,
					false);

		// prior on homogeneous point
		const double sigma_pixel = 1;

		Eigen::MatrixXd prior_cov(3, 3);

		prior_cov << sigma_pixel / pow(_fx, 2), 0, 0, 0, sigma_pixel
					/ pow(_fy, 2), 0, 0, 0, pow(_lambda / 3.0, 2);

		_filter->addPriorOnConstantParameter(Euclidean3DPrior, sensor + "_FOhp",
					fohp0, prior_cov);

		//add to current track list
		ObjectTrackDescriptor &d = _objects[id];

		d.anchorFrame = pv;
		d.isInitialized = false;
		d.initStrategy = new ObjectSufficientZChange(2.0, d.zHistory,
					_K.data());

		//_filter->setRobustKernel(sensor, true, 3.0);

		cerr << "[AnchoredRectangleHandler] New rectangle, id " << id << endl;

		return true;

		return true;

	}
	else
	{
		return AnchoredRectangleHandler::initFeature(sensor, z, pv, id);
	}
}

void AnchoredRectangleHandlerBootstrap::voteFixedPoseCandidates(
			std::map<double, unsigned int>& candidates,
			ObjectObservationMap& map)
{
	auto it = map.begin();
	Eigen::Matrix<double, 8, 1>& firstObservation = it->second.z;

	while (it != map.end())
	{
		if (it->second.z == firstObservation)
		{ //TODO threshold over squared norm of difference
			candidates[it->second.pose->getTimestamp()]++;
			it++;
		}
		else
		{
			break;
		}

	}
}

}
