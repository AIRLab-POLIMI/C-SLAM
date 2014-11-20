/*
 * c_slam_roamfree,
 *
 *
 * Copyright (C) 2014 Davide Tateo
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

#ifndef TRACKSHANDLER_H_
#define TRACKSHANDLER_H_

#include <map>

#include <Eigen/Dense>

#include <tf/tf.h>

#include <ROAMestimation/ROAMestimation.h>

class TracksHandler
{
public:
	TracksHandler(ROAMestimation::FactorGraphFilter* filter,
				tf::Transform& T_OC_tf);
	void addMeasurement(double t, size_t id, Eigen::VectorXd z);

private:
	bool poseExists(double t);
	void computePossibleLandmarkLocation(const Eigen::VectorXd& z,
				const Eigen::VectorXd& x, Eigen::VectorXd& Lw);
	void initTrack(const std::string& sensor, const Eigen::VectorXd& z,
			ROAMestimation::PoseVertexWrapper_Ptr pv, size_t id);

	void initTrack_FHP(const std::string& sensor, const Eigen::VectorXd& z,
			ROAMestimation::PoseVertexWrapper_Ptr pv, size_t id);

	void computeCameraPose(const Eigen::VectorXd& x, Eigen::Matrix3d& R_WC,
				Eigen::Vector3d& t_WC);

private:
	ROAMestimation::FactorGraphFilter* filter;
	tf::Transform& T_OC_tf;

	typedef std::map<size_t, size_t> TracksMap;
	TracksMap tracks;



};

#endif /* TRACKSHANDLER_H_ */
