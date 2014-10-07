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

#include "TracksHandler.h"

#include <tf_conversions/tf_eigen.h>

using namespace ROAMestimation;

TracksHandler::TracksHandler(FactorGraphFilter* filter, tf::Transform& T_OC_tf) :
			filter(filter), T_OC_tf(T_OC_tf)
{

}

void TracksHandler::addMeasurement(double t, size_t id, Eigen::VectorXd z)
{
	std::set<size_t>::iterator s_it = tracks.find(id);

	// produce the sensor name
	std::stringstream s;
	s << "Track_" << id;
	std::string sensor = s.str();

	if (s_it == tracks.end())
	{

		// there must already exist a pose
		ROAMestimation::PoseVertexWrapper_Ptr pose_ptr =
					filter->getNearestPoseByTimestamp(t);
		if (!pose_ptr)
		{
			return;
		}

		// we need to add a new sensor
		initTrack(sensor, z, pose_ptr->getEstimate(), id);
	}

	Eigen::MatrixXd cov(2, 2);
	cov = Eigen::MatrixXd::Identity(2, 2);

	filter->addSequentialMeasurement(sensor, t, z, cov);
}

void TracksHandler::initTrack(const std::string& sensor,
			const Eigen::VectorXd& z, const Eigen::VectorXd& x, size_t id)
{
	// we need to add a new sensor
	filter->addSensor(sensor, ROAMestimation::ImagePlaneProjection, false,
				true);
	filter->shareSensorFrame("Camera", sensor);
	filter->shareParameter("Camera_CM", sensor + "_CM");

	// place the marker somewhere on the direction where it was seen
	Eigen::VectorXd Lw(3);
	computePossibleLandmarkLocation(z, x, Lw);
	filter->addConstantParameter(ROAMestimation::Euclidean3D, sensor + "_Lw",
				Lw, false);
	filter->setRobustKernel(sensor, true, 0.1);

	//add to current track list
	tracks.insert(id);
}

void TracksHandler::computePossibleLandmarkLocation(const Eigen::VectorXd& z,
			const Eigen::VectorXd& x, Eigen::VectorXd& Lw)
{
	// place the marker somewhere on the direction where it was seen
	const Eigen::Map<const Eigen::Matrix3d> cm(
				filter->getParameterByName("Camera_CM")->getEstimate().data());

	Eigen::Matrix3d cm_inv = cm.transpose().inverse(); // the inverse of the camera intrinsic calibration matrix
	cm_inv /= cm_inv(2, 2);

	Eigen::Vector3d Limg; // the landmark on the image plane
	Limg << z(0), z(1), 1.0;

	Eigen::Vector3d dc; // the direction of the landmark in the camera reference frame
	dc = cm_inv * Limg;

	// the pose of the odometric center wrt world
	Eigen::Matrix3d R_WC;
	Eigen::Vector3d t_WC;

	computeCameraPose(x, R_WC, t_WC);
	//Compute a possible pose for the landmark
	double alpha = 5;
	Lw = alpha * R_WC * dc + t_WC;
}

void TracksHandler::computeCameraPose(const Eigen::VectorXd& x,
			Eigen::Matrix3d& R_WC, Eigen::Vector3d& t_WC)
{
	Eigen::Quaterniond q(x(3), x(4), x(5), x(6));
	tf::Quaternion q_tf;
	tf::quaternionEigenToTF(q, q_tf);
	tf::Transform T_WO_tf(q_tf, tf::Vector3(x(0), x(1), x(2)));
	tf::Transform T_WC_tf = T_WO_tf * T_OC_tf;
	tf::Matrix3x3 R_WC_tf = T_WC_tf.getBasis();
	tf::Vector3 t_WC_tf = T_WC_tf.getOrigin();
	tf::matrixTFToEigen(R_WC_tf, R_WC);
	tf::vectorTFToEigen(t_WC_tf, t_WC);
}

