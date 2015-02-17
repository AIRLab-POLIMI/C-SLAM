/*
 Copyright (c) 2013-2014 Politecnico di Milano.
 All rights reserved. This program and the accompanying materials
 are made available under the terms of the GNU Lesser Public License v3
 which accompanies this distribution, and is available at
 https://www.gnu.org/licenses/lgpl.html

 Contributors:
 Davide A. Cucci (cucci@elet.polimi.it)
 */

/*
 * AnchoredRectangleHandler.cpp
 *
 *  Created on: Jan 08, 2015
 *      Author: davide
 */

#include "roamfree_extra/AnchoredRectangleHandler.h"
#include "roamfree_extra/ObjectSufficientParallax.h"

#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace ROAMestimation;

namespace ROAMvision
{

AnchoredRectangleHandler::AnchoredRectangleHandler(double initialDepth) :
			_lambda(initialDepth)
{
	_timestampOffsetTreshold = 0;
	_filter = NULL;
	_fx = 1;
	_fy = 1;
}

bool AnchoredRectangleHandler::init(FactorGraphFilter* f, const string &name,
			const Eigen::VectorXd & T_OS, const Eigen::VectorXd & K)
{

	_filter = f;
	_sensorName = name;

	// TODO: currently FHP works only if system is camera-centric, i.e., T_OS = I

	_filter->addConstantParameter("Camera_SOx", T_OS(0), true);
	_filter->addConstantParameter("Camera_SOy", T_OS(1), true);
	_filter->addConstantParameter("Camera_SOz", T_OS(2), true);

	_filter->addConstantParameter("Camera_qOSx", T_OS(4), true);
	_filter->addConstantParameter("Camera_qOSy", T_OS(5), true);
	_filter->addConstantParameter("Camera_qOSz", T_OS(6), true);

	_filter->addConstantParameter(Matrix3D, "Camera_CM", K, true);

	Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > KasMatrix(
				K.data());
	_K = KasMatrix;

	_fx = _K(0, 0);
	_fy = _K(1, 1);

	return true;

}

bool AnchoredRectangleHandler::addFeatureObservation(long int id, double t,
			const Eigen::VectorXd &z, const Eigen::MatrixXd &cov)
{

	const string &sensor = getFeatureSensor(id);

	// there must already exist a pose
	PoseVertexWrapper_Ptr cur_frame = _filter->getNearestPoseByTimestamp(t);
	if (!cur_frame)
	{
		return false;
	}

	// time of the message must match the vertex found
	if (fabs(t - cur_frame->getTimestamp()) > _timestampOffsetTreshold)
	{
		return false;
	}

	if (_objects.find(id) == _objects.end())
	{
		// we need to add a new sensor
		initFeature(sensor, z, cur_frame, id);

	}

	ObjectTrackDescriptor &d = _objects[id];

	// only one reading per frame
	if (d.lastFrame && cur_frame->sameVertexAs(d.lastFrame))
	{
		return false;
	}
	d.lastFrame = cur_frame;

	if (!d.isInitialized)
	{

		// add the current observation to history
		ObjectObservationDescriptor &obs = d.zHistory[t];
		obs.pose = cur_frame;
		obs.z = z;
		obs.t = t;

		if (d.initStrategy->initialize())
		{
			// add all the edges
			// TODO: make the AnchoredRectangle special case in which anchor = current pose, so we can use also the first observation
			for (auto it = ++d.zHistory.begin(); it != d.zHistory.end(); ++it)
			{
				const ObjectObservationDescriptor &obs = it->second;

				MeasurementEdgeWrapper_Ptr ret = _filter->addMeasurement(sensor,
							obs.t, obs.z, cov, obs.pose);

				assert(ret);
			}

			// done
			d.zHistory.clear();
			d.isInitialized = true;

#			ifdef DEBUG_PRINT_VISION_INFO_MESSAGES
			cerr << "[AnchoredRectangleHandler] Ready to estimate depth for track " << id
			<< endl;
#			endif
		}
	}
	else
	{

		MeasurementEdgeWrapper_Ptr ret = _filter->addMeasurement(sensor, t, z,
					cov, cur_frame);
		assert(ret);
	}

	return true;
}

bool AnchoredRectangleHandler::initFeature(const std::string& sensor,
			const Eigen::VectorXd& z, ROAMestimation::PoseVertexWrapper_Ptr av,
			long int id)
{

	const Eigen::VectorXd &anchor_frame = av->getEstimate();
	Eigen::VectorXd dim0(2), f0(7), foq0(4), fohp0(3);

	initRectangle(anchor_frame, _lambda, z, dim0, fohp0, foq0);

	_filter->addSensor(sensor, AnchoredRectangularObject, false, false);
	_filter->shareSensorFrame("Camera", sensor);
	_filter->shareParameter("Camera_CM", sensor + "_CM");

	_filter->addConstantParameter(Euclidean2D, sensor + "_Dim", 0.0, dim0,
				false);

	_filter->poseVertexAsParameter(av, sensor + "_F");

	_filter->addConstantParameter(Quaternion, sensor + "_FOq", 0.0, foq0,
				false);

	_filter->addConstantParameter(Euclidean3D, sensor + "_FOhp", 0.0, fohp0,
				false);

// prior on homogeneous point

	const double sigma_pixel = 1;

	Eigen::MatrixXd prior_cov(3, 3);

	prior_cov << sigma_pixel / pow(_fx, 2), 0, 0, 0, sigma_pixel / pow(_fy, 2), 0, 0, 0, pow(
				_lambda / 3.0, 2);

	_filter->addPriorOnConstantParameter(Euclidean3DPrior, sensor + "_FOhp",
				fohp0, prior_cov);

//add to current track list
	ObjectTrackDescriptor &d = _objects[id];

	d.anchorFrame = av;
	d.isInitialized = false;
	d.initStrategy = new ObjectSufficientParallax(0.25, d.zHistory, _K.data());

//_filter->setRobustKernel(sensor, true, 3.0);

	cerr << "[RectangleHandler] New rectangle, id " << id << endl;

	return true;
}

void AnchoredRectangleHandler::initRectangle(const Eigen::VectorXd& Fw,
			double lambda, const Eigen::VectorXd& z,
			Eigen::VectorXd& shapeParamshat, Eigen::VectorXd& FOhphat,
			Eigen::VectorXd &FOqhat)
{

//Get the points
	Eigen::Vector3d m1(z[0], z[1], 1);
	Eigen::Vector3d m2(z[2], z[3], 1);
	Eigen::Vector3d m3(z[4], z[5], 1);
	Eigen::Vector3d m4(z[6], z[7], 1);

	Eigen::Vector3d Ft(Fw[0], Fw[1], Fw[2]);
	Eigen::Quaterniond Fq(Fw[3], Fw[4], Fw[5], Fw[6]);

//compute normals
	double c2 = (m1.cross(m3).transpose() * m4)[0]
				/ (m2.cross(m3).transpose() * m4)[0];
	double c3 = (m1.cross(m3).transpose() * m2)[0]
				/ (m4.cross(m3).transpose() * m2)[0];

	Eigen::Vector3d n2 = c2 * m2 - m1;
	Eigen::Vector3d n3 = c3 * m4 - m1;

//Compute rotation matrix columns
	Eigen::Vector3d R1 = _K.inverse() * n2;
	R1 = R1 / R1.norm();

	Eigen::Vector3d R2 = _K.inverse() * n3;
	R2 = R2 / R2.norm();

	Eigen::Vector3d R3 = R1.cross(R2);

//Compute rotation from camera to object
	Eigen::Matrix3d R;
	R << R1, R2, R3;
	Eigen::Quaterniond FOq_e(R);

// and initialize the of the object with respect to the anchor frame
	FOqhat << FOq_e.w(), FOq_e.x(), FOq_e.y(), FOq_e.z();

// now initialize lower left corner homogeneous point
	FOhphat << z[0], z[1], 1.0;
	FOhphat = _K.inverse() * FOhphat;
	FOhphat(2) = 1.0 / lambda; // 1/d distance of the plane parallel to the image plane on which features are initialized.

//Compute frame transaltion
	Eigen::Matrix3d omega = _K.transpose().inverse() * _K.inverse();
	double ff = sqrt(
				(n2.transpose() * omega * n2)[0]
							/ (n3.transpose() * omega * n3)[0]);

//compute shape parameters
	Eigen::Vector3d X = _K * R1;
	Eigen::Vector3d Y = c2 * lambda * m2 - lambda * m1;

	double w = ((X.transpose() * X).inverse() * X.transpose() * Y)[0];

//Write the results
	shapeParamshat << ff, w / lambda;

}

bool AnchoredRectangleHandler::getFeaturePoseInWorldFrame(long int id,
			Eigen::VectorXd& c) const
{

	const string &sensor = getFeatureSensor(id);

	ParameterWrapper_Ptr f_par = _filter->getParameterByName(sensor + "_F"); // anchor frame
	ParameterWrapper_Ptr fohp_par = _filter->getParameterByName(
				sensor + "_FOhp"); // anchor frame
	ParameterWrapper_Ptr foq_par = _filter->getParameterByName(sensor + "_FOq"); // anchor frame

	if (!f_par || !fohp_par || !foq_par)
	{
		return false;
	}

	const Eigen::VectorXd &fw = f_par->getEstimate();

	const Eigen::VectorXd &FOhp = fohp_par->getEstimate();
	const Eigen::VectorXd &FOq = foq_par->getEstimate();

	Eigen::Quaterniond Fq_e(fw(3), fw(4), fw(5), fw(6)); // anchor frame orientation wrt world

// compute the position of the lower left corner of the object starting from anchor frame and homogeneous point

	Eigen::Vector3d FOhp_e;
	FOhp_e << FOhp(0), FOhp(1), 1.0; // construct a proper homogeneous point from the first two components of the parameter

	Eigen::Vector3d Ow;
	Ow = fw.head(3) + 1 / FOhp(2) * (Fq_e.toRotationMatrix() * FOhp_e);

// orientation of the object
	Eigen::Quaterniond FOq_e(FOq(0), FOq(1), FOq(2), FOq(3));
	Eigen::Quaterniond R_WO = Fq_e * FOq_e;

	Eigen::VectorXd dim(2);
	getFeatureDimensions(id, dim);

	Eigen::Vector3d t_OC(dim(0) / 2.0, dim(1) / 2.0, 0.0);

	c.head(3) = Ow + R_WO.toRotationMatrix() * t_OC;
	c.tail(4) << R_WO.w(), R_WO.x(), R_WO.y(), R_WO.z();

	return true;
}

bool AnchoredRectangleHandler::getFeatureDimensions(long int id,
			Eigen::VectorXd& dim) const
{
	const string &sensor = getFeatureSensor(id);

	ParameterWrapper_Ptr dim_par = _filter->getParameterByName(sensor + "_Dim"); // dimensions
	ParameterWrapper_Ptr fohp_par = _filter->getParameterByName(
				sensor + "_FOhp"); // we need also inverse depth for computing actual dimensions

	if (!dim_par)
	{
		return false;
	}

	const Eigen::VectorXd &ffandwbar = dim_par->getEstimate();
	const Eigen::VectorXd &FOhp = fohp_par->getEstimate();

	dim << ffandwbar(1) / FOhp(2), ffandwbar(1) / (ffandwbar(0) * FOhp(2));

	return true;
}

long int AnchoredRectangleHandler::getNActiveFeatures() const
{
	long int N = 0;

	for (auto it = _objects.begin(); it != _objects.end(); ++it)
	{
		N++;
	}

	return N;
}

bool AnchoredRectangleHandler::getFeaturesIds(std::vector<long int>& to) const
{
	to.clear();

	for (auto it = _objects.begin(); it != _objects.end(); ++it)
	{
		to.push_back(it->first);
	}

	return true;
}

string AnchoredRectangleHandler::getFeatureSensor(long int id) const
{
	stringstream s;
	s << _sensorName << "_" << id;
	return s.str();
}

void AnchoredRectangleHandler::setTimestampOffsetTreshold(double dt)
{
	_timestampOffsetTreshold = dt;
}

AnchoredRectangleHandler::~AnchoredRectangleHandler()
{

}

}

