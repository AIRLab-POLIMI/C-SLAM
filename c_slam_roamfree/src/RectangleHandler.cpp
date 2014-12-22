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
 * RectangleHandler.cpp
 *
 *  Created on: Dec 19, 2014
 *      Author: davide
 */

#include "RectangleHandler.h"

#include <iostream>

using namespace std;
using namespace ROAMestimation;

RectangleHandler::RectangleHandler(double initialDepth) :
		_lambda(initialDepth) {
}

bool RectangleHandler::init(FactorGraphFilter* f, const string &name,
		const Eigen::VectorXd & T_OS, const Eigen::VectorXd & K) {

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

}

bool RectangleHandler::addFeatureObservation(long int id, double t,
		const Eigen::VectorXd &z, const Eigen::MatrixXd &cov) {

	const string &sensor = getFeatureSensor(id);

	// there must already exist a pose
	PoseVertexWrapper_Ptr cur_frame = _filter->getNearestPoseByTimestamp(t);
	if (!cur_frame) {
		return false;
	}

	// time of the message must match the vertex found
	if (fabs(t - cur_frame->getTimestamp()) > _timestampOffsetTreshold) {
		return false;
	}

	if (_features.find(id) == _features.end()) {
		// we need to add a new sensor
		initFeature(sensor, z, cur_frame, id);
	}

	RectangleDescriptor &d = _features[id];

	// only one reading per frame
	if (d.lastFrame && cur_frame->sameVertexAs(d.lastFrame)) {
		return false;
	}
	d.lastFrame = cur_frame;

	MeasurementEdgeWrapper_Ptr ret = _filter->addMeasurement(sensor, t, z, cov,
			cur_frame);
}

bool RectangleHandler::initFeature(const std::string& sensor,
		const Eigen::VectorXd& z, ROAMestimation::PoseVertexWrapper_Ptr pv,
		long int id) {

	const Eigen::VectorXd &cur_frame = pv->getEstimate();
	Eigen::VectorXd dim0(2), f0(7);

	initRectangle(cur_frame, _lambda, z, dim0, f0);

	_filter->addSensor(sensor, RectangularObject, false, false);
	_filter->shareSensorFrame("Camera", sensor);
	_filter->shareParameter("Camera_CM", sensor + "_CM");

	_filter->addConstantParameter(Euclidean2D, sensor + "_Dim", 0.0, dim0, false);
	_filter->addConstantParameter(SE3, sensor + "_F", 0.0, f0, false);

	//add to current track list
	RectangleDescriptor &d = _features[id];

	//_filter->setRobustKernel(sensor, true, 3.0);

	cerr << "[RectangleHandler] New rectangle, id " << id << endl;

	return true;
}

void RectangleHandler::initRectangle(const Eigen::VectorXd& Sw, double lambda,
		const Eigen::VectorXd& z, Eigen::VectorXd& shapeParams,
		Eigen::VectorXd& F) {

	cerr << "initRectangle" << endl;

//Get the points
	Eigen::Vector3d m1(z[0], z[1], 1);
	Eigen::Vector3d m2(z[2], z[3], 1);
	Eigen::Vector3d m3(z[4], z[5], 1);
	Eigen::Vector3d m4(z[6], z[7], 1);

	Eigen::Vector3d Ct(Sw[0], Sw[1], Sw[2]);
	Eigen::Quaterniond Cq(Sw[3], Sw[4], Sw[5], Sw[6]);

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

//Compute frame quaternion
	Eigen::Matrix3d R;
	R << R1, R2, R3;

	const Eigen::Quaterniond qOC(R);

	Eigen::Quaterniond Fqhat = Cq * qOC;

//Compute frame transaltion
	Eigen::Matrix3d omega = _K.transpose().inverse() * _K.inverse();
	double ff = sqrt(
			(n2.transpose() * omega * n2)[0] / (n3.transpose() * omega * n3)[0]);

	Eigen::Vector3d&& CtOhat = lambda * _K.inverse() * m1;
	Eigen::Vector3d&& Fthat = Ct + Cq.toRotationMatrix() * CtOhat;

//compute shape parameters
	Eigen::Vector3d&& X = _K * R1;
	Eigen::Vector3d&& Y = c2 * lambda * m2 - lambda * m1;

	double w = ((X.transpose() * X).inverse() * X.transpose() * Y)[0];
	double h = w / ff;

//Write the results
	shapeParams << w, h;
	F << Fthat[0], Fthat[1], Fthat[2], Fqhat.w(), Fqhat.x(), Fqhat.y(), Fqhat.z();

}

bool RectangleHandler::getFeaturePoseInWorldFrame(long int id,
		Eigen::VectorXd& fw) const {

	const string &sensor = getFeatureSensor(id);

	ParameterWrapper_Ptr f_par = _filter->getParameterByName(sensor + "_F"); // anchor frame

	const Eigen::VectorXd &f = f_par->getEstimate();

	fw = f;

}

long int RectangleHandler::getNActiveFeatures() const {
	long int N = 0;

	for (auto it = _features.begin(); it != _features.end(); ++it) {
		N++;
	}

	return N;
}

bool RectangleHandler::getFeaturesIds(std::vector<long int>& to) const {
	to.clear();

	for (auto it = _features.begin(); it != _features.end(); ++it) {
		to.push_back(it->first);
	}
}

string RectangleHandler::getFeatureSensor(long int id) const {
	stringstream s;
	s << _sensorName << "_" << id;
	return s.str();
}

void RectangleHandler::setTimestampOffsetTreshold(double dt) {
	_timestampOffsetTreshold = dt;
}
