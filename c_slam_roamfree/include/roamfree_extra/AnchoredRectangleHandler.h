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
 * AnchoredRectangleHandler.h
 *
 *  Created on: Jan 08, 2015
 *      Author: davide
 */

#ifndef ANCHOREDRECTANGLEHANDLER_H_
#define ANCHOREDRECTANGLEHANDLER_H_

#include <map>
#include <vector>

#include "ROAMestimation/ROAMestimation.h"

namespace roamfree_c_slam
{

class AnchoredRectangleHandler {

public:
	AnchoredRectangleHandler(double initialDepth);

	virtual bool init(ROAMestimation::FactorGraphFilter* f,
			const std::string &name, const Eigen::VectorXd & T_OS,
			const Eigen::VectorXd & K);
	virtual bool addFeatureObservation(long int id, double t,
			const Eigen::VectorXd &z, const Eigen::MatrixXd &cov);

	virtual bool getFeaturePoseInWorldFrame(long int id,
			Eigen::VectorXd &lw) const;
	virtual bool getFeatureDimensions(long int id, Eigen::VectorXd &dim) const;
	virtual long int getNActiveFeatures() const;
	bool getFeaturesIds(std::vector<long int>& to) const;

	virtual void setTimestampOffsetTreshold(double dt);

	virtual ~AnchoredRectangleHandler();

protected:

	class RectangleDescriptor {
	public:
		ROAMestimation::PoseVertexWrapper_Ptr anchorFrame, lastFrame;

		int nObservations;
	};

	virtual bool initFeature(const std::string& sensor,
			const Eigen::VectorXd& z, ROAMestimation::PoseVertexWrapper_Ptr av,
			long int id);
	void initRectangle(const Eigen::VectorXd& Fw, double lambda,
			const Eigen::VectorXd& z, Eigen::VectorXd& shapeParams,
			Eigen::VectorXd &FOhp, Eigen::VectorXd& FOq);

	std::string getFeatureSensor(long int id) const;

	double _lambda; // initial depth

	typedef std::map<long unsigned int, RectangleDescriptor> FeatureMap;

	FeatureMap _features;

	ROAMestimation::FactorGraphFilter* _filter;
	std::string _sensorName;

	double _timestampOffsetTreshold;

	Eigen::Matrix3d _K;
	double _fx, _fy;

};

}

#endif /* ANCHOREDRECTANGLEHANDLER_H_ */
