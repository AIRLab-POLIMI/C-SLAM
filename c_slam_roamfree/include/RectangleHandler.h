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
 * RectangleHandler.h
 *
 *  Created on: Dec 19, 2014
 *      Author: davide
 */

#ifndef RECTANGLEHANDLER_H_
#define RECTANGLEHANDLER_H_

#include <map>
#include <vector>

#include "ROAMestimation/ROAMestimation.h"

namespace roamfree_c_slam
{

class RectangleHandler
{

public:
	RectangleHandler(double initialDepth);

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

	virtual ~RectangleHandler();

protected:

	class RectangleDescriptor
	{
	public:
		ROAMestimation::PoseVertexWrapper_Ptr lastFrame;

		int nObservations;
	};

	virtual bool initFeature(const std::string& sensor,
				const Eigen::VectorXd& z,
				ROAMestimation::PoseVertexWrapper_Ptr pv, long int id);
	void initRectangle(const Eigen::VectorXd& Sw, double lambda,
				const Eigen::VectorXd& z, Eigen::VectorXd& shapeParams,
				Eigen::VectorXd& F);

	std::string getFeatureSensor(long int id) const;

	double _lambda; // initial depth

	typedef std::map<long unsigned int, RectangleDescriptor> FeatureMap;

	FeatureMap _features;

	ROAMestimation::FactorGraphFilter* _filter;
	std::string _sensorName;

	double _timestampOffsetTreshold;

	Eigen::Matrix3d _K;

};

}

#endif /* RECTANGLEHANDLER_H_ */
