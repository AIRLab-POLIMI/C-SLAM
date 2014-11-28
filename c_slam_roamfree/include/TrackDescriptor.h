/*
 * TrackDescriptor.h
 *
 *  Created on: Nov 21, 2014
 *      Author: davide
 */

#ifndef TRACKDESCRIPTOR_H_
#define TRACKDESCRIPTOR_H_

#include <Eigen/Dense>
#include <ROAMestimation/ROAMestimation.h>

namespace roamfree_c_slam {

class TrackDescriptor {

public:
	int _n_observations;

	Eigen::Vector2d _last_z;
	double _total_z_change;

	ROAMestimation::PoseVertexWrapper_Ptr _last_vertex;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} /* namespace roamfree_c_slam */

#endif /* TRACKDESCRIPTOR_H_ */
