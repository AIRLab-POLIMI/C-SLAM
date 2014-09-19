/*
 * FullSlamImu.h
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#ifndef FULLSLAMIMU_H_
#define FULLSLAMIMU_H_

#include <Eigen/Dense>
#include <set>

#include <ros/ros.h>
#include <tf/tf.h>

#include "ROAMestimation/ROAMestimation.h"
#include "ROAMimu/IMUIntegralHandler.h"

#include <sensor_msgs/Imu.h>
#include <c_slam_msgs/TrackedObject.h>

using namespace ROAMestimation;
using namespace ROAMimu;

namespace roamfree_c_slam {

class FullSlamImu {

public:

	FullSlamImu();
	virtual ~FullSlamImu();

	void init();

	void run();

protected:

	bool _initialized;

	FactorGraphFilter *_filter;
	IMUIntegralHandler *_imu;

	tf::Transform _T_OC_tf; // transformation from robot to camera;

	ros::Subscriber _tracks_sub, _imu_sub;

	std::set<int> _tracks;

	void imuCb(const sensor_msgs::Imu &msg);
	void tracksCb(const c_slam_msgs::TrackedObject &msg);




};

} /* namespace roamfree_c_slam */

#endif /* FULLSLAMIMU_H_ */
