/*
 * FullSlamImu.h
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#ifndef FULLSLAMIMU_H_
#define FULLSLAMIMU_H_

#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf/tf.h>

#include <sensor_msgs/Imu.h>
#include <c_slam_msgs/TrackedObject.h>

#include "ImuHandler.h"
#include "TracksHandler.h"

namespace roamfree_c_slam
{

class FullSlamImu
{

public:

	FullSlamImu(std::string imuTopic);
	virtual ~FullSlamImu();
	void run();

protected:
	void imuCb(const sensor_msgs::Imu &msg);
	void tracksCb(const c_slam_msgs::TrackedObject &msg);

private:
	void initRoamfree();
	void initCamera();

private:
	ros::NodeHandle n;
	ros::Subscriber tracks_sub;
	ros::Subscriber imu_sub;

	tf::Transform T_OC_tf; // transformation from robot to camera;

private:
	ROAMestimation::FactorGraphFilter* filter;
	ImuHandler* imuHandler;
	TracksHandler* tracksHandler;


};

} /* namespace roamfree_c_slam */

#endif /* FULLSLAMIMU_H_ */
