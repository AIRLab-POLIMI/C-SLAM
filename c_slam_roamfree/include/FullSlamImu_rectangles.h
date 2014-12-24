/*
 * FullSlamImu.h
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#ifndef FULLSLAMIMU_RECTANGLES_H_
#define FULLSLAMIMU_RECTANGLES_H_

#include <Eigen/Dense>

#include <ros/ros.h>
#include <tf/tf.h>

#include <sensor_msgs/Imu.h>
#include <c_slam_msgs/TrackedObject.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include "FullSlamImu.h"
#include "RectangleHandler.h"

namespace roamfree_c_slam
{

class FullSlamImu_rectangles : public FullSlamImu
{

public:
	FullSlamImu_rectangles(std::string imuTopic);
	virtual ~FullSlamImu_rectangles();
	void run();

protected:
	void tracksCb(const c_slam_msgs::TrackedObject &msg);

private:
	void initCamera();
	void publishFeatureMarkers();

private:
	ros::Subscriber tracks_sub;

private:
	RectangleHandler *tracksHandler;

};

} /* namespace roamfree_c_slam */

#endif /* FULLSLAMIMU_RECTANGLES_H_ */
