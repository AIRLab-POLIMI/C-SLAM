/*
 * FullSlamImu.h
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#ifndef FULLSLAMIMU_FHP_H_
#define FULLSLAMIMU_FHP_H_

#include <c_slam_msgs/TrackedObject.h>
#include <ROAMvision/ROAMvision.h>

#include "FullSlamImu.h"

namespace roamfree_c_slam
{

class FullSlamImu_FHP : public FullSlamImu
{

public:
	FullSlamImu_FHP(FullSlamConfig& config);
	virtual ~FullSlamImu_FHP();
	void run();

protected:
	void tracksCb(const c_slam_msgs::TrackedObject &msg);

private:
	void initCamera();
	void publishFeatureMarkers();

private:
	//FIXME only for tests with track
	bool pointInImage(Eigen::VectorXd& z);

private:
	ROAMvision::FHPFeatureHandlerBootstrap *tracksHandler;
	ros::Subscriber tracks_sub;

};

} /* namespace roamfree_c_slam */

#endif /* FULLSLAMIMU_H_ */
