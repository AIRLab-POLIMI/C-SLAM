/*
 * FullSlamImu.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#include "FullSlamImu.h"

#include <tf_conversions/tf_eigen.h>

using namespace ROAMestimation;

namespace roamfree_c_slam
{

FullSlamImu::FullSlamImu(std::string imuTopic) :
			filter(NULL), imuHandler(NULL), tracksHandler(NULL)
{
	//setup roamfree
	initRoamfree();

	//setup the camera
	initCamera();

	//setup the handlers
	imuHandler = new ImuHandler(filter);

	//subscribe to sensor topics
	imu_sub = n.subscribe(imuTopic, 60000, &FullSlamImu::imuCb, this);
	tracks_sub = n.subscribe("/tracks", 60000, &FullSlamImu::tracksCb, this);
}

FullSlamImu::~FullSlamImu()
{
	if (filter != NULL)

		delete filter;

	if (imuHandler)
		delete imuHandler;

	if (tracksHandler)
		delete tracksHandler;

}

void FullSlamImu::run()
{
	ros::NodeHandle n("~");

	ros::Rate rate(5);

	while (ros::ok())
	{
		rate.sleep();

		ros::spinOnce();

		if (filter->getNthOldestPose(1))
		{
			//filter->getOldestPose()->setFixed(true);

			filter->getNthOldestPose(0)->setFixed(true);
			//filter->getNthOldestPose(1)->setFixed(true);

			ROS_INFO("Run estimation");
			bool ret = filter->estimate(20);

			assert(ret);
		}
	};

}

void FullSlamImu::imuCb(const sensor_msgs::Imu& msg)
{
	//ROS_INFO("imu callback");
	double t = msg.header.stamp.toSec();

// fill temporaries with measurements
	double za[] =
	{ msg.linear_acceleration.x, msg.linear_acceleration.y,
	msg.linear_acceleration.z };
	double zw[] =
	{ msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z };

	imuHandler->addMeasurement(za, zw, t);

}

void FullSlamImu::tracksCb(const c_slam_msgs::TrackedObject& msg)
{
	//ROS_INFO("Tracks callback");

	double t = msg.imageStamp.toSec();

// compute the center of mass
	Eigen::VectorXd z(2);
	z
				<< 0.25
							* (msg.polygon.points[0].x + msg.polygon.points[1].x
										+ msg.polygon.points[2].x
										+ msg.polygon.points[3].x), 0.25
				* (msg.polygon.points[0].y + msg.polygon.points[1].y
							+ msg.polygon.points[2].y + msg.polygon.points[3].y);

	static const Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(2,2);

	tracksHandler->addFeatureObservation(msg.id, t, z, cov);

}

void FullSlamImu::initRoamfree()
{
	filter = FactorGraphFilterFactory::getNewFactorGraphFilter();
	filter->setLowLevelLogging(true); // default log folder
	system("mkdir -p /tmp/roamfree/");
	system("rm -f /tmp/roamfree/*.log");
	filter->setDeadReckoning(false);
	filter->setSolverMethod(GaussNewton);
}

void FullSlamImu::initCamera()
{

	tracksHandler = new ROAMvision::FHPFeatureHandler(2.0);
	tracksHandler->setTimestampOffsetTreshold(5e-3);

	//the camera intrinsic calibration matrix
	Eigen::VectorXd CM(9);
	CM << 565.59102697808, 0.0, 337.839450567586, 0.0, 563.936510489792, 199.522081717361, 0.0, 0.0, 1.0;

	Eigen::VectorXd T_OC(7);
	T_OC << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

	tracksHandler->init(filter, "Track", T_OC, CM);
}

} /* namespace roamfree_c_slam */

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "roamfree_full_slam_imu");

	roamfree_c_slam::FullSlamImu n(argv[1]);

	ROS_INFO("Localization node started");
	n.run();
	ROS_INFO("Localization node shut down");

	return 0;
}
