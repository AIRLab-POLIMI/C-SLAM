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

FullSlamImu::FullSlamImu() :
			filter(NULL), imuHandler(NULL), tracksHandler(NULL)
{
	//setup roamfree
	initRoamfree();

	//setup the camera
	initCamera();

	//setup the handlers
	imuHandler = new ImuHandler(filter);
	tracksHandler = new TracksHandler(filter, T_OC_tf);

	//subscribe to sensor topics
	imu_sub = n.subscribe("/ardrone/imu", 6000, &FullSlamImu::imuCb, this);
	tracks_sub = n.subscribe("/tracks", 6000, &FullSlamImu::tracksCb, this);
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

	ros::Rate rate(0.1);

	while (ros::ok())
	{
		rate.sleep();

		ros::spinOnce();

		if (filter->getOldestPose())
		{
			filter->getOldestPose()->setFixed(true);
			filter->estimate(500);
		}
	};

}

void FullSlamImu::imuCb(const sensor_msgs::Imu& msg)
{
	ROS_INFO("imu callback");
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
	ROS_INFO("Tracks callback");

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

	tracksHandler->addMeasurement(t, msg.id, z);

}

void FullSlamImu::initRoamfree()
{
	filter = FactorGraphFilterFactory::getNewFactorGraphFilter();
	filter->setLowLevelLogging(true); // default log folder
	system("mkdir -p /tmp/roamfree/");
	system("rm -f /tmp/roamfree/*.log");
	filter->setDeadReckoning(false);
	filter->setSolverMethod(LevenbergMarquardt);
}

void FullSlamImu::initCamera()
{

	filter->addConstantParameter("Camera_SOx", 0.000, true);
	filter->addConstantParameter("Camera_SOy", 0.000, true);
	filter->addConstantParameter("Camera_SOz", 0.000, true);

	filter->addConstantParameter("Camera_qOSx", -0.5, true);
	filter->addConstantParameter("Camera_qOSy", 0.5, true);
	filter->addConstantParameter("Camera_qOSz", -0.5, true);

// the pose of the camera wrt odometric center
	T_OC_tf = tf::Transform(tf::Quaternion(-0.5, 0.5, -0.5, 0.5),
				tf::Vector3(0.0, 0.00, 0.00));

	Eigen::VectorXd CM(9); //the camera intrinsic calibration matrix
	CM << 565.59102697808, 0.0, 337.839450567586, 0.0, 563.936510489792, 199.522081717361, 0.0, 0.0, 1.0;

	filter->addConstantParameter(Matrix3D, "Camera_CM", CM, true);

}

} /* namespace roamfree_c_slam */

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "roamfree_full_slam_imu");

	roamfree_c_slam::FullSlamImu n;

	ROS_INFO("Localization node started");
	n.run();
	ROS_INFO("Localization node shut down");

	return 0;
}
