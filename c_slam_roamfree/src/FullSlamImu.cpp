/*
 * FullSlamImu.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#include "FullSlamImu.h"

#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

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

	//setup the imu handler
	initIMU();

	//subscribe to sensor topics
	imu_sub = n.subscribe(imuTopic, 60000, &FullSlamImu::imuCb, this);
	tracks_sub = n.subscribe("/tracks", 60000, &FullSlamImu::tracksCb, this);
	markers_pub = n.advertise<visualization_msgs::Marker>(
				"/visualization/features", 1);
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
		//rate.sleep();

		ros::spinOnce();

		if (filter->getWindowLenght() > 1.0 && tracksHandler->getNActiveFeatures() >= 3)
		{
			filter->getNthOldestPose(0)->setFixed(true);

			int cnt = 1;
			double curTs = filter->getNewestPose()->getTimestamp();
			PoseVertexWrapper_Ptr cur;

			while ((cur=filter->getNthOldestPose(cnt))->getTimestamp() < curTs - 5.0) {
				cur->setFixed(true);
				cnt++;
			}

			ROS_INFO("Run estimation");
			bool ret = filter->estimate(iterationN);
		}

		if (filter->getOldestPose()) {
			publishFeatureMarkers();
			publishCameraPose();
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

	static const Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(2, 2);

	tracksHandler->addFeatureObservation(msg.id, t, z, cov);

}

void FullSlamImu::initRoamfree()
{
	filter = FactorGraphFilterFactory::getNewFactorGraphFilter();
	filter->setLowLevelLogging(false); // default log folder
	system("mkdir -p /tmp/roamfree/");
	system("rm -f /tmp/roamfree/*.log");
	filter->setDeadReckoning(false);
	filter->setSolverMethod(GaussNewton);
}

void FullSlamImu::initCamera()
{

	tracksHandler = new ROAMvision::FHPFeatureHandler(10.0);
	tracksHandler->setTimestampOffsetTreshold(5e-3);

	//the camera intrinsic calibration matrix
	Eigen::VectorXd CM(9);
	CM << 565.59102697808, 0.0, 337.839450567586, 0.0, 563.936510489792, 199.522081717361, 0.0, 0.0, 1.0;

	Eigen::VectorXd T_OC(7);
	T_OC << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

	tracksHandler->init(filter, "Track", T_OC, CM);
}

void FullSlamImu::initIMU()
{
	//setup the handlers
	imuHandler = new ImuHandler(filter, false, false);

	//Firefly initial pose and sensor pose
	/*Eigen::VectorXd T_OS_IMU(7), x0(7);
	T_OS_IMU << 0.0, 0.0, 0.0, 0.5, 0.5, -0.5, 0.5;
	x0 << 0.0, 0.0, 0.0, 0.5, -0.5, 0.5, -0.5;
	imuHandler->setSensorframe(T_OS_IMU, x0);

	//Firefly bias
	Eigen::VectorXd accBias(3);
	accBias << 0.1939, 0.0921, -0.2989;
	Eigen::VectorXd gyroBias(3);
	gyroBias << -0.0199, 0.0077, -0.0099;
	imuHandler->setBias(accBias, gyroBias);*/
}

void FullSlamImu::publishFeatureMarkers()
{
	std::vector<long int> ids;

	tracksHandler->getFeaturesIds(ids);

	visualization_msgs::Marker msg;

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "/world";
	msg.type = visualization_msgs::Marker::CUBE_LIST;
	//msg.lifetime = ros::Duration(0.2);
	msg.frame_locked = false;
	msg.ns = "roamfree_visualodometry";
	msg.id = 0;
	msg.action = visualization_msgs::Marker::ADD;

	msg.color.r = 1.0;
	msg.color.g = 0.0;
	msg.color.b = 0.0;
	msg.color.a = 1.0;

	msg.scale.x = 0.30;
	msg.scale.y = 0.30;
	msg.scale.z = 0.30;

	msg.pose.position.x = 0.0;
	msg.pose.position.y = 0.0;
	msg.pose.position.z = 0.0;

	msg.pose.orientation.w = 1.0;

	msg.points.resize(ids.size());

	for (int k = 0; k < ids.size(); ++k)
	{
		Eigen::VectorXd fw(3);

		tracksHandler->getFeaturePositionInWorldFrame(ids[k], fw);

		msg.points[k].x = fw(0);
		msg.points[k].y = fw(1);
		msg.points[k].z = fw(2);
	}

	markers_pub.publish(msg);
}

void FullSlamImu::publishCameraPose()
{
	ROAMestimation::PoseVertexWrapper_Ptr cameraPose_ptr =
				filter->getNewestPose();
	const Eigen::VectorXd &camera = cameraPose_ptr->getEstimate();

	tf::Transform T_WR_tf(
				tf::Quaternion(camera(4), camera(5), camera(6), camera(3)),
				tf::Vector3(camera(0), camera(1), camera(2)));

	pose_tf_br.sendTransform(
				tf::StampedTransform(T_WR_tf,
							ros::Time(cameraPose_ptr->getTimestamp()), "world",
							"camera_link"));

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
