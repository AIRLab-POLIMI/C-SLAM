/*
 * FullSlamImu.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#include "FullSlamImu_FHP.h"

#include <visualization_msgs/Marker.h>

using namespace ROAMestimation;

namespace roamfree_c_slam
{

FullSlamImu_FHP::FullSlamImu_FHP(FullSlamConfig& config) :
			FullSlamImu(config), tracksHandler(NULL)
{
	//setup the camera
	initCamera();

	//subscribe to sensor topics
	tracks_sub = n.subscribe("/tracks", 60000, &FullSlamImu_FHP::tracksCb,
				this);
}

FullSlamImu_FHP::~FullSlamImu_FHP()
{
	if (tracksHandler)
		delete tracksHandler;
}

void FullSlamImu_FHP::run()
{
	ros::NodeHandle n("~");

	ros::Rate rate(5);

	while (ros::ok())
	{
		//rate.sleep();

		ros::spinOnce();

		if (filter->getWindowLenght() > config.minWindowLenghtSecond
					&& tracksHandler->getNActiveFeatures() >= 3)
		{
			filter->getNthOldestPose(0)->setFixed(true);

			//ROS_INFO("Run estimation");
			bool ret = filter->estimate(config.iterationN);
		}

		if (filter->getOldestPose())
		{
			publishFeatureMarkers();
			publishPose();
		}
	};

}

void FullSlamImu_FHP::tracksCb(const c_slam_msgs::TrackedObject& msg)
{
	//ROS_INFO("Tracks callback");

	double t = msg.imageStamp.toSec();

	static const Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(2, 2);

	/* compute the center of mass FIXME levami
	 Eigen::VectorXd z(2);

	 z
	 << 0.25
	 * (msg.polygon.points[0].x + msg.polygon.points[1].x
	 + msg.polygon.points[2].x + msg.polygon.points[3].x), 0.25
	 * (msg.polygon.points[0].y + msg.polygon.points[1].y
	 + msg.polygon.points[2].y + msg.polygon.points[3].y);

	 tracksHandler->addFeatureObservation(msg.id, t, z, cov);
	 //*/

	//
	for (int k = 0; k < 4; k++)
	{
		Eigen::VectorXd z(2);

		z << msg.polygon.points[k].x, msg.polygon.points[k].y;

		tracksHandler->addFeatureObservation(msg.id * 4 + k, t, z, cov);
	}
	//*/

}

void FullSlamImu_FHP::initCamera()
{
	tracksHandler = new ROAMvision::FHPFeatureHandler(config.initialScale);
	tracksHandler->setTimestampOffsetTreshold(
				1.0 / 2.0 / imuHandler->getPoseRate());

	tracksHandler->init(filter, "Track", config.T_O_CAMERA, config.K);
}

void FullSlamImu_FHP::publishFeatureMarkers()
{
	std::vector<long int> ids;

	tracksHandler->getFeaturesIds(ids);

	visualization_msgs::Marker msg;

	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = "/world";
	msg.type = visualization_msgs::Marker::CUBE_LIST;
	msg.frame_locked = false;
	msg.ns = "c_slam";
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

} /* namespace roamfree_c_slam */

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "c_localization");

	try
	{
		roamfree_c_slam::FullSlamConfig config;

		ROS_INFO("Configuration loaded");

		roamfree_c_slam::FullSlamImu_FHP n(config);

		ROS_INFO("Localization node started");

		n.run();

		ROS_INFO("Localization node shut down");
	}
	catch (std::runtime_error& e)
	{
		return -1;
	}

	return 0;
}
