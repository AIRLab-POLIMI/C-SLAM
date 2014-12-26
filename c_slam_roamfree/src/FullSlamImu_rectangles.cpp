/*
 * FullSlamImu.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#include "FullSlamImu_rectangles.h"

#include <visualization_msgs/Marker.h>

using namespace ROAMestimation;
using namespace std;

namespace roamfree_c_slam
{

FullSlamImu_rectangles::FullSlamImu_rectangles(FullSlamConfig& config) :
			FullSlamImu(config), tracksHandler(NULL)
{
	//setup the camera
	initCamera();

	//subscribe to sensor topics
	tracks_sub = n.subscribe("/tracks", 60000,
				&FullSlamImu_rectangles::tracksCb, this);
}

FullSlamImu_rectangles::~FullSlamImu_rectangles()
{
	if (tracksHandler)
		delete tracksHandler;
}

void FullSlamImu_rectangles::run()
{
	ros::NodeHandle n("~");

	ros::Rate rate(5);

	while (ros::ok())
	{
		//rate.sleep();

		ros::spinOnce();

		if (filter->getWindowLenght() > config.minWindowLenghtSecond
					&& tracksHandler->getNActiveFeatures() >= config.minActiveFeatures)
		{

			//filter->getNthOldestPose(0)->setFixed(true);

			double curTs = filter->getNewestPose()->getTimestamp();
			PoseVertexWrapper_Ptr cur;

			//ROS_INFO("Run estimation");
			bool ret = filter->estimate(config.iterationN);

			filter->forgetOldNodes(2.5);
		}

		if (filter->getOldestPose())
		{
			publishFeatureMarkers();
			publishPose();
		}
	};

}

void FullSlamImu_rectangles::tracksCb(const c_slam_msgs::TrackedObject& msg)
{
	//ROS_INFO("Tracks callback");

	double t = msg.imageStamp.toSec();

	// compute the center of mass
	Eigen::VectorXd z(8);
	z << msg.polygon.points[0].x, msg.polygon.points[0].y, msg.polygon.points[1].x, msg.polygon.points[1].y, msg.polygon.points[2].x, msg.polygon.points[2].y, msg.polygon.points[3].x, msg.polygon.points[3].y;

	static const Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(8, 8);

	tracksHandler->addFeatureObservation(msg.id, t, z, cov);

}

void FullSlamImu_rectangles::initCamera()
{

	tracksHandler = new RectangleHandler(config.initialScale);
	tracksHandler->setTimestampOffsetTreshold(1.0 / 5.0 / 2.0);

	tracksHandler->init(filter, "Track", config.T_O_CAMERA, config.K);
}

void FullSlamImu_rectangles::publishFeatureMarkers()
{
	std::vector<long int> ids;

	tracksHandler->getFeaturesIds(ids);

	for (int k = 0; k < ids.size(); ++k)
	{

		visualization_msgs::Marker msg;

		msg.header.stamp = ros::Time::now();
		msg.header.frame_id = "/world";
		msg.type = visualization_msgs::Marker::CUBE;
		msg.frame_locked = false;
		msg.ns = "c_slam";
		msg.id = k;
		msg.action = visualization_msgs::Marker::ADD;

		msg.color.r = 1.0;
		msg.color.g = 0.0;
		msg.color.b = 0.0;
		msg.color.a = 1.0;

		Eigen::VectorXd fw(7);
		Eigen::VectorXd dim(2);

		bool ret;

		ret = tracksHandler->getFeaturePoseInWorldFrame(ids[k], fw);
		assert(ret);
		ret = tracksHandler->getFeatureDimensions(ids[k], dim);
		assert(ret);

		msg.pose.position.x = fw(0);
		msg.pose.position.y = fw(1);
		msg.pose.position.z = fw(2);

		msg.pose.orientation.w = fw(3);
		msg.pose.orientation.x = fw(4);
		msg.pose.orientation.y = fw(5);
		msg.pose.orientation.z = fw(6);

		msg.scale.x = dim(0);
		msg.scale.y = dim(1);
		msg.scale.z = 0.05;

		markers_pub.publish(msg);

	}
}

} /* namespace roamfree_c_slam */

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "c_localization");

	roamfree_c_slam::FullSlamConfig config;

	roamfree_c_slam::FullSlamImu_rectangles n(config);

	ROS_INFO("Localization node started");
	n.run();
	ROS_INFO("Localization node shut down");

	return 0;
}
