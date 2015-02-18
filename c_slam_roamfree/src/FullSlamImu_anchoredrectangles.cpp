/*
 * FullSlamImu.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#include "FullSlamImu_anchoredrectangles.h"

#include <visualization_msgs/Marker.h>
#include <roscpp/Empty.h>

using namespace ROAMestimation;
using namespace ROAMvision;
using namespace std;

namespace roamfree_c_slam
{

FullSlamImu_anchoredrectangles::FullSlamImu_anchoredrectangles(
			FullSlamConfig& config) :
			FullSlamImu(config), tracksHandler(NULL)
{
	//setup the camera
	initCamera();

	//subscribe to sensor topics
	tracks_sub = n.subscribe("/objects", 60000,
				&FullSlamImu_anchoredrectangles::tracksCb, this);
}

FullSlamImu_anchoredrectangles::~FullSlamImu_anchoredrectangles()
{
	if (tracksHandler)
		delete tracksHandler;
}

void FullSlamImu_anchoredrectangles::run()
{
	ros::NodeHandle n("~");

	ros::Rate rate(5);

	while (ros::ok() && !tracksHandler->bootstrapCompleted())
	{
		ros::spinOnce();

		if (filter->getOldestPose()
		/*&& tracksHandler->getNActiveFeatures() >= 3*/)
		{
			filter->getOldestPose()->setFixed(true);
			tracksHandler->fixImmutableFeaturePoses(
						filter->getOldestPose()->getEstimate(), 1.0);

			//ROS_INFO("Run estimation");
			/*std::cerr << "Run estimation" << std::endl;
			 bool ret = filter->estimate(config.iterationN);*/
		}

		if (filter->getOldestPose())
		{
			publishFeatureMarkers();
			publishPose();
		}

	}

	std::cerr << ("Bootstrap completed") << std::endl;

	while (ros::ok())
	{
		rate.sleep();

		ros::spinOnce();

		if (filter->getWindowLenght() > config.minWindowLenghtSecond
					&& tracksHandler->getNActiveFeatures() >= 3)
		{
			filter->getNthOldestPose(0)->setFixed(true);

			//ROS_INFO("Run estimation");

			roscpp::Empty answ;

			ros::service::call("/pause_bag", answ);
			bool ret = filter->estimate(config.iterationN);
			ros::service::call("/unpause_bag", answ);
		}

		if (filter->getOldestPose())
		{
			publishFeatureMarkers();
			publishPose();
		}
	};

}

void FullSlamImu_anchoredrectangles::tracksCb(
			const c_slam_msgs::NamedPolygon& msg)
{
	//ROS_INFO("Tracks callback");

	double t = msg.header.stamp.toSec();

	// compute the center of mass
	Eigen::VectorXd z(8);
	z << msg.polygon.points[0].x, msg.polygon.points[0].y, msg.polygon.points[1].x, msg.polygon.points[1].y, msg.polygon.points[2].x, msg.polygon.points[2].y, msg.polygon.points[3].x, msg.polygon.points[3].y;

	static const Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(8, 8);

	tracksHandler->addFeatureObservation(msg.id, t, z, cov);

}

void FullSlamImu_anchoredrectangles::initCamera()
{

	tracksHandler = new AnchoredRectangleHandlerBootstrap(config.initialScale);
	tracksHandler->setTimestampOffsetTreshold(1.0 / 5.0 / 2.0);

	tracksHandler->init(filter, "Track", config.T_O_CAMERA, config.K);
}

void FullSlamImu_anchoredrectangles::publishFeatureMarkers()
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

		/* debug output
		 cout << "Feature " << ids[k] << " dim (" << dim(0) << "," << dim(1) << ")" << endl;
		 cout << "Feature " << ids[k] << " pos (" << fw(0) << "," << fw(1) << "," << fw(2) << ")" << endl;
		 */

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

	try
	{
		// wait for the service to appear
		// (but NOT with ros::Duration(xx)::sleep(), use_sim_time might be set)

		ROS_INFO("wait for the bag_controller");
		while (!ros::service::exists("/unpause_bag", false))
		{
			sleep(1.0);
		}

		ROS_INFO("starting playback");
		roscpp::Empty answ;
		ros::service::call("/unpause_bag", answ);

		roamfree_c_slam::FullSlamConfig config;

		roamfree_c_slam::FullSlamImu_anchoredrectangles n(config);

		ROS_INFO("Localization node started");
		n.run();
		ROS_INFO("Localization node shut down");

	}
	catch (std::runtime_error& error)
	{
		return -1;
	}

	return 0;
}
