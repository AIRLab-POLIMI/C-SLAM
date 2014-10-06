/*
 * FullSlamImu.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: davide
 */

#include "FullSlamImu.h"

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <ROAMimu/IMUIntegralHandler.h>

namespace roamfree_c_slam
{

FullSlamImu::FullSlamImu() :
			_filter(NULL), _imu(NULL), _initialized(false), _initCalled(false)
{
}

FullSlamImu::~FullSlamImu()
{
	if (_filter != NULL)
	{
		delete _filter;
	}
	if (_imu != NULL)
	{
		delete _imu;
	}
}

void FullSlamImu::init()
{

	/* configure roamfree */

	_filter = FactorGraphFilterFactory::getNewFactorGraphFilter();

	_filter->setLowLevelLogging(true); // default log folder
	system("mkdir -p /tmp/roamfree/");
	system("rm -f /tmp/roamfree/*.log");

	_filter->setDeadReckoning(false);
	_filter->setSolverMethod(LevenbergMarquardt);

	/* configure IMU handler */

	double imu_N = 10;
	double imu_dt = 0.02; //ardrone imu runs at 50 Hz

	_imu = new ROAMimu::IMUIntegralHandler(imu_N, imu_dt); // instantiate the new handler

	_imu->getSensorNoises() = 1e-2*Eigen::Matrix<double, 6, 6>::Identity(); // init the sensor noises

	/* configure the camera	 */

	_filter->addConstantParameter("Camera_SOx", 0.210, true);
	_filter->addConstantParameter("Camera_SOy", 0.000, true);
	_filter->addConstantParameter("Camera_SOz", 0.000, true);

	_filter->addConstantParameter("Camera_qOSx", -0.5, true);
	_filter->addConstantParameter("Camera_qOSy", 0.5, true);
	_filter->addConstantParameter("Camera_qOSz", -0.5, true);

	// the pose of the camera wrt odometric center
	_T_OC_tf = tf::Transform(tf::Quaternion(-0.5, 0.5, -0.5, 0.5),
				tf::Vector3(0.21, 0.00, 0.00));

	Eigen::VectorXd CM(9); //the camera intrinsic calibration matrix
	CM << 565.59102697808, 0.0, 337.839450567586, 0.0, 563.936510489792, 199.522081717361, 0.0, 0.0, 1.0;

	_filter->addConstantParameter(Matrix3D, "Camera_CM", CM, true);

	/* subscribe to sensor topics */

	ros::NodeHandle n("~");

	_imu_sub = n.subscribe("/ardrone/imu", 2048, &FullSlamImu::imuCb, this);

	_tracks_sub = n.subscribe("/tracks", 1024, &FullSlamImu::tracksCb, this);

	_initCalled = true;

}

void FullSlamImu::run()
{
	ros::NodeHandle n("~");

	ros::Rate rate(0.1);

	while (ros::ok())
	{
		ros::spinOnce();

		if (_filter->getNthOldestPose(1))
		{

			_filter->getNthOldestPose(0)->setFixed(true);
			//_filter->getNthOldestPose(1)->setFixed(true);

			_filter->estimate(200);
		}

		rate.sleep();
	};

}

void computePose(Eigen::VectorXd& pose, double t, double theta0, double w0,
			double alpha, double r)
{
	double thetaRobot = theta0 + w0 * t + 0.5 * alpha * std::pow(t, 2);
	double theta = thetaRobot - M_PI / 2;

	double x = r * cos(theta);
	double y = r * sin(theta);
	double z = 0;

	Eigen::Matrix3d R;

	R << cos(thetaRobot), -sin(thetaRobot), 0, //
	sin(thetaRobot), cos(thetaRobot), 0, //
	0, 0, 1;

	Eigen::Quaterniond q(R);

	pose << x, y, z, q.w(), q.x(), q.y(), q.z();

}

void FullSlamImu::imuCb(const sensor_msgs::Imu& msg)
{
	ROS_INFO("imu callback");
	double t = msg.header.stamp.toSec();

	if (!_initialized)
	{

		// we have to initialize the IMUIntegralHandler

		Eigen::VectorXd accBias(3);  // Accelerometer and Gyroscope biases
		accBias << 0.0, 0.0, 0.0;
		Eigen::VectorXd gyroBias(3);
		gyroBias << 0.0, 0.0, 0.0;

		Eigen::VectorXd x0(7);
		x0 << 0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0;

		Eigen::VectorXd T_OS_IMU(7); // Transformation between Odometer and robot frame
		T_OS_IMU << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

		_imu->init(_filter, "IMUintegral", T_OS_IMU, accBias, true, gyroBias,
					true, x0, t);

		// fix first pose to remove gauge freedom
		_filter->getOldestPose()->setFixed(true);

		_initialized = true;
	}

	// fill temporaries with measurements
	double za[] =
	{ msg.linear_acceleration.x, msg.linear_acceleration.y,
	msg.linear_acceleration.z };
	double zw[] =
	{ msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z };

	if (_imu->step(za, zw))
	{
		//TODO delete this
		double r = 1; // meters
		double alpha = 0.01; // radians / s^2
		double w0 = 0.0; //initial angular speed
		double thetaRobot0 = 0;
		double imuRate = 50;
		Eigen::VectorXd pose(7);
		computePose(pose, t, thetaRobot0, w0, alpha, r);
		_filter->getNewestPose()->setEstimate(pose);

	}
}

void FullSlamImu::tracksCb(const c_slam_msgs::TrackedObject& msg)
{
	ROS_INFO("Tracks callback");
	double t = msg.imageStamp.toSec();

	// produce the sensor name
	std::stringstream s;
	s << "Track_" << msg.id;
	std::string sensor = s.str();

	// compute the center of mass
	Eigen::VectorXd z(2);
	z
				<< 0.25
							* (msg.polygon.points[0].x + msg.polygon.points[1].x
										+ msg.polygon.points[2].x
										+ msg.polygon.points[3].x), 0.25
				* (msg.polygon.points[0].y + msg.polygon.points[1].y
							+ msg.polygon.points[2].y + msg.polygon.points[3].y);

	std::set<int>::iterator s_it = _tracks.find(msg.id);

	if (s_it == _tracks.end())
	{

		// there must already exist a pose
		ROAMestimation::PoseVertexWrapper_Ptr pose_ptr =
					_filter->getNearestPoseByTimestamp(t);
		if (!pose_ptr)
		{
			return;
		}

		// we need to add a new sensor
		_filter->addSensor(sensor, ROAMestimation::ImagePlaneProjection, false,
					true);
		_filter->shareSensorFrame("Camera", sensor);
		_filter->shareParameter("Camera_CM", sensor + "_CM");

		// place the marker somewhere on the direction where it was seen

		const Eigen::Map<const Eigen::Matrix3d> cm(
					_filter->getParameterByName("Camera_CM")->getEstimate().data());
		Eigen::Matrix3d cm_inv = cm.transpose().inverse(); // the inverse of the camera intrinsic calibration matrix
		cm_inv /= cm_inv(2, 2);

		ROS_ERROR_STREAM("cm: " << std::endl << cm << std::endl);
		ROS_ERROR_STREAM("cm_inv: " << std::endl << cm_inv << std::endl);

		Eigen::Vector3d Limg; // the landmark on the image plane
		Limg << z(0), z(1), 1.0;

		ROS_ERROR_STREAM("Limg :" << std::endl << Limg << std::endl);

		Eigen::Vector3d dc; // the direction of the landmark in the camera reference frame
		dc = cm_inv * Limg;

		// the pose of the odometric center wrt world
		const Eigen::VectorXd &x = pose_ptr->getEstimate();
		tf::Transform T_WO_tf(tf::Quaternion(x(4), x(5), x(6), x(3)),
					tf::Vector3(x(0), x(1), x(2)));

		tf::Transform T_WC_tf = T_WO_tf * _T_OC_tf;

		tf::Matrix3x3 R_WC_tf = T_WC_tf.getBasis();
		tf::Vector3 t_WC_tf = T_WC_tf.getOrigin();

		Eigen::Matrix3d R_WC;
		Eigen::Vector3d t_WC;

		tf::matrixTFToEigen(R_WC_tf, R_WC);
		tf::vectorTFToEigen(t_WC_tf, t_WC);
		ROS_ERROR_STREAM("R_WC :" << std::endl << R_WC << std::endl);
		ROS_ERROR_STREAM("t_WC :" << std::endl << t_WC << std::endl);

		//Compute a possible pose for the landmark
		double alpha = 2;
		Eigen::VectorXd Lw(3);
		Lw = alpha * R_WC * dc + t_WC;

		ROS_ERROR_STREAM("INITIAL Lw " << sensor << " " << Lw.transpose());

		_filter->addConstantParameter(ROAMestimation::Euclidean3D,
					sensor + "_Lw", Lw, false);

		_filter->setRobustKernel(sensor, true, 0.1);

		_tracks.insert(msg.id);

	}

	Eigen::MatrixXd cov(2, 2);
	cov = Eigen::MatrixXd::Identity(2, 2);

	_filter->addSequentialMeasurement(sensor, t, z, cov);
}

} /* namespace roamfree_c_slam */

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "roamfree_full_slam_imu");

	roamfree_c_slam::FullSlamImu n;

	ROS_INFO("Localization node started");
	n.init();
	ROS_INFO("Localization node initialized");
	n.run();
	ROS_INFO("Localization node shut down");

	return 0;
}
