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

namespace roamfree_c_slam {

FullSlamImu::FullSlamImu() :
		_filter(NULL), _imu(NULL), _initialized(false) {
}

FullSlamImu::~FullSlamImu() {
	if (_filter != NULL) {
		delete _filter;
	}
	if (_imu != NULL) {
		delete _imu;
	}
}

void FullSlamImu::init() {

	/* configure roamfree */

	_filter = FactorGraphFilterFactory::getNewFactorGraphFilter();

	_filter->setLowLevelLogging(true); // default log folder
	system("rm /tmp/roamfree/*.log");

	_filter->setDeadReckoning(false);
	_filter->setSolverMethod(LevenbergMarquardt);

	/* configure IMU handler */

	double imu_N = 10;
	double imu_dt = 0.02; //ardrone imu runs at 50 Hz

	_imu = new ROAMimu::IMUIntegralHandler(imu_N, imu_dt); // instantiate the new handler

	_imu->getSensorNoises() = Eigen::Matrix<double, 6, 6>::Identity(); // init the sensor noises

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

	_imu_sub = n.subscribe("/ardrone/imu", 1024, &FullSlamImu::imuCb, this);

	_tracks_sub = n.subscribe("/tracks", 1024, &FullSlamImu::tracksCb, this);

}

void FullSlamImu::run() {
	ros::NodeHandle n("~");

	ros::Rate rate(0.2);

	while (ros::ok()) {
		ros::spinOnce();

		if (_filter->getNthOldestPose(1)) {

			_filter->getNthOldestPose(0)->setFixed(true);
			_filter->getNthOldestPose(1)->setFixed(true);

			_filter->estimate(25);
		}

		rate.sleep();
	};
}

void FullSlamImu::imuCb(const sensor_msgs::Imu& msg) {

	double t = msg.header.stamp.toSec();

	if (!_initialized) {

		// we have to initialize the IMUIntegralHandler

		Eigen::VectorXd accBias(3);  // Accelerometer and Gyroscope biases
		accBias << 0.0, 0.0, 0.0;
		Eigen::VectorXd gyroBias(3);
		gyroBias << 0.0, 0.0, 0.0;

		Eigen::VectorXd x0(7);
		x0 << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

		Eigen::VectorXd T_OS_IMU(7); // Transformation between Odometer and robot frame
		T_OS_IMU << 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

		_imu->init(_filter, "IMUintegral", T_OS_IMU, accBias, true, gyroBias, true,
				x0, t);

		// fix first pose to remove gauge freedom
		_filter->getOldestPose()->setFixed(true);

		_initialized = true;
	}

	// fill temporaries with measurements
	double za[] = { msg.linear_acceleration.x, msg.linear_acceleration.y,
			msg.linear_acceleration.z };
	double zw[] = { msg.angular_velocity.x, msg.angular_velocity.y,
			msg.angular_velocity.z };

	_imu->step(za, zw);
}

void FullSlamImu::tracksCb(const c_tracking::TrackedObject& msg) {

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
							+ msg.polygon.points[2].x + msg.polygon.points[3].x), 0.25
			* (msg.polygon.points[0].y + msg.polygon.points[1].y
					+ msg.polygon.points[2].y + msg.polygon.points[3].y);

	std::set<int>::iterator s_it = _tracks.find(msg.id);

	if (s_it == _tracks.end()) {

		// there must already exist a pose
		ROAMestimation::PoseVertexWrapper_Ptr pose_ptr =
				_filter->getNearestPoseByTimestamp(t);
		if (!pose_ptr) {
			return;
		}

		// we need to add a new sensor
		_filter->addSensor(sensor, ROAMestimation::ImagePlaneProjection, false,
				true);
		_filter->shareSensorFrame("Camera", sensor);
		_filter->shareParameter("Camera_CM", sensor + "_CM");

		// place the marker somewhere on the direction where it was seen

		double alpha = 2;

		const Eigen::Map<const Eigen::Matrix3d> cm(
				_filter->getParameterByName("Camera_CM")->getEstimate().data());
		Eigen::Matrix3d cm_inv; // the inverse of the camera intrinsic calibration matrix
		std::cout << cm(0, 0);

		cm_inv << 1.0 / cm(0, 0), 0.0, -cm(0, 2) / cm(0, 0), 0.0, 1.0 / cm(1, 1), -cm(
				1, 2) / cm(1, 1), 0.0, 0.0, 1.0;

		Eigen::Vector3d Limg; // the landmark on the image plane
		Limg << z(0), z(1), 1.0;

		Eigen::Vector3d Lc; // the landmark in the camera reference frame at given depth;
		Lc = alpha * cm_inv * Limg;

		// the pose of the odometric center wrt world
		const Eigen::VectorXd &x = pose_ptr->getEstimate();
		tf::Transform T_WO_tf(tf::Quaternion(x(4), x(5), x(6), x(3)),
				tf::Vector3(x(0), x(1), x(2)));

		// TODO: a little bit of messy code between eigen and tf
		tf::Vector3 Lc_tf(Lc(0), Lc(1), Lc(2));
		tf::Vector3 Lw_tf = T_WO_tf * _T_OC_tf * Lc_tf;

		Eigen::VectorXd Lw(3);
		Lw << Lw_tf.x(), Lw_tf.y(), Lw_tf.z();

		std::cout << "INITIAL Lw " << sensor << " " << Lw.transpose() << std::endl;

		//

		_filter->addConstantParameter(ROAMestimation::Euclidean3D, sensor + "_Lw",
				Lw, false);

		_filter->setRobustKernel(sensor, true, 0.1);

		_tracks.insert(msg.id);

	}

	Eigen::MatrixXd cov(2, 2);
	cov = Eigen::MatrixXd::Identity(2, 2);

	_filter->addSequentialMeasurement(sensor, t, z, cov);
}

} /* namespace roamfree_c_slam */

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "roamfree_full_slam_imu");

	roamfree_c_slam::FullSlamImu n;

	n.init();

	n.run();

	return 0;
}
