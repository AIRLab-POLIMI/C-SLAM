/*
 * c_slam_roamfree,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_slam_roamfree.
 *
 * c_slam_roamfree is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_slam_roamfree is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_slam_roamfree.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FullSlamImu.h"

#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>

#include "NoiseGeneration.h"

using namespace ROAMestimation;

namespace roamfree_c_slam {

FullSlamImu::FullSlamImu(FullSlamConfig& config) :
    config(config), filter(NULL), imuHandler(NULL) {
  //setup roamfree
  initRoamfree();

  //setup the imu handler
  initIMU();

  //subscribe to sensor topics
  imu_sub = n.subscribe(config.imuTopic, 60000, &FullSlamImu::imuCb, this);

  //advertise markers topic
  markers_pub = n.advertise<visualization_msgs::Marker>(
      "/visualization/features", 1);

}

FullSlamImu::~FullSlamImu() {
  if (filter)
    delete filter;

  if (imuHandler)
    delete imuHandler;

}

void FullSlamImu::publishPose() {
  ROAMestimation::PoseVertexWrapper_Ptr cameraPose_ptr =
      filter->getNewestPose();
  const Eigen::VectorXd &camera = cameraPose_ptr->getEstimate();

  tf::Transform T_WR_tf(
      tf::Quaternion(camera(4), camera(5), camera(6), camera(3)),
      tf::Vector3(camera(0), camera(1), camera(2)));

  br.sendTransform(
      tf::StampedTransform(T_WR_tf, ros::Time(cameraPose_ptr->getTimestamp()),
          "world", config.trackedFrame));

}

void FullSlamImu::imuCb(const sensor_msgs::Imu& msg) {
  //ROS_INFO("imu callback");
  double t = msg.header.stamp.toSec();

  Eigen::VectorXd eig_za(3), eig_zw(3);

  // fill temporaries with measurements
  eig_za << msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z;
  eig_zw << msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z;

  eig_za += config.acc_generated_bias;
  eig_zw += config.gyro_generated_bias;

  add_gaussian_noise(eig_za.data(), 3, 0.0, config.acc_stdev);
  add_gaussian_noise(eig_zw.data(), 3, 0.0, config.gyro_stdev);

  imuHandler->addMeasurement(eig_za.data(), eig_zw.data(), t);
}

void FullSlamImu::initRoamfree() {
  filter = FactorGraphFilterFactory::getNewFactorGraphFilter();
  filter->setLowLevelLogging(true); // default log folder
  system("mkdir -p /tmp/roamfree/");
  system("rm -f /tmp/roamfree/*.log");
  filter->setDeadReckoning(false);

  if (config.useGaussNewton) {
    filter->setSolverMethod(GaussNewton);
  } else {
    filter->setSolverMethod(LevenbergMarquardt);
  }
}

void FullSlamImu::initIMU() {
  //setup the handlers
  imuHandler = new ImuHandler(filter, config.imu_N, config.imu_dt,
      config.isAccBiasFixed, config.isGyroBiasFixed);

  imuHandler->setSensorframe(config.T_O_IMU, config.x0);

  if (config.accBias.size() == 3 && config.gyroBias.size() == 3) {
    imuHandler->setBias(config.accBias, config.gyroBias);
  } else {
    imuHandler->setBias(config.acc_generated_bias, config.gyro_generated_bias);
  }
}

}
