/*
 * c_slam_roamfree,
 *
 *
 * Copyright (C) 2015 Davide Tateo
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

#ifndef INCLUDE_TEST_RECTANGLEGENERATOR_H_
#define INCLUDE_TEST_RECTANGLEGENERATOR_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace roamfree_c_slam
{

class RectangleGenerator
{
public:
	RectangleGenerator(double w, double h);
	void setPosition(double x, double y, double z);
	void setRPY(double roll, double pitch, double yaw);

	void generateTrack(std::vector<std::vector<Eigen::Vector4d>>& trackVector,
				std::vector<Eigen::Vector3d>& trackCM);


private:
	void getObjectCoordinates(int i, Eigen::Vector3d& Mi_O);
	void getQuaternion(Eigen::Quaterniond& q);
	void getTranslation(Eigen::Vector3d& t);

private:
	//rectangle dimensions
	double w;
	double h;

	//rectanglePosition
	double x;
	double y;
	double z;

	//set RPY
	double roll;
	double pitch;
	double yaw;
};


}
#endif /* INCLUDE_TEST_RECTANGLEGENERATOR_H_ */
