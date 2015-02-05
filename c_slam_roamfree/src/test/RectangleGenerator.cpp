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

#include "test/RectangleGenerator.h"

using namespace std;

namespace roamfree_c_slam
{

RectangleGenerator::RectangleGenerator(double w, double h) :
			w(w), h(h), x(0), y(0), z(0), roll(0), pitch(0), yaw(0)
{

}

void RectangleGenerator::setPosition(double x, double y, double z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

void RectangleGenerator::setRPY(double roll, double pitch, double yaw)
{
	this->roll = roll;
	this->pitch = pitch;
	this->yaw = yaw;
}

void RectangleGenerator::generateTrack(vector<Eigen::Vector4d>& track)
{
	Eigen::Quaterniond q;
	getQuaternion(q);

	Eigen::Vector3d t;
	getTranslation(t);

	for(int i = 0; i < 4; i++)
	{
		Eigen::Vector3d Mi_O;
		Eigen::Vector4d Mi_W;
		getObjectCoordinates(i, Mi_O);

		Mi_W.head<3>() = t + q.toRotationMatrix() * Mi_O;
		Mi_W(3) = 1;

		track.push_back(Mi_W);
	}


}

void RectangleGenerator::getObjectCoordinates(int i, Eigen::Vector3d& Mi_O)
{
	double x = ((i == 1 || i == 2) ? -1 : 1) * w / 2;
	double y = 0;
	double z = ((i >= 2) ? -1 : 1) * h / 2;

	Mi_O << x, y, z;
}

void RectangleGenerator::getQuaternion(Eigen::Quaterniond& q)
{
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

	q = rollAngle * yawAngle * pitchAngle;
}

void RectangleGenerator::getTranslation(Eigen::Vector3d& t)
{
	t << x, y, z;
}

}
