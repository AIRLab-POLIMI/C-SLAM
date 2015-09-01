/*
 * c_vision,
 *
 *
 * Copyright (C) 2015 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_vision.
 *
 * c_vision is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_vision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_vision.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef INCLUDE_LIB_COGNITIVE_VISION_LINES_H_
#define INCLUDE_LIB_COGNITIVE_VISION_LINES_H_

#include <opencv2/opencv.hpp>

class Lines
{
public:
	inline static void getPointsCoordinates(cv::Vec4i l, cv::Point& i,
				cv::Point& j)
	{
		i = cv::Point(l[0], l[1]);
		j = cv::Point(l[2], l[3]);
	}

	inline static void getPointsCoordinates(cv::Vec4i l, cv::Vec3d& i,
				cv::Vec3d& j)
	{
		i = cv::Vec3d(l[0], l[1], 1);
		j = cv::Vec3d(l[2], l[3], 1);
	}

	inline static cv::Point findInterception(cv::Vec4i l1, cv::Vec4i l2)
	{
		cv::Vec3d p0, p1, p2, p3;
		cv::Vec3d hl, vl;
		cv::Vec3d p;

		//get the homogeneus coordinates
		getPointsCoordinates(l1, p0, p1);
		getPointsCoordinates(l2, p2, p3);

		//get the lines and the interception point
		hl = p0.cross(p1);
		hl = hl / norm(hl);

		vl = p2.cross(p3);
		vl = vl / norm(vl);

		p = hl.cross(vl);
		p = p / p[2];

		return cv::Point(p[0], p[1]);
	}

	inline static bool between_h(cv::Vec4i& v1, cv::Vec4i& v2, cv::Vec4i& h)
	{
		int min1 = std::min(v1[0], v1[2]);
		int min2 = std::min(v2[0], v2[2]);
		int minx = std::min(min1, min2);

		int max1 = std::max(v1[0], v1[2]);
		int max2 = std::max(v2[0], v2[2]);
		int maxx = std::max(max1, max2);

		int minh = std::min(h[0], h[2]);
		int maxh = std::max(h[0], h[2]);

		bool isAtLeft = maxh < minx;
		bool isAtRight = minh > maxx;

		return !(isAtLeft || isAtRight);
	}

	inline static bool above(const cv::Vec4i& l, unsigned int y)
	{
		return l[1] < y && l[3] < y;
	}

	inline static bool below(const cv::Vec4i& l, unsigned int y)
	{
		return l[1] > y && l[3] > y;
	}

	inline static bool atLeft(const cv::Vec4i& l, unsigned int x)
	{
		return l[0] < x && l[2] < x;
	}

	inline static bool atRight(const cv::Vec4i& l, unsigned int x)
	{
		return l[0] > x && l[2] > x;
	}

	inline static unsigned int getMidLine(const cv::Vec4i& v1,
				const cv::Vec4i& v2)
	{
		return (v1[1] + v1[3] + v2[1] + v2[3]) / 4;
	}

	inline static cv::Vec2d projectPoint(unsigned int x, unsigned int y,
				const cv::Vec4i& l)
	{
		cv::Point a, b;

		getPointsCoordinates(l, a, b);

		cv::Vec2d v(a.x - b.x, a.y - b.y);
		cv::Vec2d p(x, y);
		cv::Vec2d p0(a.x, a.y);
		cv::Vec2d p1(b.x, b.y);

		cv::Matx22d k = (v * v.t()) * (1.0 / (v.t() * v)[0]);

		return k * (p - p0) + p0;

	}

	inline static double projectionDistanceFromSegment(unsigned int x,
				unsigned int y, const cv::Vec4i& l)
	{
		cv::Point a, b;

		getPointsCoordinates(l, a, b);

		cv::Vec2d v(a.x - b.x, a.y - b.y);
		cv::Vec2d p(x, y);
		cv::Vec2d p1(a.x, a.y);
		cv::Vec2d p2(b.x, b.y);

		double test1 = -(v.t()*(p - p1))[0];
		double test2 = (v.t()*(p - p2))[0];

		double dist = std::min(test1, test2);

		if(dist >= 0)
		{
			return 0.0;
		}
		else
		{
			return -dist / cv::norm(v);
		}

	}

};

#endif /* INCLUDE_LIB_COGNITIVE_VISION_LINES_H_ */
