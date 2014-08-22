/*
 * c_vision,
 *
 *
 * Copyright (C) 2014 Davide Tateo
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

#include "MetricRectification.h"

using namespace cv;
using namespace std;

void metric_rectification::findLine(const cv::Point& a, const cv::Point& b,
			cv::Vec3d& l)
{
	cv::Vec3d ah(a.x, a.y, 1);
	cv::Vec3d bh(b.x, b.y, 1);

	l = ah.cross(bh);
	l = l / norm(l);
}

Mat metric_rectification::metricRectify(const Mat& K, Vec3d& v1, Vec3d& v2)
{

	Mat Cinf;
	Mat W = (K * K.t()).inv();
	//  find the line at the infinity of the plane
	Vec3d linf = v1.cross(v2);
	linf = linf / norm(linf);

	// find the circular points of the plane
	findConicDualCircularPoints(W, linf, Cinf);

	//find the euclidean rectification
	return findHomography(Cinf);

}

void metric_rectification::findConicDualCircularPoints(const Mat& W,
			const Vec3d& linf, Mat& Cinf)
{
	vector<complex<double> > I, J;

	intersectConicLine(W, linf, I, J);

	Cinf = scalarProduct(I, J) + scalarProduct(J, I);
}

Mat metric_rectification::scalarProduct(const vector<complex<double> >& I,
			const vector<complex<double> >& J)
{
	Mat_<double> P(3, 3);
	for (int i = 0; i < I.size(); i++)
		for (int j = 0; j < J.size(); j++)
			P(i, j) = real(I[i] * J[j]);
	return P;
}

Mat metric_rectification::findHomography(const Mat& Cinf)
{
	//find the euclidean rectification
	Mat_<double> H, U, V, S, Sr(3, 3);
	SVD::compute(Cinf, S, U, V);
	S(2, 2) = 100;
	Sr.setTo(0);
	Sr(0, 0) = sqrt(S(0, 0));
	Sr(1, 1) = sqrt(S(0, 1));
	Sr(2, 2) = 10;
	H = (U * Sr).inv();
	H = H / H(2, 2);

	return H;
}

void metric_rectification::intersectConicLine(const Mat& C, const Vec3d& l,
			vector<complex<double> >& I, vector<complex<double> >& J)
{
	Vec3d p1, p2;
	// take the generic point p = p1 + t*p2
	getPointsOnLine(l, p1, p2);

	// the point is on the conic c if p'*C*p=0
	// the equation is: a*t^2 + 2*b*t + c = 0
	Mat_<double> af = (Mat(p2).t() * C * Mat(p2));
	Mat_<double> bf = (Mat(p1).t() * C * Mat(p2));
	Mat_<double> cf = (Mat(p1).t() * C * Mat(p1));

	complex<double> a(af(0, 0), 0);
	complex<double> b(bf(0, 0), 0);
	complex<double> c(cf(0, 0), 0);

	// compute the two t
	complex<double> deltaSqrt = sqrt(pow(b, 2) - a * c);
	complex<double> t1 = (-b + deltaSqrt) / a;
	complex<double> t2 = (-b - deltaSqrt) / a;

	//compute the two points
	for (int i = 0; i < 3; i++)
	{
		I.push_back(p1[i] + t1 * p2[i]);
		J.push_back(p1[i] + t2 * p2[i]);
	}
}

void metric_rectification::getPointsOnLine(const Vec3d& l, Vec3d& p1, Vec3d& p2)
{
	if (l[0] == 0 && l[1] == 0) //line at infinity
	{
		p1 = Vec3d(1, 0, 0);
		p2 = Vec3d(0, 1, 0);
	}
	else
	{
		p2 = Vec3d(-l[1], l[0], 0);
		if (abs(l[0]) < abs(l[1]))
			p1 = Vec3d(0, -l[2], l[1]);
		else
			p1 = Vec3d(-l[2], 0, l[0]);

	}
}

