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

#ifndef INCLUDE_PROBQUADDETECTOR_H_
#define INCLUDE_PROBQUADDETECTOR_H_

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "ParameterServer.h"

#include "Rectangle.h"
#include "Pole.h"

#include <random>

class ProbQuadDetector
{
	template<int N, typename type>
	struct vote_s
	{
		typedef std::map<unsigned int, typename vote_s<N-1, type>::t_multimap> t_multimap;
	};

	template<typename type>
	struct vote_s<1, type>
	{
		typedef std::map<unsigned int, type> t_multimap;
	};

	typedef vote_s<4, int>::t_multimap t_vote;
	typedef vote_s<4, double>::t_multimap t_counter;


public:
	ProbQuadDetector(QDetectorParam& quadP);

	void detect(std::vector<cv::Vec4i>& verticalLines,
				std::vector<cv::Vec4i>& horizontalLines);

	inline std::vector<Pole>* getPoles() const
	{
		return poles;
	}

	inline std::vector<Rectangle>* getRectangles() const
	{
		return rectangles;
	}

private:
	unsigned int sampleCoordinate(unsigned int lo, unsigned int hi);
	void buildRectangle(cv::Vec4i& v1, cv::Vec4i& v2, cv::Vec4i& h1, cv::Vec4i& h2);
	void voteRectangles(unsigned int x, unsigned int y,
				std::vector<cv::Vec4i>& verticalLines,
				std::vector<cv::Vec4i>& horizontalLines);

private:
	std::vector<Rectangle>* rectangles;
	std::vector<Pole>* poles;

	t_vote votes;
	t_counter counts;

	QDetectorParam& quadP;

	//RANDOM STUFF
	static std::random_device rd;
	static std::mt19937 gen;
};

#endif /* INCLUDE_PROBQUADDETECTOR_H_ */
