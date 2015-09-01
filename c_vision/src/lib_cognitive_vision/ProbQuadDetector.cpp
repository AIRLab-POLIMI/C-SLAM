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

#include "ProbQuadDetector.h"
#include <random>
#include "../../include/lib_cognitive_vision/Lines.h"

using namespace cv;
using namespace std;

std::random_device ProbQuadDetector::rd;
std::mt19937 ProbQuadDetector::gen(ProbQuadDetector::rd());

ProbQuadDetector::ProbQuadDetector(QDetectorParam& quadP) :
			quadP(quadP)
{
	rectangles = new vector<Rectangle>();
	poles = new vector<Pole>();

	//TODO delete
	points = new vector<Point>();
}

unsigned int ProbQuadDetector::sampleCoordinate(unsigned int lo,
			unsigned int hi)
{
	std::uniform_real_distribution<> dist(lo, hi);
	return dist(gen);
}

bool ProbQuadDetector::crispScore(unsigned int x, unsigned int y,
			Vec4i& h1, Vec4i& h2, Vec4i& v1, Vec4i& v2, const Mat& canny)
{
	Vec2d p[4];
	p[0] = Lines::projectPoint(x, y, h1);
	p[1] = Lines::projectPoint(x, y, h2);
	p[2] = Lines::projectPoint(x, y, v1);
	p[3] = Lines::projectPoint(x, y, v2);

	for(unsigned int i = 0; i < 4; i++)
	{
		int xp = p[i][0];
		int yp = p[i][1];

		int xr = std::min(std::max(0, xp-1), canny.cols-1);
		int yr = std::min(std::max(0, yp-1), canny.rows-1);
		int w = std::min(canny.cols - xr, 3);
		int h = std::min(canny.rows - yr, 3);

		Rect roi(xr, yr, w, h);

		if(countNonZero(canny(roi)) == 0)
			return false;

	}

	return true;
}

double ProbQuadDetector::fuzzyScore(unsigned int x, unsigned int y,
			Vec4i& h1, Vec4i& h2, Vec4i& v1, Vec4i& v2)
{
	double dist[4];

	dist[0] = Lines::projectionDistanceFromSegment(x, y, h1);
	dist[1] = Lines::projectionDistanceFromSegment(x, y, h2);
	dist[2] = Lines::projectionDistanceFromSegment(x, y, v1);
	dist[3] = Lines::projectionDistanceFromSegment(x, y, v2);

	double score = 1.0;

	for(unsigned int i = 0; i < 4; i++)
	{
		score *= std::max(0.0,(quadP.maxDistance - dist[i])/quadP.maxDistance);
	}

	return score;
}

void ProbQuadDetector::voteRectangles(unsigned int x, unsigned int y,
			std::vector<cv::Vec4i>& verticalLines,
			std::vector<cv::Vec4i>& horizontalLines,
			const cv::Mat& canny)
{
	for (size_t i = 0; i + 1 < verticalLines.size(); i++)
	{
		//i+1 instead size-1 to avoid integer overflow
		Vec4i& v1 = verticalLines[i];
		if (Lines::atLeft(v1, x))
		{
			for (size_t k1 = i + 1; k1 < i + 3 && k1 < verticalLines.size();
						k1++)
			{
				Vec4i& v2 = verticalLines[k1];
				unsigned int midLine = Lines::getMidLine(v1, v2);
				if (Lines::atRight(v2, x))
				{
					for (size_t j = 0; j + 1 < horizontalLines.size(); j++)
					{
						Vec4i& h1 = horizontalLines[j];
						if (Lines::between_h(v1, v2, h1) && Lines::above(h1, y)
									&& Lines::above(h1, midLine))
						{
							for (size_t k2 = j + 1; k2 < horizontalLines.size();
										k2++)
							{
								Vec4i& h2 = horizontalLines[k2];
								if (Lines::between_h(v1, v2, h2)
											&& Lines::below(h2, y)
											&& Lines::below(h2, midLine))
								{

									bool ok = crispScore(x, y, h1, h2, v1, v2, canny);
									double score2 = fuzzyScore(x, y, h1, h2, v1, v2);

									//Dave test
									votes[i][k1][j][k2] += ok ? 1 : 0;
									counts[i][k1][j][k2] += 1;
								}
							}
						}
					}
				}
			}
		}
	}
}

void ProbQuadDetector::detect(std::vector<cv::Vec4i>& verticalLines,
			std::vector<cv::Vec4i>& horizontalLines, const Mat& canny)
{

	for (int n = 0; n < quadP.points; n++)
	{
		unsigned int x = sampleCoordinate(0, canny.cols - 1);
		unsigned int y = sampleCoordinate(0, canny.rows - 1);

		voteRectangles(x, y, verticalLines, horizontalLines, canny);

		//TODO delete me
		Point p(x,y);
		points->push_back(p);
	}

	for (auto& pair1 : votes)
	{

		for (auto& pair2 : pair1.second)
		{

			for (auto& pair3 : pair2.second)
			{

				for (auto& pair4 : pair3.second)
				{
					double hits = pair4.second;
					double tot = counts[pair1.first][pair2.first][pair3.first][pair4.first];

					if(hits/tot > quadP.threshold)
					{
						Vec4i& v1 = verticalLines[pair1.first];
						Vec4i& v2 = verticalLines[pair2.first];
						Vec4i& h1 = horizontalLines[pair3.first];
						Vec4i& h2 = horizontalLines[pair4.first];

						buildRectangle(v1, v2, h1, h2);
					}
				}
			}
		}
	}

}

void ProbQuadDetector::buildRectangle(cv::Vec4i& v1, cv::Vec4i& v2,
			cv::Vec4i& h1, cv::Vec4i& h2)
{
	Point px, py, pz, pw;

	px = Lines::findInterception(h1, v1);
	py = Lines::findInterception(h1, v2);
	pz = Lines::findInterception(h2, v2);
	pw = Lines::findInterception(h2, v1);

	Rectangle rectangle(px, py, pz, pw, quadP.omega);
	rectangles->push_back(rectangle);
}
