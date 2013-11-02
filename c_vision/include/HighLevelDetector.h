/*
 * c_vision,
 *
 *
 * Copyright (C) 2013 Davide Tateo
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

#ifndef HIGHLEVELDETECTOR_H_
#define HIGHLEVELDETECTOR_H_

#include <vector>
#include <opencv2/core/core.hpp>

class HighLevelDetector
{
public:
	void detect(std::vector<cv::Vec4i> verticalLines,
			std::vector<cv::Vec4i> horizontalLines);

	const std::vector<std::vector<cv::Point> >& getPoles() const
	{
		return poles;
	}

	const std::vector<std::vector<cv::Point> >& getSquares() const
	{
		return squares;
	}

private:
	cv::Point findInterceptions(cv::Vec4i l1, cv::Vec4i l2);

private:
	std::vector<std::vector<cv::Point> > squares;
	std::vector<std::vector<cv::Point> > poles;

};

#endif /* HIGHLEVELDETECTOR_H_ */
