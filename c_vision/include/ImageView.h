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

#ifndef IMAGEVIEW_H_
#define IMAGEVIEW_H_

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Cluster.h"
#include "Rectangle.h"
#include "Pole.h"

#include "DBSCAN.h"

class ImageView
{
public:
	ImageView(std::string viewName);

	~ImageView();

	void display(cv::Mat& frame);

	inline void setClusters(const std::vector<Cluster>* clusters)
	{
		this->clusters = clusters;
	}

	inline void setKeyPoints(const std::vector<cv::KeyPoint>* keyPoints)
	{
		this->keyPoints = keyPoints;
	}

	inline void setRoll(double roll)
	{
		this->roll = roll;
	}

	inline void setVerticalLines(const std::vector<cv::Vec4i>* verticalLines)
	{
		this->verticalLines = verticalLines;
	}

	inline void setHorizontalLines(
				const std::vector<cv::Vec4i>* horizontalLines)
	{
		this->horizontalLines = horizontalLines;
	}

	inline void setRectangles(const std::vector<Rectangle>* rectangles)
	{
		this->rectangles = rectangles;
	}

	inline void setPoles(const std::vector<Pole>* poles)
	{
		this->poles = poles;
	}

private:
	void drawAxis(cv::Mat& input);
	void displayClusterResults(const std::vector<cv::KeyPoint>& keyPoints,
				const std::vector<Cluster>& clusters, cv::Mat& frame);
	void displayLineResults(std::vector<cv::Vec4i>& lines, cv::Mat& frame);
	void displayRectanglesResults(cv::Mat& frame);
	void displayPoleResults(cv::Mat& frame);

	typedef std::vector<std::vector<cv::Point> > Contours;
	template<class T>
	void drawContours(cv::Mat& frame, const std::vector<T>* features)
	{
		Contours contours;

		typedef typename std::vector<T> FeatureVector;

		for (typename FeatureVector::const_iterator i = features->begin();
					i != features->end(); ++i)
		{
			contours.push_back(i->getPointsVector());
		}

		cv::drawContours(frame, contours, -1, cv::Scalar(0, 255, 0)); //FIXME!!!
	}

private:
	std::string viewName;

	const std::vector<cv::KeyPoint>* keyPoints;
	const std::vector<cv::Vec4i>* verticalLines;
	const std::vector<cv::Vec4i>* horizontalLines;

	const std::vector<Cluster>* clusters;
	const std::vector<Rectangle>* rectangles;
	const std::vector<Pole>* poles;

	double roll;

};

#endif /* IMAGEVIEW_H_ */
