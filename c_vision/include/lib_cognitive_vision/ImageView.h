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
#include <opencv2/imgproc/imgproc.hpp>

#include "Cluster.h"
#include "Rectangle.h"
#include "Pole.h"

#include "DBSCAN.h"

class ImageView
{
public:
	ImageView();
	ImageView(std::string viewName);
	ImageView(std::size_t id);

	~ImageView();

	void display(cv::Mat& frame);

	inline void setClusters(std::vector<Cluster>* clusters)
	{
		this->clusters = clusters;
	}

	inline void setKeyPoints(std::vector<cv::KeyPoint>* keyPoints)
	{
		this->keyPoints = keyPoints;
	}

	inline void setPoints(std::vector<cv::Point>* points)
	{
		this->points = points;
	}

	inline void setRoll(double roll)
	{
		this->roll = roll;
	}

	inline void setVerticalLines(std::vector<cv::Vec4i>* verticalLines)
	{
		this->verticalLines = verticalLines;
	}

	inline void setHorizontalLines(std::vector<cv::Vec4i>* horizontalLines)
	{
		this->horizontalLines = horizontalLines;
	}

	inline void setRectangles(std::vector<Rectangle>* rectangles)
	{
		this->rectangles = rectangles;
	}

	inline void setPoles(std::vector<Pole>* poles)
	{
		this->poles = poles;
	}

private:
	void drawAxis(cv::Mat& input);
	void displayKeypointsResults(const std::vector<cv::KeyPoint>& keyPoints,
				cv::Mat& frame);
	void displayPointsResults(const std::vector<cv::Point>& points,
				cv::Mat& frame);
	void displayLineResults(std::vector<cv::Vec4i>& lines, cv::Mat& frame);
	void displayRectanglesResults(cv::Mat& frame);
	void displayPolesResults(cv::Mat& frame);
	void displayClustersResults(cv::Mat& frame);
	void writeFeaturesClassifications(cv::Mat& frame, Feature& feature);

	typedef std::vector<std::vector<cv::Point> > Contours;
	template<class T>
	void drawContours(cv::Mat& frame, std::vector<T>* features,
				cv::Scalar color)
	{
		Contours contours;

		typedef typename std::vector<T> FeatureVector;

		for (typename FeatureVector::iterator i = features->begin();
					i != features->end(); ++i)
		{
			contours.push_back(i->getPointsVector());
		}

		cv::drawContours(frame, contours, -1, color);
	}

private:
	std::string viewName;

	std::vector<cv::KeyPoint>* keyPoints;
	std::vector<cv::Point>* points;
	std::vector<cv::Vec4i>* verticalLines;
	std::vector<cv::Vec4i>* horizontalLines;

	std::vector<Cluster>* clusters;
	std::vector<Rectangle>* rectangles;
	std::vector<Pole>* poles;

	double roll;

};

#endif /* IMAGEVIEW_H_ */
