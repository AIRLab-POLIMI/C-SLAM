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

#include "DBSCAN.h"

class ImageView
{
public:
	ImageView(std::string viewName, void* featureObject, void* clusterObjetc,
				void* lineObject) :
				viewName(viewName)
	{
		cv::namedWindow(viewName);
		createTrackBars(featureObject, clusterObjetc, lineObject);
	}

	~ImageView()
	{
		cv::destroyWindow(viewName);
	}

	void display(cv::Mat& frame);

	void setClusters(const std::vector<Cluster>* clusters)
	{
		this->clusters = clusters;
	}

	void setKeyPoints(const std::vector<cv::KeyPoint>* keyPoints)
	{
		this->keyPoints = keyPoints;
	}

	void setRoll(double roll)
	{
		this->roll = roll;
	}

	void setVerticalLines(const std::vector<cv::Vec4i>* verticalLines)
	{
		this->verticalLines = verticalLines;
	}

	void setHorizontalLines(const std::vector<cv::Vec4i>* horizontalLines)
	{
		this->horizontalLines = horizontalLines;
	}

	void setRectangles(const std::vector<std::vector<cv::Point> >* rectangles)
	{
		this->rectangles = rectangles;
	}

	void setPoles(const std::vector<std::vector<cv::Point> >* poles)
	{
		this->poles = poles;
	}

private:
	void drawAxis(cv::Mat& input);
	void displayClusterResults(const std::vector<cv::KeyPoint>& keyPoints,
				const std::vector<Cluster>& clusters, cv::Mat& frame);
	void displayLineResults(std::vector<cv::Vec4i>& lines, cv::Mat& frame);
	void displayRectanglesResults(
				const std::vector<std::vector<cv::Point> >& rectangles,
				cv::Mat& frame);
	void displayPoleResults(const std::vector<std::vector<cv::Point> >& poles,
				cv::Mat& frame);

	void createTrackBars(void* featureObject, void* clusterObjetc,
				void* lineObject);

private:
	std::string viewName;

	const std::vector<cv::KeyPoint>* keyPoints;
	const std::vector<Cluster>* clusters;
	const std::vector<cv::Vec4i>* verticalLines;
	const std::vector<cv::Vec4i>* horizontalLines;
	const std::vector<std::vector<cv::Point> >* rectangles;
	const std::vector<std::vector<cv::Point> >* poles;
	double roll;

};

#endif /* IMAGEVIEW_H_ */
