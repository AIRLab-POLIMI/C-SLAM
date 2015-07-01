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

#ifndef INCLUDE_FHOMT_FUZZYMORPHOLOGICALOPERATORS_H_
#define INCLUDE_FHOMT_FUZZYMORPHOLOGICALOPERATORS_H_

#include "FuzzyConjuction.h"
#include "FuzzyImplication.h"

#include <opencv2/opencv.hpp>

class MorphOp
{
public:
	static inline cv::Mat f_homt(const cv::Mat_<double>& inputImage,
				const cv::Mat_<double>& structuringElement, const FuzzyConjunction& T,
				const FuzzyImplication& I)
	{
		cv::Size seSize = structuringElement.size();
		cv::Size imageSize = inputImage.size();

		cv::Mat_<double> R(imageSize, 0);

		unsigned int startX = std::floor(seSize.width / 2);
		unsigned int startY = floor(seSize.height / 2);

		unsigned int endX = imageSize.width - startX;
		unsigned int endY = imageSize.height - startY;

		cv::Mat Be = cv::max(0, structuringElement);
		cv::Mat Bd = -cv::min(0, structuringElement);

		for (unsigned int i = startX; i < endX; i++)
		{
			for (unsigned j = startY; j < endY; j++)
			{
				R(j, i) = f_homt_local(inputImage, Be, Bd, i, j, T, I);
			}
		}

		return R;

	}

	static inline cv::Mat f_erode(const cv::Mat_<double>& inputImage,
				const cv::Mat_<double>& structuringElement, const FuzzyImplication& I)
	{
		cv::Size seSize = structuringElement.size();
		cv::Size imageSize = inputImage.size();

		cv::Mat_<double> R(imageSize, 0);

		unsigned int startX = std::floor(seSize.width / 2);
		unsigned int startY = floor(seSize.height / 2);

		unsigned int endX = imageSize.width - startX;
		unsigned int endY = imageSize.height - startY;

		for (unsigned int i = startX; i < endX; i++)
		{
			for (unsigned j = startY; j < endY; j++)
			{
				R(j, i) = f_erode_local(inputImage, structuringElement, i, j,
							I);
			}
		}

		return R;
	}

	static inline cv::Mat f_dilate(const cv::Mat_<double>& inputImage,
				const cv::Mat_<double>& structuringElement, const FuzzyConjunction& T)
	{
		cv::Size seSize = structuringElement.size();
		cv::Size imageSize = inputImage.size();

		cv::Mat_<double> R(imageSize, 0);

		unsigned int startX = std::floor(seSize.width / 2);
		unsigned int startY = floor(seSize.height / 2);

		unsigned int endX = imageSize.width - startX;
		unsigned int endY = imageSize.height - startY;

		for (unsigned int i = startX; i < endX; i++)
		{
			for (unsigned j = startY; j < endY; j++)
			{
				R(j, i) = f_dilate_local(inputImage, structuringElement, i, j,
							T);
			}
		}

		return R;
	}

private:
	static inline double f_homt_local(const cv::Mat_<double>& inputImage,
				const cv::Mat_<double>& Be, const cv::Mat_<double>& Bd, unsigned int x,
				unsigned int y, const FuzzyConjunction& T,
				const FuzzyImplication& I)
	{
		double minVe = 1.0;
		double maxVd = 0.0;

		cv::Size seSize = Be.size();

		unsigned int offsetX = floor(seSize.width / 2);
		unsigned int offsetY = floor(seSize.height / 2);

		for (unsigned int i = 0; i < seSize.width; i++)
		{
			for (unsigned int j = 0; j < seSize.height; j++)
			{
				//Erosion
				double currentVe = I(Be(j, i),
							inputImage(y - offsetY + j, x - offsetX + i));
				minVe = std::min(minVe, currentVe);

				//Dilation
				double currentVd = T(Bd(j, i),
							inputImage(y - offsetY + j, x - offsetX + i));
				maxVd = std::max(maxVd, currentVd);
			}
		}

		return T(minVe, 1 - maxVd);
	}

	static inline double f_dilate_local(const cv::Mat_<double>& inputImage,
				const cv::Mat_<double>& B, unsigned int x, unsigned int y,
				const FuzzyConjunction& T)
	{

		double maxV = 0.0;

		cv::Size seSize = B.size();

		unsigned int offsetX = floor(seSize.width / 2);
		unsigned int offsetY = floor(seSize.height / 2);

		for (unsigned int i = 0; i < seSize.width; i++)
		{
			for (unsigned int j = 0; j < seSize.height; j++)
			{
				double currentV = T(B(j, i),
							inputImage(y - offsetY + j, x - offsetX + i));
				maxV = std::max(maxV, currentV);
			}
		}

		return maxV;
	}

	static inline double f_erode_local(const cv::Mat_<double>& inputImage,
				const cv::Mat_<double>& B, unsigned int x, unsigned int y,
				const FuzzyImplication& I)
	{
		double minV = 1.0;

		cv::Size seSize = B.size();

		unsigned int offsetX = floor(seSize.width / 2);
		unsigned int offsetY = floor(seSize.height / 2);

		for (unsigned int i = 0; i < seSize.width; i++)
		{
			for (unsigned int j = 0; j < seSize.height; j++)
			{
				double currentV = I(B(j, i),
							inputImage(y - offsetY + j, x - offsetX + i));
				minV = std::min(minV, currentV);
			}
		}

		return minV;
	}

};

#endif /* INCLUDE_FHOMT_FUZZYMORPHOLOGICALOPERATORS_H_ */
