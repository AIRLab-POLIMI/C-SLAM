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

#include "Rectangle.h"

using namespace cv;

Rectangle::Rectangle(Point x, Point y, Point z, Point w) :
			x(x), y(y), z(z), w(w)
{
}

void Rectangle::setFeature()
{
	int xMin, xMax, yMin, yMax;
	int FormFactor;

	int deltaX = (xMax - xMin);
	int deltaY = (yMax - yMin);
	if (deltaY != 0)
	{
		FormFactor = 1000 * deltaX / deltaY;
		featureMap["xMax"] = xMax;
		featureMap["yMax"] = yMax;
		featureMap["xMin"] = xMin;
		featureMap["yMin"] = yMin;
		featureMap["formFactor"] = FormFactor;
		featureMap["color"] = 100; //FIXME levami!!!!
	}
}
