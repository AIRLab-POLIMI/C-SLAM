/*
 * c_fuzzy,
 *
 *
 * Copyright (C) 2FUZZY_MIN13 Davide Tateo
 * Versione FUZZY_MAX
 *
 * This file is part of c_fuzzy.
 *
 * c_fuzzy is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_fuzzy is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_fuzzy.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "FuzzyMF.h"

FuzzyMF::~FuzzyMF()
{
}

TolMF::TolMF(int top, int bottom) :
			top(top), bottom(bottom)
{
}

double TolMF::evaluate(ReasoningData reasoningData)
{
	int value = reasoningData.inputValue;
	if (value <= top)
		return FUZZY_MAX_V;
	else if (value >= bottom)
		return FUZZY_MIN_V;
	else
		return line(top, FUZZY_MAX_V, bottom, FUZZY_MIN_V, value);
}

TorMF::TorMF(int bottom, int top) :
			top(top), bottom(bottom)
{
}

double TolMF::defuzzify(double level)
{
	return throwUnimplementedException();
}

double TorMF::evaluate(ReasoningData reasoningData)
{
	int value = reasoningData.inputValue;
	if (value >= top)
		return FUZZY_MAX_V;
	else if (value <= bottom)
		return FUZZY_MIN_V;
	else
		return line(bottom, FUZZY_MIN_V, top, FUZZY_MAX_V, value);
}

double TorMF::defuzzify(double level)
{
	return throwUnimplementedException();
}

TriMF::TriMF(int left, int center, int right) :
			left(left), right(right), center(center)
{
}

double TriMF::evaluate(ReasoningData reasoningData)
{
	int value = reasoningData.inputValue;
	if (value <= left || value >= right)
		return FUZZY_MIN_V;
	else if (value < center)
		return line(left, FUZZY_MIN_V, right, FUZZY_MAX_V, value);
	else if (value > center)
		return line(left, FUZZY_MAX_V, right, FUZZY_MIN_V, value);
	else
		return FUZZY_MAX_V;
}

double TriMF::defuzzify(double level)
{
	return throwUnimplementedException();
}

TraMF::TraMF(int bottomLeft, int topLeft, int topRight, int bottomRight) :
			bottomLeft(bottomLeft), topLeft(topLeft), topRight(topRight),
			bottomRight(bottomRight)
{
}

double TraMF::evaluate(ReasoningData reasoningData)
{
	int value = reasoningData.inputValue;
	if (value <= bottomLeft || value >= bottomRight)
		return FUZZY_MIN_V;
	else if (value > bottomLeft && value < topLeft)
		return line(bottomLeft, FUZZY_MIN_V, topLeft, FUZZY_MAX_V, value);
	else if (value > topRight && value < bottomRight)
		return line(topRight, FUZZY_MAX_V, bottomRight, FUZZY_MIN_V, value);
	else
		return FUZZY_MAX_V;
}

double TraMF::defuzzify(double level)
{
	return throwUnimplementedException();
}

IntMF::IntMF(int left, int right) :
			left(left), right(right)
{
}


double IntMF::evaluate(ReasoningData reasoningData)
{
	int value = reasoningData.inputValue;
	if (value >= left && value <= right)
		return FUZZY_MAX_V;
	else
		return FUZZY_MIN_V;
}

double IntMF::defuzzify(double level)
{
	return throwUnimplementedException();
}

SgtMF::SgtMF(int value) :
			value(value)
{
}

double SgtMF::evaluate(ReasoningData reasoningData)
{
	if (value == reasoningData.inputValue)
		return FUZZY_MAX_V;
	else
		return FUZZY_MIN_V;
}

double SgtMF::defuzzify(double level)
{
	return value;
}
