/*
 * c_fuzzy,
 *
 *
 * Copyright (C) 2013 Davide Tateo
 * Versione 1.0
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

#ifndef FUZZYMF_H_
#define FUZZYMF_H_

#include <string>
#include <map>

#include "Node.h"

/**
 * FuzzyMF base class
 *
 *
 */
class FuzzyMF: public Node
{
public:
	virtual double defuzzify(double level) = 0;
	virtual ~FuzzyMF()
	{
	}

protected:
	double inline line(int x1, double y1, int x2, double y2, int x)
	{
		double m = (y2 - y1) / (x2 - x1);
		return m * (x - x1) + y1;
	}

public:
	static const double FUZZY_MAX_V = 1.0, FUZZY_MIN_V = 0;
};

/**
 * Triangle Open Left Membership function
 *
 *
 */
class TolMF: public FuzzyMF
{
public:
	TolMF(int top, int bottom) :
				top(top), bottom(bottom)
	{
	}
	virtual double evaluate(ReasoningData reasoningData);
	double defuzzify(double level);

private:
	int top, bottom;
};

/**
 * Triangle Open Right Membership function
 *
 *
 */
class TorMF: public FuzzyMF
{

public:
	TorMF(int bottom, int top) :
				top(top), bottom(bottom)
	{
	}
	virtual double evaluate(ReasoningData reasoningData);
	double defuzzify(double level);

private:
	int top, bottom;
};

/**
 * Triangle Membership Function
 *
 *
 *
 */
class TriMF: public FuzzyMF
{

public:
	TriMF(int left, int center, int right) :
				left(left), right(right), center(center)
	{
	}
	virtual double evaluate(ReasoningData reasoningData);
	double defuzzify(double level);

private:
	int left, right, center;
};

/**
 * Trapezoid Membership Function
 *
 *
 */
class TraMF: public FuzzyMF
{

public:
	TraMF(int bottomLeft, int topLeft, int topRight, int bottomRight) :
				bottomLeft(bottomLeft), topLeft(topLeft), topRight(topRight),
				bottomRight(bottomRight)
	{
	}
	virtual double evaluate(ReasoningData reasoningData);
	double defuzzify(double level);

private:
	int bottomLeft, topLeft, topRight, bottomRight;

};

/**
 * Intrerval Membership Function
 *
 *
 */
class IntMF: public FuzzyMF
{

public:
	IntMF(int left, int right) :
				left(left), right(right)
	{
	}
	virtual double evaluate(ReasoningData reasoningData);
	double defuzzify(double level);

private:
	int left, right;
};

/**
 * Singleton Membership Function
 *
 *
 *
 */
class SgtMF: public FuzzyMF
{
public:
	SgtMF(int value) :
				value(value)
	{
	}
	virtual double evaluate(ReasoningData reasoningData);
	double defuzzify(double level);

private:
	int value;
};

typedef std::map<std::string, FuzzyMF*> MFTable;
typedef std::map<std::string, MFTable*> DomainTable;

#endif /* FUZZYMF_H_ */
