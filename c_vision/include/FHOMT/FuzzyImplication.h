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

#ifndef INCLUDE_FHOMT_FUZZYIMPLICATION_H_
#define INCLUDE_FHOMT_FUZZYIMPLICATION_H_

#include <algorithm>

class FuzzyImplication
{
public:
	virtual double operator()(double x, double y) const = 0;

	virtual ~FuzzyImplication()
	{
	}
};

class LukasiewiczImplication: public FuzzyImplication
{
public:
	virtual inline double operator()(double x, double y) const
	{

		return std::min(1.0 - x + y, 1.0);
	}

	virtual ~LukasiewiczImplication()
	{
	}
};

class MinimumImplication: public FuzzyImplication
{
public:
	virtual inline double operator()(double x, double y) const
	{
		if (x <= y)
			return 1.0;
		else
			return y;

	}

	virtual ~MinimumImplication()
	{
	}
};

class ProductImplication: public FuzzyImplication
{
public:
	virtual inline double operator()(double x, double y) const
	{
		if (x <= y)
			return 1.0;
		else
			return y / x;

	}

	virtual ~ProductImplication()
	{
	}
};

class NilpMinimumImplication: public FuzzyImplication
{
public:
	virtual inline double operator()(double x, double y) const
	{

		if (x <= y)
			return 0.0;
		else
			return std::max(1.0 - x, y);

	}

	virtual ~NilpMinimumImplication()
	{}
};

#endif /* INCLUDE_FHOMT_FUZZYIMPLICATION_H_ */
