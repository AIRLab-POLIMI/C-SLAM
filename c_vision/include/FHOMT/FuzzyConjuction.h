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

#ifndef INCLUDE_FHOMT_FUZZYCONJUCTION_H_
#define INCLUDE_FHOMT_FUZZYCONJUCTION_H_

#include <algorithm>

class FuzzyConjunction
{
public:
	virtual double operator()(double x, double y) const = 0;

	virtual ~FuzzyConjunction()
	{
	}
};

class LukasiewiczConjunction: public FuzzyConjunction
{
public:
	virtual inline double operator()(double x, double y) const
	{
		return std::max(x + y - 1.0, 0.0);
	}

	virtual ~LukasiewiczConjunction()
	{
	}
};

class MinimumConjunction: public FuzzyConjunction
{
public:
	virtual inline double operator()(double x, double y) const
	{
		return std::min(x, y);
	}

	virtual ~MinimumConjunction()
	{
	}
};

class ProductConjunction: public FuzzyConjunction
{
public:
	virtual inline double operator()(double x, double y) const
	{
		return x * y;
	}

	virtual ~ProductConjunction()
	{
	}
};

class NilpMinimumConjunction: public FuzzyConjunction
{
public:
	virtual inline double operator()(double x, double y) const
	{
		if (x + y <= 1)
			return 0;
		else
			return std::min(x, y);
	}

	virtual ~NilpMinimumConjunction()
	{
	}
};

#endif /* INCLUDE_FHOMT_FUZZYCONJUCTION_H_ */
