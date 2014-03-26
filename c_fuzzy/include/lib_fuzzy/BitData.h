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

#ifndef BITDATA_H_
#define BITDATA_H_

#include <boost/dynamic_bitset.hpp>

struct BitData
{
	BitData() :
				index(0), bits(NULL)
	{
	}
	BitData(int index, boost::dynamic_bitset<>* bits) :
				index(index), bits(bits)
	{
	}
	int index;
	boost::dynamic_bitset<>* bits;

};

#endif /* BITDATA_H_ */
