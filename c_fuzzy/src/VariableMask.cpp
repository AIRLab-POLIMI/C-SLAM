/*
 * c_fuzzy,
 *
 *
 * Copyright (C) 2014 Davide Tateo
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

#include "VariableMasks.h"

using namespace std;

void VariableMasks::newVariableMask(std::string variable)
{
	size_t variableIndex = variableMasks.size();
	variableMasks[variable] = BitData(variableIndex,
				new boost::dynamic_bitset<>());
}

void VariableMasks::updateVariableMask(string& label, size_t currentRule)
{
	map<string, BitData>& maskMap = *variableMasks;
	boost::dynamic_bitset<>& bitset = *maskMap[label].bits;
	if (currentRule >= bitset.size())
		bitset.resize(currentRule + 1, false);

	bitset[currentRule] = true;
}

void VariableMasks::normalizeVariableMasks(size_t size)
{
	map<string, BitData>::iterator it;
	for (it = variableMasks.begin(); it != variableMasks.end(); ++it)
	{
		it->second.bits->resize(size, false);
	}

}
