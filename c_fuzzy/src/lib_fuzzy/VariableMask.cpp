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

void NamespaceMasks::updateVariableMask(pair<string, string>& variable,
		size_t currentRule)
{
	string nameSpace = variable.first;
	string domain = variable.second;
	size_t index = indexMap[nameSpace][domain];
	boost::dynamic_bitset<>& bitset = variableMasks[index];
	if (currentRule >= bitset.size())
		bitset.resize(currentRule + 1, false);

	bitset[currentRule] = true;
}

void NamespaceMasks::normalizeVariableMasks(size_t size)
{
	for (MasksVector::iterator it = variableMasks.begin();
			it != variableMasks.end(); ++it)
	{
		it->resize(size, false);
	}
}

size_t NamespaceMasks::size()
{
	return variableMasks.size();
}

bool NamespaceMasks::contains(pair<string, string> variable)
{
	string nameSpace = variable.first;
	string domain = variable.second;
	return indexMap.count(nameSpace) == 1
			&& indexMap[nameSpace].count(domain) == 1;
}

boost::dynamic_bitset<>& NamespaceMasks::operator[](size_t index)
{
	return variableMasks[index];
}

void NamespaceMasks::newVariableMask(
		std::pair<std::string, std::string>& variable)
{
	size_t index = variableMasks.size();
	string nameSpace = variable.first;
	string domain = variable.second;
	boost::dynamic_bitset<> mask;
	variableMasks.push_back(mask);
	indexMap[nameSpace][domain] = index;
}

size_t NamespaceMasks::getMaskIndex(pair<string, string>& variable)
{
	string nameSpace = variable.first;
	string domain = variable.second;

	return indexMap[nameSpace][domain];
}

