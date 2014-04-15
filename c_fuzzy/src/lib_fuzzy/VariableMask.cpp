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
	boost::dynamic_bitset<>& bitset = *variableMasks[label].bits;
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

size_t VariableMasks::size()
{
	return variableMasks.size();
}

bool VariableMasks::contains(string name)
{
	return variableMasks.count(name) != 0;
}

map<string, BitData>::iterator VariableMasks::begin()
{
	return variableMasks.begin();
}

map<string, BitData>::iterator VariableMasks::end()
{
	return variableMasks.end();
}

BitData& VariableMasks::operator[](string variable)
{
	return variableMasks[variable];
}

VariableMasks::~VariableMasks()
{
	for (map<string, BitData>::iterator it = variableMasks.begin();
				it != variableMasks.end(); ++it)
	{
		delete it->second.bits;
	}
}

void NamespaceMasks::addNameSpace(std::string nameSpace,
			VariableMasks* variableMasks)
{
	namespaceMasks[nameSpace] = variableMasks;
}

void NamespaceMasks::normalizeVariableMasks(size_t size)
{
	for (map<string, VariableMasks*>::iterator it = namespaceMasks.begin();
				it != namespaceMasks.end(); ++it)
	{
		VariableMasks* mask = it->second;
		mask->normalizeVariableMasks(size);
	}
}

size_t NamespaceMasks::size()
{
	return namespaceMasks.size();
}

bool NamespaceMasks::contains(string name)
{
	return namespaceMasks.count(name) != 0;
}

map<string, VariableMasks*>::iterator NamespaceMasks::begin()
{
	return namespaceMasks.begin();
}

map<string, VariableMasks*>::iterator NamespaceMasks::end()
{
	return namespaceMasks.end();
}

VariableMasks* NamespaceMasks::operator[](string nameSpace)
{
	return namespaceMasks[nameSpace];
}

NamespaceMasks::~NamespaceMasks()
{
	for (map<string, VariableMasks*>::iterator it = namespaceMasks.begin();
				it != namespaceMasks.end(); ++it)
	{
		delete it->second;
	}
}
