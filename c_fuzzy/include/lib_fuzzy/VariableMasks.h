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

#ifndef VARIABLEMASK_H_
#define VARIABLEMASK_H_

#include <string>
#include <map>

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

class VariableMasks
{
public:
	VariableMasks()
	{
	}

	void newVariableMask(std::string variable);
	void updateVariableMask(std::string& label, size_t currentRule);
	void normalizeVariableMasks(size_t size);

	size_t size();
	bool contains(std::string name);
	std::map<std::string, BitData>::iterator begin();
	std::map<std::string, BitData>::iterator end();
	BitData& operator[](std::string variable);

	~VariableMasks();

private:
	std::map<std::string, BitData> variableMasks;
};

class NamespaceMasks
{
public:
	void addNameSpace(std::string nameSpace, VariableMasks* variableMasks);
	void normalizeVariableMasks(size_t size);

	size_t size();
	bool contains(std::string name);
	std::map<std::string, VariableMasks*>::iterator begin();
	std::map<std::string, VariableMasks*>::iterator end();
	VariableMasks* operator[](std::string nameSpace);

	~NamespaceMasks();

private:
	std::map<std::string, VariableMasks*> namespaceMasks;
};

#endif /* VARIABLEMASK_H_ */
