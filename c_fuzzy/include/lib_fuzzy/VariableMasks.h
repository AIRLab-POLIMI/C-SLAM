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
#include <vector>
#include <map>

#include <boost/dynamic_bitset.hpp>

class NamespaceMasks
{

private:
	typedef std::vector<boost::dynamic_bitset<> > MasksVector;
	typedef std::map<std::string, std::map<std::string, size_t> > IndexMap;

public:
	boost::dynamic_bitset<>& operator[](size_t index);
	void newVariableMask(std::pair<std::string, std::string>& variable);
	size_t getMaskIndex(std::pair<std::string, std::string>& variable);
	void updateVariableMask(std::pair<std::string, std::string>& variable,
			size_t currentRule);
	void normalizeVariableMasks(size_t size);

	size_t size();
	bool contains(std::pair<std::string, std::string> variable);

private:
	IndexMap indexMap;
	MasksVector variableMasks;
};

#endif /* VARIABLEMASK_H_ */
