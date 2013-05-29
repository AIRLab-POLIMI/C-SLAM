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

#ifndef CRISPDATA_H_
#define CRISPDATA_H_

/**
 * Crisp data class
 * Class used to match crisp data label with the current crisp value
 *
 */
class CrispData: public Node
{

public:
	CrispData(std::map<std::string, int>* lookUpTable, std::string label) :
			lookUpTable(*lookUpTable), label(label)
	{
	}

	double evaluate()
	{
		return (float) lookUpTable[label];
	}

	inline int evaluateInt()
	{
		return lookUpTable[label];
	}

private:
	std::map<std::string, int>& lookUpTable;
	std::string label;
};


#endif /* CRISPDATA_H_ */
