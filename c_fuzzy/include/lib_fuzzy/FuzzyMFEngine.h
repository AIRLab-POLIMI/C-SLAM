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

#ifndef FUZZYMFENGINE_H_
#define FUZZYMFENGINE_H_

#include <string>
#include <map>
#include <vector>

#include "FuzzyMF.h"

class FuzzyMFEngine
{

private:
	enum FuzzySets
	{
		TOL, TOR, TRA, TRI, INT, SGT
	};

	typedef std::map<std::string, FuzzySets> FuzzyMap;
public:
	static FuzzyMF* buildMF(std::string name, std::string shape,
				std::vector<int>& parameters);
	static FuzzyMF* buildTor(int bottom, int top);
	static FuzzyMF* buildTol(int top, int bottom);
	static FuzzyMF* buildTra(int bottomLeft, int topLeft, int topRight,
				int bottomRight);
	static FuzzyMF* buildTri(int left, int center, int right);
	static FuzzyMF* buildInt(int left, int right);
	static FuzzyMF* buildSgt(int value);

private:
	static void checkParameters(std::string name, std::vector<int>& parameters,
				FuzzySets fuzzySetType);
	static void chekParametersNumber(std::string name, FuzzySets fuzzySetType,
				size_t parametersSize);

private:
	static FuzzyMap createMap();
	static const FuzzyMap fuzzyMap;

};

#endif /* FUZZYMFENGINE_H_ */
