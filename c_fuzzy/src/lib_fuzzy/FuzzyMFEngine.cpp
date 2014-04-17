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

#include "FuzzyMFEngine.h"

#include <sstream>

using namespace std;

FuzzyMF* FuzzyMFEngine::buildMF(string name, string shape,
			vector<int>& parameters)
{
	FuzzySets fuzzySetType = fuzzyMap.at(shape);

	switch (fuzzySetType)
	{
		case TOL:
			checkParameters(name, parameters, fuzzySetType);
			return buildTol(parameters[1], parameters[0]);
		case TOR:
			checkParameters(name, parameters, fuzzySetType);
			return buildTor(parameters[1], parameters[0]);
		case TRA:
			checkParameters(name, parameters, fuzzySetType);
			return buildTra(parameters[3], parameters[2], parameters[1],
						parameters[0]);
		case TRI:
			checkParameters(name, parameters, fuzzySetType);
			return buildTri(parameters[2], parameters[1], parameters[0]);
		case INT:
			checkParameters(name, parameters, fuzzySetType);
			return buildInt(parameters[1], parameters[0]);
		case SGT:
			checkParameters(name, parameters, fuzzySetType);
			return buildSgt(parameters[0]);
		default:
			return NULL;
	}
}

FuzzyMF* FuzzyMFEngine::buildTor(int bottom, int top)
{
	return new TorMF(bottom, top);
}

FuzzyMF* FuzzyMFEngine::buildTol(int top, int bottom)
{
	return new TolMF(top, bottom);
}

FuzzyMF* FuzzyMFEngine::buildTra(int bottomLeft, int topLeft, int topRight,
			int bottomRight)
{
	return new TraMF(bottomLeft, topLeft, topRight, bottomRight);
}

FuzzyMF* FuzzyMFEngine::buildTri(int left, int center, int right)
{
	return new TriMF(left, center, right);
}

FuzzyMF* FuzzyMFEngine::buildInt(int left, int right)
{
	return new IntMF(left, right);
}

FuzzyMF* FuzzyMFEngine::buildSgt(int value)
{
	return new SgtMF(value);
}

void FuzzyMFEngine::chekParametersNumber(string name, FuzzySets fuzzySetType,
			size_t parametersSize)
{
	bool error = false;

	switch (fuzzySetType)
	{
		case SGT:
			if (parametersSize != 1)
				error = true;
			break;
		case TOL:
		case TOR:
		case INT:
			if (parametersSize != 2)
				error = true;
			break;
		case TRI:
			if (parametersSize != 3)
				error = true;
			break;
		case TRA:
			if (parametersSize != 4)
				error = true;
			break;
	}

	if (error)
	{
		stringstream ss;
		ss << "The parameters number is wrong for label " << name;
		throw runtime_error(ss.str());
	}
}

void FuzzyMFEngine::checkParameters(string name, vector<int>& parameters,
			FuzzySets fuzzySetType)
{
	size_t parametersSize = parameters.size();
	chekParametersNumber(name, fuzzySetType, parametersSize);

	for (size_t i = 1; i < parametersSize; i++)
	{
		if (parameters[i] > parameters[i - 1])
		{
			stringstream ss;
			ss << "The parameters ordering is wrong for label " << name;
			throw runtime_error(ss.str());
		}
	}
}

//Static initialization
FuzzyMFEngine::FuzzyMap FuzzyMFEngine::createMap()
{
	FuzzyMap fuzzyMap;

	fuzzyMap["tol"] = TOL;
	fuzzyMap["tor"] = TOR;
	fuzzyMap["tra"] = TRA;
	fuzzyMap["tri"] = TRI;
	fuzzyMap["int"] = INT;
	fuzzyMap["sgt"] = SGT;

	return fuzzyMap;
}

const FuzzyMFEngine::FuzzyMap FuzzyMFEngine::fuzzyMap =
			FuzzyMFEngine::createMap();
