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

#include "FuzzyClassifier.h"

FuzzyClass* FuzzyClassifier::getClass(std::string name)
{
	if (classList.count(name) == 1)
		return classList[name];
	else
		return NULL;
}

void FuzzyClassifier::addClass(FuzzyClass* fuzzyClass)
{
	classList[fuzzyClass->getName()] = fuzzyClass;

	dGraph.addClass(fuzzyClass);
}

bool FuzzyClassifier::contains(std::string name)
{
	return classList.count(name) == 1;
}

ClassList::iterator FuzzyClassifier::begin()
{
	return classList.begin();
}

ClassList::iterator FuzzyClassifier::end()
{
	return classList.end();
}
