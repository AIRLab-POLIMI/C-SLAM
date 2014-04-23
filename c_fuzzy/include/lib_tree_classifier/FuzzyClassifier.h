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

#ifndef FUZZYCLASSIFIER_H_
#define FUZZYCLASSIFIER_H_

#include <string>
#include <map>
#include <vector>
#include <set>

#include "DependencyGraph.h"
#include "FuzzyClass.h"

typedef std::map<std::string, FuzzyClass*> ClassList;
typedef std::vector<ClassList> ReasoningList;

class FuzzyClassifier
{
public:
	FuzzyClass* getClass(std::string);
	void addClass(FuzzyClass* fuzzyClass);
	void addDependency(std::string fuzzyClass, std::string dependency);
	bool contains(std::string name);
	void setupClassifier();
	ClassList::iterator begin();
	ClassList::iterator end();
	ReasoningList::iterator beginReasoning();
	ReasoningList::iterator endReasoning();

	~FuzzyClassifier();


public:
	void drawDependencyGraph(std::string path);
	void drawReasoningGraph(std::string path);

private:
	ClassList classList;
	DependencyGraph dGraph;
	ReasoningGraph* rGraph;
	ReasoningList reasoningList;
};

#endif /* FUZZYCLASSIFIER_H_ */
