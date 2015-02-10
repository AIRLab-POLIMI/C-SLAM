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

#include <fstream>

using namespace std;

FuzzyClass* FuzzyClassifier::getClass(string name)
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

void FuzzyClassifier::addDependency(std::string fuzzyClass,
			std::string dependency)
{
	dGraph.addDependency(fuzzyClass, dependency);
}

bool FuzzyClassifier::contains(string name)
{
	return classList.count(name) == 1;
}

vector<string> FuzzyClassifier::getDependenciesNames(FuzzyClass* fuzzyClass)
{
	return dGraph.getDependencies(fuzzyClass->getName());
}

ClassList::iterator FuzzyClassifier::begin()
{
	return classList.begin();
}

ClassList::iterator FuzzyClassifier::end()
{
	return classList.end();
}

void FuzzyClassifier::setupClassifier()
{
	rGraph = dGraph.buildReasoningGraph();

	vector<size_t> order;
	rGraph->getReasonigOrder(order);

	for (auto index : order)
	{
		const vector<string>& nodeNames = rGraph->getNodeNames(index);
		ClassList list;

		for (auto& name : nodeNames)
		{
			FuzzyClass* fuzzyClass = classList[name];
			list[name] = fuzzyClass;
		}

		reasoningList.push_back(list);
	}

}

ReasoningList::iterator FuzzyClassifier::beginReasoning()
{
	return reasoningList.begin();
}

ReasoningList::iterator FuzzyClassifier::endReasoning()
{
	return reasoningList.end();
}

void FuzzyClassifier::drawDependencyGraph(string path)
{
	ofstream out;
	out.open(path.c_str());
	dGraph.drawGraph(out);
}

void FuzzyClassifier::drawDependencyGraph(std::ostream& stream)
{
	dGraph.drawGraph(stream);
}

void FuzzyClassifier::drawReasoningGraph(string path)
{
	if (rGraph != NULL)
	{
		ofstream out;
		out.open(path.c_str());
		rGraph->drawGraph(out);
	}
}

void FuzzyClassifier::drawReasoningGraph(std::ostream& stream)
{
	rGraph->drawGraph(stream);
}

FuzzyClassifier::~FuzzyClassifier()
{
	delete rGraph;
}

