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

#ifndef DEPENDENCYGRAPH_H_
#define DEPENDENCYGRAPH_H_

#include <string>
#include <map>
#include <fstream>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/strong_components.hpp>

#include "FuzzyClass.h"
#include "ReasoningGraph.h"

class DependencyGraph
{

private:
	typedef boost::adjacency_list<boost::setS, boost::vecS, boost::directedS> Graph;
	typedef std::map<std::string, size_t> IndexMap;
	typedef std::vector<std::string> NameList;

public:
	void addClass(FuzzyClass* fuzzyClass);
	void addDependency(FuzzyClass* fuzzyClass, FuzzyClass* dependency);
	void addDependency(std::string fuzzyClass, std::string dependency);
	NameList getDependencies(std::string className);
	ReasoningGraph* buildReasoningGraph();
	void drawGraph(std::ostream& out);

private:
	Graph graph;
	IndexMap indexes;
	NameList names;
};

#endif /* DEPENDENCYGRAPH_H_ */
