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

#include "DependencyGraph.h"

#include <boost/graph/graphviz.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace boost;

void DependencyGraph::addClass(FuzzyClass* fuzzyClass)
{
	size_t id = num_vertices(graph);
	string name = fuzzyClass->getName();
	indexes[name] = id;
	names.push_back(name);

	add_vertex(graph);

	FuzzyClass* superClass = fuzzyClass->getSuperClass();

	if (superClass != NULL)
	{
		addDependency(fuzzyClass, superClass);
	}

}

void DependencyGraph::addDependency(FuzzyClass* fuzzyClass,
			FuzzyClass* dependency)
{
	string className = fuzzyClass->getName();
	string dependencyName = dependency->getName();

	addDependency(className, dependencyName);
}

void DependencyGraph::addDependency(string fuzzyClass, string dependency)
{

	size_t classId = indexes[fuzzyClass];
	size_t dependencyId = indexes[dependency];

	add_edge(classId, dependencyId, graph);
}

void DependencyGraph::drawGraph(std::ostream& out)
{
	//Ugly, but works...
	string* name = &names[0];

	write_graphviz(out, graph, make_label_writer(name));
}

ReasoningGraph* DependencyGraph::buildReasoningGraph()
{
	vector<size_t> components(num_vertices(graph));
	vector<vector<string> > componentsNames;

	size_t num = strong_components(graph,
				make_iterator_property_map(components.begin(),
							get(vertex_index, graph)));

	componentsNames.resize(num);

	BOOST_FOREACH(size_t i, vertices(graph))
	{
		size_t index = components[i];
		componentsNames[index].push_back(names[i]);
	}

	ReasoningGraph* reasoningGraph = new ReasoningGraph(num, componentsNames);

	BOOST_FOREACH(size_t i, vertices(graph))
	{
		BOOST_FOREACH(size_t j, adjacent_vertices(i, graph))
		{
			size_t ci = components[i];
			size_t cj = components[j];
			if (ci != cj)
				reasoningGraph->addEdge(ci, cj);
		}
	}

	return reasoningGraph;

}
