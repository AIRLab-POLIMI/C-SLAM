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

	Graph::vertex_descriptor v = add_vertex(graph);

	graph[v].name = name;

	FuzzyClass* superClass = fuzzyClass->getSuperClass();

	if (superClass != NULL)
	{
		addDependency(fuzzyClass, superClass, true);
	}

}

void DependencyGraph::addDependency(FuzzyClass* fuzzyClass,
			FuzzyClass* dependency, bool isSuperClass)
{
	string className = fuzzyClass->getName();
	string dependencyName = dependency->getName();

	addDependency(className, dependencyName, isSuperClass);
}

void DependencyGraph::addDependency(string fuzzyClass, string dependency,
			bool isSuperClass)
{

	Graph::vertex_descriptor classId = indexes[fuzzyClass];
	Graph::vertex_descriptor dependencyId = indexes[dependency];

	Graph::edge_descriptor edge = add_edge(classId, dependencyId, graph).first;
	graph[edge].isSuperClass = isSuperClass;
}

DependencyGraph::NameList DependencyGraph::getDependencies(string className)
{
	NameList list;
	Graph::vertex_descriptor node = indexes[className];

	BOOST_FOREACH(Graph::vertex_descriptor i, adjacent_vertices(node, graph))
	{
		Graph::edge_descriptor e = edge(node, i, graph).first;
		if (!graph[e].isSuperClass)
			list.push_back(graph[i].name);
	}

	return list;

}

void DependencyGraph::drawGraph(std::ostream& out)
{
	write_graphviz(out, graph, make_label_writer(get(&Vertex::name, graph)),
				make_label_writer(get(&Edge::isSuperClass, graph)));
}

ReasoningGraph* DependencyGraph::buildReasoningGraph()
{
	vector<Graph::vertex_descriptor> components(num_vertices(graph));
	vector<vector<string> > componentsNames;

	size_t num = strong_components(graph,
				make_iterator_property_map(components.begin(),
							get(vertex_index, graph)));

	componentsNames.resize(num);

	BOOST_FOREACH(Graph::vertex_descriptor i, vertices(graph))
	{
		size_t index = components[i];
		componentsNames[index].push_back(graph[i].name);
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
