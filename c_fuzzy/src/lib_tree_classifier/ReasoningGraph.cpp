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

#include "ReasoningGraph.h"

#include <boost/graph/graphviz.hpp>
#include <boost/graph/topological_sort.hpp>

using namespace std;
using namespace boost;

void ReasoningGraph::addEdge(size_t i, size_t j)
{
	add_edge(i, j, graph);
}

void ReasoningGraph::drawGraph(ostream& out)
{
	string* name = &names[0];

	write_graphviz(out, graph, make_label_writer(name));
}

void ReasoningGraph::getReasonigOrder(vector<size_t>& order)
{
	topological_sort(graph, back_inserter(order));
}
