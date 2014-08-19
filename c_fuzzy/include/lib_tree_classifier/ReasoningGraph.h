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

#ifndef REASONINGGRAPH_H_
#define REASONINGGRAPH_H_

#include <vector>
#include <string>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

class ReasoningGraph
{
private:
	struct Vertex
	{
		std::string name;
	    std::vector<std::string> nodeNames;
	};

	typedef boost::adjacency_list<boost::setS, boost::vecS, boost::directedS, Vertex> Graph;

public:
	ReasoningGraph(size_t n, std::vector<std::vector<std::string> > names);
	void addEdge(size_t i, size_t j);
	void getReasonigOrder(std::vector<size_t>& order);
	std::vector<std::string> getNodeNames(size_t index);

public:
	struct graph_writer
	{
		void operator()(std::ostream& out) const
		{
			out << "graph [rankdir=BT];" << std::endl;
		}
	};
	void drawGraph(std::ostream& out);

private:
	Graph graph;
};

#endif /* REASONINGGRAPH_H_ */
