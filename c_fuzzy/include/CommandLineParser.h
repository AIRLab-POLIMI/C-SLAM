/*
 * c_vision,
 *
 *
 * Copyright (C) 2014 Davide Tateo
 * Versione 1.0
 *
 * This file is part of c_vision.
 *
 * c_vision is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * c_vision is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with c_vision.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef COMMANDLINEPARSER_H_
#define COMMANDLINEPARSER_H_

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include <string>

class CommandLineParser
{
public:
	CommandLineParser();
	void parseCommandLine(int argc, char **argv);
	bool validParameters();

	std::string getKnowledgeBase();
	std::string getClassifier();
	std::string getClassifierKnowledgeBase();

	bool hasReasoner();
	bool hasClassifier();

private:
	boost::program_options::options_description desc;
	boost::program_options::variables_map vm;

	bool reasoner;
	bool classifier;

};



#endif /* COMMANDLINEPARSER_H_ */
