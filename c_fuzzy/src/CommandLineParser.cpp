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

#include "CommandLineParser.h"

using namespace std;
using namespace boost::program_options;

#include <ros/ros.h>

CommandLineParser::CommandLineParser() :
			desc("Usage")
{

	desc.add_options() //
	("help,h", "produce help message") //
	("reasoner,r", value<string>(), "set up a reasoner from knowledgebase") //
	("classifier,c", value<vector<string> >()->multitoken(), "set up a classifier from\n"
				"\t- a classifier file\n"
				"\t- a knowledgebase\n");

	reasoner = false;
	classifier = false;

}

void CommandLineParser::parseCommandLine(int argc, char **argv)
{
	try
	{
		store(parse_command_line(argc, argv, desc), vm);
		if (vm.count("help"))
		{
			ROS_INFO_STREAM(desc);
			return;
		}

		notify(vm);

	}
	catch (error& e)
	{
		cout << e.what() << endl;
		ROS_FATAL_STREAM(desc);
		return;
	}

	reasoner = vm.count("reasoner");
	classifier = vm.count("classifier")
				&& vm["classifier"].as<vector<string> >().size() == 2;

	if(!reasoner && !classifier)
		ROS_FATAL_STREAM(desc);
}


bool CommandLineParser::validParameters()
{
	return reasoner || classifier;
}

string CommandLineParser::getKnowledgeBase()
{
	return vm["reasoner"].as<string>();
}

string CommandLineParser::getClassifier()
{
	return vm["classifier"].as<vector<string> >()[1];
}

string CommandLineParser::getClassifierKnowledgeBase()
{
	return vm["classifier"].as<vector<string> >()[0];
}

bool CommandLineParser::hasReasoner()
{
	return reasoner;
}

bool CommandLineParser::hasClassifier()
{
	return classifier;
}

