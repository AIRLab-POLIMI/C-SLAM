/*
 * c_fuzzy,
 *
 *
 * Copyright (C) 2013 Davide Tateo
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

#include <ros/ros.h>
#include <stdexcept>

#include "ReasonerServiceHandler.h"
#include "ClassifierServiceHandler.h"

#include "CommandLineParser.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "fuzzy_reasoner_server");
	ros::NodeHandle n;

	CommandLineParser clParser;

	clParser.parseCommandLine(argc, argv);
	if (!clParser.validParameters())
		return -1;

	ReasonerServiceHandler* reasonerHandler;
	ClassifierServiceHandler* classifierHandler;

	try
	{

		if (clParser.hasReasoner())
		{
			reasonerHandler = new ReasonerServiceHandler(
						clParser.getKnowledgeBase());
			ros::ServiceServer reasonerService = n.advertiseService("reasoning",
						&ReasonerServiceHandler::reasoningCallback,
						reasonerHandler);

			ROS_INFO("Reasoner setup correctly");
		}

		if (clParser.hasClassifier())
		{
			classifierHandler = new ClassifierServiceHandler(
						clParser.getClassifierKnowledgeBase(),
						clParser.getClassifier());
			ros::ServiceServer classifierService = n.advertiseService(
						"classification",
						&ClassifierServiceHandler::classificationCallback,
						classifierHandler);

			ros::ServiceServer rGraphService =
						n.advertiseService("getReasoningGraph",
									&ClassifierServiceHandler::reasoningGraphRequestCallback,
									classifierHandler);

			ros::ServiceServer dGraphService =
						n.advertiseService("getDependencyGraph",
									&ClassifierServiceHandler::dependencyGraphRequestCallback,
									classifierHandler);

			ROS_INFO("Classifier setup correctly");

		}

		ros::spin();

		if (reasonerHandler)
			delete reasonerHandler;
		if (classifierHandler)
			delete classifierHandler;

		ROS_INFO("Reasoner shut down");

	}
	catch (std::runtime_error& e)
	{
		ROS_FATAL(e.what());
		ROS_FATAL("Check the knowledge base file");

		if (reasonerHandler)
			delete reasonerHandler;
		if (classifierHandler)
			delete classifierHandler;
		return -1;
	}

	return 0;
}
