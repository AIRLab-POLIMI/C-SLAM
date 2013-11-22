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

#include "ReasonerServiceHandler.h"

//main del servizio
int main(int argc, char **argv)
{
	ros::init(argc, argv, "fuzzy_reasoner_server");
	ros::NodeHandle n;

	if (argc > 1)
	{
		ReasonerServiceHandler handler(argv[1]);

		ros::ServiceServer service = n.advertiseService("reasoning",
				&ReasonerServiceHandler::reasoningCallback, &handler);
		ROS_INFO("Reasoner setup correctly");
		ros::spin();
		ROS_INFO("Reasoner shut down");
	}

	ROS_INFO("No knowledge base specified");

	return 0;
}
