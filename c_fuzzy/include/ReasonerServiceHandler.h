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

#ifndef REASONERSERVICEHANDLER_H_
#define REASONERSERVICEHANDLER_H_

#include <vector>
#include <string>

#include "FuzzyReasoner.h"

#include "c_fuzzy/Reasoning.h"

#include <ros/ros.h>

class ReasonerServiceHandler
{
public:
	ReasonerServiceHandler(ros::NodeHandle& n,
				const std::string& knowledgeBasePath);

	bool reasoningCallback(c_fuzzy::Reasoning::Request& request,
				c_fuzzy::Reasoning::Response& response);
	~ReasonerServiceHandler();

private:
	FuzzyKnowledgeBase* knowledgeBase;

	ros::ServiceServer reasonerService;

};

#endif /* REASONERSERVICEHANDLER_H_ */
