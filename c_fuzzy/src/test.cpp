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

#include "FuzzyBuilder.h"
#include "FuzzyKnowledgeBase.h"
#include "FuzzyReasoner.h"
#include <map>
#include <stdexcept>
#include <sstream>

int main(int argc, char *argv[])
{

	try
	{
		FuzzyBuilder builder;

		builder.parse(argv[1]);

		FuzzyKnowledgeBase* knowledgebase = builder.createKnowledgeBase();
		FuzzyReasoner reasoner(*knowledgebase);

		if (argc == 2)
		{
			char stop;

			do
			{
				int value;
				std::string name;
				std::cout << "insert input name: " << std::endl;
				std::cin >> name;
				//name = "Input";
				std::cout << "insert input value: " << std::endl;
				std::cin >> value;
				//value = 150;

				reasoner.addInput(name, value);

				do
				{
					std::cout << "another input? [y/n]" << std::endl;
					std::cin >> stop;
					//stop = 'n';
				} while (stop != 'n' && stop != 'y');

			} while (stop != 'n');
		}
		else
		{
			reasoner.addInput("Input", 189);
		}

		OutputTable results = reasoner.run();

		std::cout << "Reasoning terminato, risultati:" << std::endl;

		for (OutputTable::iterator i = results.begin(); i != results.end(); ++i)
		{
			DomainOutputTable& map = i->second;
			for (DomainOutputTable::iterator j = map.begin(); j != map.end();
						++j)
			{
				std::stringstream ss;

				if (!i->first.empty())
				{
					ss << i->first << ".";
				}

				ss << j->first;

				std::cout << ss.str() << " = " << j->second.value
							<< " truth value = " << j->second.truth
							<< std::endl;
			}
		}

		delete knowledgebase;

	} catch (const std::runtime_error& e)
	{
		std::cout << e.what() << std::endl;
		std::cout << "Check the input file an try again" << std::endl;
	}

}
