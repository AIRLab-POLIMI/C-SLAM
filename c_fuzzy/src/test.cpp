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

char askYNQuestion(std::string question)
{
	char stop;

	do
	{
		std::cout << question <<" [y/n]" << std::endl;
		std::cin >> stop;
	} while (stop != 'n' && stop != 'y');

	return stop;

}

void getInputs(FuzzyReasoner& reasoner)
{
	char stop;

	stop = askYNQuestion("insert simple input?");

	while (stop != 'n')
	{
		int value;
		std::string name;
		std::cout << "insert input name: " << std::endl;
		std::cin >> name;
		std::cout << "insert input value: " << std::endl;
		std::cin >> value;
		reasoner.addInput(name, value);

		stop = askYNQuestion("another input?");

	}

	stop = askYNQuestion("insert class input?");

	while (stop != 'n')
	{
		int value;
		std::string nameSpace;
		std::string name;
		std::cout << "insert input class: " << std::endl;
		std::cin >> nameSpace;
		std::cout << "insert input name: " << std::endl;
		std::cin >> name;
		std::cout << "insert input value: " << std::endl;
		std::cin >> value;
		reasoner.addInput(nameSpace, name, value);

		stop = askYNQuestion("another input?");
	}
}

int main(int argc, char *argv[])
{

	try
	{
		FuzzyBuilder builder;

		builder.parse(argv[1]);

		FuzzyKnowledgeBase* knowledgebase = builder.createKnowledgeBase();
		FuzzyReasoner reasoner(*knowledgebase);

		getInputs(reasoner);

		OutputTable results = reasoner.run();

		std::cout << "Reasoning terminato, risultati:" << std::endl;

		std::cout << results << std::endl;

		delete knowledgebase;

	} catch (const std::runtime_error& e)
	{
		std::cout << e.what() << std::endl;
		std::cout << "Check the input file an try again" << std::endl;
	}

}
