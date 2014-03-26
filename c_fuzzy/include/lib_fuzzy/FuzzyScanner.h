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

#ifndef FUZZYSCANNER_H_
#define FUZZYSCANNER_H_

#if !defined(yyFlexLexerOnce)
#include <FlexLexer.h>
#endif

#undef  YY_DECL
#define YY_DECL int  FuzzyScanner::yylex()

#include "FuzzyParser.tab.h"

class FuzzyScanner: public yyFlexLexer
{
public:

	FuzzyScanner(std::istream* in) :
				yyFlexLexer(in), yylval(NULL), column(0), line(1)
	{
	}

	int yylex(yy::FuzzyParser::semantic_type* lval)
	{
		yylval = lval;
		return (yylex());
	}

	inline void count(int lenght)
	{
		column += lenght;
	}

	inline void newLine()
	{
		column = 0;
		line++;
	}

	inline int getColumn()
	{
		return column;
	}

	inline int getLine()
	{
		return line;
	}

	inline void saveLastState(int state)
	{
		caller.push_back(state);
	}

	inline int getLastState()
	{
		int lastState = caller.back();
		caller.pop_back();
		return lastState;
	}

private:
	/* hide this one from public view */
	int yylex();
	/* yyval ptr */
	yy::FuzzyParser::semantic_type *yylval;
	int column, line;
	/*comment eater stack*/
	std::vector<int> caller;
};

#endif /* FUZZYSCANNER_H_ */
