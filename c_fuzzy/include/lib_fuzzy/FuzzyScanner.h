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

#include "FuzzyParser.tab.h"

#undef YY_DECL
#define YY_DECL fz::FuzzyParser::symbol_type fz::FuzzyScanner::lex()

#ifndef __FLEX_LEXER_H
#define yyFlexLexer fzFlexLexer
#include "FlexLexer.h"
#undef yyFlexLexer
#endif

namespace fz
{

class FuzzyScanner: public fzFlexLexer
{
public:
	FuzzyScanner(std::istream* in) :
			fzFlexLexer(in)
	{
	}

	virtual ~FuzzyScanner()
	{
	}

	fz::FuzzyParser::symbol_type lex();

	fz::location loc;
};

}

#endif /* FUZZYSCANNER_H_ */
