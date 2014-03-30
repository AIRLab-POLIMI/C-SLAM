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

#ifndef TREECLASSIFIERSCANNER_H_
#define TREECLASSIFIERSCANNER_H_

#include "TreeClassifierParser.tab.h"

#undef YY_DECL
#define YY_DECL tc::TreeClassifierParser::symbol_type lex(TreeClassifierBuilder& builder)

YY_DECL;

#ifndef __FLEX_LEXER_H
#define yyFlexLexer tcFlexLexer
#include "FlexLexer.h"
#undef yyFlexLexer
#endif

namespace tc
{

class TreeClassifierScanner: public tcFlexLexer
{
public:
	TreeClassifierScanner(std::istream* in) :
			tcFlexLexer(in)
	{
	}

	virtual ~TreeClassifierScanner()
	{
	}

	virtual tc::TreeClassifierParser::symbol_type lex(TreeClassifierBuilder& builder);
};

}

#ifdef yylex
#undef yylex
#endif

int tcFlexLexer::yylex()
{
	std::cerr << "in ExampleFlexLexer::yylex() !" << std::endl;
	return 0;
}

#endif /* TREECLASSIFIERSCANNER_H_ */
