%{
	#include <sstream>
	#include <stdexcept>
	#include <string>
	
	#include "FuzzyParser.tab.h"
	#include "FuzzyScanner.h"
	
	#undef yywrap
	#define yywrap() 1
	
	using namespace std;
%}


%option noyywrap nounput batch debug noinput c++
%option prefix="fz"

%x COMMENT

DIGIT [0-9]
CAPITAL [A-Z]
LOWERCASE [a-z]
LETTER {LOWERCASE}|{CAPITAL}
BLANK [ \t]
NEWLINE \n\r|\n
FUZZYLABEL tol|tor|tri|tra|int|sgt
SIGN [-+]
UNDERSCORE [_]

%{
	// Code run each time a pattern is matched.
	#define YY_USER_ACTION  loc.columns (yyleng);
%}
%%
%{
	// Code run each time yylex is called.
	loc.step ();
%}


 /* C++ stle nested comment eater */
"/*" { yy_push_state(YY_START); BEGIN(COMMENT); }
<COMMENT>
{
	{NEWLINE} { loc.lines(yyleng); loc.step(); }
	"/*" { yy_push_state(YY_START); BEGIN(COMMENT); }
	"*/" { BEGIN(yy_top_state()); yy_pop_state(); }
	. {}
}

 /* blank and new line */
{BLANK} { loc.step (); }
{NEWLINE} { loc.lines(yyleng); loc.step(); }

 /* keywords and symbols */
"fuzzify"|"FUZZIFY" { return fz::FuzzyParser::make_FUZZIFY(loc); }
"end_fuzzify"|"END_FUZZIFY" { return fz::FuzzyParser::make_END_FUZZIFY(loc); }
"fuzzify_class"|"FUZZIFY_CLASS" { return fz::FuzzyParser::make_FUZZIFY_CLASS(loc); }
"end_fuzzify_class"|"END_FUZZIFY_CLASS" { return fz::FuzzyParser::make_END_FUZZIFY_CLASS(loc); }
"fuzzify_predicate"|"FUZZIFY_PREDICATE" { return fz::FuzzyParser::make_FUZZIFY_PREDICATE(loc); }
"end_fuzzify_predicate"|"END_FUZZIFY_PREDICATE"  { return fz::FuzzyParser::make_END_FUZZIFY_PREDICATE(loc); }
"and"|"AND" { return fz::FuzzyParser::make_OP_AND(loc); }
"or"|"OR" { return fz::FuzzyParser::make_OP_OR(loc); }
"~"|"not"|"NOT" { return fz::FuzzyParser::make_OP_NOT(loc); }
"then"|"THEN" { return fz::FuzzyParser::make_THEN(loc); }
"is"|"IS" { return fz::FuzzyParser::make_IS(loc); }
"if"|"IF" { return fz::FuzzyParser::make_IF(loc); }
"(" { return fz::FuzzyParser::make_OPEN_B(loc); }
")" { return fz::FuzzyParser::make_CLOSE_B(loc); }
";" { return fz::FuzzyParser::make_END_RULE(loc); }
":=" { return fz::FuzzyParser::make_LIKE(loc); }
"," { return fz::FuzzyParser::make_COMMA(loc); }
"." { return fz::FuzzyParser::make_PERIOD(loc); }
"?" { return fz::FuzzyParser::make_QUESTION(loc); }

 /* identificators */
{FUZZYLABEL} { return fz::FuzzyParser::make_F_LABEL(yytext, loc); }
{CAPITAL}({LETTER}|{DIGIT}|{UNDERSCORE})* { return fz::FuzzyParser::make_ID(yytext, loc); }
{LOWERCASE}({LETTER}|{DIGIT}|{UNDERSCORE})* { return fz::FuzzyParser::make_VAR_ID(yytext, loc); }
-?{DIGIT}+ { return fz::FuzzyParser::make_PARAMETER(atoi(yytext), loc); }

 /* unknown tokens */
. { std::stringstream ss; ss << "Error: Unknown char " << *yytext << " at " << loc << endl; throw std::runtime_error(ss.str()); }

 /* End of file */
<<EOF>> { return fz::FuzzyParser::make_END(loc); }

%%

int fzFlexLexer::yylex()
{
	return 0;
}