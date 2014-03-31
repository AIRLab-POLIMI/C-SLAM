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
LETTER [a-z]|{CAPITAL}
BLANK [ \t]
NEWLINE \n\r|\n
FUZZYLABEL tol|tor|tri|tra|int|sgt
SIGN [-+]

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
	"/*" { yy_push_state(COMMENT); BEGIN(COMMENT); }
	"*/" { BEGIN(yy_top_state()); yy_pop_state(); }
	. {}
}

 /* blank and new line */
{BLANK} { }
{NEWLINE} { this->newLine(); }

 /* keywords and symbols */
"fuzzify"|"FUZZIFY" { return fz::Fuzzyparser::make_FUZZIFY(loc); }
"end_fuzzify"|"END_FUZZIFY" { return fz::Fuzzyparser::make_END_FUZZIFY(loc); }
"and"|"AND" { return fz::Fuzzyparser::make_OP_AND(loc); }
"or"|"OR" { return fz::Fuzzyparser::make_OP_OR(loc); }
"~"|"not"|"NOT" { return fz::Fuzzyparser::make_OP_NOT(loc); }
"then"|"THEN" { return fz::Fuzzyparser::make_THEN(loc); }
"is"|"IS" { return fz::Fuzzyparser::make_IS(loc); }
"if"|"IF" { return fz::Fuzzyparser::make_IF(loc); }
"(" { return fz::Fuzzyparser::make_OPEN_B(loc); }
")" { return fz::Fuzzyparser::make_CLOSE_B(loc); }
";" { return fz::Fuzzyparser::make_END_RULE(loc); }
":=" { return fz::Fuzzyparser::make_LIKE(loc); }
"," { return fz::Fuzzyparser::make_COMMA(loc); }

 /* identificators */
{CAPITAL}({LETTER}|{DIGIT})* { return fz::Fuzzyparser::make_ID(yytext, loc); }
-?{DIGIT}+ { return fz::Fuzzyparser::make_PARAMETER(atoi(yytext), loc); }
{FUZZYLABEL} { return fz::Fuzzyparser::make_F_LABEL(yytext, loc); }

 /* unknown tokens */
. { std::stringstream ss; ss << "Error: Unknown char " << *yytext << " at line " << this->getLine() << ", column " << this->getColumn() << endl; throw std::runtime_error(ss.str()); }

%%

int tcFlexLexer::yylex()
{
	return 0;
}