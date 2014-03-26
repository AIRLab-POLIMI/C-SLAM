%{
	#include <sstream>
	#include <stdexcept>
	#include <string>
	#include "FuzzyScanner.h"
	
	#define YY_USER_ACTION this->count(yyleng);
	
	#define UNKNOWN -1
	
	typedef yy::FuzzyParser::token token;
	
	using namespace std;
%}


%option yylineno
%option debug 
%option nodefault 
%option yyclass="FuzzyScanner" 
%option noyywrap 
%option c++

%x COMMENT

DIGIT [0-9]
CAPITAL [A-Z]
LETTER [a-z]|{CAPITAL}
BLANK [ \t]
NEWLINE \n\r|\n
FUZZYLABEL tol|tor|tri|tra|int|sgt
SIGN [-+]

%%

 /* C++ stle nested comment eater */
"/*" { saveLastState(YY_START); BEGIN(COMMENT); }
<COMMENT>
{
	[^/*]*
	"/"+[^*/]*
	"/*" { saveLastState(YY_START); BEGIN(COMMENT); }
	"*"+[^*/]*
	"*"+"/" { BEGIN(getLastState()); }
}
 /* blank and new line */
{BLANK} { }
{NEWLINE} { this->newLine(); }

 /* keywords and symbols */
"fuzzify"|"FUZZIFY" { return token::FUZZIFY; }
"end_fuzzify"|"END_FUZZIFY" { return token::END_FUZZIFY; }
"and"|"AND" { return token::OP_AND; }
"or"|"OR" { return token::OP_OR; }
"~"|"not"|"NOT" { return token::OP_NOT; }
"then"|"THEN" { return token::THEN; }
"is"|"IS" { return token::IS; }
"if"|"IF" { return token::IF; }
"(" { return token::OPEN_B; }
")" { return token::CLOSE_B; }
";" { return token::END_RULE; }
":=" { return token::LIKE; }
"," { return token::COMMA; }

 /* identificators */
{CAPITAL}({LETTER}|{DIGIT})* { yylval->str = new std::string(yytext); return token::ID; }
-?{DIGIT}+ { yylval->integer = atoi(yytext); return token::PARAMETER; }
{FUZZYLABEL} { yylval->str = new std::string(yytext); return token::F_LABEL; }

 /* unknown tokens */
. { std::stringstream ss; ss << "Error: Unknown char " << *yytext << " at line " << this->getLine() << ", column " << this->getColumn() << endl; throw std::runtime_error(ss.str()); }
