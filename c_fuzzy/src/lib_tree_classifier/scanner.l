%{
	#include <iostream>
	#include <string>
	#include "TreeClassifierScanner.h"
	
	#define YY_USER_ACTION this->count(yyleng);
	
	#define UNKNOWN -1
	
	typedef tc::TreeClassifierParser::token token;
	
	using namespace std;
%}


%option yylineno
%option debug 
%option nodefault 
%option yyclass="TreeClassifierScanner" 
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


 /* identificators */
{CAPITAL}({LETTER}|{DIGIT})* { yylval->str = new std::string(yytext); return token::ID; }
-?{DIGIT}+ { yylval->integer = atoi(yytext); return token::PARAMETER; }
{FUZZYLABEL} { yylval->str = new std::string(yytext); return token::F_LABEL; }

 /* unknown tokens */
. { cerr << "Unknown char " << *yytext << " at line " << this->getLine() << ", column " << this->getColumn() << endl; return UNKNOWN; }
