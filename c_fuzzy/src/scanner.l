%{
	#include<iostream>
	#include<string>
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

DIGIT [0-9]
CAPITAL [A-Z]
LETTER [a-z]|{CAPITAL}
BLANK [ \t]
NEWLINE \n\r|\n
FUZZYLABEL tol|tor|tri|tra|int|sgt

%%

{BLANK} { }
{NEWLINE} {this->newLine();}
{CAPITAL}({LETTER}|{DIGIT})* { yylval->str = new std::string(yytext); return token::ID; }
{DIGIT}+ { yylval->integer = atoi(yytext); return token::PARAMETER; }
{FUZZYLABEL} { yylval->str = new std::string(yytext); return token::F_LABEL; }
"and" { return token::OP_AND; }
"or" { return token::OP_OR; }
"~" { return token::OP_NOT; }
"==>" { return token::THEN; }
"is" { return token::IS; }
"(" { return token::OPEN_B; }
")" { return token::CLOSE_B; }
";" { return token::END_RULE; }
"->" { return token::LIKE; }
"," { return token::COMMA; }
. { cerr << "Unknown char " << *yytext << " at line " << this->getLine() << ", column " << this->getColumn() << endl; return UNKNOWN; }
