%skeleton "lalr1.cc"
%debug
%defines
%define parser_class_name { TreeClassifierParser }
%define api.namespace {tc}
 
%code requires
{
	#include<vector>
	#include<map>
	class TreeClassifierBuilder;
	class TreeClassifierScanner;
}
 
%lex-param   { TreeClassifierScanner  &scanner  }
%parse-param { TreeClassifierScanner  &scanner  }
 
%lex-param   { TreeClassifierBuilder  &builder  }
%parse-param { TreeClassifierBuilder  &builder  }

%error-verbose
%locations
 
%code
{
	#include<iostream>
	#include<cstdlib>
	#include<fstream>
	
	#include "TreeClassifierBuilder.h"
	
	static int yylex(tc::TreeClassifierParser::semantic_type *yylval, tc::TreeClassifierParser::location_type* l, TreeClassifierScanner& scanner, TreeClassifierBuilder& builder);
}

%union { std::string* str; int integer; }

%token<str> ID
%token IS
%token MATCH
%token BETWEEN

%token CLASS
%token VARIABLES
%token END_VARIABLES
%token CONSTANTS
%token END_CONSTANTS
%token END_CLASS
%token EXTENDS
%token IMPORTANT

%token PERIOD
%token SEMICOLON
%token COMMA
%token LPAR
%token RPAR
%token EQUAL

%token <integer> NUMBER

%%

fuzzyClassifiers	: fuzzyClass fuzzyClassifiers
			|
			;

fuzzyClass		: CLASS ID fuzzySuperclass importantFlag fuzzyClassElements fuzzyAttributes END_CLASS
			;

fuzzySuperclass		: EXTENDS ID 
			|
			;

importantFlag		: IMPORTANT
			|
			;

fuzzyClassElements	: constants variables
			| variables constants
			| constants
			| variables
			|
			;

constants		: CONSTANTS constantList END_CONSTANTS
			;
			
constantList		: ID EQUAL NUMBER SEMICOLON constantList
			|
			;

variables		: VARIABLES variableList END_VARIABLES
			;

variableList		: ID SEMICOLON variableList
			|
			;

fuzzyAttributes		: fuzzyAttribute fuzzyAttributes
			| fuzzyRelation fuzzyAttributes
			;

fuzzyAttribute		: ID IS ID SEMICOLON
			;

fuzzyRelation		: ID PERIOD ID MATCH ID SEMICOLON
			| ID PERIOD ID fuzzyConstraint BETWEEN LPAR ID COMMA ID RPAR SEMICOLON
			;


fuzzyConstraint		: IS ID
			| 
			;
%%

void tc::TreeClassifierParser::error(const tc::TreeClassifierParser::location_type& l, const std::string& msg)
{
    std::cerr << "Error: " << msg << ", between " << l.begin << " and " << l.end << std::endl;
    exit(-1);
}

/* include for access to scanner.yylex */
#include "FuzzyScanner.h"
static int yylex(tc::TreeClassifierParser::semantic_type *yylval, tc::TreeClassifierParser::location_type* l, TreeClassifierScanner& scanner, TreeClassifierBuilder& builder)
{
	l->step();
	return(scanner.yylex(yylval));
}

