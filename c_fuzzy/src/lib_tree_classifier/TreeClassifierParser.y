%skeleton "lalr1.cc"
%debug
%defines
%define parser_class_name { TreeClassifierParser }
%define api.namespace {tc}
 
%code requires
{
	#include <string>
	#include "FuzzyClass.h" 
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

%union { std::string* str; int integer; bool boolean; VariableList* vlist; ConstantList* clist; std::pair<VariableList*, ConstantList*>* elists;  }

%token <str> ID
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

%type <boolean> importantFlag
%type <str> fuzzySuperclass fuzzyConstraint
%type <vlist> variables variableList
%type <clist> constants 
%type <clist> constantList
%type <elists> fuzzyClassElements
 

%%

fuzzyClassifiers	: fuzzyClass fuzzyClassifiers
			| fuzzyClass
			;

fuzzyClass		: CLASS ID fuzzySuperclass importantFlag fuzzyClassElements fuzzyAttributes END_CLASS
			{
				builder.buildClass(*$2, *$3, $5->first, $5->second, $4);
				free($2);
				free($3);
			}
			;

fuzzySuperclass		: EXTENDS ID 
			{
				$$ = $2;
			}
			| /* empty */
			{
				$$ = NULL;
			}
			;

importantFlag		: IMPORTANT 
			{
				$$ = true;
			}
			| /* empty */
			{
				$$ = false;
			}
			;

fuzzyClassElements	: constants variables
			{
				$$ = new std::pair<VariableList*, ConstantList*>();
				$$->first = $2;
				$$->second = $1;
			}
			| variables constants
			{
				$$ = new std::pair<VariableList*, ConstantList*>();
				$$->first = $1;
				$$->second = $2;
			}
			| constants
			{
				$$ = new std::pair<VariableList*, ConstantList*>();
				$$->first = NULL;
				$$->second = $1;
			}
			| variables
			{
				$$ = new std::pair<VariableList*, ConstantList*>();
				$$->first = $1;
				$$->second = NULL;
			}
			| /* empty */
			{
				$$ = new std::pair<VariableList*, ConstantList*>();
				$$->first = NULL;
				$$->second = NULL;
			}
			;

constants		: CONSTANTS constantList END_CONSTANTS
			{
				$$ = $2;
			}
			;
			
constantList		: ID EQUAL NUMBER SEMICOLON constantList
			{
				$$ = builder.buildCostantList($5, *$1, $3);
				free($1);
			}
			| /* empty */
			{
				$$ = NULL;
			}
			;

variables		: VARIABLES variableList END_VARIABLES
			{
				$$ = $2;
			}
			;

variableList		: ID SEMICOLON variableList
			{
				$$ = builder.buildVariableList($3, *$1);
				free($1);
			}
			| /* empty */
			{
				$$ = NULL;
			}
			;

fuzzyAttributes		: fuzzyAttribute fuzzyAttributes
			| fuzzyRelation fuzzyAttributes
			| /* empty */
			;

fuzzyAttribute		: ID IS ID SEMICOLON
			;

fuzzyRelation		: ID PERIOD ID MATCH ID SEMICOLON
			| ID PERIOD ID fuzzyConstraint BETWEEN LPAR ID COMMA ID RPAR SEMICOLON
			;


fuzzyConstraint		: IS ID
			{
				$$ = $2;
			}
			| /* empty */
			{
				$$ = NULL;
			}
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

