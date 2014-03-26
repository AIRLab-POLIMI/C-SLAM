%skeleton "lalr1.cc"
%debug
%defines
%define parser_class_name { TreeClassifierParser }
%define api.namespace {tc}
 
%code requires
{
	#include <sstream>
	#include <stdexcept>

	#include <string>
	#include <vector>
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
	#include<cstdlib>
	#include<fstream>
	
	#include "TreeClassifierBuilder.h"
	
	static int yylex(tc::TreeClassifierParser::semantic_type *yylval, tc::TreeClassifierParser::location_type* l, TreeClassifierScanner& scanner, TreeClassifierBuilder& builder);
}

%union 
{ 
	std::string* str; 
	std::vector<std::string>* vstr; 
	int integer; bool boolean;
	VariableList* vlist; 
	ConstantList* clist; 
	std::pair<VariableList*, ConstantList*>* elists;
	FuzzyFeatureList* flist;
}

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
%type <vstr> fuzzySimpleFeature fuzzyRelation
%type <vlist> variables variableList
%type <clist> constants 
%type <clist> constantList
%type <elists> fuzzyClassElements
%type <flist> fuzzyFeatures
 

%%

fuzzyClassifiers	: fuzzyClass fuzzyClassifiers
			| fuzzyClass
			;

fuzzyClass		: CLASS ID fuzzySuperclass importantFlag fuzzyClassElements fuzzyFeatures END_CLASS
			{
				builder.buildClass(*$2, *$3, $5->first, $5->second, $6, $4);
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
				$$ = new std::string("");
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

fuzzyFeatures		: fuzzySimpleFeature fuzzyFeatures
			{
				$$ = builder.buildFeaturesList($2, *$1);
				free($1);
				
			}
			| fuzzyRelation fuzzyFeatures
			{
				$$ = builder.buildFeaturesList($2, *$1);
				free($1);
			}
			| /* empty */
			{
				$$ = NULL;
			}
			;

fuzzySimpleFeature	: ID IS ID SEMICOLON
			{
				$$ = new std::vector<std::string>();
				$$->push_back(*$1);
				$$->push_back(*$3);
				delete $1;
				delete $3;
			}
			;

fuzzyRelation		: ID PERIOD ID MATCH ID SEMICOLON
			{
				$$ = new std::vector<std::string>();
				$$->push_back(*$1);
				$$->push_back(*$3);
				$$->push_back(*$5);
				delete $1;
				delete $3;
				delete $5;
			}
			| ID PERIOD ID fuzzyConstraint BETWEEN LPAR ID COMMA ID RPAR SEMICOLON
			{
				$$ = new std::vector<std::string>();
				$$->push_back(*$1);
				$$->push_back(*$3);
				$$->push_back(*$7);
				$$->push_back(*$9);
				delete $1;
				delete $3;
				delete $7;
				delete $9;

				if($4 != NULL)
				{
					$$->push_back(*$4);
					delete $4;
				}
			}
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
	std::stringstream ss;
	ss << "Error: " << msg << ", between " << l.begin << " and " << l.end << std::endl;
	throw std::runtime_error(ss.str());
}

/* include for access to scanner.yylex */
#include "FuzzyScanner.h"
static int yylex(tc::TreeClassifierParser::semantic_type *yylval, tc::TreeClassifierParser::location_type* l, TreeClassifierScanner& scanner, TreeClassifierBuilder& builder)
{
	l->step();
	return(scanner.yylex(yylval));
}

