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
%token END_RULE
%token OP_OR
%token OP_AND
%token OP_NOT
%token OPEN_B
%token CLOSE_B
%token THEN
%token IS
%token IF

%token FUZZIFY
%token END_FUZZIFY

%token LIKE
%token COMMA
%token <str> F_LABEL
%token <integer> PARAMETER


%left OP_AND
%left OP_OR
%right OP_NOT

%%

test: ID;

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

