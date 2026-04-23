%{
#include <stdio.h>
#include "ex1_subdiv.tab.h"
%}

%%

[0-9]+          { yylval.entier = atoi(yytext); return NB; }
"somme"         { return SOM; }
"produit"       { return PROD; }
"soustraction"  { return SOUS; }
"division"      { return DIV; }
[,.]            { return yytext[0]; }
[$]             { return FIN; }
[ \t\n]         { }
.               { printf("Erreur lexicale: %s\n", yytext); }

%%

int yywrap() { return 1; }