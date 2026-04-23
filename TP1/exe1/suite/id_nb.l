%{
#include <stdio.h>
%}

LETTRE   [a-zA-Z]
CHIFFRE  [0-9]
ID       {LETTRE}({LETTRE}|{CHIFFRE})*
NB       {CHIFFRE}+

%%
{ID}        { printf("ID  : [%s]\n", yytext); }

{NB}        { printf("NB  : [%s]\n", yytext); }

[ \t\n]     { /* ignorer */ }

.           { printf("AUTRE: [%s]\n", yytext); }
%%

int yywrap() { return 1; }

int main(int argc, char *argv[])
{
    if(argc > 1)
        yyin = fopen(argv[1], "r");
    else
        yyin = stdin;

    yylex();
    return 0;
}
