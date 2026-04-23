%{
#include <stdio.h>
%}

nom     [A-Z][a-z]+
prenom  [a-z]+
tel     [0-9][0-9]" "[0-9][0-9]" "[0-9][0-9]" "[0-9][0-9]" "[0-9][0-9]

%%

{nom}    { printf("Nom     : %s\n", yytext); }
{prenom} { printf("Prenom  : %s\n", yytext); }
"@"      { }
{tel}    { printf("Telephone: %s\n\n", yytext); }
[ \t\n]  { }

%%

int yywrap() { return 1; }

int main(int argc, char *argv[])
{
    if (argc > 1)
        yyin = fopen(argv[1], "r");
    yylex();
    return 0;
}
