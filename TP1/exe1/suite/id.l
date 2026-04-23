%{
#include <stdio.h>
%}

letter  [a-zA-Z]
digit   [0-9]
id      {letter}({letter}|{digit})*

%%

{id}    { printf("[%s] : identificateur valide (id)\n", yytext); }

.       { printf("[%s] : n'est pas un id valide\n", yytext); }

%%

int yywrap() { return 1; }

int main(int argc, char *argv[]) {
    ++argv; --argc;
    if (argc > 0)
        yyin = fopen(argv[0], "r");
    else
        yyin = stdin;
    yylex();
    return 0;
}
