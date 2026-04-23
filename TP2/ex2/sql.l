%{
#include <stdio.h>
#include "sql.tab.h"
%}

%%

[Cc][Rr][Ee][Aa][Tt][Ee]                          { return CREATE; }
[Tt][Aa][Bb][Ll][Ee]                               { return TABLE_KW; }
[Ii][Nn][Tt]                                       { return DATATYPE; }
[Vv][Aa][Rr][Cc][Hh][Aa][Rr]                      { return DATATYPE; }
[Dd][Aa][Tt][Ee]                                   { return DATATYPE; }
[Pp][Rr][Ii][Mm][Aa][Rr][Yy][ \t]+[Kk][Ee][Yy]   { return PRIMARY_KEY; }

[a-zA-Z_][a-zA-Z0-9_]*   { yylval.str = strdup(yytext); return IDENTIFIER; }
[0-9]+                    { yylval.num = atoi(yytext);   return NUMBER; }

"("       { return '('; }
")"       { return ')'; }
","       { return ','; }

[ \t\n]+  { /* ignorer espaces */ }
.         { printf("Erreur lexicale: %s\n", yytext); }

%%

int yywrap() { return 1; }