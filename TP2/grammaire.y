%{
#include <stdio.h>
int yylex(void);
int yyerror(char *s);
%}

%%

mot : S '$'  { printf("mot correct\n"); }
;

S : 'a' S 'a'
  | 'b' S 'b'
  | 'c'
;

%%

int yylex() {
    char c;
    do {
        c = getchar();
    } while (c == ' ' || c == '\t' || c == '\n' || c == '\r');
    
    if (c == 'a' || c == 'b' || c == 'c' || c == '$') return c;
    else if (c == EOF) return 0;
    else { printf("erreur lexicale: '%c'\n", c); return 0; }
}

int yyerror(char *s) {
    printf("%s\n", s);
    return 0;
}

int main() {
    yyparse();
    printf("\n");
    return 0;
}