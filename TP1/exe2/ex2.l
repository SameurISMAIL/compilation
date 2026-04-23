%{
#include <stdio.h>
#include <stdlib.h>

int nb_mots   = 0;
int nb_lignes = 0;
int nb_chars  = 0;
long somme    = 0;
%}

MOT    [a-zA-Z]+
NOMBRE [0-9]+
BLANC  [ \t]+

%%

{NOMBRE}  {
              nb_mots++;
              nb_chars += yyleng;
              somme    += atol(yytext);
          }

{MOT}     {
              nb_mots++;
              nb_chars += yyleng;
          }

\n        {
              nb_lignes++;
              nb_chars++;
          }

{BLANC}   { nb_chars += yyleng; }

.         { nb_chars++; }

%%

int yywrap() { return 1; }

int main(int argc, char *argv[])
{
    if (argc > 1)
        yyin = fopen(argv[1], "r");
    else
        yyin = stdin;

    yyout = fopen("resultat.txt", "w");

    yylex();

    fprintf(yyout, "Nombre de mots       : %d\n", nb_mots);
    fprintf(yyout, "Nombre de lignes     : %d\n", nb_lignes);
    fprintf(yyout, "Nombre de caracteres : %d\n", nb_chars);
    fprintf(yyout, "Somme des nombres    : %ld\n", somme);

    fclose(yyout);
    return 0;
}
