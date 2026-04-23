%{
#include <stdio.h>
%}

%%
(0|1)+    { printf("C'est un nombre binaire\n"); }
.*        { printf("Ce n'est pas un nombre binaire\n"); }
%%

int yywrap()
{
 return 1;
}

int main(int argc,char *argv[])
{
 if(argc>1)
 {
  yyin=fopen(argv[1],"r");
 }

 yylex();

 return 0;
}
