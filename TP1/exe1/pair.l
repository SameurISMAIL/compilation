%{
#include <stdio.h>
%}

pairpair (aa|bb)*((ab|ba)(aa|bb)*(ab|ba)(aa|bb)*)*

%%



{pairpair} { printf("pair a et b\n"); }
a*b* { printf("a puis b\n"); }


%%

int yywrap(){return 1;}

int main()
{
 yylex();
}
