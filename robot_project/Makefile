CC     = gcc
CFLAGS = -Wall -O2 -I/usr/local/include
LIBS   = -L/usr/local/lib -lraylib -lm -lpthread -ldl

all: robot

robot: robot.tab.c lex.yy.c robot3d.c semantic.c
	$(CC) $(CFLAGS) robot.tab.c robot3d.c semantic.c -o robot $(LIBS)

robot.tab.c robot.tab.h: robot.y
	bison -d robot.y

lex.yy.c: robot.l robot.tab.h
	flex robot.l

clean:
	rm -f robot.tab.c robot.tab.h lex.yy.c robot