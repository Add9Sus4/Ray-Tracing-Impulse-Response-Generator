#
# Makefile
#

# OS X:
CC=gcc -g -D__MACOSX_CORE__ -Wno-deprecated
LIBS=-framework OpenGL -framework GLUT -framework CoreFoundation -lsndfile -lncurses
EXE = generateImpulse
FLAGS=-c -Wall 
SRCS = generateImpulse.c

# Linux:
#CC=gcc -g -Wno-deprecated
#LIBS=-lGL -lGLU -lglut -lportaudio -lsndfile

all: $(EXE)

$(EXE): $(SRCS) $(HDRS)
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LIBS)

clean:
	rm -f *~ core $(EXE) *.o output.wav
	rm -rf $(EXE).dSYM
