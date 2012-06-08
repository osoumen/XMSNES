CXX = g++
RM = rm
CXXFLAGS = -O3 -g -Wall
LINK =
LIBS =

TARGET = xm2snes
OBJS = main.o brr.o files.o patterns.o samples.o \
	simpmath.o spc.o wav.o

$(TARGET): $(OBJS)
	$(CXX) $(LINK) -o $@ $^ $(LIBS)

clean:
	$(RM) $(OBJS)
