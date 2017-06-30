CC = g++
DEBUG = -g
CFLAGS = -Wall -c $(DEBUG)
LFLAGS = -Wall $(DEBUG)

OBJS = MovieList.o Movie.o NameList.o Name.o Iterator.o

all: p1 p2 p3


p1 : $(OBJS)
    $(CC) $(LFLAGS) $(OBJS) -o p1

p2: Bar.o main2.o
   g++ -Wall Bar.o main2.o -o p2

p3: Baz.o main3.o
   g++ -Wall Baz.o main3.o -o p3


clean:
     \rm *.o *~ p1

# g++ -std=c++11 -c main.cpp -o main.o -I/usr/local/include/vtk -I/usr/local/include/opencv2/xfeatures2d -I/usr/local/include/opencv2
