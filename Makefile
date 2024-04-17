CC = g++ 
LDLIBS = -lsfml-graphics -lsfml-window -lsfml-system -lm
CFLAGS = -Iphysics -Ihelper -ggdb3

# all: ball_collider debug

ball_collider: main.cpp
	$(CC) main.cpp $(LDLIBS) $(CFLAGS) -o ball_collider

debug: debug.cpp
	$(CC) debug.cpp $(LDLIBS) $(CFLAGS) -o debug

clean:
	rm ball_collider debug
