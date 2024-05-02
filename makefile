MAIN = main

CC = gcc

CFLAGS = -Wall -lpthread -lrt -lm `allegro-config --libs`

$(MAIN): $(MAIN).o mypthlib.o flyrand.o motor.o
	$(CC) -o $(MAIN) $(MAIN).o mypthlib.o flyrand.o motor.o $(CFLAGS)

$(MAIN).o: $(MAIN).c 
	$(CC) -c $(MAIN).c 

mypthlib.o: mypthlib.c 
	$(CC) -c mypthlib.c 
	
flyrand.o: flyrand.c 
	$(CC) -c flyrand.c

motor.o: motor.c 
	$(CC) -c motor.c

clean:
	rm -rf *o $(MAIN)
