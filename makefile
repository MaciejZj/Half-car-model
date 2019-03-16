CC = gcc
CFLAGS = -Wall
LDFLAGS = -L/usr/local/lib
LDLIBS = -lgsl -lgslcblas -lm 
RM = rm -f

.DEFAULT_GOAL := all

.PHONY: sim
sim: simHalf.out
	./simHalf.out
	@if [ "`uname`" = "Darwin" ]; then sleep 1; open *.eps; fi

.PHONY: all
all: simHalf.out

%.out: %.o
	$(CC) $(LDFLAGS) $^ $(LDLIBS) -o $@
	@make clean

%.o: %.c
	$(CC) $(CFLAGS) -c $^

clean:
	@$(RM) *.o
