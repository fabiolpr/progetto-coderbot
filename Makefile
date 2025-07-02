# Compiler e flag
CC = cc
CFLAGS = -Wall -Wextra

# librerie da linkare
LIBS = -lm -lpthread -lpigpio

# source e file oggetto
SRCS := $(wildcard *.c)
OBJS := $(SRCS:.c=.o)
EXE  := coderbot.exe

# target di default
all: $(EXE)

# linking
$(EXE): $(OBJS)
	$(CC) $(CFLAGS) -o $@ cbdef.h $^ $(LIBS)

# compila i file oggetto
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# rimuovi i file oggetto e l'eseguibile
clean:
	rm -f $(OBJS) $(EXE)

.PHONY: all clean
