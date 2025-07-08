# compiler e flag
CC = cc
CFLAGS = -Wall -Wextra -Wno-missing-field-initializers -Iinclude

# librerie da linkare
LIBS = -lm -lpthread -lpigpio

# cartelle
SRC_DIR = src
INC_DIR = include
BUILD_DIR = build

# file sorgente, file oggetto e file eseguibile
SRCS := $(wildcard $(SRC_DIR)/*.c)
OBJS := $(patsubst $(SRC_DIR)/%.c, $(BUILD_DIR)/%.o, $(SRCS))
EXE  := $(BUILD_DIR)/coderbot.exe

# obbiettivo di default
all: clean $(EXE)

# linking dei file oggetto nel file eseguibile
$(EXE): $(OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LIBS)

# compilazione dei file sorgente in file oggetto
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.c
	@mkdir -p $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# rimozione della cartella di build
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean
