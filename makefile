# Makefile per struttura: src/ include/ obj/ bin/
CC := gcc

SRC_DIR := src
INCLUDE_DIR := include
OBJ_DIR := obj
BIN_DIR := bin

ALLEGRO_CFLAGS := $(shell allegro-config --cflags)
ALLEGRO_LIBS   := $(shell allegro-config --libs)

CFLAGS := -Wall -I$(INCLUDE_DIR) $(ALLEGRO_CFLAGS)
LIBS   := -lpthread -lrt -lm $(ALLEGRO_LIBS)

SRCS := $(wildcard $(SRC_DIR)/*.c)
OBJS := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRCS))

TARGET := $(BIN_DIR)/main

.PHONY: all clean dirs

all: dirs $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LIBS)

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | dirs
	$(CC) $(CFLAGS) -c $< -o $@

dirs:
	mkdir -p $(OBJ_DIR) $(BIN_DIR)

clean:
	rm -rf $(OBJ_DIR)/* $(BIN_DIR)/*

