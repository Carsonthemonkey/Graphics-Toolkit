CC=gcc
CFLAGS=-I./include -g -I/opt/X11/include -I./vendor/OpenImageDenoise/include -Ofast
SRC_DIR=lib
OBJ_DIR=out
INCLUDE_DIR=include

# Automatically find all C source files
SRCS=$(wildcard $(SRC_DIR)/*.c)

# Convert the .c filenames to .o to determine the object file names
OBJS=$(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRCS))

# Target to compile everything
all: $(OBJS)


$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c | $(OBJ_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Create the obj directory if it doesn't exist
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

clean:
	rm -rf $(OBJ_DIR)/*.o

.PHONY: all clean
