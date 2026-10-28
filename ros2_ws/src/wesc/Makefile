# Compilation hors ROS (le_loto/RoboCar) : make -> libwesc.a
LIB = libwesc.a

SRC = $(wildcard src/*.c)
OBJ = $(SRC:.c=.o)
CFLAGS += -Iinclude -D_DEFAULT_SOURCE -Wall -Wextra -std=c11

all: $(LIB)
$(LIB): $(OBJ)
	ar rcs $@ $^

clean:
	rm -f $(OBJ)

fclean: clean
	rm -f $(LIB)

re: fclean all
