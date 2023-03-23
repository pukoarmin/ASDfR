CC=gcc
CFLAGS=-Wall -Werror -I/usr/xenomai/include
LDFLAGS=-L/usr/xenomai/lib -lnative -lxenomai

TARGET=rt
SRCS=rt.c
OBJS=$(SRCS:.c=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $^ -o $@

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)
