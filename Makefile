PREFIX ?= /usr/local

#PREFIX_APRILTAG_EXAMPLE ?=/udd/pmordel/ros/indigo/catkin_src/apriltag2/apriltag2_example/lib

CC = gcc
AR = ar

CFLAGS = -std=gnu99 -fPIC -Wall -Wno-unused-parameter -Wno-unused-function -I. -O4
#CFLAGS += -I. -O4 -fno-strict-overflow

APRILTAG_SRCS := $(wildcard *.c common/*.c)
APRILTAG_HEADERS := $(wildcard *.h common/*.h)
APRILTAG_OBJS := $(APRILTAG_SRCS:%.c=%.o)
TARGETS := libapriltag.a libapriltag.so 
# LIBS := -Lusr/include/flycapture

.PHONY: all
all: $(TARGETS)
	@$(MAKE) -C apriltag2_example/_build all

.PHONY: install
install: libapriltag.so
	#@chmod +x install.sh
	@./install.sh $(PREFIX)/lib libapriltag.so #this should be the line that install the library
	#@./install.sh $(PREFIX_APRILTAG_EXAMPLE) libapriltag.so
	@./install.sh $(PREFIX)/include/apriltag $(APRILTAG_HEADERS)
	@sed 's:^prefix=$$:prefix=$(PREFIX):' < apriltag.pc.in > apriltag.pc
	@./install.sh $(PREFIX)/lib/pkgconfig apriltag.pc
	@rm apriltag.pc
	@ldconfig

libapriltag.a: $(APRILTAG_OBJS)
	@echo "   [$@]"
	@$(AR) -cq $@ $(APRILTAG_OBJS)

libapriltag.so: $(APRILTAG_OBJS)
	@echo "   [$@]"
	@$(CC) -fPIC -shared -o $@ $^

%.o: %.c
	@echo "   $@"
	@$(CC) -o $@ -c $< $(CFLAGS)

.PHONY: clean

clean:
	@rm -rf *.o common/*.o $(TARGETS)
	@$(MAKE) -C apriltag2_example/_build clean
