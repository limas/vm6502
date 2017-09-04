.PHONY: all clean

EXE_NAME := nes

EXEDIR := .
SRCDIR := .
INCDIR := .

SRC_C := $(foreach dir,$(SRCDIR),$(wildcard $(dir)/*.c))
SRC_H := $(foreach dir,$(INCDIR),$(wildcard $(dir)/*.h))

INC_OPT := $(foreach dir,$(INCDIR),-I$(dir)/)

CC_OPT := -g

all:
	$(CC) $(CC_OPT) -o $(EXE_NAME) $(SRC_C) $(INC_OPT)

clean:
	rm -f $(EXEDIR)/$(EXE_NAME)
