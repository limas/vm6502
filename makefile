.PHONY: all clean run

Q := @

EXE_NAME := nes

EXEDIR := .
SRCDIR := .
INCDIR := .
OBJDIR := ./obj

SRC_C := $(foreach dir,$(SRCDIR),$(wildcard $(dir)/*.c))
SRC_H := $(foreach dir,$(INCDIR),$(wildcard $(dir)/*.h))
OBJS  := $(patsubst %.c,%.o,$(SRC_C))

INC_OPT := $(foreach dir,$(INCDIR),-I$(dir)/)

CC_OPT := -g -Wall

all: $(EXE_NAME)

clean:
	$(Q)rm -f $(EXEDIR)/$(EXE_NAME)
	$(Q)rm -f $(OBJS)

$(EXE_NAME) : $(OBJS)
	$(Q)echo [LD] $@
	$(Q)$(CC) $(CC_OPT) -o $@ $^ $(INC_OPT)

$(OBJS) : %.o : %.c
	$(Q)echo [CC] $@
	$(Q)$(CC) $(CC_OPT) -c -o $@ $^ $(INC_OPT)

run: $(EXE_NAME)
	$(Q)echo [RUN] $@
	$(Q)./$(EXE_NAME) -f rom/Super_Mario_Bros.nes
