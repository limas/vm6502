#include <stdint.h>

#include "ppu.h"

struct ppu ppu;

void ppu_reset(void)
{
    ppu.regs.PPUCTRL = 0;
    ppu.regs.PPUMASK = 0;
    ppu.regs.PPUSTATUS = 0x80;
    ppu.regs.OAMADDR = 0;
    ppu.regs.OAMDATA = 0;
    ppu.regs.PPUSCROLL = 0;
    ppu.regs.PPUADDR = 0;
    ppu.regs.PPUDATA = 0;
}

