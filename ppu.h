#pragma once

struct ppu_regs
{
    uint8_t PPUCTRL;
    uint8_t PPUMASK;
    uint8_t PPUSTATUS;
    uint8_t OAMADDR;
    uint8_t OAMDATA;
    uint8_t PPUSCROLL;
    uint8_t PPUADDR;
    uint8_t PPUDATA;
};

struct ppu
{
    struct ppu_regs regs;
};

extern struct ppu ppu;

void ppu_reset(void);

