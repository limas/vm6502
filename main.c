#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>


#include "cpu.h"
#include "ppu.h"
/*
#include "apu.h"
*/

struct config
{
    char *rom_name;
};

struct array
{
    uint32_t size;
    uint8_t *data;
};

struct nes_header
{
    uint8_t magic[4];
    uint8_t num_16kB_rom;
    uint8_t num_8kB_vrom;
    uint8_t mirror:1; /* 1 for vertical mirroring, 0 for horizontal mirroring */
    uint8_t bk_ram:1;
    uint8_t has_trainer:1;
    uint8_t four_screen_vram:1;
    uint8_t mapper_type_low:4;
    uint8_t vs_sys_cart:1;
    uint8_t resv1:3;
    uint8_t mapper_type_high:4;
    uint8_t num_8kB_ram;
    uint8_t pal_cart:1; /* 1 for PAL cartridges, otherwise assume NTSC */
    uint8_t resv2:7;
    uint8_t resv3[6];
};

struct cartridge
{
    struct nes_header header;
    struct array trainer;
    struct array rom;
    struct array vrom;
};

typedef bool (*io_handle_write)(uint16_t addr, uint8_t *data, uint16_t len);
typedef bool (*io_handle_read)(uint16_t addr, uint8_t *data, uint16_t len);

static bool ram_write(uint16_t addr, uint8_t *data, uint16_t len);
static bool ram_read(uint16_t addr, uint8_t *data, uint16_t len);
static bool ppu_reg_write(uint16_t addr, uint8_t *data, uint16_t len);
static bool ppu_reg_read(uint16_t addr, uint8_t *data, uint16_t len);
static bool cartridge_rom_write(uint16_t addr, uint8_t *data, uint16_t len);
static bool cartridge_rom_read(uint16_t addr, uint8_t *data, uint16_t len);

struct memory_map_entry
{
    char *name;
    uint16_t start;
    uint16_t end;
    io_handle_write write;
    io_handle_read read;
};

struct apu
{

};

static struct config gconfig;
static uint8_t *gram;
static struct cartridge gcartridge;

struct memory_map_entry cpu_map[]=
{
    {
        .name="ram",
        .start=0x0000,
        .end=0x1fff,
        .write=ram_write,
        .read=ram_read,
    },
    {
        .name="ppu_regs",
        .start=0x2000,
        .end=0x3fff,
        .write=ppu_reg_write,
        .read=ppu_reg_read,
    },
    {
        .name="extent_module",
        .start=0x5000,
        .end=0x6fff,
    },
    {
        .name="cartridge_ram",
        .start=0x6000,
        .end=0x7fff,
    },
    {
        .name="cartridge_rom_lower",
        .start=0x8000,
        .end=0xbfff,
        .write=cartridge_rom_write,
        .read=cartridge_rom_read,
    },
    {
        .name="cartridge_rom_upper",
        .start=0xc000,
        .end=0xffff,
        .write=cartridge_rom_write,
        .read=cartridge_rom_read,
    },
};

#define NUM_CPU_MEM_ENTRY (sizeof(cpu_map)/sizeof(struct memory_map_entry))

static void print_help(void)
{
    printf("usage:\n");
    printf("  nes [option1] [option2] ...\n");
    printf("\n");
    printf("options:\n");
    printf("  -h print this list\n");
    printf("  -f specify input rom name\n");
}

static bool handle_args(int argc, char **argv, struct config *config)
{
    int opt;

    while((opt = getopt(argc, argv, "f:h")) != -1)
    {
        switch(opt)
        {
        case 'f':
            config->rom_name=optarg;
            break;
        case '?':
        case 'h':
            print_help();
            return false;
        default:
            break;
        }
    }

    return true;
}

static struct memory_map_entry *_find_mem_entry(uint16_t addr)
{
    uint8_t idx;

    for(idx=0; idx<NUM_CPU_MEM_ENTRY; idx++)
    {
        if((addr>=cpu_map[idx].start) && (addr<=cpu_map[idx].end))
        {
            return &cpu_map[idx];
        }
    }

    return NULL;
}

bool cpu_mem_read(uint16_t addr, uint8_t *data, uint16_t len)
{
    struct memory_map_entry *ip;

    ip = _find_mem_entry(addr);

    if(ip == NULL)
    {
        printf("no corresponding ip to address: 0x%04x\n", addr);
        return false;
    }

    if(ip->read)
        return ip->read(addr, data, len);
    else
        printf("there is no read handler installed to address: 0x%04x\n", addr);

    return false;
}

bool cpu_mem_write(uint16_t addr, uint8_t *data, uint8_t len)
{
    struct memory_map_entry *ip;

    ip = _find_mem_entry(addr);

    if(ip == NULL)
    {
        printf("no corresponding ip to address: 0x%04x\n", addr);
        return false;
    }

    if(ip->write)
        return ip->write(addr, data, len);
    else
        printf("there is no read handler installed to address: 0x%04x\n", addr);

    return false;
}

static bool ram_write(uint16_t addr, uint8_t *data, uint16_t len)
{
    uint16_t index;

    for(index=0; index<len; index++)
    {
        gram[addr+index] = data[index];
    }

    return true;
}

static bool ram_read(uint16_t addr, uint8_t *data, uint16_t len)
{
    uint16_t index;

    for(index=0; index<len; index++)
    {
        data[index] = gram[addr+index];
    }

    return true;
}

static bool ram_deinit(void)
{
    if(gram)
    {
        free(gram);
        gram = NULL;
    }

    return true;
}

static bool ram_init(void)
{
    if(!gram)
        gram = (uint8_t *)malloc(0x800);

    if(!gram)
        return false;

    return true;
}

static bool ppu_reg_write(uint16_t addr, uint8_t *data, uint16_t len)
{
    uint8_t *regs = (uint8_t *)&ppu.regs;
    regs[addr % 8] = *data;

    return true;
}

static bool ppu_reg_read(uint16_t addr, uint8_t *data, uint16_t len)
{
    uint8_t *regs = (uint8_t *)&ppu.regs;
    *data = regs[addr % 8];

    return true;
}

static bool cartridge_rom_write(uint16_t addr, uint8_t *data, uint16_t len)
{
    return false;
}

static bool cartridge_rom_read(uint16_t addr, uint8_t *data, uint16_t len)
{
    uint16_t offset;

    if(gcartridge.rom.size <= 16*1024)
    {
        if(addr < 0xC000)
        {
            printf("invalid rom address to read.\n");
            return false;
        }
        else
        {
            offset = addr-0xC000;
            memcpy((void *)data, (void *)(gcartridge.rom.data+(offset)), len);
        }
    }
    else if(gcartridge.rom.size <= 32*1024)
    {
        offset = addr-0x8000;
        memcpy((void *)data, (void *)(gcartridge.rom.data+(offset)), len);
    }
    else
    {
    
    }

    return true;
}

static bool unload_rom(void)
{
    gcartridge.trainer.size = 0;
    if(gcartridge.trainer.data)
    {
        free(gcartridge.trainer.data);
        gcartridge.trainer.data = NULL;
    }

    gcartridge.rom.size = 0;
    if(gcartridge.rom.data)
    {
        free(gcartridge.rom.data);
        gcartridge.rom.data = NULL;
    }

    gcartridge.vrom.size = 0;
    if(gcartridge.vrom.data)
    {
        free(gcartridge.vrom.data);
        gcartridge.vrom.data = NULL;
    }

    return true;
}

static bool load_rom(char *file_name)
{
    FILE *fp = NULL;
    struct nes_header *header = &gcartridge.header;

    fp = fopen(file_name, "rb");
    if(!fp)
        return false;

    /* read header of nes format file */
    fread((void *)header, 1, sizeof(struct nes_header), fp);
    if(strncmp((char *)header->magic, "NES\x1a", 4))
    {
        printf("unknown magic number.\n");
        return false;
    }

    do
    {
        if(header->has_trainer)
        {
            gcartridge.trainer.size = 512;
            gcartridge.trainer.data = (uint8_t *)malloc(gcartridge.trainer.size);
            if(!gcartridge.trainer.data)
            {
                break;
            }
            fread((void *)gcartridge.trainer.data, 1, 512, fp);
        }

        gcartridge.rom.size = header->num_16kB_rom*16*1024;
        gcartridge.rom.data = (uint8_t *)malloc(gcartridge.rom.size);
        if(!gcartridge.rom.data)
        {
            break;
        }
        fread((void *)gcartridge.rom.data, 1, gcartridge.rom.size, fp);

        gcartridge.vrom.size = header->num_8kB_vrom*8*1024;
        gcartridge.vrom.data = (uint8_t *)malloc(gcartridge.vrom.size);
        if(!gcartridge.vrom.data)
        {
            break;
        }
        fread((void *)gcartridge.vrom.data, 1, gcartridge.vrom.size, fp);
    
        fclose(fp);

        return true;
    }while(0);

    fclose(fp);

    unload_rom();

    return true;
}

int main(int argc, char **argv)
{
    if(handle_args(argc, argv, &gconfig) == false)
        return -1;

    //cpu_dump_regs();

    if(ram_init() == false)
    {
        printf("init ram fail.\n");
        return -1;
    }

    if(load_rom(gconfig.rom_name) == false)
    {
        printf("load rom image fail.\n");
        return -1;
    }

    cpu_reset();
    ppu_reset();

    while(1)
    {
        if(cpu_run() == false)
            break;
    }

    unload_rom();

    ram_deinit();

    return 0;
}
