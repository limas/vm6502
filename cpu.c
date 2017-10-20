#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "cpu.h"

#define BIT(b) (1<<b)

#define STATUS_REG_CARRY     ((cpu6502.regs.sr&BIT(0))?(1):(0))
#define STATUS_REG_ZERO      ((cpu6502.regs.sr&BIT(1))?(1):(0))
#define STATUS_REG_INTERRUPT ((cpu6502.regs.sr&BIT(2))?(1):(0))
#define STATUS_REG_DECIMAL   ((cpu6502.regs.sr&BIT(3))?(1):(0))
#define STATUS_REG_OVERFLOW  ((cpu6502.regs.sr&BIT(6))?(1):(0))
#define STATUS_REG_NEGATIVE  ((cpu6502.regs.sr&BIT(7))?(1):(0))

#define STATUS_REG_SET_CARRY()     do{cpu6502.regs.sr|=BIT(0);}while(0)
#define STATUS_REG_SET_ZERO()      do{cpu6502.regs.sr|=BIT(1);}while(0)
#define STATUS_REG_SET_INTERRUPT() do{cpu6502.regs.sr|=BIT(2);}while(0)
#define STATUS_REG_SET_DECIMAL()   do{cpu6502.regs.sr|=BIT(3);}while(0)
#define STATUS_REG_SET_OVERFLOW()  do{cpu6502.regs.sr|=BIT(6);}while(0)
#define STATUS_REG_SET_NEGATIVE()  do{cpu6502.regs.sr|=BIT(7);}while(0)

#define STATUS_REG_CLR_CARRY()     do{cpu6502.regs.sr&=(~BIT(0));}while(0)
#define STATUS_REG_CLR_ZERO()      do{cpu6502.regs.sr&=(~BIT(1));}while(0)
#define STATUS_REG_CLR_INTERRUPT() do{cpu6502.regs.sr&=(~BIT(2));}while(0)
#define STATUS_REG_CLR_DECIMAL()   do{cpu6502.regs.sr&=(~BIT(3));}while(0)
#define STATUS_REG_CLR_OVERFLOW()  do{cpu6502.regs.sr&=(~BIT(6));}while(0)
#define STATUS_REG_CLR_NEGATIVE()  do{cpu6502.regs.sr&=(~BIT(7));}while(0)

struct regs
{
    uint16_t pc;
    uint8_t acc;
    uint8_t idx_x;
    uint8_t idx_y;
    uint8_t sp;
    uint8_t sr;
};

struct cpu
{
    struct regs regs;
};

struct instruction
{
    uint8_t code;
    uint8_t num_op;
};

struct instruction isa[]=
{
    /* nop */
    {0xea, 1},
    /* adc */
    {0x69, 2}, /* Immediate */
    {0x65, 2}, /* Zero Page */
    {0x75, 2}, /* Zero Page,X */
    {0x6d, 3}, /* Absolute */
    {0x7d, 3}, /* Absolute,X */
    {0x79, 3}, /* Absolute,Y */
    {0x61, 2}, /* Indirect,X */
    {0x71, 2}, /* Indirect,Y */
    /* and */
    {0x29, 2}, /* Immediate */
    {0x25, 2}, /* Zero Page */
    {0x35, 2}, /* Zero Page,X */
    {0x2d, 3}, /* Absolute */
    {0x3d, 3}, /* Absolute,X */
    {0x39, 3}, /* Absolute,Y */
    {0x21, 2}, /* Indirect,X */
    {0x31, 2}, /* Indirect,Y */
    /* asl */
    {0x0a, 1}, /* Accumulator */
    {0x06, 2}, /* Zero Page */
    {0x16, 2}, /* Zero Page,X */
    {0x0e, 3}, /* Absolute */
    {0x1e, 3}, /* Absolute,X */
    /* bit(test bits) */
    {0x24, 2}, /* Zero Page */
    {0x2c, 3}, /* Absolute */
    /* branch */
    {0x10, 2}, /* BPL (Branch on PLus) */
    {0x30, 2}, /* BMI (Branch on MInus) */
    {0x50, 2}, /* BVC (Branch on oVerflow Clear) */
    {0x70, 2}, /* BVS (Branch on oVerflow Set) */
    {0x90, 2}, /* BCC (Branch on Carry Clear) */
    {0xb0, 2}, /* BCS (Branch on Carry Set) */
    {0xd0, 2}, /* BNE (Branch on Not Equal) */
    {0xf0, 2}, /* BEQ (Branch on EQual) */
    /* break */
    {0x00, 1}, /* Implied */
    /* cmp (compare accumulator) */
    {0xc9, 2}, /* Immediate */
    {0xc5, 2}, /* Zero Page */
    {0xd5, 2}, /* Zero Page,X */
    {0xcd, 3}, /* Absolute */
    {0xdd, 3}, /* Absolute,X */
    {0xd9, 3}, /* Absolute,Y */
    {0xc1, 2}, /* Indirect,X */
    {0xd1, 2}, /* Indirect,Y */
    /* cpx (compare x register) */
    {0xe0, 2}, /* Immediate */
    {0xe4, 2}, /* Zero Page */
    {0xec, 3}, /* Absolute */
    /* cpy (compare y register) */
    {0xc0, 2}, /* Immediate */
    {0xc4, 2}, /* Zero Page */
    {0xcc, 3}, /* Absolute */
    /* dec (decrement memory) */
    {0xc6, 2}, /* Zero Page */
    {0xd6, 2}, /* Zero Page,X */
    {0xce, 3}, /* Absolute,X */
    {0xde, 3}, /* Absolute,Y */
    /* eor (bitwise exclusive or) */
    {0x49, 2}, /* Immediate */
    {0x45, 2}, /* Zero Page */
    {0x55, 2}, /* Zero Page,X */
    {0x4d, 3}, /* Absolute */
    {0x5d, 3}, /* Absolute,X */
    {0x59, 3}, /* Absolute,Y */
    {0x41, 2}, /* Indirect,X */
    {0x51, 2}, /* Indirect,Y */
    /* flag (process status) */
    {0x18, 1}, /* CLC (CLear Carry) */
    {0x38, 1}, /* SEC (SEt Carry) */
    {0x58, 1}, /* CLI (CLear Interrupt) */
    {0x78, 1}, /* SEI (SEt Interrupt) */
    {0xb8, 1}, /* CLV (CLear oVerflow) */
    {0xd8, 1}, /* CLD (CLear Decimal) */
    {0xf8, 1}, /* SED (SEt Decimal) */
    /* inc (incremental memory) */
    {0xe6, 2}, /* Zero Page */
    {0xf6, 2}, /* Zero Page,X */
    {0xee, 3}, /* Absolute */
    {0xfe, 3}, /* Absolute,X */
    /* jmp */
    {0x4c, 3}, /* Absolute */
    {0x6c, 3}, /* Indirect */
    /* jsr (jump to sub-routin) */
    {0x20, 3}, /* Absolute */
    /* lda (load accumulator) */
    {0xa9, 2}, /* Immediate */
    {0xa5, 2}, /* Zero Page */
    {0xb5, 2}, /* Zero Page,X */
    {0xad, 3}, /* Absolute */
    {0xbd, 3}, /* Absolute,X */
    {0xb9, 3}, /* Absolute,Y */
    {0xa1, 2}, /* Indirect,X */
    {0xb1, 2}, /* Indirect,Y */
    /* ldx (load x register) */
    {0xa2, 2}, /* Immediate */
    {0xa6, 2}, /* Zero Page */
    {0xb6, 2}, /* Zero Page,Y */
    {0xae, 3}, /* Absolute */
    {0xbe, 3}, /* Absolute,Y */
    /* ldy (load y register) */
    {0xa0, 2}, /* Immediate */
    {0xa4, 2}, /* Zero Page */
    {0xb4, 2}, /* Zero Page,Y */
    {0xac, 3}, /* Absolute */
    {0xbc, 3}, /* Absolute,Y */
    /* lsr (logical shift right) */
    {0x4a, 1}, /* Accumulator */
    {0x46, 2}, /* Zero Page */
    {0x56, 2}, /* Zero Page,X */
    {0x4e, 3}, /* Absolute */
    {0x5e, 3}, /* Absolute,X */
    /* ora (bitwise or with accumulator) */
    {0x09, 2}, /* Immediate */
    {0x05, 2}, /* Zero Page */
    {0x15, 2}, /* Zero Page,X */
    {0x0d, 3}, /* Absolute */
    {0x1d, 3}, /* Absolute,X */
    {0x19, 3}, /* Absolute,Y */
    {0x01, 2}, /* Indirect,X */
    {0x11, 2}, /* Indirect,Y */
    /* register instruction */
    {0xaa, 1}, /* TAX (Transfer A to X) */
    {0x8a, 1}, /* TXA (Transfer X to A) */
    {0xca, 1}, /* DEX (DEcrement X) */
    {0xe8, 1}, /* INX (INcrement X) */
    {0xa8, 1}, /* TAY (Transfer A to Y) */
    {0x98, 1}, /* TYA (Transfer Y to A) */
    {0x88, 1}, /* DEY (DEcrement Y) */
    {0xc8, 1}, /* INY (INcrement Y) */
    /* rol (rotate left) */
    {0x2a, 1}, /* Accumulator */
    {0x26, 2}, /* Zero Page */
    {0x36, 2}, /* Zero Page,X */
    {0x2e, 3}, /* Absolute */
    {0x3e, 3}, /* Absolute,X */
    /* ror (rotate right) */
    {0x6a, 1}, /* Accumulator */
    {0x66, 2}, /* Zero Page */
    {0x76, 2}, /* Zero Page,X */
    {0x6e, 3}, /* Absolute */
    {0x7e, 3}, /* Absolute,X */
    /* rti (return from interrupt) */
    {0x40, 1}, /* Implied */
    /* rts (return from sub-routin) */
    {0x60, 1}, /* Implied */
    /* sbc (substract with carry) */
    {0xe9, 2}, /* Immediate */
    {0xe5, 2}, /* Zero Page */
    {0xf5, 2}, /* Zero Page,X */
    {0xed, 3}, /* Absolute */
    {0xfd, 3}, /* Absolute,X */
    {0xf9, 3}, /* Absolute,Y */
    {0xe1, 2}, /* Indirect,X */
    {0xf1, 2}, /* Indirect,Y */
    /* sta (store accumulator) */
    {0x85, 2}, /* Zero Page */
    {0x95, 2}, /* Zero Page,X */
    {0x8d, 3}, /* Absolute */
    {0x9d, 3}, /* Absolute,X */
    {0x99, 3}, /* Absolute,Y */
    {0x81, 2}, /* Indirect,X */
    {0x91, 2}, /* Indirect,Y */
    /* stack instruction */
    {0x9a, 1}, /* TXS (Transfer X to Stack ptr) */
    {0xba, 1}, /* TSX (Transfer Stack ptr to X) */
    {0x48, 1}, /* PHA (PusH Accumulator) */
    {0x68, 1}, /* PLA (PuLl Accumulator) */
    {0x08, 1}, /* PHP (PusH Processor status) */
    {0x28, 1}, /* PLP (PuLl Processor status) */
    /* sta (store x register) */
    {0x86, 2}, /* Zero Page */
    {0x96, 2}, /* Zero Page,Y */
    {0x8e, 3}, /* Absolute */
    /* sty (store y register) */
    {0x84, 2}, /* Zero Page */
    {0x94, 2}, /* Zero Page,Y */
    {0x8c, 3}, /* Absolute */
};

static struct cpu cpu6502;

extern bool cpu_mem_read(uint16_t addr, uint8_t *data, uint16_t len);
extern bool cpu_mem_write(uint16_t addr, uint8_t *data, uint16_t len);

void cpu_dump_regs(void)
{
    printf("   $pc: 0x%04x\n", cpu6502.regs.pc);
    printf("  $acc: 0x%02x\n", cpu6502.regs.acc);
    printf("$idx_x: 0x%02x\n", cpu6502.regs.idx_x);
    printf("$idx_y: 0x%02x\n", cpu6502.regs.idx_y);
    printf("   $sp: 0x%02x\n", cpu6502.regs.sp);
    printf("   $sr: 0x%02x\n", cpu6502.regs.sr);
    printf("        carry: 1'b%d\n", STATUS_REG_CARRY);
    printf("         zero: 1'b%d\n", STATUS_REG_ZERO);
    printf("    interrupt: 1'b%d\n", STATUS_REG_INTERRUPT);
    printf("      decimal: 1'b%d\n", STATUS_REG_DECIMAL);
    printf("     overflow: 1'b%d\n", STATUS_REG_OVERFLOW);
    printf("     negative: 1'b%d\n", STATUS_REG_NEGATIVE);
}

void cpu_reset(void)
{
    uint8_t data;

    /* set pc to reset vector */
    cpu_mem_read(0xfffd, &data, 1);
    cpu6502.regs.pc = (data<<8)&0xff00;
    cpu_mem_read(0xfffc, &data, 1);
    cpu6502.regs.pc |= (data&0x00ff);

    /* set registers to default value */
    cpu6502.regs.acc = 0x00;
    cpu6502.regs.idx_x = 0x00;
    cpu6502.regs.idx_y = 0x00;
    cpu6502.regs.sr = 0x34;
    cpu6502.regs.sp = 0xfd;
}

static struct instruction *_find_instruction(uint8_t op_code)
{
    uint8_t index;

    for(index=0; index<sizeof(isa)/sizeof(struct instruction); index++)
    {
        if(isa[index].code == op_code)
        {
            return &isa[index];
        }
    }
    
    return NULL;
}

static bool _handle_interrupt(void)
{
    return true;
}

static bool instr_fetch(uint8_t *op_code, uint8_t *data, uint8_t *num_data)
{
    uint16_t pc;
    struct instruction *instr;

    pc = cpu6502.regs.pc;
    cpu_mem_read(pc, op_code, 1);
    instr = _find_instruction(*op_code);
    if(!instr)
    {
        /* error handle for illigle instruction */
        printf("can not find op code [0x%02x] on pc [0x%04x]\n", *op_code, pc);
        return false;
    }

    *num_data = instr->num_op-1;
    if(*num_data)
    {
        cpu_mem_read(pc+1, data, *num_data);
    }

    /* update pc */
    cpu6502.regs.pc+=(instr->num_op);

    return true;
}

static bool instr_adc(uint8_t op_code, uint8_t *data, uint8_t num_data)
{
    if(STATUS_REG_DECIMAL) /* do BCD addition */
    {
    
    }
    else
    {
    
    }

    return true;
}

static bool instr_exec(uint8_t op_code, uint8_t *data, uint8_t num_data)
{
    bool ret=true;

    switch(op_code)
    {
        /* Jump Instructions */
        case 0x20: /* JSR (Jump to SubRoutine) */
            {
                uint8_t value[2];

                value[0] = (cpu6502.regs.pc+2)&0x00ff;
                value[1] = ((cpu6502.regs.pc+2)&0xff00)>>8;
                cpu_mem_write(cpu6502.regs.sp+0x100, value, 2);
                cpu6502.regs.sp -= 2;
            }
        case 0x4c: /* JMP (JuMP) Absolute */
            cpu6502.regs.pc = (data[1]<<8 | data[0]);
            break;
        case 0x6c: /* JMP (JuMP) Indirect */
            {
                uint8_t value[2];

                cpu_mem_read(((data[1]<<8) |  data[0]), value, 2);
                cpu6502.regs.pc = ((value[1]<<8) | value[0]);
            }
            break;
        case 0x60:
            {
                uint8_t value[2];

                cpu_mem_read(cpu6502.regs.sp+0x100+2, value, 2);
                cpu6502.regs.sp += 2;
                cpu6502.regs.pc = ((value[1]<<8) | value[0])+1;
            }
            break;
        /* ADC */
        case 0x69:
        case 0x65:
        case 0x75:
        case 0x6d:
        case 0x7d:
        case 0x79:
        case 0x61:
        case 0x71:
            ret = instr_adc(op_code, data, num_data);
            break;
        /* Stack Instructions */
        case 0x08: /* PHP (PusH Processor status) */
            cpu_mem_write(cpu6502.regs.sp+0x100, &cpu6502.regs.sr, 1);
            cpu6502.regs.sp--;
            break;
        case 0x28: /* PLP (PuLl Processor status) */
            cpu_mem_read(cpu6502.regs.sp+0x100+1, &cpu6502.regs.sr, 1);
            cpu6502.regs.sp++;
            break;
        case 0x48: /* PHA (PusH Accumulator) */
            cpu_mem_write(cpu6502.regs.sp+0x100, &cpu6502.regs.acc, 1);
            cpu6502.regs.sp--;
            break;
        case 0x68: /* PLA (PuLl Accumulator) */
            cpu_mem_read(cpu6502.regs.sp+0x100+1, &cpu6502.regs.acc, 1);
            cpu6502.regs.sp++;

            if(cpu6502.regs.acc == 0)
                STATUS_REG_SET_ZERO();
            else
                STATUS_REG_CLR_ZERO();

            if(cpu6502.regs.acc & 0x80)
                STATUS_REG_SET_NEGATIVE();
            else
                STATUS_REG_CLR_NEGATIVE();

            break;
        case 0x9A: /* TXS (Transfer X to Stack ptr) */
            cpu6502.regs.sp = cpu6502.regs.idx_x;
            break;
        case 0xBA: /* TSX (Transfer Stack ptr to X) */
            cpu6502.regs.idx_x = cpu6502.regs.sp;

            if(cpu6502.regs.idx_x == 0)
                STATUS_REG_SET_ZERO();
            else
                STATUS_REG_CLR_ZERO();

            if(cpu6502.regs.idx_x & 0x80)
                STATUS_REG_SET_NEGATIVE();
            else
                STATUS_REG_CLR_NEGATIVE();
            break;
        /* Flag (Processor Status) Instructions */
        case 0x18: /* CLC (CLear Carry) */
            STATUS_REG_CLR_CARRY();
            break;
        case 0x38: /* SEC (SEt Carry) */
            STATUS_REG_SET_CARRY();
            break;
        case 0x58: /* CLI (CLear Interrupt)*/
            STATUS_REG_CLR_INTERRUPT();
            break;
        case 0x78: /* SEI (SEt Interrupt) */
            STATUS_REG_SET_INTERRUPT();
            break;
        case 0xB8: /* CLV (CLear oVerflow) */
            STATUS_REG_CLR_OVERFLOW();
            break;
        case 0xD8: /* CLD (CLear Decimal) */
            STATUS_REG_CLR_DECIMAL();
            break;
        case 0xF8: /* SED (SEt Decimal) */
            STATUS_REG_SET_DECIMAL();
            break;
        default:
            //return false;
            break;
    }

    return ret;
}

bool cpu_run(void)
{
    uint8_t op_code;
    uint8_t data[4];
    uint8_t num_data;

    while(1)
    {
        if(STATUS_REG_INTERRUPT)
            _handle_interrupt();

        if(instr_fetch(&op_code, data, &num_data) == false)
            continue;

        if(instr_exec(op_code, data, num_data) == false)
        {
        }
    }

    return true;
}

