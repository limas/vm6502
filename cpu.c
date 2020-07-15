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

#define STATUS_REG_CHK_CARRY(r) \
    do{ \
        if((r) & 0x0100) \
            STATUS_REG_SET_CARRY(); \
        else \
            STATUS_REG_CLR_CARRY(); \
    }while(0)

#define STATUS_REG_CHK_ZERO(r) \
    do{ \
        if((r) == 0) \
            STATUS_REG_SET_ZERO(); \
        else \
            STATUS_REG_CLR_ZERO(); \
    }while(0)

#define STATUS_REG_CHK_OVERFLOW(a, b, r) \
    do{ \
        if((((a) & 0x80) == ((b) & 0x80)) && \
           (((a) & 0x80) != ((r) & 0x80))) \
            STATUS_REG_SET_OVERFLOW(); \
        else \
            STATUS_REG_CLR_OVERFLOW(); \
    }while(0)

#define STATUS_REG_CHK_NEGATIVE(r) \
    do{ \
        if((r) & 0x80) \
            STATUS_REG_SET_NEGATIVE(); \
        else \
            STATUS_REG_CLR_NEGATIVE(); \
    }while(0)

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

/*
 * Address Modes:
 *
 * A      ....  Accumulator          OPC A        operand is AC (implied single byte instruction)
 * abs    ....  absolute             OPC $LLHH    operand is address $HHLL *
 * abs,X  ....  absolute, X-indexed  OPC $LLHH,X  operand is address; effective address is address incremented by X with carry **
 * abs,Y  ....  absolute, Y-indexed  OPC $LLHH,Y  operand is address; effective address is address incremented by Y with carry **
 * #      ....  immediate            OPC #$BB     operand is byte BB
 * impl   ....  implied              OPC          operand implied
 * ind    ....  indirect             OPC ($LLHH)  operand is address; effective address is contents of word at address: C.w($HHLL)
 * X,ind  ....  X-indexed, indirect  OPC ($LL,X)  operand is zeropage address; effective address is word in (LL + X, LL + X + 1), inc. without carry: C.w($00LL + X)
 * ind,Y  ....  indirect, Y-indexed  OPC ($LL),Y  operand is zeropage address; effective address is word in (LL, LL + 1) incremented by Y with carry: C.w($00LL) + Y
 * rel    ....  relative             OPC $BB      branch target is PC + signed offset BB ***
 * zpg    ....  zeropage             OPC $LL      operand is zeropage address (hi-byte is zero, address = $00LL)
 * zpg,X  ....  zeropage, X-indexed  OPC $LL,X    operand is zeropage address; effective address is address incremented by X without carry **
 * zpg,Y  ....  zeropage, Y-indexed  OPC $LL,Y    operand is zeropage address; effective address is address incremented by Y without carry **
 */

enum addr_mode
{
    AM_ACC = 0, /* accumulator */
    AM_ABS, /* absolute */
    AM_ABS_X, /* absolute, X-indexed */
    AM_ABS_Y, /* absolute, Y-indexed */
    AM_IMM, /* immediate */
    AM_IMPL, /* implied */
    AM_IND, /* indirect */
    AM_X_IND, /* X-indexed, indirect */
    AM_IND_Y, /* indirect, Y-indexed */
    AM_REL, /* relative */
    AM_ZPG, /* zeropage */
    AM_ZPG_X, /* zeropage, X-indexed */
    AM_ZPG_Y, /* zeropage, Y-indexed */
};

struct instruction
{
    uint8_t code;
    uint8_t num_op;
    enum addr_mode addr_mode;
};

struct instruction isa[]=
{
    /* nop */
    {0xea, 1, AM_IMPL},
    /* adc */
    {0x69, 2, AM_IMM},
    {0x65, 2, AM_ZPG},
    {0x75, 2, AM_ZPG_X},
    {0x6d, 3, AM_ABS},
    {0x7d, 3, AM_ABS_X},
    {0x79, 3, AM_ABS_Y},
    {0x61, 2, AM_X_IND},
    {0x71, 2, AM_IND_Y},
    /* and */
    {0x29, 2, AM_IMM},
    {0x25, 2, AM_ZPG},
    {0x35, 2, AM_ZPG_X},
    {0x2d, 3, AM_ABS},
    {0x3d, 3, AM_ABS_X},
    {0x39, 3, AM_ABS_Y},
    {0x21, 2, AM_X_IND},
    {0x31, 2, AM_IND_Y},
    /* asl */
    {0x0a, 1, AM_ACC},
    {0x06, 2, AM_ZPG},
    {0x16, 2, AM_ZPG_X},
    {0x0e, 3, AM_ABS},
    {0x1e, 3, AM_ABS_X},
    /* bit (test bits) */
    {0x24, 2, AM_ZPG},
    {0x2c, 3, AM_ABS},
    /* branch */
    {0x10, 2, AM_REL}, /* BPL (Branch on PLus) */
    {0x30, 2, AM_REL}, /* BMI (Branch on MInus) */
    {0x50, 2, AM_REL}, /* BVC (Branch on oVerflow Clear) */
    {0x70, 2, AM_REL}, /* BVS (Branch on oVerflow Set) */
    {0x90, 2, AM_REL}, /* BCC (Branch on Carry Clear) */
    {0xb0, 2, AM_REL}, /* BCS (Branch on Carry Set) */
    {0xd0, 2, AM_REL}, /* BNE (Branch on Not Equal) */
    {0xf0, 2, AM_REL}, /* BEQ (Branch on EQual) */
    /* brk */
    {0x00, 1, AM_IMPL},
    /* cmp (compare accumulator) */
    {0xc9, 2, AM_IMM},
    {0xc5, 2, AM_ZPG},
    {0xd5, 2, AM_ZPG_X},
    {0xcd, 3, AM_ABS},
    {0xdd, 3, AM_ABS_X},
    {0xd9, 3, AM_ABS_Y},
    {0xc1, 2, AM_X_IND},
    {0xd1, 2, AM_IND_Y},
    /* cpx (compare x register) */
    {0xe0, 2, AM_IMM},
    {0xe4, 2, AM_ZPG},
    {0xec, 3, AM_ABS},
    /* cpy (compare y register) */
    {0xc0, 2, AM_IMM},
    {0xc4, 2, AM_ZPG},
    {0xcc, 3, AM_ABS},
    /* dec (decrement memory) */
    {0xc6, 2, AM_ZPG},
    {0xd6, 2, AM_ZPG_X},
    {0xce, 3, AM_ABS},
    {0xde, 3, AM_ABS_X},
    /* eor (bitwise exclusive or) */
    {0x49, 2, AM_IMM},
    {0x45, 2, AM_ZPG},
    {0x55, 2, AM_ZPG_X},
    {0x4d, 3, AM_ABS},
    {0x5d, 3, AM_ABS_X},
    {0x59, 3, AM_ABS_Y},
    {0x41, 2, AM_X_IND},
    {0x51, 2, AM_IND_Y},
    /* flag (process status) */
    {0x18, 1, AM_IMPL}, /* CLC (CLear Carry) */
    {0x38, 1, AM_IMPL}, /* SEC (SEt Carry) */
    {0x58, 1, AM_IMPL}, /* CLI (CLear Interrupt) */
    {0x78, 1, AM_IMPL}, /* SEI (SEt Interrupt) */
    {0xb8, 1, AM_IMPL}, /* CLV (CLear oVerflow) */
    {0xd8, 1, AM_IMPL}, /* CLD (CLear Decimal) */
    {0xf8, 1, AM_IMPL}, /* SED (SEt Decimal) */
    /* inc (incremental memory) */
    {0xe6, 2, AM_ZPG},
    {0xf6, 2, AM_ZPG_X},
    {0xee, 3, AM_ABS},
    {0xfe, 3, AM_ABS_X},
    /* jmp */
    {0x4c, 3, AM_ABS},
    {0x6c, 3, AM_IND},
    /* jsr (jump to sub-routin) */
    {0x20, 3, AM_ABS},
    /* lda (load accumulator) */
    {0xa9, 2, AM_IMM},
    {0xa5, 2, AM_ZPG},
    {0xb5, 2, AM_ZPG_X},
    {0xad, 3, AM_ABS},
    {0xbd, 3, AM_ABS_X},
    {0xb9, 3, AM_ABS_Y},
    {0xa1, 2, AM_X_IND},
    {0xb1, 2, AM_IND_Y},
    /* ldx (load x register) */
    {0xa2, 2, AM_IMM},
    {0xa6, 2, AM_ZPG},
    {0xb6, 2, AM_ZPG_Y},
    {0xae, 3, AM_ABS},
    {0xbe, 3, AM_ABS_Y},
    /* ldy (load y register) */
    {0xa0, 2, AM_IMM},
    {0xa4, 2, AM_ZPG},
    {0xb4, 2, AM_ZPG_X},
    {0xac, 3, AM_ABS},
    {0xbc, 3, AM_ABS_X},
    /* lsr (logical shift right) */
    {0x4a, 1, AM_ACC},
    {0x46, 2, AM_ZPG},
    {0x56, 2, AM_ZPG_X},
    {0x4e, 3, AM_ABS},
    {0x5e, 3, AM_ABS_X},
    /* ora (bitwise or with accumulator) */
    {0x09, 2, AM_IMM},
    {0x05, 2, AM_ZPG},
    {0x15, 2, AM_ZPG_X},
    {0x0d, 3, AM_ABS},
    {0x1d, 3, AM_ABS_X},
    {0x19, 3, AM_ABS_Y},
    {0x01, 2, AM_X_IND},
    {0x11, 2, AM_IND_Y},
    /* register instruction */
    {0xaa, 1, AM_IMPL}, /* TAX (Transfer A to X) */
    {0x8a, 1, AM_IMPL}, /* TXA (Transfer X to A) */
    {0xca, 1, AM_IMPL}, /* DEX (DEcrement X) */
    {0xe8, 1, AM_IMPL}, /* INX (INcrement X) */
    {0xa8, 1, AM_IMPL}, /* TAY (Transfer A to Y) */
    {0x98, 1, AM_IMPL}, /* TYA (Transfer Y to A) */
    {0x88, 1, AM_IMPL}, /* DEY (DEcrement Y) */
    {0xc8, 1, AM_IMPL}, /* INY (INcrement Y) */
    /* rol (rotate left) */
    {0x2a, 1, AM_ACC},
    {0x26, 2, AM_ZPG},
    {0x36, 2, AM_ZPG_X},
    {0x2e, 3, AM_ABS},
    {0x3e, 3, AM_ABS_X},
    /* ror (rotate right) */
    {0x6a, 1, AM_ACC},
    {0x66, 2, AM_ZPG},
    {0x76, 2, AM_ZPG_X},
    {0x6e, 3, AM_ABS},
    {0x7e, 3, AM_ABS_X},
    /* rti (return from interrupt) */
    {0x40, 1, AM_IMPL},
    /* rts (return from sub-routin) */
    {0x60, 1, AM_IMPL},
    /* sbc (substract with carry) */
    {0xe9, 2, AM_IMM},
    {0xe5, 2, AM_ZPG},
    {0xf5, 2, AM_ZPG_X},
    {0xed, 3, AM_ABS},
    {0xfd, 3, AM_ABS_X},
    {0xf9, 3, AM_ABS_Y},
    {0xe1, 2, AM_X_IND},
    {0xf1, 2, AM_IND_Y},
    /* sta (store accumulator) */
    {0x85, 2, AM_ZPG},
    {0x95, 2, AM_ZPG_X},
    {0x8d, 3, AM_ABS},
    {0x9d, 3, AM_ABS_X},
    {0x99, 3, AM_ABS_Y},
    {0x81, 2, AM_X_IND},
    {0x91, 2, AM_IND_Y},
    /* stack instruction */
    {0x9a, 1, AM_IMPL}, /* TXS (Transfer X to Stack ptr) */
    {0xba, 1, AM_IMPL}, /* TSX (Transfer Stack ptr to X) */
    {0x48, 1, AM_IMPL}, /* PHA (PusH Accumulator) */
    {0x68, 1, AM_IMPL}, /* PLA (PuLl Accumulator) */
    {0x08, 1, AM_IMPL}, /* PHP (PusH Processor status) */
    {0x28, 1, AM_IMPL}, /* PLP (PuLl Processor status) */
    /* stx (store x register) */
    {0x86, 2, AM_ZPG},
    {0x96, 2, AM_ZPG_Y},
    {0x8e, 3, AM_ABS},
    /* sty (store y register) */
    {0x84, 2, AM_ZPG},
    {0x94, 2, AM_ZPG_X},
    {0x8c, 3, AM_ABS},
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

static void cpu_halt(void)
{
    printf("[CPU] cpu halt\n");
    while(1) ;
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

static struct instruction *instr_fetch(uint8_t *op_code, uint8_t *data, uint8_t *num_data)
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
        return NULL;
    }

    *num_data = instr->num_op-1;
    if(*num_data)
    {
        cpu_mem_read(pc+1, data, *num_data);
    }

    return instr;
}

static void cpu_stack_push(uint8_t val)
{
    cpu_mem_write(cpu6502.regs.sp + 0x100, &val, 1);
    cpu6502.regs.sp--;
}

static uint8_t cpu_stack_pop(void)
{
    uint8_t val;

    cpu6502.regs.sp++;
    cpu_mem_read(cpu6502.regs.sp + 0x100, &val, 1);

    return val;
}

static uint16_t cal_addr(enum addr_mode addr_mode, uint8_t *data)
{
    uint16_t addr = 0;

    switch(addr_mode)
    {
        case AM_ABS:
            addr = data[0] + (data[1] << 8);
            break;
        case AM_ABS_X:
            addr = data[0] + (data[1] << 8);
            addr += cpu6502.regs.idx_x;
            break;
        case AM_ABS_Y:
            addr = data[0] + (data[1] << 8);
            addr += cpu6502.regs.idx_y;
            break;
        case AM_IND:
            {
                uint8_t l_data[2];
                addr = data[0] + (data[1] << 8);
                cpu_mem_read(addr, l_data, 2);
                addr = l_data[0] + (l_data[1] << 8);
            }
            break;
        case AM_X_IND:
            {
                uint8_t l_data[2];
                addr = (data[0] + cpu6502.regs.idx_x) & 0x00ff;
                cpu_mem_read(addr, l_data, 1);
                addr = (data[0] + cpu6502.regs.idx_x + 1) & 0x00ff;
                cpu_mem_read(addr, &l_data[1], 1);
                addr = l_data[0] + (l_data[1] << 8);
            }
            break;
        case AM_IND_Y:
            {
                uint8_t l_data[2];
                addr = data[0];
                cpu_mem_read(addr, l_data, 1);
                addr = (data[0] + 1) & 0x00ff;
                cpu_mem_read(addr, &l_data[1], 1);
                addr = l_data[0] + (l_data[1] << 8);
                addr += cpu6502.regs.idx_y;
            }
            break;
        case AM_REL:
            {
                int8_t l_data = *((int8_t *)data);
                addr = cpu6502.regs.pc + 2 + l_data;
            }
            break;
        case AM_ZPG:
            addr = data[0];
            break;
        case AM_ZPG_X:
            addr = (data[0] + cpu6502.regs.idx_x) & 0x00ff;
            break;
        case AM_ZPG_Y:
            addr = (data[0] + cpu6502.regs.idx_y) & 0x00ff;
            break;
        default:
            printf("invalid address mode\n");
            cpu_halt();
    }

    return addr;
}

static uint8_t cpu_addr_mode_read(enum addr_mode addr_mode, uint8_t *data)
{
    uint8_t ret;
    uint16_t addr;

    if (addr_mode == AM_IMM)
    {
        ret = data[0];
    }
    else if(addr_mode == AM_ACC)
    {
        ret = cpu6502.regs.acc;
    }
    else
    {
        addr = cal_addr(addr_mode, data);
        cpu_mem_read(addr, &ret, 1);
    }

    return ret;
}

static void cpu_addr_mode_write(enum addr_mode addr_mode, uint8_t *data, uint8_t val)
{
    uint16_t addr;

    if(addr_mode == AM_ACC)
    {
        cpu6502.regs.acc = val;
    }
    else
    {
        addr = cal_addr(addr_mode, data);
        cpu_mem_write(addr, &val, 1);
    }
}

static uint16_t instr_exec(struct instruction *instr, uint8_t *data, uint8_t num_data)
{
    uint16_t next_pc = cpu6502.regs.pc + instr->num_op;
    uint8_t op_code = instr->code;

    switch(op_code)
    {
        /* nop */
        case 0xea:
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
            {
                uint8_t addend;
                uint16_t result;

                addend = cpu_addr_mode_read(instr->addr_mode, data);
                result = cpu6502.regs.acc + addend + STATUS_REG_CARRY;

                STATUS_REG_CHK_CARRY(result);
                STATUS_REG_CHK_ZERO(result);
                STATUS_REG_CHK_OVERFLOW(cpu6502.regs.acc, addend, result);
                STATUS_REG_CHK_NEGATIVE(result);

                cpu6502.regs.acc = (uint8_t)result;
                break;
            }
        /* AND */
        case 0x29:
        case 0x25:
        case 0x35:
        case 0x2d:
        case 0x3d:
        case 0x39:
        case 0x21:
        case 0x31:
            cpu6502.regs.acc &= cpu_addr_mode_read(instr->addr_mode, data);

            STATUS_REG_CHK_ZERO(cpu6502.regs.acc);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.acc);
            break;
        /* ASL */
        case 0x0a:
        case 0x06:
        case 0x16:
        case 0x0e:
        case 0x1e:
            {
                uint16_t result;

                result = cpu_addr_mode_read(instr->addr_mode, data);
                result <<= 1;
                cpu_addr_mode_write(instr->addr_mode, data, (uint8_t)result);

                STATUS_REG_CHK_CARRY(result);
                STATUS_REG_CHK_ZERO(result);
                STATUS_REG_CHK_NEGATIVE(result);
                break;
            }
        /* BIT */
        case 0x24:
        case 0x2c:
            {
                uint8_t operand;

                operand = cpu_addr_mode_read(instr->addr_mode, data);
                if(operand & 0x80)
                    STATUS_REG_SET_NEGATIVE();
                else
                    STATUS_REG_CLR_NEGATIVE();

                if(operand & 0x40)
                    STATUS_REG_SET_OVERFLOW();
                else
                    STATUS_REG_CLR_OVERFLOW();

                operand &= cpu6502.regs.acc;

                STATUS_REG_CHK_ZERO(operand);
                break;
            }
        /* BPL (Branch on PLus) */
        case 0x10:
            if(!STATUS_REG_NEGATIVE)
                next_pc = cal_addr(instr->addr_mode, data);
            break;
        /* BMI (Branch on MInus) */
        case 0x30:
            if(STATUS_REG_NEGATIVE)
                next_pc = cal_addr(instr->addr_mode, data);
            break;
        /* BVC (Branch on oVerflow Clear) */
        case 0x50:
            if(!STATUS_REG_OVERFLOW)
                next_pc = cal_addr(instr->addr_mode, data);
            break;
        /* BVS (Branch on oVerflow Set) */
        case 0x70:
            if(STATUS_REG_OVERFLOW)
                next_pc = cal_addr(instr->addr_mode, data);
            break;
        /* BCC (Branch on Carry Clear) */
        case 0x90:
            if(!STATUS_REG_CARRY)
                next_pc = cal_addr(instr->addr_mode, data);
            break;
        /* BCS (Branch on Carry Set) */
        case 0xb0:
            if(STATUS_REG_CARRY)
                next_pc = cal_addr(instr->addr_mode, data);
            break;
        /* BNE (Branch on Not Equal) */
        case 0xd0:
            if(!STATUS_REG_ZERO)
                next_pc = cal_addr(instr->addr_mode, data);
            break;
        /* BEQ (Branch on EQual) */
        case 0xf0:
            if(STATUS_REG_ZERO)
                next_pc = cal_addr(instr->addr_mode, data);
            break;
        /* BRK */
        case 0x00:
            {
                uint8_t value[2];

                value[0] = ((cpu6502.regs.pc+2)>>8)&0x00ff;
                value[1] = (cpu6502.regs.pc+2)&0x00ff;
                cpu_stack_push(value[0]);
                cpu_stack_push(value[1]);
                cpu_stack_push(cpu6502.regs.sr);
                break;
            }
        /* CMP */
        case 0xc9:
        case 0xc5:
        case 0xd5:
        case 0xcd:
        case 0xdd:
        case 0xd9:
        case 0xc1:
        case 0xd1:
            {
                uint8_t value;

                value = cpu_addr_mode_read(instr->addr_mode, data);
                if(cpu6502.regs.acc == value)
                {
                    STATUS_REG_SET_CARRY();
                    STATUS_REG_SET_ZERO();
                    STATUS_REG_CLR_NEGATIVE();
                }
                else if(cpu6502.regs.acc < value)
                {
                    STATUS_REG_CLR_CARRY();
                    STATUS_REG_CLR_ZERO();
                    STATUS_REG_SET_NEGATIVE();
                }
                else
                {
                    STATUS_REG_SET_CARRY();
                    STATUS_REG_CLR_ZERO();
                    STATUS_REG_CLR_NEGATIVE();
                }
                break;
            }
        /* CPX */
        case 0xe0:
        case 0xe4:
        case 0xec:
            {
                uint8_t value;

                value = cpu_addr_mode_read(instr->addr_mode, data);
                if(cpu6502.regs.idx_x == value)
                {
                    STATUS_REG_SET_CARRY();
                    STATUS_REG_SET_ZERO();
                    STATUS_REG_CLR_NEGATIVE();
                }
                else if(cpu6502.regs.idx_x < value)
                {
                    STATUS_REG_CLR_CARRY();
                    STATUS_REG_CLR_ZERO();
                    STATUS_REG_SET_NEGATIVE();
                }
                else
                {
                    STATUS_REG_SET_CARRY();
                    STATUS_REG_CLR_ZERO();
                    STATUS_REG_CLR_NEGATIVE();
                }
                break;
            }
        /* CPY */
        case 0xc0:
        case 0xc4:
        case 0xcc:
            {
                uint8_t value;

                value = cpu_addr_mode_read(instr->addr_mode, data);
                if(cpu6502.regs.idx_y == value)
                {
                    STATUS_REG_SET_CARRY();
                    STATUS_REG_SET_ZERO();
                    STATUS_REG_CLR_NEGATIVE();
                }
                else if(cpu6502.regs.idx_y < value)
                {
                    STATUS_REG_CLR_CARRY();
                    STATUS_REG_CLR_ZERO();
                    STATUS_REG_SET_NEGATIVE();
                }
                else
                {
                    STATUS_REG_SET_CARRY();
                    STATUS_REG_CLR_ZERO();
                    STATUS_REG_CLR_NEGATIVE();
                }
                break;
            }
        /* DEC */
        case 0xc6:
        case 0xd6:
        case 0xce:
        case 0xde:
            {
                uint16_t addr;
                uint8_t operand;

                addr = cal_addr(instr->addr_mode, data);
                cpu_mem_read(addr, &operand, 1);
                operand -= 1;
                cpu_mem_write(addr, &operand, 1);

                STATUS_REG_CHK_ZERO(operand);
                STATUS_REG_CHK_NEGATIVE(operand);
                break;
            }
        /* EOR */
        case 0x49:
        case 0x45:
        case 0x55:
        case 0x4d:
        case 0x5d:
        case 0x59:
        case 0x41:
        case 0x51:
            cpu6502.regs.acc ^= cpu_addr_mode_read(instr->addr_mode, data);

            STATUS_REG_CHK_ZERO(cpu6502.regs.acc);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.acc);
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
        case 0xb8: /* CLV (CLear oVerflow) */
            STATUS_REG_CLR_OVERFLOW();
            break;
        case 0xd8: /* CLD (CLear Decimal) */
            STATUS_REG_CLR_DECIMAL();
            break;
        case 0xf8: /* SED (SEt Decimal) */
            STATUS_REG_SET_DECIMAL();
            break;
        /* INC */
        case 0xe6:
        case 0xf6:
        case 0xee:
        case 0xfe:
            {
                uint16_t addr;
                uint8_t operand;

                addr = cal_addr(instr->addr_mode, data);
                cpu_mem_read(addr, &operand, 1);
                operand += 1;
                cpu_mem_write(addr, &operand, 1);

                STATUS_REG_CHK_ZERO(operand);
                STATUS_REG_CHK_NEGATIVE(operand);
                break;
            }
        /* Jump Instructions */
        case 0x20: /* JSR (Jump to SubRoutine) */
            {
                uint8_t value[2];

                value[0] = ((cpu6502.regs.pc+2)&0xff00)>>8;
                value[1] = (cpu6502.regs.pc+2)&0x00ff;
                cpu_stack_push(value[0]);
                cpu_stack_push(value[1]);
            }
        case 0x4c: /* JMP (JuMP) Absolute */
        case 0x6c: /* JMP (JuMP) Indirect */
            next_pc = cal_addr(instr->addr_mode, data);
            break;
        /* LDA */
        case 0xa9: /* Immediate */
        case 0xa5: /* Zero Page */
        case 0xb5: /* Zero Page,X */
        case 0xad: /* Absolute */
        case 0xbd: /* Absolute,X */
        case 0xb9: /* Absolute,Y */
        case 0xa1: /* X-indexed,Indirect */
        case 0xb1: /* Indirect,Y-indexed */
            cpu6502.regs.acc = cpu_addr_mode_read(instr->addr_mode, data);

            STATUS_REG_CHK_ZERO(cpu6502.regs.acc);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.acc);
            break;
        /* LDX */
        case 0xa2: /* Immediate */
        case 0xa6: /* Zero Page */
        case 0xb6: /* Zero Page,Y */
        case 0xae: /* Absolute */
        case 0xbe: /* Absolute,Y */
            cpu6502.regs.idx_x = cpu_addr_mode_read(instr->addr_mode, data);

            STATUS_REG_CHK_ZERO(cpu6502.regs.idx_x);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.idx_x);
            break;
        /* LDY */
        case 0xa0: /* Immediate */
        case 0xa4: /* Zero Page */
        case 0xb4: /* Zero Page,X */
        case 0xac: /* Absolute */
        case 0xbc: /* Absolute,X */
            cpu6502.regs.idx_y = cpu_addr_mode_read(instr->addr_mode, data);

            STATUS_REG_CHK_ZERO(cpu6502.regs.idx_y);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.idx_y);
            break;
        /* LSR */
        case 0x4a:
        case 0x46:
        case 0x56:
        case 0x4e:
        case 0x5e:
            {
                uint8_t result;

                result = cpu_addr_mode_read(instr->addr_mode, data);
                result >>= 1;
                cpu_addr_mode_write(instr->addr_mode, data, result);

                STATUS_REG_CHK_CARRY(result);
                STATUS_REG_CHK_ZERO(result);
                STATUS_REG_CHK_NEGATIVE(result);
                break;
            }
        /* ORA */
        case 0x09:
        case 0x05:
        case 0x15:
        case 0x0d:
        case 0x1d:
        case 0x19:
        case 0x01:
        case 0x11:
            cpu6502.regs.acc |= cpu_addr_mode_read(instr->addr_mode, data);

            STATUS_REG_CHK_ZERO(cpu6502.regs.acc);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.acc);
            break;
        /* register instruction */
        case 0xaa: /* TAX (Transfer A to X) */
            cpu6502.regs.idx_x = cpu6502.regs.acc;

            STATUS_REG_CHK_ZERO(cpu6502.regs.idx_x);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.idx_x);
            break;
        case 0x8a: /* TXA (Transfer X to A) */
            cpu6502.regs.acc = cpu6502.regs.idx_x;

            STATUS_REG_CHK_ZERO(cpu6502.regs.acc);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.acc);
            break;
        case 0xca: /* DEX (DEcrement X) */
            cpu6502.regs.idx_x--;

            STATUS_REG_CHK_ZERO(cpu6502.regs.idx_x);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.idx_x);
            break;
        case 0xe8: /* INX (INcrement X) */
            cpu6502.regs.idx_x++;

            STATUS_REG_CHK_ZERO(cpu6502.regs.idx_x);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.idx_x);
            break;
        case 0xa8: /* TAY (Transfer A to Y) */
            cpu6502.regs.idx_y = cpu6502.regs.acc;

            STATUS_REG_CHK_ZERO(cpu6502.regs.idx_y);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.idx_y);
            break;
        case 0x98: /* TYA (Transfer Y to A) */
            cpu6502.regs.acc = cpu6502.regs.idx_y;

            STATUS_REG_CHK_ZERO(cpu6502.regs.acc);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.acc);
            break;
        case 0x88: /* DEY (DEcrement Y) */
            cpu6502.regs.idx_y--;

            STATUS_REG_CHK_ZERO(cpu6502.regs.idx_y);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.idx_y);
            break;
        case 0xc8: /* INY (INcrement Y) */
            cpu6502.regs.idx_y++;

            STATUS_REG_CHK_ZERO(cpu6502.regs.idx_y);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.idx_y);
            break;
        /* ROL */
        case 0x2a:
        case 0x26:
        case 0x36:
        case 0x2e:
        case 0x3e:
            {
                uint16_t result;

                result = cpu_addr_mode_read(instr->addr_mode, data);
                result <<= 1;
                result |= STATUS_REG_CARRY;
                cpu_addr_mode_write(instr->addr_mode, data, (uint8_t)result);

                STATUS_REG_CHK_CARRY(result);
                STATUS_REG_CHK_ZERO(result);
                STATUS_REG_CHK_NEGATIVE(result);
                break;
            }
        /* ROR */
        case 0x6a:
        case 0x66:
        case 0x76:
        case 0x6e:
        case 0x7e:
            {
                uint8_t result;
                uint8_t carry;

                result = cpu_addr_mode_read(instr->addr_mode, data);
                carry = result & 0x01;
                result >>= 1;
                result |= (STATUS_REG_CARRY?0x80:0);
                cpu_addr_mode_write(instr->addr_mode, data, result);

                if(carry)
                    STATUS_REG_SET_CARRY();
                else
                    STATUS_REG_CLR_CARRY();
                STATUS_REG_CHK_ZERO(result);
                STATUS_REG_CHK_NEGATIVE(result);
                break;
            }
        /* RTI */
        case 0x40:
            {
                uint8_t value[2];

                cpu6502.regs.sr = cpu_stack_pop();
                value[0] = cpu_stack_pop();
                value[1] = cpu_stack_pop();
                next_pc = ((value[0]<<8) | value[1]);
            }
            break;
        /* RTS */
        case 0x60:
            {
                uint8_t value[2];

                value[1] = cpu_stack_pop();
                value[0] = cpu_stack_pop();
                next_pc = ((value[0]<<8) | value[1]) + 1;
            }
            break;
        /* SBC */
        case 0xe9:
        case 0xe5:
        case 0xf5:
        case 0xed:
        case 0xfd:
        case 0xf9:
        case 0xe1:
        case 0xf1:
            {
                uint8_t minuend;
                uint16_t result;

                minuend = cpu_addr_mode_read(instr->addr_mode, data);
                result = cpu6502.regs.acc - minuend - STATUS_REG_CARRY;

                STATUS_REG_CHK_CARRY(result);
                STATUS_REG_CHK_ZERO(result);
                STATUS_REG_CHK_OVERFLOW(cpu6502.regs.acc, minuend ^ 0x80, result);
                STATUS_REG_CHK_NEGATIVE(result);

                cpu6502.regs.acc = (uint8_t)result;
            }
            break;
        /* STA */
        case 0x85:
        case 0x95:
        case 0x8d:
        case 0x9d:
        case 0x99:
        case 0x81:
        case 0x91:
            cpu_addr_mode_write(instr->addr_mode, data, cpu6502.regs.acc);
            break;
        /* Stack Instructions */
        case 0x08: /* PHP (PusH Processor status) */
            cpu_stack_push(cpu6502.regs.sr);
            break;
        case 0x28: /* PLP (PuLl Processor status) */
            cpu6502.regs.sr = cpu_stack_pop();
            break;
        case 0x48: /* PHA (PusH Accumulator) */
            cpu_stack_push(cpu6502.regs.acc);
            break;
        case 0x68: /* PLA (PuLl Accumulator) */
            cpu6502.regs.acc = cpu_stack_pop();

            STATUS_REG_CHK_ZERO(cpu6502.regs.acc);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.acc);
            break;
        case 0x9A: /* TXS (Transfer X to Stack ptr) */
            cpu6502.regs.sp = cpu6502.regs.idx_x;
            break;
        case 0xBA: /* TSX (Transfer Stack ptr to X) */
            cpu6502.regs.idx_x = cpu6502.regs.sp;

            STATUS_REG_CHK_ZERO(cpu6502.regs.idx_x);
            STATUS_REG_CHK_NEGATIVE(cpu6502.regs.idx_x);
            break;
        /* STX */
        case 0x86:
        case 0x96:
        case 0x8e:
            cpu_addr_mode_write(instr->addr_mode, data, cpu6502.regs.idx_x);
            break;
        /* STY */
        case 0x84:
        case 0x94:
        case 0x8c:
            cpu_addr_mode_write(instr->addr_mode, data, cpu6502.regs.idx_y);
            break;
        default:
            fprintf(stderr, "[CPU] unhandle instruction [0x%02x]\n", op_code);
            cpu_halt();
    }

    return next_pc;
}

bool cpu_run(void)
{
    uint8_t op_code;
    uint8_t data[4];
    uint8_t num_data;
    struct instruction *instr;

    if(STATUS_REG_INTERRUPT)
        _handle_interrupt();

    if((instr = instr_fetch(&op_code, data, &num_data)) == NULL)
        return false;

    cpu6502.regs.pc = instr_exec(instr, data, num_data);

    return true;
}

