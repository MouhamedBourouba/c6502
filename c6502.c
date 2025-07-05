#include <stdbool.h>
#include <stdint.h>

static const uint32_t ticktable[256];
static void (*optable[256])();
static const uint32_t ticktable[256];

#define COMBINE_BYTES(high, low) (high << 8 | low);

#define RESET_VECTOR_LOW 0xFFFA
#define STACK_PAGE 0x0100

// Forward decleration
// The user of the libary the resposible for implementing those functions
uint8_t read6502(uint16_t address);
void write6502(uint16_t address, uint8_t value);

static struct {
  uint8_t stack_ptr;

  uint16_t program_counter;

  uint8_t accumulator, x, y;

  union {
    struct {
      bool carry : 1;
      bool zero : 1;
      bool interrupt_disable : 1;
      bool decimal_mode : 1;
      bool breake : 1;
      bool unused : 1;
      bool overflow : 1;
      bool negative : 1;
    };
    uint8_t processor_status;
  };

  uint8_t cycles;
  uint8_t opcode;

  uint16_t oprand_address;
  uint16_t relative_address;

  bool addr_plus_cycle, opcode_plus_cycle;

  unsigned num_of_instr;
} state;

void c6502_reset() {
  state.processor_status = 0;

  state.stack_ptr = 0xFD;

  state.x = state.y = state.accumulator = 0;
  state.cycles = state.oprand_address = 0;

  uint8_t reset_vector_lo = read6502(RESET_VECTOR_LOW);
  uint8_t reset_vector_hi = read6502(RESET_VECTOR_LOW + 1);

  state.processor_status = COMBINE_BYTES(reset_vector_hi, reset_vector_lo);

  state.cycles = 8;
}

bool c6502_getCarry() { return state.carry; }
bool c6502_getZero() { return state.zero; }
bool c6502_getInterruptDisable() { return state.interrupt_disable; }
bool c6502_getDecimalMode() { return state.decimal_mode; }
bool c6502_getBreake() { return state.breake; }
bool c6502_getOverflow() { return state.overflow; }
bool c6502_getNegative() { return state.negative; }

void c6502_tick() {
  // remember to unset the plus cytle in the state and clear the state of any
  // cycle spicifc data
}

inline static uint16_t read_pc() { return state.program_counter++; }

// Addression modes
static void imp() { return; }
static void acc() { return; }

static void imm() {
  state.oprand_address = read_pc();
  return;
}

static void zp() {
  state.oprand_address = (uint16_t)read6502(read_pc());
  return;
}

static void zpx() {
  state.oprand_address = (uint16_t)(read6502(read_pc()) + state.x) & 0x00FF;
  return;
}

static void zpy() {
  state.oprand_address = (uint16_t)(read6502(read_pc()) + state.y) & 0x00FF;
  return;
}

static void rel() {
  state.relative_address = read_pc();
  if (state.relative_address & 0x80) {
    state.relative_address |= 0xFF00;
  }
  return;
}

static void abso() {
  uint8_t lo = read_pc();
  uint8_t hi = read_pc();

  state.oprand_address = COMBINE_BYTES(hi, lo);
  return;
}

static void absx() {
  uint8_t lo = read_pc();
  uint8_t hi = read_pc();

  state.oprand_address = COMBINE_BYTES(hi, lo);
  state.oprand_address += state.x;

  if (state.oprand_address >> 8 != hi) {
    state.addr_plus_cycle = true;
  }
  return;
}

static void absy() {
  uint8_t lo = read_pc();
  uint8_t hi = read_pc();

  state.oprand_address = COMBINE_BYTES(hi, lo);
  state.oprand_address += state.y;

  if (state.oprand_address >> 8 != hi) {
    state.addr_plus_cycle = true;
  }
  return;
}

static void ind() {
  uint8_t lo = read_pc();
  uint8_t hi = read_pc();

  uint16_t pointer = COMBINE_BYTES(lo, hi);
  uint8_t target_lo = read6502(pointer);

  uint16_t pointer_next = (pointer & 0xFF00) | ((pointer + 1) & 0x00FF);
  uint8_t target_hi = read6502(pointer_next);

  state.oprand_address = COMBINE_BYTES(target_lo, target_hi);
  return;
}

static void indx() {
  uint16_t base = (read_pc() + state.x) & 0x00FF;

  uint8_t target_lo = read6502(base);

  uint16_t pointer_next = ((base + 1) & 0x00FF);
  uint8_t target_hi = read6502(pointer_next);

  state.oprand_address = COMBINE_BYTES(target_lo, target_hi);
  return;
}

static void indy() {
  uint16_t base = read_pc() & 0x00FF;

  uint8_t target_lo = read6502(base);
  uint8_t target_hi = read6502((base + 1) & 0x00FF);

  uint16_t target = COMBINE_BYTES(target_lo, target_hi);

  state.oprand_address = target + state.y;

  if ((state.oprand_address & 0xFF00) != (target & 0xFF00)) {
    state.addr_panalty_cycle = true;
  }
  return;
}

static uint16_t getvalue() {
  if (addrtable[state.opcode] == acc)
    return ((uint16_t)state.accumulator);
  else
    return ((uint16_t)read6502(state.oprand_address));
}

static uint16_t getvalue16() {
  return COMBINE_BYTES(read6502(state.oprand_address),
                       read6502(state.oprand_address + 1));
}

static void putvalue(uint16_t value) {
  if (addrtable[state.opcode] == acc)
    state.accumulator = (uint8_t)(value & 0x00FF);
  else
    write6502(state.oprand_address, (value & 0x00FF));
}

// Instructions
static void adc() {}

static void and () {}

static void asl() {}

static void bcc() {}

static void bcs() {}

static void beq() {}

static void bit() {}

static void bmi() {}

static void bne() {}

static void bpl() {}

static void brk() {}

static void bvc() {}

static void bvs() {}

static void clc() {}

static void cld() {}

static void cli() {}

static void clv() {}

static void cmp() {}

static void cpx() {}

static void cpy() {}

static void dcp() {}

static void dec() {}

static void dex() {}

static void dey() {}

static void eor() {}

static void inc() {}

static void inx() {}

static void iny() {}

static void isb() {}

static void jmp() {}

static void jsr() {}

static void lax() {}

static void lda() {}

static void ldx() {}

static void ldy() {}

static void lsr() {}

static void nop() {}

static void ora() {}

static void pha() {}

static void php() {}

static void pla() {}

static void plp() {}

static void rla() {}

static void rol() {}

static void ror() {}

static void rra() {}

static void rti() {}

static void rts() {}

static void sax() {}

static void sbc() {}

static void sec() {}

static void sed() {}

static void sei() {}

static void slo() {}

static void sre() {}

static void sta() {}

static void stx() {}

static void sty() {}

static void tax() {}

static void tay() {}

static void tsx() {}

static void txa() {}

static void txs() {}

static void tya() {}

// clang-format off
static void (*addrtable[256])() = {
/*        |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  A  |  B  |  C  |  D  |  E  |  F  |     */
/* 0 */     imp, indx,  imp, indx,   zp,   zp,   zp,   zp,  imp,  imm,  acc,  imm, abso, abso, abso, abso, /* 0 */
/* 1 */     rel, indy,  imp, indy,  zpx,  zpx,  zpx,  zpx,  imp, absy,  imp, absy, absx, absx, absx, absx, /* 1 */
/* 2 */    abso, indx,  imp, indx,   zp,   zp,   zp,   zp,  imp,  imm,  acc,  imm, abso, abso, abso, abso, /* 2 */
/* 3 */     rel, indy,  imp, indy,  zpx,  zpx,  zpx,  zpx,  imp, absy,  imp, absy, absx, absx, absx, absx, /* 3 */
/* 4 */     imp, indx,  imp, indx,   zp,   zp,   zp,   zp,  imp,  imm,  acc,  imm, abso, abso, abso, abso, /* 4 */
/* 5 */     rel, indy,  imp, indy,  zpx,  zpx,  zpx,  zpx,  imp, absy,  imp, absy, absx, absx, absx, absx, /* 5 */
/* 6 */     imp, indx,  imp, indx,   zp,   zp,   zp,   zp,  imp,  imm,  acc,  imm,  ind, abso, abso, abso, /* 6 */
/* 7 */     rel, indy,  imp, indy,  zpx,  zpx,  zpx,  zpx,  imp, absy,  imp, absy, absx, absx, absx, absx, /* 7 */
/* 8 */     imm, indx,  imm, indx,   zp,   zp,   zp,   zp,  imp,  imm,  imp,  imm, abso, abso, abso, abso, /* 8 */
/* 9 */     rel, indy,  imp, indy,  zpx,  zpx,  zpy,  zpy,  imp, absy,  imp, absy, absx, absx, absy, absy, /* 9 */
/* A */     imm, indx,  imm, indx,   zp,   zp,   zp,   zp,  imp,  imm,  imp,  imm, abso, abso, abso, abso, /* A */
/* B */     rel, indy,  imp, indy,  zpx,  zpx,  zpy,  zpy,  imp, absy,  imp, absy, absx, absx, absy, absy, /* B */
/* C */     imm, indx,  imm, indx,   zp,   zp,   zp,   zp,  imp,  imm,  imp,  imm, abso, abso, abso, abso, /* C */
/* D */     rel, indy,  imp, indy,  zpx,  zpx,  zpx,  zpx,  imp, absy,  imp, absy, absx, absx, absx, absx, /* D */
/* E */     imm, indx,  imm, indx,   zp,   zp,   zp,   zp,  imp,  imm,  imp,  imm, abso, abso, abso, abso, /* E */
/* F */     rel, indy,  imp, indy,  zpx,  zpx,  zpx,  zpx,  imp, absy,  imp, absy, absx, absx, absx, absx  /* F */
};

static void (*optable[256])() = {
/*        |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  A  |  B  |  C  |  D  |  E  |  F  |      */
/* 0 */      brk,  ora,  nop,  slo,  nop,  ora,  asl,  slo,  php,  ora,  asl,  nop,  nop,  ora,  asl,  slo, /* 0 */
/* 1 */      bpl,  ora,  nop,  slo,  nop,  ora,  asl,  slo,  clc,  ora,  nop,  slo,  nop,  ora,  asl,  slo, /* 1 */
/* 2 */      jsr,  and,  nop,  rla,  bit,  and,  rol,  rla,  plp,  and,  rol,  nop,  bit,  and,  rol,  rla, /* 2 */
/* 3 */      bmi,  and,  nop,  rla,  nop,  and,  rol,  rla,  sec,  and,  nop,  rla,  nop,  and,  rol,  rla, /* 3 */
/* 4 */      rti,  eor,  nop,  sre,  nop,  eor,  lsr,  sre,  pha,  eor,  lsr,  nop,  jmp,  eor,  lsr,  sre, /* 4 */
/* 5 */      bvc,  eor,  nop,  sre,  nop,  eor,  lsr,  sre,  cli,  eor,  nop,  sre,  nop,  eor,  lsr,  sre, /* 5 */
/* 6 */      rts,  adc,  nop,  rra,  nop,  adc,  ror,  rra,  pla,  adc,  ror,  nop,  jmp,  adc,  ror,  rra, /* 6 */
/* 7 */      bvs,  adc,  nop,  rra,  nop,  adc,  ror,  rra,  sei,  adc,  nop,  rra,  nop,  adc,  ror,  rra, /* 7 */
/* 8 */      nop,  sta,  nop,  sax,  sty,  sta,  stx,  sax,  dey,  nop,  txa,  nop,  sty,  sta,  stx,  sax, /* 8 */
/* 9 */      bcc,  sta,  nop,  nop,  sty,  sta,  stx,  sax,  tya,  sta,  txs,  nop,  nop,  sta,  nop,  nop, /* 9 */
/* A */      ldy,  lda,  ldx,  lax,  ldy,  lda,  ldx,  lax,  tay,  lda,  tax,  nop,  ldy,  lda,  ldx,  lax, /* A */
/* B */      bcs,  lda,  nop,  lax,  ldy,  lda,  ldx,  lax,  clv,  lda,  tsx,  lax,  ldy,  lda,  ldx,  lax, /* B */
/* C */      cpy,  cmp,  nop,  dcp,  cpy,  cmp,  dec,  dcp,  iny,  cmp,  dex,  nop,  cpy,  cmp,  dec,  dcp, /* C */
/* D */      bne,  cmp,  nop,  dcp,  nop,  cmp,  dec,  dcp,  cld,  cmp,  nop,  dcp,  nop,  cmp,  dec,  dcp, /* D */
/* E */      cpx,  sbc,  nop,  isb,  cpx,  sbc,  inc,  isb,  inx,  sbc,  nop,  sbc,  cpx,  sbc,  inc,  isb, /* E */
/* F */      beq,  sbc,  nop,  isb,  nop,  sbc,  inc,  isb,  sed,  sbc,  nop,  isb,  nop,  sbc,  inc,  isb  /* F */
};

static const uint32_t ticktable[256] = {
/*        |  0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |  8  |  9  |  A  |  B  |  C  |  D  |  E  |  F  |     */
/* 0 */      7,    6,    2,    8,    3,    3,    5,    5,    3,    2,    2,    2,    4,    4,    6,    6,  /* 0 */
/* 1 */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7,  /* 1 */
/* 2 */      6,    6,    2,    8,    3,    3,    5,    5,    4,    2,    2,    2,    4,    4,    6,    6,  /* 2 */
/* 3 */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7,  /* 3 */
/* 4 */      6,    6,    2,    8,    3,    3,    5,    5,    3,    2,    2,    2,    3,    4,    6,    6,  /* 4 */
/* 5 */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7,  /* 5 */
/* 6 */      6,    6,    2,    8,    3,    3,    5,    5,    4,    2,    2,    2,    5,    4,    6,    6,  /* 6 */
/* 7 */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7,  /* 7 */
/* 8 */      2,    6,    2,    6,    3,    3,    3,    3,    2,    2,    2,    2,    4,    4,    4,    4,  /* 8 */
/* 9 */      2,    6,    2,    6,    4,    4,    4,    4,    2,    5,    2,    5,    5,    5,    5,    5,  /* 9 */
/* A */      2,    6,    2,    6,    3,    3,    3,    3,    2,    2,    2,    2,    4,    4,    4,    4,  /* A */
/* B */      2,    5,    2,    5,    4,    4,    4,    4,    2,    4,    2,    4,    4,    4,    4,    4,  /* B */
/* C */      2,    6,    2,    8,    3,    3,    5,    5,    2,    2,    2,    2,    4,    4,    6,    6,  /* C */
/* D */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7,  /* D */
/* E */      2,    6,    2,    8,    3,    3,    5,    5,    2,    2,    2,    2,    4,    4,    6,    6,  /* E */
/* F */      2,    5,    2,    8,    4,    4,    6,    6,    2,    4,    2,    7,    4,    4,    7,    7   /* F */
};
