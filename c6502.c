#include <cmath>
#include <stdbool.h>
#include <stdint.h>

#define RESET_VECTOR_LOW 0xFFFA
#define STACK_PAGE 0x0100

#define COMBINE_BYTES(low, high) (((uint16_t)high) << 8 | (uint16_t)low)

#define ZEROCALC(n)                                                            \
  {                                                                            \
    if ((n) & 0x00FF)                                                          \
      state.zero = false;                                                      \
    else                                                                       \
      state.zero = true;                                                       \
  }

#define SIGNCALC(n)                                                            \
  {                                                                            \
    if ((n) & 0x0080)                                                          \
      state.negative = true;                                                   \
    else                                                                       \
      state.negative = false;                                                  \
  }

#define CARRYCALC(n)                                                           \
  {                                                                            \
    if ((n) & 0xFF00)                                                          \
      state.carry = true;                                                      \
    else                                                                       \
      state.carry = false;                                                     \
  }

#define OVERFLOWCALC(n, m, o)                                                  \
  { /* n = result, m = accumulator, o = memory */                              \
    if (((n) ^ (uint16_t)(m)) & ((n) ^ (o)) & 0x0080)                          \
      state.overflow = true;                                                   \
    else                                                                       \
      state.overflow = false;                                                  \
  }

#define SAVEACCUM(n) state.accumulator = (uint8_t)((n) & 0x00FF)

// Forward decleration
// The user of the libary the resposible for implementing those functions
uint8_t read6502(uint16_t address);
void write6502(uint16_t address, uint8_t value);

static void (*addrtable[256])();
static void (*optable[256])();
static const uint32_t ticktable[256];

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

  bool addr_panalty_cycle, opcode_panalty_cycle;

  unsigned num_of_cyles;
} state;

void c6502_reset() {
  state.processor_status = 0;

  state.stack_ptr = 0xFD;

  state.x = state.y = state.accumulator = 0;
  state.cycles = state.oprand_address = 0;

  uint8_t reset_vector_lo = read6502(RESET_VECTOR_LOW);
  uint8_t reset_vector_hi = read6502(RESET_VECTOR_LOW + 1);

  state.processor_status = COMBINE_BYTES(reset_vector_lo, reset_vector_hi);

  state.cycles = 8;
}

bool c6502_getCarry() { return state.carry; }
bool c6502_getZero() { return state.zero; }
bool c6502_getInterruptDisable() { return state.interrupt_disable; }
bool c6502_getDecimalMode() { return state.decimal_mode; }
bool c6502_getBreake() { return state.breake; }
bool c6502_getOverflow() { return state.overflow; }
bool c6502_getNegative() { return state.negative; }

void c6502_tick() {}

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

  state.oprand_address = COMBINE_BYTES(lo, hi);
  return;
}

static void absx() {
  uint8_t lo = read_pc();
  uint8_t hi = read_pc();

  state.oprand_address = COMBINE_BYTES(lo, hi);
  state.oprand_address += state.x;

  if (state.oprand_address >> 8 != hi) {
    state.addr_panalty_cycle = true;
  }
  return;
}

static void absy() {
  uint8_t lo = read_pc();
  uint8_t hi = read_pc();

  state.oprand_address = COMBINE_BYTES(lo, hi);
  state.oprand_address += state.y;

  if (state.oprand_address >> 8 != hi) {
    state.addr_panalty_cycle = true;
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

void push16(uint16_t pushval) {
  write6502(STACK_PAGE + state.stack_ptr, (pushval >> 8) & 0xFF);
  write6502(STACK_PAGE + ((state.stack_ptr - 1) & 0xFF), pushval & 0xFF);
  state.stack_ptr -= 2;
}

void push8(uint8_t pushval) {
  write6502(STACK_PAGE + state.stack_ptr--, pushval);
}

uint16_t pull16() {
  uint16_t temp16;
  temp16 =
      read6502(STACK_PAGE + ((state.stack_ptr + 1) & 0xFF)) |
      ((uint16_t)read6502(STACK_PAGE + ((state.stack_ptr + 2) & 0xFF)) << 8);
  state.stack_ptr += 2;
  return (temp16);
}

uint8_t pull8() { return (read6502(STACK_PAGE + ++state.stack_ptr)); }

uint8_t value;
uint8_t result;

// Instructions
static void adc() {
  state.opcode_panalty_cycle = 1;
  value = getvalue();
  result = (uint16_t)state.accumulator + value + (uint16_t)(state.carry);

  CARRYCALC(result);
  ZEROCALC(result);
  OVERFLOWCALC(result, state.accumulator, value);
  SIGNCALC(result);

#ifndef NES_CPU
  if (state.decimal_mode) {
    state.carry = false;

    if ((state.accumulator & 0x0F) > 0x09) {
      state.accumulator += 0x06;
    }
    if ((state.accumulator & 0xF0) > 0x90) {
      state.accumulator += 0x60;
      state.carry = true;
    }

    state.num_of_cyles++;
  }
#endif

  state.accumulator = result;
  return;
}

static void and () {
  state.opcode_panalty_cycle = true;

  value = getvalue();
  result = (uint16_t)(state.accumulator) & value;

  SIGNCALC(result);
  CARRYCALC(result);

  SAVEACCUM(result);
}

static void asl() {
  value = getvalue();
  result = value << 1;

  SIGNCALC(result);
  CARRYCALC(result);
  ZEROCALC(result);

  putvalue(result);
}

static void bcc() {
  if (state.carry == false) {
    uint16_t oldpc = state.program_counter;
    state.program_counter += state.relative_address;

    if ((oldpc & 0xFF00) != (state.program_counter & 0xFF00))
      state.cycles += 2;
    else
      state.cycles++;
  }
}

static void bcs() {
  if (state.carry) {
    uint16_t oldpc = state.program_counter;

    state.program_counter += state.relative_address;

    if ((oldpc & 0xFF00) != (state.program_counter & 0xFF00))
      state.cycles += 2; // check if jump crossed a page boundary
    else
      state.cycles++;
  }
}

static void beq() {
  if (state.zero) {
    uint16_t oldpc = state.program_counter;
    state.program_counter += state.relative_address;
    if ((oldpc & 0xFF00) != (state.program_counter & 0xFF00))
      state.cycles += 2; // check if jump crossed a page boundary
    else
      state.cycles++;
  }
}

static void bit() {
  value = getvalue();
  result = (uint16_t)state.accumulator & value;

  ZEROCALC(result);
  state.processor_status =
      (state.processor_status & 0x3F) | (uint8_t)(value & 0xC0);
}

static void bmi() {
  if (state.negative) {
    uint16_t oldpc = state.program_counter;
    state.program_counter += state.relative_address;
    if ((oldpc & 0xFF00) != (state.program_counter & 0xFF00))
      state.cycles += 2; // check if jump crossed a page boundary
    else
      state.cycles++;
  }
}

static void bne() {
  if (state.zero == false) {
    uint16_t oldpc = state.program_counter;
    state.program_counter += state.relative_address;
    if ((oldpc & 0xFF00) != (state.program_counter & 0xFF00))
      state.cycles += 2; // check if jump crossed a page boundary
    else
      state.cycles++;
  }
}

static void bpl() {
  if (state.negative == false) {
    uint16_t oldpc = state.program_counter;
    state.program_counter += state.relative_address;
    if ((oldpc & 0xFF00) != (state.program_counter & 0xFF00))
      state.cycles += 2; // check if jump crossed a page boundary
    else
      state.cycles++;
  }
}

static void brk() {
  state.program_counter++;
  push16(state.program_counter);
  uint16_t status =
      state.processor_status | 0x10; // status wiht the breake setk
  push8(status);
  state.interrupt_disable = true;
  state.program_counter =
      (uint16_t)read6502(0xFFFE) | ((uint16_t)read6502(0xFFFF) << 8);
}

static void bvc() {
  if (state.overflow == false) {
    uint16_t oldpc = state.program_counter;
    state.program_counter += state.relative_address;
    if ((oldpc & 0xFF00) != (state.program_counter & 0xFF00))
      state.cycles += 2; // check if jump crossed a page boundary
    else
      state.cycles++;
  }
}

static void bvs() {
  if (state.overflow == true) {
    uint16_t oldpc = state.program_counter;
    state.program_counter += state.relative_address;
    if ((oldpc & 0xFF00) != (state.program_counter & 0xFF00))
      state.cycles += 2; // check if jump crossed a page boundary
    else
      state.cycles++;
  }
}

static void clc() { state.carry = false; }

static void cld() { state.decimal_mode = false; }

static void cli() { state.interrupt_disable = false; }

static void clv() { state.overflow = false; }

static void cmp() {
  state.opcode_panalty_cycle = true;
  value = getvalue();
  result = (uint16_t)state.accumulator - value;

  if (state.accumulator >= (uint8_t)(value & 0x00FF))
    state.carry = true;
  else
    state.carry = false;
  if (state.accumulator == (uint8_t)(value & 0x00FF))
    state.zero = true;
  else
    state.zero = false;

  SIGNCALC(result);
}

static void cpx() {
  value = getvalue();
  result = (uint16_t)state.x - value;

  if (state.x >= (uint8_t)(value & 0x00FF))
    state.carry = true;
  else
    state.carry = false;
  if (state.x == (uint8_t)(value & 0x00FF))
    state.zero = true;
  else
    state.zero = false;

  SIGNCALC(result);
}

static void cpy() {
  value = getvalue();
  result = (uint16_t)state.y - value;

  if (state.y >= (uint8_t)(value & 0x00FF))
    state.carry = true;
  else
    state.carry = false;
  if (state.y == (uint8_t)(value & 0x00FF))
    state.zero = true;
  else
    state.zero = false;
  SIGNCALC(result);
}

static void dec() {
  value = getvalue();
  result = value - 1;

  ZEROCALC(result);
  SIGNCALC(result);

  putvalue(result);
}

static void dex() {
  state.x--;

  ZEROCALC(state.x);
  SIGNCALC(state.x);
}

static void dey() {
  state.y--;

  ZEROCALC(state.y);
  SIGNCALC(state.y);
}

static void eor() {
  state.opcode_panalty_cycle = true;
  value = getvalue();
  result = (uint16_t)state.accumulator ^ value;

  ZEROCALC(result);
  SIGNCALC(result);

  SAVEACCUM(result);
}

static void inc() {
  value = getvalue();
  result = value + 1;

  ZEROCALC(result);
  SIGNCALC(result);

  putvalue(result);
}

static void inx() {
  state.x++;

  ZEROCALC(state.x);
  SIGNCALC(state.x);
}

static void iny() {
  state.y++;

  ZEROCALC(state.y);
  SIGNCALC(state.y);
}

static void jmp() { state.program_counter = state.oprand_address; }

static void jsr() {
  push16(state.program_counter - 1);
  state.program_counter = state.oprand_address;
}

static void lda() {
  state.opcode_panalty_cycle = 1;

  value = getvalue();
  state.accumulator = (uint8_t)(value & 0x00FF);

  ZEROCALC(state.accumulator);
  SIGNCALC(state.accumulator);
}

static void ldx() {
  state.opcode_panalty_cycle = 1;
  value = getvalue();
  state.x = (uint8_t)(value & 0x00FF);

  ZEROCALC(state.x);
  SIGNCALC(state.x);
}

static void ldy() {
  state.opcode_panalty_cycle = 1;
  value = getvalue();
  state.y = (uint8_t)(value & 0x00FF);

  ZEROCALC(state.y);
  SIGNCALC(state.y);
}

static void lsr() {
  value = getvalue();
  result = value >> 1;

  if (value & 1)
    state.carry = true;
  else
    state.carry = false;
  ZEROCALC(result);
  SIGNCALC(result);

  putvalue(result);
}

static void nop() {
  switch (state.opcode) {
  case 0x1C:
  case 0x3C:
  case 0x5C:
  case 0x7C:
  case 0xDC:
  case 0xFC:
    state.opcode_panalty_cycle = 1;
    break;
  }
}

static void ora() {
  state.opcode_panalty_cycle = 1;
  value = getvalue();
  result = (uint16_t)state.accumulator | value;

  ZEROCALC(result);
  SIGNCALC(result);

  SAVEACCUM(result);
}

static void pha() { push8(state.accumulator); }

static void php() {
  uint8_t status = state.processor_status | 0x10;
  push8(status);
}

static void pla() {
  state.accumulator = pull8();

  ZEROCALC(state.accumulator);
  SIGNCALC(state.accumulator);
}

static void plp() {
  state.processor_status = pull8();
  state.unused = true;
}

static void rol() {
  value = getvalue();
  result = (value << 1) | (state.carry);

  CARRYCALC(result);
  ZEROCALC(result);
  SIGNCALC(result);

  putvalue(result);
}

static void ror() {
  value = getvalue();
  result = (value >> 1) | ((state.carry) << 7);

  if (value & 1)
    state.carry = true;
  else
    state.carry = false;
  ZEROCALC(result);
  SIGNCALC(result);

  putvalue(result);
}

static void rti() {
  state.processor_status = pull8();
  value = pull16();
  state.program_counter = value;
}

static void rts() {
  value = pull16();
  state.program_counter = value + 1;
}

static void sbc() {
  state.opcode_panalty_cycle = 1;
  value = getvalue() ^ 0x00FF;
  result = (uint16_t)state.accumulator + value + (uint16_t)(state.carry);

  CARRYCALC(result);
  ZEROCALC(result);
  OVERFLOWCALC(result, state.accumulator, value);
  SIGNCALC(result);

#ifndef NES_CPU
  if (state.decimal_mode) {
    state.carry = false;

    state.accumulator -= 0x66;
    if ((state.accumulator & 0x0F) > 0x09) {
      state.accumulator += 0x06;
    }
    if ((state.accumulator & 0xF0) > 0x90) {
      state.accumulator += 0x60;
      state.carry = true;
    }

    state.cycles++;
  }
#endif

  SAVEACCUM(result);
}

static void sec() { state.carry = true; }

static void sed() { state.decimal_mode = true; }

static void sei() { state.interrupt_disable = true; }

static void sta() { putvalue(state.accumulator); }

static void stx() { putvalue(state.x); }

static void sty() { putvalue(state.y); }

static void tax() {
  state.x = state.accumulator;

  ZEROCALC(state.x);
  SIGNCALC(state.x);
}

static void tay() {
  state.y = state.accumulator;

  ZEROCALC(state.y);
  SIGNCALC(state.y);
}

static void tsx() {
  state.x = state.stack_ptr;

  ZEROCALC(state.x);
  SIGNCALC(state.x);
}

static void txa() {
  state.accumulator = state.x;

  ZEROCALC(state.accumulator);
  SIGNCALC(state.accumulator);
}

static void txs() { state.stack_ptr = state.x; }

static void tya() {
  state.accumulator = state.y;

  ZEROCALC(state.accumulator);
  SIGNCALC(state.accumulator);
}

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

// undocumented ops
#define lax nop
#define sax nop
#define dcp nop
#define isb nop
#define slo nop
#define rla nop
#define sre nop
#define rra nop

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
