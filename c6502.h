#ifndef C6502_H
#define C6502_H

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef uint8_t (*read_func_t)(uint16_t);
typedef void (*write_func_t)(uint16_t, uint8_t);

typedef struct {
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
  bool implied;

  read_func_t read;
  write_func_t write;
} c6502_t;

c6502_t *c6502_create(read_func_t read, write_func_t write);
void c6502_destroy(c6502_t *cpu);
void c6502_reset(c6502_t *cpu);
void c6502_tick(c6502_t *cpu);
void c6502_execute_instruction(c6502_t *cpu);
void c6502_interrupt(c6502_t *cpu);

// flags
bool c6502_get_carry(c6502_t *cpu);
bool c6502_get_zero(c6502_t *cpu);
bool c6502_get_interrupt_disable(c6502_t *cpu);
bool c6502_get_decimal_mode(c6502_t *cpu);
bool c6502_get_break(c6502_t *cpu);
bool c6502_get_overflow(c6502_t *cpu);
bool c6502_get_negative(c6502_t *cpu);

// todo remove
#define C6502_IMPLEMENTATION
/// for develepment puprposes

#ifdef C6502_IMPLEMENTATION

// prevent multiple implimentaions
#undef C6502_IMPLEMENTATION

#define RESET_VECTOR_LOW 0xFFFA
#define STACK_PAGE 0x0100

void c6502_reset(c6502_t *cpu) {
  cpu->processor_status = 0;
  cpu->stack_ptr = 0xFD;
  cpu->x = cpu->y = cpu->accumulator = 0;
  cpu->cycles = cpu->implied = cpu->oprand_address = 0;

  uint8_t resetVectorLow = cpu->read(RESET_VECTOR_LOW);
  uint8_t resetVectorHigh = cpu->read(RESET_VECTOR_LOW + 1);
  cpu->processor_status= (resetVectorHigh << 8) | resetVectorLow;

  cpu->cycles = 8;
}

c6502_t *c6502_create(read_func_t read, write_func_t write) {
  c6502_t *cpu = malloc(sizeof(c6502_t));
  cpu->read = read;
  cpu->write = write;
  c6502_reset(cpu);
  return cpu;
}

void c6502_destroy(c6502_t *cpu) {
  free(cpu);
}

#endif // C6502_IMPLEMENTATION

#endif // C6502_H
