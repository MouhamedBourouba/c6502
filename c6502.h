#ifndef C6502_H
#define C6502_H

#include <stdint.h>

typedef uint8_t (*read_func_t)(uint16_t);
typedef void (*write_func_t)(uint16_t, uint8_t);

struct c6502_t {
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
};

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

#ifndef C6502_IMPLEMENTATION

#endif // C6502_IMPLEMENTATION

#endif // C6502_H
