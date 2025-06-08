#ifndef C6502_H
#define C6502_H

#include <stdint.h>

typedef struct c6502_t c6502_t;

typedef uint8_t (*read_func_t)(uint16_t);
typedef void (*write_func_t)(uint16_t, uint8_t);

c6502_t *c6502_create(read_func_t read, write_func_t write);
void c6502_destroy(c6502_t *cpu);
void c6502_reset(c6502_t *cpu);
void c6502_tick(c6502_t *cpu);
void c6502_exeInstruction(c6502_t *cpu);
void c6502_interrupt(c6502_t *cpu);

// flags
bool c6502_getCarry(c6502_t *cpu);
bool c6502_getZero(c6502_t *cpu);
bool c6502_getInterruptDisable(c6502_t *cpu);
bool c6502_getDecimalMode(c6502_t *cpu);
bool c6502_getBreake(c6502_t *cpu);
bool c6502_getOverflow(c6502_t *cpu);
bool c6502_getNegative(c6502_t *cpu);

#endif // C6502_H
