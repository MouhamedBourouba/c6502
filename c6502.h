#include <stdbool.h>
#include <stdint.h>

#define RESET_VECTOR_LOW 0xFFFC
#define STACK_PAGE 0x0100

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

  bool addr_penalty_cycle, opcode_penalty_cycle;

  unsigned total_cycles;
  unsigned toatal_instructions;
} state;

// Forward decleration
// The user of the libary the resposible for implementing those functions
uint8_t read6502(uint16_t address);
void write6502(uint16_t address, uint8_t value);

void c6502_reset();

void c6502_tick();

bool c6502_getCarry();
bool c6502_getZero();
bool c6502_getInterruptDisable();
bool c6502_getDecimalMode();
bool c6502_getBreake();
bool c6502_getOverflow();
bool c6502_getNegative();

bool c6502_getTotaCylesRan();
bool c6502_getTotaInstructionsRan();
