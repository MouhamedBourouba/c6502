#include "../c6502.h"
#include <stdint.h>
#include <stdio.h>

uint8_t ram[0xFFFF] = {0};

uint8_t read6502(uint16_t address) { return ram[address]; }
void write6502(uint16_t address, uint8_t value) { ram[address] = value; }

int main() {
  printf("hello world \n");
  c6502_tick();
  return 0;
}
