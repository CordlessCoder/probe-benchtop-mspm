/* Fixture for the Thumb-bit rule. Generic on purpose: it is a shape, not a product. */
#include <stdint.h>

/* Two single-byte variables, so the linker has to put one of them at an odd address.
 * That is the whole fixture: a byte at an odd address is what a data symbol carrying
 * its own low bit looks like, and what clearing that bit resolves onto its neighbour. */
volatile uint8_t firstByte;
volatile uint8_t secondByte;

/* A function beside them, so the other half of the rule is covered: code keeps having
 * its Thumb bit cleared, and a test that only checked the data half would pass against
 * an implementation that stopped clearing it anywhere. */
int main(void) { return (int)firstByte + (int)secondByte; }

/*
 * Built with:
 *   arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -g -Os -nostdlib -nostartfiles \
 *     -ffile-prefix-map=$(pwd)=. -Wl,-e,main -o thumb-gcc.elf thumb.c
 *
 * The prefix map is what keeps the builder's own directory out of the committed DWARF, which
 * a published repository has no reason to carry.
 *
 * The ELF is committed beside this file so the test needs no cross-compiler.
 */
