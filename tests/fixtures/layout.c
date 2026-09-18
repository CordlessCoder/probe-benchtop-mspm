/* Fixture for the DWARF layout walk. Generic on purpose: it is a shape, not a product. */
#include <stdint.h>

typedef struct { uint16_t low; uint16_t high; } Pair;
typedef struct { uint8_t  flag; Pair     pair; } Inner;

typedef struct {
    Inner    inner;          /* nested struct, so a path has two components   */
    uint32_t counts[4];      /* array, so indices are exercised               */
    uint8_t  narrow : 3;     /* bitfield, which must not appear at all        */
    uint8_t  alsoNarrow : 5;
    uint8_t  after;          /* the field following the bitfields             */
    union { uint32_t word; uint16_t halves[2]; } overlay;  /* union: shared offset */
    void    *pointer;        /* a leaf, never followed                        */
} Fixture;

volatile Fixture fixtureInstance;
volatile uint32_t plainScalar;

int main(void) { return (int)fixtureInstance.after + (int)plainScalar; }

/*
 * Built with:
 *   arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -g -Os -nostdlib -nostartfiles \
 *     -ffile-prefix-map=$(pwd)=. -Wl,-e,main -o layout-gcc.elf layout.c
 *
 * The prefix map is what keeps the builder's own directory out of the committed DWARF, which a
 * published repository has no reason to carry.
 *
 * The ELF is committed beside this file so the test needs no cross-compiler. Rebuild it only to
 * change the shape, and update the offsets in tests/layout.rs when you do — they are the C
 * compiler's answer, not the walk's, which is what makes them worth asserting.
 */
