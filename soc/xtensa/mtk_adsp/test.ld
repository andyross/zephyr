MEMORY { ram (rwx) : ORIGIN = 0x40000000, LENGTH = 0x00040000 }
SECTIONS {

  .text : {
    *(.init);
    *(.literal .literal.* .text .text.*);
    *(.rodata .rodata.*);
    *(.data .data.*);
    *(.bss .bss.*);
  } > ram

#include "vecs.ld"
  > ram

  img_sram_end = .;
}
