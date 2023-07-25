MEMORY { ram (rwx) : ORIGIN = 0x40000000, LENGTH = 0x00040000 }
SECTIONS {
  .text : {
    *(*.init);
    *(*.literal*);
    *(*.rodata*);
    *(*.text*);
    *(*.data*);
    *(*.bss*);
    . = ALIGN(16);
    img_sram_end = .;
  } > ram
}
