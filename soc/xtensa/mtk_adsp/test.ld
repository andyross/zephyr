MEMORY { ram (rwx) : ORIGIN = 0x40000000, LENGTH = 0x00040000 }
SECTIONS { .text : { *(*.init); *(*.literal); *(*.text) } > ram }
