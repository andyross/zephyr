// Super-trivial asm demonstrator.  Jumps forward a bit (because
// Xtensa can only back-reference its constants), writes four
// repetitions of a four byte pattern to the start of DRAM at
// 0x60000000, then enters an infinite loop.
//
// Note that absolutely no work is done here to initialize the CPU for
// register windowing, to set up a stack, initialize caching, etc...
// Don't try to link C code into this rig, it won't work.

.section .init
_start:
	j after_immediates

.section .text
after_immediates:
	// Start of DRAM
	movi a4, 0x60000000
	movi a5, 0x59444e41
	s32i a5, a4, 0
	s32i a5, a4, 4
	s32i a5, a4, 8
	s32i a5, a4, 12

	// End of (non-DMA) DRAM mapping
	movi a4, 0x60d7fff0
	movi a5, 0x59444e41
	s32i a5, a4, 0
	s32i a5, a4, 4
	s32i a5, a4, 8
	s32i a5, a4, 12

	// There is a hole in the mapping.  128k at 0x60d80000 is a
	// noop when written, and reads back zeros.  (There is a
	// conspicuous 1M hole in the device-tree regions there too).
	// Usable memory (no idea what it is) picks back up here at
	// 0x60da0000.
	movi a4, 0x60da0000
	movi a5, 0x59444e41
	s32i a5, a4, 0
	s32i a5, a4, 4
	s32i a5, a4, 8
	s32i a5, a4, 12

	// End of hole
	movi a4, 0x60e7fff0
	movi a5, 0x59444e41
	s32i a5, a4, 0
	s32i a5, a4, 4
	s32i a5, a4, 8
	s32i a5, a4, 12

	// Start of "DMA" mapping
	movi a4, 0x60e80000
	movi a5, 0x59444e41
	s32i a5, a4, 0
	s32i a5, a4, 4
	s32i a5, a4, 8
	s32i a5, a4, 12

	// End of DRAM
	movi a4, 0x60fffff0
	movi a5, 0x59444e41
	s32i a5, a4, 0
	s32i a5, a4, 4
	s32i a5, a4, 8
	s32i a5, a4, 12

	// End of SRAM
	movi a4, 0x4003fff0
	movi a5, 0x59444e41
	s32i a5, a4, 0
	s32i a5, a4, 4
	s32i a5, a4, 8
	s32i a5, a4, 12
loop:
	j loop
