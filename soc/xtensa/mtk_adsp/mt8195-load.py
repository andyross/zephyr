#!/usr/bin/env python
import ctypes
import sys
import mmap

# MT8195 audio firmware load/debug gadget

# Note that the handling here is only partial: in practice the audio
# DSP depends on clock and power well devices whose drivers live
# elsewhere in the kernel.  Those aren't duplicated here.  Make sure
# the DSP has been started by a working kernel driver first.

# The DSP device has a host register block and two shared memory regions:
#
# * 256kb of fast device SRAM.  Sometimes referred to as "IRAM" --
#   internal RAM?  Mapped on the DSP at 0x40000000
#
# * 16MB of shared host DRAM.  Slow from the device, but large.
#   Visible at the same address (0x60000000) on the DSP.  See below
#   for details.
#
# In the kernel driver, the address/size values come from device-tree.
# But currently the Mediatek architecture is one kernel driver per
# SOC, so it really doesn't matter.  There's also a seeming bug with
# the setup in current kernels: the DRAM region is split between a
# 13.5MB prefix for general use and a 2.5MB suffix seemingly dedicated
# to DMA.  But the start address of the DMA region is 1MB too high
# (0x60e80000 and not 0x60d80000), leading to an overflow of the
# hardware mapping at the end.  This region seems to be unused by
# existing firmware, probably explaining why it hasn't been fixed.
#
# (For reference: in /proc/device-tree on current ChromeOS kernels,
# the host registers are a "cfg" platform resource on the
# "adsp@10803000" node.  The sram is likewise the "sram" resource on
# that device node, and the two dram areas are "memory-region"
# phandles pointing to "adsp_mem_region" and "adsp_dma_mem_region"
# nodes under "/reserved-memory").
#
# Note that there seems to be a bug with the current devicetree
# schema.

MAPPINGS = { "regs" : (0x10803000,   0xa000),
             "sram" : (0x10840000,  0x40000),
             "dram" : (0x60000000, 0x1000000) }

# Runtime mmap objects for each MAPPINGS entry
maps = {}

def main():
    # Open device and establish mappings
    devmem_fd = open("/dev/mem", "wb+")
    for mp in MAPPINGS.keys():
        paddr = MAPPINGS[mp][0]
        mapsz = MAPPINGS[mp][1]
        maps[mp] = mmap.mmap(devmem_fd.fileno(), mapsz, offset=paddr,
                             flags=mmap.MAP_SHARED, prot=mmap.PROT_WRITE|mmap.PROT_READ)

    # Create a Regs object for the registers
    cfg = Regs(ctypes.addressof(ctypes.c_int.from_buffer(maps["regs"])))
    cfg.ALTRESETVEC   = 0x0004 # Xtensa boot address
    cfg.RESET_SW      = 0x0024 # Xtensa halt/reset/boot control
    cfg.PDEBUGBUS0    = 0x000c # Unclear, enabled by host, unused by SOF?
    cfg.SRAM_POOL_CON = 0x0930 # SRAM power control: low 4 bits (banks?) enable
    cfg.EMI_MAP_ADDR  = 0x981c # == host SRAM mapping - 0x40000000 (controls MMIO map?)
    cfg.freeze()

    if sys.argv[1] == "stop":
        # Stop DSP
        cfg.RESET_SW |= 8 # Set RUNSTALL: halt CPU
        cfg.RESET_SW |= 3 # Set low two bits: "BRESET|DRESET"

    elif sys.argv[1] == "start":
        # Start DSP (restarts if not already halted)
        cfg.RESET_SW |= 8 # Set RUNSTALL: halt CPU
        cfg.RESET_SW |= 3 # Set low two bits: "BRESET|DRESET"

        cfg.RESET_SW |= 0x10         # Enable "alternate reset" boot vector
        cfg.ALTRESETVEC = 0x40000000 # Start of SRAM, firmware default

        cfg.RESET_SW &= ~3 # Release reset bits
        cfg.RESET_SW &= ~8  # Clear RUNSTALL: go!

    elif sys.argv[1] == "load":
        # Load named mapping from file argument
        assert(sys.argv[2] != "regs")
        dat = open(sys.argv[3], "rb").read()
        mm = maps[sys.argv[2]]
        n = min(len(mm), len(dat))
        print(f"Copying {n} bytes to {sys.argv[2]}\n")
        for i in range(n):
            mm[i] = dat[i]

    elif sys.argv[1] == "dump":
        # Dump named mapping to stdout
        assert(sys.argv[2] != "regs")
        sys.stdout.buffer.write(maps[sys.argv[2]])

    elif sys.argv[1] == "log":
        # Custom logging protocol, watch the 1M null-terminated log
        # stream at 0x60700000 (the top of the linkable region of
        # existing SOF firmware, before the heap.  Nothing uses this
        # currently.)
        msg = b''
        for i in range(0x700000, 0x800000):
            x = maps["dram"][i]
            if x == 0:
                break
            msg += x.to_bytes(1, "little")
        sys.stdout.buffer.write(msg)

    elif sys.argv[1] == "regs":
        # Register dump
        print(f"ALTRESETVEC = 0x{cfg.ALTRESETVEC:x}")
        print(f"RESET_SW = 0x{cfg.RESET_SW:x}")
        print(f"PDEBUGBUS0 = 0x{cfg.PDEBUGBUS0:x}")
        print(f"SRAM_POOL_CON = 0x{cfg.SRAM_POOL_CON:x}")
        print(f"EMI_MAP_ADDR = 0x{cfg.EMI_MAP_ADDR:x}")

    else:
        print(f"Usage: {sys.argv[0]} stop|start|regs|log")
        print(f"       {sys.argv[0]} dump <regs|sram|dram>")
        print(f"       {sys.argv[0]} load <regs|sram|dram> <file>")

# (Cribbed from cavstool.py)
class Regs:
    def __init__(self, base_addr):
        vars(self)["base_addr"] = base_addr
        vars(self)["ptrs"] = {}
        vars(self)["frozen"] = False
    def freeze(self):
        vars(self)["frozen"] = True
    def __setattr__(self, name, val):
        if not self.frozen and name not in self.ptrs:
            addr = self.base_addr + val
            self.ptrs[name] = ctypes.c_uint32.from_address(addr)
        else:
            self.ptrs[name].value = val
    def __getattr__(self, name):
        return self.ptrs[name].value

if __name__ == "__main__":
    main()
