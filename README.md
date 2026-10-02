# FDC1793-Emul — fork with fixes against the FD179X-01 data sheet

This fork keeps the core of [solegstar/FDC1793-Emul](https://github.com/solegstar/FDC1793-Emul)
and fixes what we found while putting it into a Vector-06C expansion board and
a Vector-06C replica on a Terasic DE1. On real hardware, with the start-up
changes below, the core boots MicroDOS, reads and writes a Gotek and a 3.5"
drive, formats disks (`FORMAT`) and copies them (`PIP`). The Read Track and
data sheet fixes have passed the test benches and still wait for a hardware
run. Every change in the original files is marked `(наше)` ("ours") in the
code. The original README follows below.

The reference was the Western Digital FD179X-01 data sheet, October 1979:
https://www.bitsavers.org/components/westernDigital/FD179X-01_Data_Sheet_Oct1979.pdf.
Times are for CLK = 1 MHz (5.25" MFM, 250 kbit/s). The core runs at 16 MHz
and produces those times.

## Changes to the original files

Start-up (`fdc_emul.v`, `DPLL.v`, `AMD.v`, `Main_CTRL.v`). Six registers had
no power-up value. In simulation each of them failed as something that looked
real rather than as an X: no clock (`clk_16`), the data separator never
started (`w288`), sync marks were never found (`o3WORDS`, `rBIT`), Write Track
waited forever at stage 10 (`rIPTRG`, `rIPTRG0`). They now have initial
values, as the neighbouring registers already did.

New outputs of `Main_CTRL.v`:
- `oBUSY` -- BUSY straight from the state machine;
- `oRDTRK` -- Read Track is running past the index.

`DPLL.v`: the separator table is kept in logic (`romstyle = "logic"`), so
Quartus does not spend an M4K block on it.

Read Track returns the gaps, as the data sheet says ("Gaps are included in
the input data stream"). The decoder used to stay silent until the first
address mark, and the 28 bytes after the index were lost: 344 bytes per
revolution instead of about 390. `fdc_emul.v` now starts the decoder with
`start | rdtrk`.

`Main_CTRL.v`, against the data sheet:

| was | now |
|---|---|
| step pulse 0.94 us | 4 us (TSTP) |
| Step, Step-In, Step-Out: direction set 0.4 us before the step | set before the step when it changes (TDIR, 24 us) |
| Seek, Restore: the direction delay was skipped (the counter was checked one clock before reloading) | delay applied on a direction change |
| Restore without TR00: clean status after 255 steps | Seek Error |
| Verify: up to 9 index pulses | 5 ("within 5 revolutions") |
| no data mark after the ID field: took the next sector's data mark and returned its 256 bytes with a CRC error | data mark must come within 43 bytes, otherwise Record Not Found |
| Write Sector: a missed byte written as zero, no status bit | zero and Lost Data |
| Seek Error and CRC Error carried over to the next command | cleared at the start of a new command |
| Write Track: waited 3 bytes (96 us) for the first byte | waits until the index pulse |
| Write Track: exit on Lost Data or Write Protect went through stage 17, which compared with an index flag left from the previous command | goes straight to the end of the command |

## Additions

- `firmware/fdc_emul/rtl/fdc_core.v` -- a top level for use inside an FPGA, in
  place of `fdc_emul.v`. Input and output data buses are separate; there is no
  tri-state bus. BUSY comes from `oBUSY`. Optional clock divider: 32 MHz
  divided by two, or 16 MHz directly.
- `firmware/fdc_emul/rtl/fdc/mfm_wr.v` -- MFM encoder used by `fdc_core.v` in
  place of `MFMCDR.v`:
  - real write precompensation. `MFMCDR` stretched the interval by a clock,
    so the shifts added up along the track. Here the cell grid stays even and
    only the pulse moves: 0..8 steps of 62.5 ns, chosen by the neighbours two
    cells away.
  - WG output aligned with the data.
  - after WG drops, the started byte is finished, so an FF follows the CRC as
    on the chip.
- `firmware/fdc_emul/sim/` -- Icarus Verilog test benches, started with
  `sh firmware/fdc_emul/sim/run.sh`:
  - `tb_fdc1793.v` -- the core through `fdc_emul.v`: reset, Restore, Seek,
    Read/Write Sector on a model disk, Write Track, Read Address, Read Track,
    Force Interrupt.
  - `tb_fd179x.v` -- the data sheet section by section through `fdc_core.v`:
    status bits, step rates, Step/Step-In/Step-Out and `u`, verify, head
    load, E delay, read errors, C/S and `m`, `a0`, Lost Data, Write Protect,
    Read Address, Write Track, Force Interrupt, Restore without TR00. Before
    the data sheet fixes it failed six checks.

## Known differences, left on purpose

- There is no READY input. Type II and III commands run on a drive that is
  not ready, so status bit 7 must be supplied outside the core.
- A command written while BUSY is accepted and aborts the current one. The
  data sheet tells software not to do this and does not say what the chip
  does.
- The core runs its state machine at 16 MHz, against 1 MHz in the chip. A
  command that does not touch the disk holds BUSY for about 500 ns. A host
  that polls status may never see BUSY, so stretch it outside (we use 50 us).

Not checked: Force Interrupt on READY (I0, I1), FM (single density),
multi-sector write.

The original repository does not state a license, and this fork does not add
one.

---

# WD1793 Floppy Disk Controller Hardware FPGA Emulator

### Project Description  

This project is based on the **VG93 (FDC1793/WD1793) emulator core for FPGA** originally developed by [IanPo](https://github.com/IanPo) and later refactored by [andykarpov](https://github.com/andykarpov) for the **Karabas-Go** project.  

For this implementation:  
- All Beta Disk–related components were removed, leaving only the VG93 (FDC1793/WD1793) itself.  
- Several additional internal signals were routed to the VG pins, which makes the source code slightly different from the original, though not significantly.  

### Hardware & Firmware Versions  

The project provides three PCB versions, each with a corresponding FPGA firmware:  

- **kpro** – Board without buffers, modified v1 with repinned layout for convenience. Must be used only with 3.3v outputs!
- **v2** – Second board revision with buffers replacing level shifters. FPGA pinout differs from *kpro*.  
- **vDebug** – Debug version with several free pins exposed for firmware debugging. Pinout matches *v2*.  

### Credits  

- [IanPo](https://github.com/IanPo) – Original VG93 FPGA core for Firefly FPGA computer 
- [andykarpov](https://github.com/andykarpov) – Refactoring and integration into Karabas-Go  
- Community contributors and testers  

