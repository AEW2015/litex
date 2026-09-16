# Experimental USNative BIOS initialization

This adds an explicit BIOS startup path for the existing XEM8320 x16 native
DDR4 integration. It is an initial board-specific calibration driver, not a
portable implementation for every UltraScale board. The independent LiteDRAM
native-building-blocks commit does not yet provide the complete PHY/CSR ABI.

## Default operation

A board target opts in with:

```python
self.add_config("SDRAM_USNATIVE_XEM8320")
```

`sdram_init()` then runs native readiness initialization, direct-DFII clock and
lane searches, CPU-based per-DQ read deskew and measured +/-4-tap guards. After
successful calibration, the existing BIOS controller handoff and memory check
run normally. Startup prints the selected profile, success or an error; it does
not print per-tap scans or use DMA. Other PHYs and the existing custom-init hook
keep their existing initialization paths. When both native and custom-init flags
are present, the native driver takes precedence; new targets should use only the
native flag unless they explicitly need another custom hook elsewhere.

Training writes memory and must run from ROM/SRAM before applications use DDR.
The CPU deskew scratch region is 2 MiB at 0x41000000; direct DFII searches also
write training rows. This is not a non-destructive runtime retraining service.
On calibration or final memory-check failure, DDR remains in reset and under
software ownership, and the existing DDRCTRL error/status reporting is updated.

## Optional build features

```python
# Optional detailed scan output and diagnostic snapshot CSRs.
self.add_config("SDRAM_USNATIVE_DEBUG")

# Optional counter/PRBS refinement using the local paired DMA engine.
self.add_config("SDRAM_USNATIVE_DMA_CALIBRATION")
```

These options are independent and disabled by default. DMA refinement requires
the paired 256-bit DMA CSR interface, per-DQ error mask and its hardware timeout.
It destroys 64 MiB starting at 0x41000000. Software also bounds polling when the
hardware completion response never arrives. A failed/stuck DMA transaction must
be reset/quiesced by the system before another attempt; firmware cannot cancel
an already issued memory operation through this interface.

No DMA benchmark BIOS command or debug hardware is instantiated by this patch.
Those are separate board-target choices. Required delay selection, RIU and
readiness CSRs are calibration controls, even when verbose diagnostics are off.

## Supported initial interface

- XEM8320 x16 DDR4, four 32-bit DFI phases, eight-bit serializers, 512 delay taps,
  eight bitslips and registered TX with command latency 5.
- 2400 MT/s: controller 300 MHz, CL17/CWL12, RD2/WR3.
- 2666.667 MT/s: controller 333333333 Hz, CL19/CWL14, RD0/WR1.
- Related sys:RIU clocks 2:1, acknowledged RIU bridge and registered tap-status
  freshness. Physical tap indices and nibble ownership match the existing
  XEM8320 integration; a new board must not simply reuse this flag.
- Generated JEDEC initialization and memory timing settings remain the target's
  responsibility. Unsupported widths, phase/latency profiles, inadequate scratch
  memory and missing required CSR capabilities fail at compile time.

The firmware and PHY share one configuration: direct read/write commands use the
same generated phase values as runtime phase selection. The SDRAM frequency API
returns an unsigned frequency so rates above 2147 MT/s display correctly.

## Validation and remaining integration

Host C tests cover profile rejection, optional DMA requirements, phase selection,
RIU/tap wait failures, DMA completion bounds, init failure ownership, final
memtest failure and unsigned speed reporting. Run:

```sh
python -m pytest test/software/test_usnative.py test/software/test_litedram_accessors.py
```

The default 2400 and2666.667 BIOS images compiled and linked using the existing
local Windows build adapter with this new liblitedram and BIOS source. The native
C implementation also compiles against generated headers with DMA/debug CSRs
removed, and with both optional features enabled. This is firmware validation;
no bitstream was built/programmed and the new default CPU-only calibration path
has not yet been qualified on hardware. Previous hardware evidence included the
optional DMA refinement, so it does not establish the new default path's margin.

Four existing console/command host tests fail in the Windows environment and
also fail on unchanged upstream. They remain outstanding for suitable Linux CI.

Before an upstream PR, replace board-specific tap/nibble indices and bootstrap
settings with generated PHY metadata, wire target options into litex-boards,
validate the default path on hardware, and add an explicit controller/DMA quiesce
contract for retraining. No thermal or runtime background retraining is included.
