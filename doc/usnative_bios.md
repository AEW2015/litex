# Experimental USNative BIOS initialization

This adds an explicit BIOS startup path for the XEM8320 x16 native DDR4
integration. It is an initial board-specific calibration driver, not a
portable implementation for every UltraScale board. The coordinated LiteDRAM
feature branch supplies the complete initial PHY/CSR ABI; the litex-boards
feature branch selects it explicitly for Vivado.

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

The optional DMA benchmark command is compiled only when the board target
instantiates its DMA hardware and selects `CONFIG_SDRAM_NATIVE_DMA_TEST`.
Debug hardware is also a separate board-target choice. Required delay
selection, RIU and readiness CSRs are calibration controls, even when verbose
diagnostics are off.

## Supported initial interface

- XEM8320 x16 DDR4, four 32-bit DFI phases, eight-bit serializers, 512 delay taps,
  eight bitslips and registered TX with command latency 5.
- 2400 MT/s: controller 300 MHz, CL17/CWL12, RD2/WR3.
- 2666.667 MT/s: controller 333333333 Hz, CL19/CWL14, RD0/WR1.
- Experimental 2933.333 MT/s: controller 366666666/7 Hz, CL21/CWL16,
  RD2/WR3; requires the overclock configuration.
- Experimental 3200 MT/s: controller 400 MHz, CL24/CWL16, RD3/WR3;
  requires the overclock configuration.
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

Eleven host C tests cover profile rejection, overclock opt-in, optional DMA
requirements, phase selection, RIU/tap wait failures, DMA completion bounds,
the debug-disabled deskew seed, init failure ownership, final memory-test
failure and unsigned speed reporting. Run:

```sh
python -m pytest test/software/test_usnative.py test/software/test_litedram_accessors.py
```

The 2400 and 2666.667 BIOS configurations compile and link using the generated
SoC headers. The DMA-enabled 2666.667 configuration also compiles with the
optional `native_dma` command. See the hardware qualification section below for
the narrower set of configurations that have been exercised on hardware.

The current interface still has board-specific tap/nibble indices and bootstrap
settings. A future portable interface should provide these as generated PHY
metadata. Runtime retraining also needs an explicit controller/DMA quiesce
contract. No thermal or runtime background retraining is included.

## Coordinated board options and overclock profiles

The board target's `--with-dma` selects `CONFIG_SDRAM_NATIVE_DMA_TEST`, which
compiles the optional `native_dma` BIOS command. This is explicit destructive
bandwidth/integrity testing after normal calibration, not automatic DMA-based
calibration refinement. The default engine uses a 128/256-bit width-converted
native port. Write and read rates are reported independently, using hardware
cycle counts; percentage efficiency uses the physical x16 DDR peak rather than
DMA port width.

When the target selects `CONFIG_SDRAM_NATIVE_DMA_BANK_GROUP_INTERLEAVING`, the
same command reports `mode=paired-bank-group-interleaved`. This opt-in hardware
uses two 128-bit bank-group streams as one 256-bit benchmark interface and waits
for both write paths to drain before starting reads. A sticky error from either
stream prevents further traffic through the engine. The standard native-port
mode remains the default, and neither mode enables automatic startup DMA
calibration unless the separate `CONFIG_SDRAM_USNATIVE_DMA_CALIBRATION` option
is selected.

The optional command also supports component `USPDDRPHY` builds. These select
`CONFIG_SDRAM_DMA_SOFTWARE_ADMISSION` and provide the
`dma_bench_software_ready` CSR. `sdram_init()` clears admission before each
initialization and grants it only after standard leveling and the final
controller-path memory test complete. A failed or interrupted reinitialization
therefore leaves the hardware gate closed. The `sdram_cal` diagnostic revokes
admission because leveling alone does not validate normal controller traffic;
run `sdram_init` before using DMA again. Component-PHY debug uses LiteDRAM's
existing leveling output and commands, not HSSIO bit-slice window reporting.
Targets may map a component-PHY debug option to `CONFIG_SDRAM_PHY_DEBUG` to
include command-delay scans and write-latency calibration samples. It remains
separate from USNative debugging and does not enable DMA.

For `USDDRPHY` and `USPDDRPHY`, repeated initialization wraps each DQS output
delay back to its base before asserting the global PHY reset. The global reset
clears DQ output delay, so this preparation keeps the DQ and software-tracked
DQS offsets coherent when leveling starts again. It does not affect the
USNative initialization path.

The BIOS DMA completion wait uses LiteX's hardware timer instead of a fixed CPU
poll count. Its deadline covers two complete DMA hardware timeout intervals
plus a settling margin, with the conversion performed from system-clock cycles
using 64-bit arithmetic. This permits large transfers to finish without making
the safety timeout depend on CPU execution speed.

The 2933.333 and 3200 profiles require `CONFIG_SDRAM_USNATIVE_OVERCLOCK` in
addition to the native profile. Their CL/CWL and read-gate delays differ from
the initial 2400/2666.667 profiles. The BIOS identifies overclock operation at
startup. Acceptance of a profile is not timing or hardware qualification.

Debug logs provide direct RX/TX search endpoints and final per-bit RX deskew
windows. Report spans as last-first taps, together with scan step and censored
search boundaries. These measurements do not constitute full eye or thermal scans.

## Hardware qualification status

The 2400 MT/s, debug-disabled XEM8320 build passed three calibration runs—an
FPGA load, a reboot, and an explicit `sdram_init`—and three BIOS memory tests
after making the RX deskew pattern seed unconditional. CPU memory-speed
measurements were 44.6 MiB/s write and 48.7 MiB/s read. These CPU-loop figures
measure the firmware access path, not the sustained bandwidth available through
a native DMA port.

The 2666.667 MT/s debug/DMA256 build also passed calibration and memory tests.
Its 2 MiB CPU test measured 49.3 MiB/s write and 54.6 MiB/s read. Both 64 MiB
counter and PRBS31 DMA tests passed with zero errors and no fault: writes were
2.402195 and 2.400797 GB/s (45.04% and 45.01% of physical peak), and reads were
2.420201 and 2.420200 GB/s (45.37%). This uses the standard controller and a
width-converted port, so the result is not comparable to the older paired-port
engine without accounting for that architecture.

Final component-`USPDDRPHY` hardware testing covered three configurations. Each
passed three initialization runs, three mandatory controller-path memory tests,
and five DMA tests including a full 1 GiB PRBS write and reread.

| DDR rate | DMA interface | DMA write/read | Physical-peak efficiency |
|---:|---|---:|---:|
| 1000 MT/s | 128-bit standard | 0.905 / 0.918 GB/s | 45.24% / 45.90% |
| 1000 MT/s | 256-bit paired bank groups | 1.797 / 1.815 GB/s | 89.85% / 90.77% |
| 2000 MT/s | 256-bit paired bank groups | 3.569 / 3.584 GB/s | 89.23% / 89.60% |

Both paired configurations passed CPU/DMA interoperability tests. The 1000
MT/s paired build also verified the admission contract: calibration-only and a
subsequent standalone memory test left DMA closed, while full initialization
restored access. The hardware-timer completion deadline passed the 1 GiB tests.
These builds retain the upstream component-PHY RTL and use the BIOS pre-reset
DQS restoration described above.

The 1000 MT/s implementations passed setup, hold, and pulse-width timing. The
2000 MT/s implementation had positive fabric setup and hold slack but a
`-0.600 ns` clock pulse-width slack, so it remains an experimental hardware
result rather than a timing-qualified component-PHY configuration. Its reported
read-leveling windows are standard PHY half-windows, not HSSIO bit-slice eyes.

The packaged 2933.333 and 3200 MT/s configurations have not yet completed
hardware qualification. Both are overclock profiles and must remain explicitly
opt-in.
