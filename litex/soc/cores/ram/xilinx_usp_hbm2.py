#
# This file is part of LiteX.
#
# Copyright (c) 2020-2021 Florent Kermarrec <florent@enjoy-digital.fr>
# Copyright (c) 2020 Antmicro <www.antmicro.com>
# SPDX-License-Identifier: BSD-2-Clause

import os
from pathlib import Path

from migen import *
from migen.genlib.cdc import MultiReg

from litex.gen import *

from litex.soc.cores.clock import *
from litex.soc.integration.soc import SoCRegion
from litex.soc.integration.builder import *
from litex.soc.interconnect.axi import *
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import csr_bus

# HBM2 Pseudo-Channel Helpers ----------------------------------------------------------------------

USPHBM2_NCHANNELS    = 32
USPHBM2_CHANNEL_SIZE = 0x1000_0000 # 256MB.
USPHBM2_DEFAULT_BASE = 0x4000_0000
USPHBM2_HIGH_BASE    = 0x1_0000_0000

def parse_usphbm2_channels(channels):
    if isinstance(channels, str):
        channels = channels.strip().lower()
        if channels == "all":
            parsed_channels = list(range(USPHBM2_NCHANNELS))
        else:
            parsed_channels = []
            for item in channels.split(","):
                item = item.strip()
                if not item:
                    raise ValueError("empty HBM2 channel entry")
                if "-" in item:
                    start, end = [int(v, 0) for v in item.split("-", 1)]
                    if end < start:
                        raise ValueError("HBM2 channel ranges must be increasing")
                    parsed_channels.extend(range(start, end + 1))
                else:
                    parsed_channels.append(int(item, 0))
    else:
        parsed_channels = list(channels)

    if not parsed_channels:
        raise ValueError("at least one HBM2 channel must be selected")
    if len(parsed_channels) != len(set(parsed_channels)):
        raise ValueError("HBM2 channels must be unique")
    for channel in parsed_channels:
        if not 0 <= channel < USPHBM2_NCHANNELS:
            raise ValueError(f"HBM2 channel {channel} outside 0-{USPHBM2_NCHANNELS - 1}")
    return tuple(parsed_channels)

def usphbm2_channel_origin(channel,
    hbm_base      = USPHBM2_DEFAULT_BASE,
    hbm_high_base = USPHBM2_HIGH_BASE):

    origin = hbm_base + USPHBM2_CHANNEL_SIZE*channel
    if hbm_base == USPHBM2_DEFAULT_BASE and origin >= 0x8000_0000:
        origin = hbm_high_base + USPHBM2_CHANNEL_SIZE*(channel - 4)
    return origin

def usphbm2_channel_origins(channels,
    hbm_base      = USPHBM2_DEFAULT_BASE,
    hbm_high_base = USPHBM2_HIGH_BASE):

    return {
        channel: usphbm2_channel_origin(channel, hbm_base, hbm_high_base)
        for channel in channels
    }

def usphbm2_window_end(origins):
    return max(origin + USPHBM2_CHANNEL_SIZE for origin in origins.values())

def add_usphbm2_pseudochannels(soc, hbm, channels, main_channel,
    origins          = None,
    hbm_base         = USPHBM2_DEFAULT_BASE,
    hbm_high_base    = USPHBM2_HIGH_BASE,
    hbm_strip_origin = False):

    channels = parse_usphbm2_channels(channels)
    if main_channel not in channels:
        raise ValueError("HBM2 main channel must be one of the mapped HBM2 channels")
    if origins is None:
        origins = usphbm2_channel_origins(channels, hbm_base, hbm_high_base)
    if usphbm2_window_end(origins) > 2**33:
        hbm_strip_origin = True

    hbm_regions = {}
    for channel in channels:
        axi_hbm      = hbm.axi[channel]
        axi_lite_bus = AXILiteInterface(data_width=256, address_width=soc.bus.address_width)
        axi_lite_hbm = AXILiteInterface(data_width=256, address_width=33)
        origin       = origins[channel]
        soc.submodules += AXILiteOffset(
            master = axi_lite_bus,
            slave  = axi_lite_hbm,
            offset = origin if hbm_strip_origin else 0)
        soc.submodules += AXILite2AXI(axi_lite_hbm, axi_hbm)
        hbm_regions[channel] = origin
        soc.bus.add_slave(
            f"hbm{channel}",
            axi_lite_bus,
            SoCRegion(
                origin = origin,
                size   = USPHBM2_CHANNEL_SIZE,
                cached = not (0x8000_0000 <= origin < 0x1_0000_0000)))

    soc.add_constant("HBM_CHANNELS", len(channels))
    soc.add_constant("HBM_MAIN_CHANNEL", main_channel)
    soc.add_constant("HBM_HIGH_BASE", hbm_high_base)
    soc.bus.add_region("main_ram", SoCRegion(
        origin = hbm_regions[main_channel],
        size   = USPHBM2_CHANNEL_SIZE,
        linker = True))
    return hbm_regions

# Ultrascale + HBM2 IP Wrapper ---------------------------------------------------------------------

# Devices with 8-high HBM2 stacks (8GB/stack), others have 4-high stacks (4GB/stack).
USPHBM2_8H_DEVICES = ["xcvu45p", "xcvu47p", "xcvu57p", "xcu55c"]

class USPHBM2(LiteXModule):
    """Xilinx Virtex US+ High Bandwidth Memory 2 IP wrapper

    By default, the HBM IP is read from <cwd>/ip/hbm/<hbm_ip_name>.xci. Another .xci can be
    provided with xci_path, or the IP can be generated from parameters with generate_ip=True
    (HBM/AXI/Ref/APB clock frequencies, number of stacks, and optional raw CONFIG.* overrides
    through ip_config).
    """
    def __init__(self, platform, hbm_ip_name="hbm_0", xci_path=None,
        generate_ip  = False,
        stacks       = 2,
        hbm_clk_freq = 900e6,
        axi_clk_freq = 450e6,
        ref_clk_freq = 100e6,
        apb_clk_freq = 100e6,
        ip_config    = {}):
        assert stacks in [1, 2]
        assert not (generate_ip and xci_path is not None)
        self.platform     = platform
        self.hbm_name     = hbm_ip_name
        self.xci_path     = xci_path
        self.generate_ip  = generate_ip
        self.stacks       = stacks
        self.hbm_clk_freq = hbm_clk_freq
        self.axi_clk_freq = axi_clk_freq
        self.ref_clk_freq = ref_clk_freq
        self.apb_clk_freq = apb_clk_freq
        self.ip_config    = ip_config

        self.axi = []
        self.apb = []

        self.hbm_params = {}

        self.init_done = CSRStatus(description="HBM2 initialization done.")

        # # #

        # Clocks -----------------------------------------------------------------------------------
        # Ref = 100 MHz (HBM: 900 (225-900) MHz), drives internal PLL (1 per stack).
        for i in range(stacks):
            self.hbm_params[f"i_HBM_REF_CLK_{i:1d}"] = ClockSignal("hbm_ref")

        # APB: 100 (50-100) MHz
        for i in range(stacks):
            self.hbm_params[f"i_APB_{i:1d}_PCLK"]     = ClockSignal("apb")
            self.hbm_params[f"i_APB_{i:1d}_PRESET_N"] = ~ResetSignal("apb")

        # AXI: 450 (225-450) MHz
        for i in range(16*stacks):
            self.hbm_params[f"i_AXI_{i:02d}_ACLK"]     = ClockSignal("axi")
            self.hbm_params[f"i_AXI_{i:02d}_ARESET_N"] = ~ResetSignal("apb")

        # AXI --------------------------------------------------------------------------------------
        for i in range(16*stacks):
            axi = AXIInterface(data_width=256, address_width=33, id_width=6)
            self.axi.append(axi)

            # AW Channel.
            self.hbm_params[f"i_AXI_{i :02d}_AWADDR"]      = axi.aw.addr
            self.hbm_params[f"i_AXI_{i :02d}_AWBURST"]     = axi.aw.burst
            self.hbm_params[f"i_AXI_{i :02d}_AWID"]        = axi.aw.id
            self.hbm_params[f"i_AXI_{i :02d}_AWLEN"]       = axi.aw.len
            self.hbm_params[f"i_AXI_{i :02d}_AWSIZE"]      = axi.aw.size
            self.hbm_params[f"i_AXI_{i :02d}_AWVALID"]     = axi.aw.valid
            self.hbm_params[f"o_AXI_{i :02d}_AWREADY"]     = axi.aw.ready

            # W Channel.
            self.hbm_params[f"i_AXI_{i:02d}_WDATA"]        = axi.w.data
            self.hbm_params[f"i_AXI_{i:02d}_WLAST"]        = axi.w.last
            self.hbm_params[f"i_AXI_{i:02d}_WSTRB"]        = axi.w.strb
            self.hbm_params[f"i_AXI_{i:02d}_WDATA_PARITY"] = 0 # FIXME: Manage parity?
            self.hbm_params[f"i_AXI_{i:02d}_WVALID"]       = axi.w.valid
            self.hbm_params[f"o_AXI_{i:02d}_WREADY"]       = axi.w.ready

            # B Channel.
            self.hbm_params[f"o_AXI_{i:02d}_BID"]          = axi.b.id
            self.hbm_params[f"o_AXI_{i:02d}_BRESP"]        = axi.b.resp
            self.hbm_params[f"o_AXI_{i:02d}_BVALID"]       = axi.b.valid
            self.hbm_params[f"i_AXI_{i:02d}_BREADY"]       = axi.b.ready

            # AR Channel.
            self.hbm_params[f"i_AXI_{i:02d}_ARADDR"]       = axi.ar.addr
            self.hbm_params[f"i_AXI_{i:02d}_ARBURST"]      = axi.ar.burst
            self.hbm_params[f"i_AXI_{i:02d}_ARID"]         = axi.ar.id
            self.hbm_params[f"i_AXI_{i:02d}_ARLEN"]        = axi.ar.len
            self.hbm_params[f"i_AXI_{i:02d}_ARSIZE"]       = axi.ar.size
            self.hbm_params[f"i_AXI_{i:02d}_ARVALID"]      = axi.ar.valid
            self.hbm_params[f"o_AXI_{i:02d}_ARREADY"]      = axi.ar.ready

            # R Channel.
            self.hbm_params[f"o_AXI_{i:02d}_RDATA_PARITY"] = Open() # FIXME: Manage parity?
            self.hbm_params[f"o_AXI_{i:02d}_RDATA"]        = axi.r.data
            self.hbm_params[f"o_AXI_{i:02d}_RID"]          = axi.r.id
            self.hbm_params[f"o_AXI_{i:02d}_RLAST"]        = axi.r.last
            self.hbm_params[f"o_AXI_{i:02d}_RRESP"]        = axi.r.resp
            self.hbm_params[f"o_AXI_{i:02d}_RVALID"]       = axi.r.valid
            self.hbm_params[f"i_AXI_{i:02d}_RREADY"]       = axi.r.ready

        # APB --------------------------------------------------------------------------------------
        # FIXME: Connect to CSR or Wishbone.
        apb_complete = Signal(stacks)
        for i in range(stacks):
            self.hbm_params[f"i_APB_{i:1d}_PWDATA"]  = 0
            self.hbm_params[f"i_APB_{i:1d}_PADDR"]   = 0
            self.hbm_params[f"i_APB_{i:1d}_PENABLE"] = 0
            self.hbm_params[f"i_APB_{i:1d}_PSEL"]    = 0
            self.hbm_params[f"i_APB_{i:1d}_PWRITE"]  = 0

            self.hbm_params[f"o_APB_{i:1d}_PRDATA"]  = Open()
            self.hbm_params[f"o_APB_{i:1d}_PREADY"]  = Open()
            self.hbm_params[f"o_APB_{i:1d}_PSLVERR"] = Open()

            self.hbm_params[f"o_apb_complete_{i:1d}"] = apb_complete[i]
        self.comb += self.init_done.status.eq(apb_complete == (2**stacks - 1))

        # Temperature ------------------------------------------------------------------------------
        for i in range(stacks):
            self.hbm_params[f"o_DRAM_{i:1d}_STAT_CATTRIP"] = Open()
            self.hbm_params[f"o_DRAM_{i:1d}_STAT_TEMP"]    = Open()

    def get_ip_config(self):
        def mhz(freq):
            return f"{freq/1e6:g}"
        device     = self.platform.device.lower()
        stack_size = 8 if any(device.startswith(d) for d in USPHBM2_8H_DEVICES) else 4
        config = {
            # Stacks.
            "USER_HBM_STACK"   : self.stacks,
            "USER_HBM_DENSITY" : f"{stack_size*self.stacks}GB",
        }
        for i in range(self.stacks):
            config.update({
                # HBM / Ref Clks (1 PLL per stack).
                f"USER_HBM_TCK_{i}"     : mhz(self.hbm_clk_freq),
                f"USER_HBM_REF_CLK_{i}" : mhz(self.ref_clk_freq),
                # APB Clk.
                f"USER_APB_PCLK_{i}"    : mhz(self.apb_clk_freq),
            })
        # AXI Clk.
        config["USER_AXI_CLK_FREQ"] = mhz(self.axi_clk_freq)
        if self.stacks == 2:
            config["USER_AXI_CLK1_FREQ"] = mhz(self.axi_clk_freq)

        # User/Custom config.
        config.update(self.ip_config)
        return config

    def add_sources(self, platform):
        # Generate IP from parameters.
        if self.generate_ip:
            ip_tcl = []
            ip_tcl.append(f"create_ip -vendor xilinx.com -library ip -name hbm -module_name {self.hbm_name}")
            ip_tcl.append(f"set obj [get_ips {self.hbm_name}]")
            ip_tcl.append("set_property -dict [list \\")
            for config, value in self.get_ip_config().items():
                ip_tcl.append("CONFIG.{} {} \\".format(config, "{{" + str(value) + "}}"))
            ip_tcl.append("] $obj")
            ip_tcl.append("generate_target all $obj")
            ip_tcl.append("synth_ip $obj")
            platform.toolchain.pre_synthesis_commands += ip_tcl
        # Use provided/default .xci.
        else:
            xci_path = self.xci_path
            if xci_path is None:
                xci_path = os.path.join(os.getcwd(), "ip", "hbm", self.hbm_name + ".xci")
            platform.add_ip(os.path.abspath(xci_path))

    def do_finalize(self):
        self.add_sources(self.platform)
        self.specials += Instance(self.hbm_name, **self.hbm_params)


# Fixed-profile Primitive HBM2 Wrapper ---------------------------------------------------------------

MANAGEMENT_BASE = 0x3000
MANAGEMENT_REGISTERS = (
    ("identity", "ro"),
    ("version", "ro"),
    ("address", "rw"),
    ("write_data", "rw"),
    ("command", "wo"),
    ("status", "ro"),
    ("read_data", "ro"),
    ("last_address", "ro"),
    ("temperature0", "rw"),
    ("temperature1", "rw"),
    ("raw_temperature", "ro"),
    ("inject_timeout", "wo"),
)


class PrimitiveHBM2(USPHBM2):
    """Bind the maintained primitive/control RTL to LiteX interfaces.

    The target can map ``axi`` to SYS250 or provide AXI400 with SYS200.
    Clock selection still requires fresh implementation and hardware checks.
    ``memory_ready`` is SYS-facing; ``memory_ready_axi`` is synchronized for
    native admission and does not reset the AXI-domain control state.
    With ``with_native_ready=False``, ``memory_ready_axi`` is constant zero
    and its synchronizer is absent. Use this only when SYS owns admission
    and the application transport tracks accepted work across its own CDC.
    ``control_bus`` is a raw 32-bit, 14-bit word-addressed CSR interface. It
    must be connected to the SoC's CSR interconnect without a second CSR bank
    decoder: the control RTL already selects bank 6 and registers responses.
    The bundled RTL is used by default; ``rtl_dir`` may override it with a
    directory containing fk33_hbm_primitive.sv and fk33_hbm_control.sv.
    The fixed FK33 profile requires software initialization before READY.
    Withdrawing READY resets AXI interfaces; retention is not qualified.
    The 200 MHz reference profile boots at 900 MHz HBM. Software may select
    800/900 MHz only through complete destructive reset and recalibration.
    The primitive model remains at its supported 1800 MT/s maximum.
    """

    def __init__(self, platform, hbm_ip_name="hbm_0", *, rtl_dir=None,
                 stacks=2, hbm_clk_freq=900e6, axi_clk_freq=250e6, sys_clk_freq=250e6,
                 ref_clk_freq=200e6, apb_clk_freq=100e6, with_native_ready=True):
        expected = {
            "stacks": (stacks, 2),
            "hbm_clk_freq": (hbm_clk_freq, 900e6),
            "ref_clk_freq": (ref_clk_freq, 200e6),
            "apb_clk_freq": (apb_clk_freq, 100e6),
        }
        for name, (actual, required) in expected.items():
            if actual != required:
                raise ValueError(f"FK33 primitive profile requires {name}={required!r}")
        if (sys_clk_freq, axi_clk_freq) not in ((250e6, 250e6), (200e6, 400e6)):
            raise ValueError("FK33 primitive clocks require SYS250/AXI250 or SYS200/AXI400")
        self.sys_clk_freq = sys_clk_freq
        if hbm_ip_name != "hbm_0":
            raise ValueError("The tested primitive RTL module is named hbm_0")
        self.rtl_dir = (Path(rtl_dir).resolve() if rtl_dir is not None
                        else Path(__file__).resolve().with_suffix(""))
        self.rtl_sources = tuple(self.rtl_dir / name for name in (
            "fk33_hbm_primitive.sv", "fk33_hbm_control.sv"))
        for path in self.rtl_sources:
            if not path.is_file():
                raise FileNotFoundError(path)

        # Reuse the upstream public AXI and init_done interfaces. Its source
        # loader is overridden below, so no XCI path is read or generated.
        super().__init__(platform, hbm_ip_name=hbm_ip_name,
                         stacks=stacks, hbm_clk_freq=hbm_clk_freq,
                         axi_clk_freq=axi_clk_freq, ref_clk_freq=ref_clk_freq,
                         apb_clk_freq=apb_clk_freq, generate_ip=False,
                         xci_path=None, ip_config={})

        # Constants describe the boot profile and supported tool model, not a
        # measured clock. Firmware reports the active destructive profile.
        self.operating_clock_mhz = CSRConstant(900, name="operating_clock_mhz")
        self.operating_data_rate_mt_s = CSRConstant(1800, name="operating_data_rate_mt_s")
        self.model_data_rate_mt_s = CSRConstant(1800, name="model_data_rate_mt_s")
        self.experimental_overclock = CSRConstant(0, name="experimental_overclock")

        self.control_bus = csr_bus.Interface(data_width=32, address_width=14)
        self.sw_ready = Signal(2, name="hbm_sw_ready")
        self.memory_ready = Signal(name="hbm_memory_ready")
        self.comb += self.memory_ready.eq(self.sw_ready == 3)
        self.memory_ready_axi = Signal(name="hbm_memory_ready_axi")
        if with_native_ready:
            self.specials += MultiReg(self.memory_ready, self.memory_ready_axi, "axi")
        else:
            self.comb += self.memory_ready_axi.eq(0)
        self.sw_temperature = [Signal(7, name=f"hbm_sw_temperature_{i}") for i in range(2)]
        self.sw_temperature_valid = Signal(2, name="hbm_sw_temperature_valid")
        self.raw_temperature = [Signal(3, name=f"hbm_raw_temperature_{i}") for i in range(2)]
        self.raw_cattrip = Signal(2, name="hbm_raw_cattrip")

        # All application AXI interfaces retain their own domain reset.
        for channel in range(32):
            self.hbm_params[f"i_AXI_{channel:02d}_ARESET_N"] = ~ResetSignal("axi")

        # The two stack ports share master address/data/control but have
        # separate selects and complete 32-bit return buses.
        address = Signal(22, name="hbm_apb_address")
        write_data = Signal(32, name="hbm_apb_write_data")
        write = Signal(name="hbm_apb_write")
        enable = Signal(name="hbm_apb_enable")
        select = Signal(2, name="hbm_apb_select")
        for stack in range(2):
            apb = Record([
                ("paddr", 22), ("pwdata", 32), ("pwrite", 1),
                ("psel", 1), ("penable", 1), ("prdata", 32),
                ("pready", 1), ("pslverr", 1),
            ], name=f"hbm_apb{stack}")
            self.apb.append(apb)
            self.comb += [apb.paddr.eq(address), apb.pwdata.eq(write_data),
                          apb.pwrite.eq(write), apb.penable.eq(enable),
                          apb.psel.eq(select[stack])]
            for port in ("paddr", "pwdata", "pwrite", "psel", "penable"):
                self.hbm_params[f"i_APB_{stack}_{port.upper()}"] = getattr(apb, port)
            for port in ("prdata", "pready", "pslverr"):
                self.hbm_params[f"o_APB_{stack}_{port.upper()}"] = getattr(apb, port)

            self.hbm_params[f"o_raw_temperature_{stack}"] = self.raw_temperature[stack]
            self.hbm_params[f"i_sw_temperature_{stack}"] = self.sw_temperature[stack]
            self.hbm_params[f"o_DRAM_{stack}_STAT_CATTRIP"] = self.raw_cattrip[stack]

            # Preserve the reference's software-visible status names and
            # two-stage sampling. These Celsius values are compatibility
            # shadows from software, not decoded three-bit primitive codes.
            raw_celsius = Signal(7, name=f"hbm_stack{stack}_celsius_shadow")
            self.hbm_params[f"o_DRAM_{stack}_STAT_TEMP"] = raw_celsius
            for suffix, width, value in (
                ("temperature", 7, raw_celsius),
                ("cattrip", 1, self.raw_cattrip[stack]),
            ):
                name = f"stack{stack}_{suffix}"
                status = CSRStatus(width, name=name, description=f"HBM stack {stack} {suffix}.")
                setattr(self, name, status)
                self.specials += MultiReg(value, status.status, "sys")

        self.hbm_params["i_sw_ready"] = self.sw_ready
        self.hbm_params["i_sw_temperature_valid"] = self.sw_temperature_valid
        self.specials += Instance("fk33_hbm_control",
            # Preserve the default watchdog duration and 15-second thermal age
            # when changing SYS; APB remains at 100 MHz with its own timeout.
            p_SYS_TIMEOUT=(800064 * int(sys_clk_freq) + 250000000 - 1) // 250000000,
            p_TEMPERATURE_MAX_AGE=15 * int(sys_clk_freq),
            i_sys_clk=ClockSignal("sys"), i_sys_rst=ResetSignal("sys"),
            i_apb_clk=ClockSignal("apb"), i_apb_rst=ResetSignal("apb"),
            i_csr_adr=self.control_bus.adr, i_csr_dat_w=self.control_bus.dat_w,
            i_csr_we=self.control_bus.we, o_csr_dat_r=self.control_bus.dat_r,
            i_trip=self.raw_cattrip,
            i_raw_temp_0=self.raw_temperature[0], i_raw_temp_1=self.raw_temperature[1],
            o_sw_ready=self.sw_ready,
            o_sw_temperature_0=self.sw_temperature[0], o_sw_temperature_1=self.sw_temperature[1],
            o_sw_temperature_valid=self.sw_temperature_valid,
            o_paddr=address, o_pwdata=write_data, o_pwrite=write,
            o_psel=select, o_penable=enable,
            i_prdata_0=self.apb[0].prdata, i_prdata_1=self.apb[1].prdata,
            i_pready=Cat(self.apb[0].pready, self.apb[1].pready),
            i_pslverr=Cat(self.apb[0].pslverr, self.apb[1].pslverr))

    def add_sources(self, platform):
        for source in self.rtl_sources:
            platform.add_source(str(source), language="systemverilog")
