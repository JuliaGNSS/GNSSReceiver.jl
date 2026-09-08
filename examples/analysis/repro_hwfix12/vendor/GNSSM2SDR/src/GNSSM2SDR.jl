"""
    GNSSM2SDR

The LiteX-M2SDR vendor package for GNSSReceiver.jl's hardware-correlator
interface (GNSSReceiver.jl#107).

The FPGA — gateware from [gnss-m2sdr](https://github.com/JuliaGNSS/gnss-m2sdr) —
downconverts and correlates; this package moves its correlator dumps to the host
and the host's NCO updates back, so `GNSSReceiver.receive` can run the tracking
loops without correlating a single sample on the CPU. Acquisition, decoding and
PVT are unchanged and still run off the raw stream.

```julia
raw = # a SignalChannel of raw I/Q from the board (SoapySDR, LiteXM2SDR.jl, …)
sdr = M2SDRCorrelator("build/…/csr.csv", raw; fs = 4e6u"Hz", n_channels = 4)
start!(sdr)
data = receive(sdr, GPSL1CA(), 4e6u"Hz")
```

The raw stream must be running before `start!` and must keep running: the
tracking bank observes the RX datapath non-intrusively, so it only sees samples
while DMA0 drains.
"""
module GNSSM2SDR

using GNSSReceiver
using GNSSSignals
using PipeChannels: PipeChannel
using SignalChannels: SignalChannel
using StaticArrays: SVector
using Tracking
using Unitful
using Unitful: Hz

export M2SDRCorrelator,
    LiteXCSR,
    GNSSBank,
    GNSSBankChannel,
    detect_num_channels,
    start!,
    stop!,
    sample_count,
    apply_status,
    applied_at,
    overflow,
    clear_overflow!

include("csr.jl")
include("dma.jl")
include("bank.jl")
include("record.jl")
include("sdr.jl")

end # module GNSSM2SDR
