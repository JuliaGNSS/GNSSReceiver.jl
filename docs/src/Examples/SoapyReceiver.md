# SDR Stream

```julia
# include your SDR driver package here
# e.g.
using SoapyRTLSDR_jll
# using SoapyLMS7_jll
# using SoapyBladeRF_jll
# using SoapyUHD_jll

using GNSSReceiver, GNSSSignals, Unitful, Tracking, SoapySDR

# You'll might want to run it twice for optimal performance.
gnss_receiver_gui(;
    system = GPSL1(),
    sampling_freq = 2e6u"Hz",
    acquisition_time = 4u"ms", # A longer time increases the SNR for satellite acquisition, but also increases the computational load. Must be longer than 1ms
    run_time = 40u"s",
    num_ants = Tracking.NumAnts(1), # Number of antenna channels
    dev_args = first(Devices()) # Select device (e.g. first device)
)
```
