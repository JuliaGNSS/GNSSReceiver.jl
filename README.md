# GNSSReceiver (WIP)

![Exemplary output](media/output.png)

Currently, this includes the Manifest.toml because this implementation depends on pending pull requests.

## Installation

Git clone GNSSReceiver.jl

```julia
julia> ;
shell> cd folder/GNSSReceiver.jl
julia> ]
pkg> activate .
(GNSSReceiver) pkg> instantiate
```

## Usage

### Example to read from file

```julia
using GNSSSignals, Tracking, GNSSReceiver, Unitful
using Unitful:Hz, ms
gpsl1 = GPSL1()
files = map(i -> "antenna$i.dat", 1:4) # Could also be a single file for a single antenna channel
sampling_freq = 5e6Hz
# The number of samples must be integer multiples of 1ms.
# The number of samples determines the length of the signal that
# is passed to the acquisition of the satellites.
# Higher values result into higher chance of acquisition, but also
# demand a larger computing power.
num_samples = Int(upreferred(sampling_freq * 4ms))
measurement_channel = read_files(files, num_samples, type = Complex{Int16})
# Let's receive GPS L1 signals
data_channel = receive(measurement_channel, gpsl1, sampling_freq, num_ants = NumAnts(4))
# Get gui channel from data channel
gui_channel = get_gui_data_channel(data_channel)
# Hook up GUI
GNSSReceiver.gui(gui_channel)
# If you'd like to save the data as well, you will have to split the data channel:
# data_channel1, data_channel2 = tee(data_channel)
# data_task = @async save_data(data_channel1)
# gui_channel = get_gui_data_channel(data_channel2)
# GNSSReceiver.gui(gui_channel)
# data = fetch(data_task)
```

That's it. You can watch the GUI being updated in real time.

### Example to read from SDR


```julia
using GNSSSignals, Tracking, GNSSReceiver, Unitful
using SoapySDR, SoapyLMS7_jll
using SoapySDR: dB
gpsl1 = GPSL1()

sampling_freq = 5e6u"Hz"
four_ms_samples = Int(upreferred(sampling_freq * 4u"ms"))
num_samples = Int(upreferred(sampling_freq * 40u"s"))

Device(first(Devices())) do dev
    chan = dev.rx[1]

    chan.frequency = 1575.42u"MHz"
    chan.sample_rate = sampling_freq
    chan.bandwidth = sampling_freq
    chan.gain_mode = true

    stream = SoapySDR.Stream(ComplexF32, dev.rx)
    # Getting samples in chunks of `mtu`
    data_stream = stream_data(stream, num_samples)

    # Satellite acquisition takes about 1s to process on a recent laptop
    # Let's take a buffer length of 5s to be on the save side
    buffer_length = 5u"s"
    buffered_stream = membuffer(data_stream, ceil(Int, buffer_length * sampling_freq / stream.mtu))

    # Resizing the chunks to 4ms in length
    reshunked_stream = rechunk(buffered_stream, four_ms_samples)
    vectorized_stream = vectorize_data(reshunked_stream)

    # Performing GNSS acquisition and tracking
    data_channel = receive(vectorized_stream, gpsl1, sampling_freq)

    gui_channel = get_gui_data_channel(data_channel)

    # Display the GUI and block
    GNSSReceiver.gui(gui_channel)
end
```

