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
# save_data(data_channel1, filename = "data.jld2")
# gui_channel = get_gui_data_channel(data_channel)
# GNSSReceiver.gui(gui_channel)
```

That's it. You can watch the GUI being updated in real time.