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
data_channel, gui_channel = receive(measurement_channel, gpsl1, sampling_freq, num_ants = NumAnts(4))
# Hook up GUI
GNSSReceiver.gui(gui_channel)
# Save interesting data
save_data(data_channel, filename = "data.jld2")
```

That's it. You can watch the GUI being updated in real time.