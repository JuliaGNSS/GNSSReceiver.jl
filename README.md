# GNSSReceiver (WIP)

![Exemplary output](media/output.png)

```julia
using GNSSSignals, Tracking, GNSSReceiver
using Unitful:Hz
gpsl1 = GPSL1()
files = map(i -> "antenna$i.dat", 1:4) # Could also be a single file
measurement_channel = read_files(files, 5e6Hz)
# Let's receive GPS L1 signals
data_channel, gui_channel = receive(measurement_channel, gpsl1, 5e6Hz, num_ants = NumAnts(4))
# Hook up GUI
GNSSReceiver.gui(gui_channel)
# Save interesting data
save_data(data_channel, filename = "data.jld2")
```

That's it. You can watch the GUI being updated in real time.