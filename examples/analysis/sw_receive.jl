using GNSSReceiver, GNSSSignals, Tracking, Unitful, Printf
using Unitful: Hz, s, ms
using GNSSReceiver: read_files
cap = ARGS[1]; ant0 = replace(cap, ".bin" => "_ant0.bin")
prns = length(ARGS) >= 2 ? parse.(Int, split(ARGS[2], ",")) : [14, 21, 30, 11]
if_hz = length(ARGS) >= 3 ? parse(Float64, ARGS[3]) : -5090.0
if !isfile(ant0)
    open(cap) do io; open(ant0, "w") do out
        while !eof(io)
            raw = read(io, 8 * 400_000); w = reinterpret(Int16, raw)
            n = length(w) ÷ 4
            write(out, [Complex{Int16}(w[4k-3], w[4k-2]) for k in 1:n])
        end
    end end
end
nsamp = filesize(ant0) ÷ 4
println("samples: $nsamp  ($(nsamp/4e6) s)  PRNs $prns  IF $if_hz Hz")
ch = read_files(ant0, 8000, nsamp - 16000)
data = GNSSReceiver.receive(ch, GPSL1CA(), 4e6Hz; prns, interm_freq = if_hz * Hz,
                            max_meas = 2^11, pvt_update_interval = 500ms)
for d in data
    sats = sort(collect(pairs(d.sat_data)); by = first)
    str = join([@sprintf("%2d:%4.1f%s%s", k[2], ustrip(v.cn0), v.is_in_lock ? "L" : "-", v.is_ranging_ready ? "R" : "-") for (k, v) in sats], " ")
    @printf("t=%5.1fs  %s\n", ustrip(s, d.runtime), str)
end
