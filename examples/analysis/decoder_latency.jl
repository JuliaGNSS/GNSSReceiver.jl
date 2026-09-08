# Replay a complete 4 MSPS GPS L1 C/A hardware bit log and time each decode.
# Loading GNSSReceiver first includes the live receiver's dependency tree.
# Run in a fresh process; --trace-compile=trace.jl --trace-compile-timing names
# the methods behind any late stall. Parsing and reporting are outside the
# measured decode call. Compilation counters are process-wide.
using GNSSReceiver, GNSSDecoder, GNSSSignals
function trace_decoder(path)
    system = GPSL1CA()
    decoders = Dict{Int,typeof(GNSSDecoderState(system, 1))}()
    origin = Int64(0)
    calls = 0
    max_compile = 0.0
    max_wall = 0.0
    total_compile = 0.0
    Base.cumulative_compile_timing(true)
    for line in eachline(path)
        s = split(line)
        isempty(s) && continue
        if s[1] == "A" && parse(Int, s[4]) > 0
            prn = parse(Int, s[4])
            decoders[prn] = GNSSDecoderState(system, prn)
        elseif s[1] == "T" && origin == 0
            origin = parse(Int64, s[3])
        elseif s[1] == "B"
            prn = parse(Int, s[2])
            bits = parse.(Float32, s[4:end])
            d = decoders[prn]
            before = Base.cumulative_compile_time_ns()[1]
            t = time_ns()
            decoders[prn] = decode(d, bits, length(bits))
            wall = (time_ns() - t) / 1e9
            compiled = (Base.cumulative_compile_time_ns()[1] - before) / 1e9
            calls += 1
            max_compile = max(max_compile, compiled)
            max_wall = max(max_wall, wall)
            total_compile += compiled
            if compiled > 0.005
                println(
                    "sample_s=",
                    (parse(Int64, s[3]) - origin) / 4e6,
                    " prn=",
                    prn,
                    " wall_s=",
                    wall,
                    " compile_s=",
                    compiled,
                )
            end
        end
    end
    println(
        "decode_calls=",
        calls,
        " max_compile_ms=",
        1000max_compile,
        " total_compile_ms=",
        1000total_compile,
        " max_wall_ms=",
        1000max_wall,
    )
end
length(ARGS) == 1 || error("usage: decoder_latency.jl <bits.log>")
trace_decoder(ARGS[1])
