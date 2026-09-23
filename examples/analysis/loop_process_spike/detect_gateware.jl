include(joinpath(@__DIR__, "device", "csr.jl"))
for csv in ARGS
    try
        csr = LiteXCSR(csv)
        v = has_register(csr, "gnss_version") ? read(csr, "gnss_version") : 0
        caps = read(csr, "gnss_capabilities")
        n = 0
        while has_register(csr, "gnss_ch$(n)_control"); n += 1; end
        println(basename(dirname(csv)), ": version csr=", v & 0xFF, " record=", (v >> 8) & 0xFF,
                " caps.n_channels=", caps & 0xFF, " csv_channels=", n, " sample_count=", read(csr, "gnss_sample_count"))
        close(csr)
    catch e
        println(basename(dirname(csv)), ": ", sprint(showerror, e))
    end
end
