using Documenter
using GNSSReceiver

# The GUI screenshot lives once in the repo at `media/output.png` (also used by the
# README). Copy it into the docs assets at build time rather than committing a second
# copy, so there is a single source of truth. The generated copy is git-ignored.
let src = joinpath(@__DIR__, "..", "media", "output.png"),
    dst = joinpath(@__DIR__, "src", "assets", "gui.png")

    mkpath(dirname(dst))
    cp(src, dst; force = true)
end

makedocs(
    sitename = "GNSSReceiver.jl",
    # `gui` is defined in the `Dashboard` submodule (which keeps the Tachikoma namespace out
    # of `GNSSReceiver`) and re-imported into the parent, so list both.
    modules = [GNSSReceiver, GNSSReceiver.Dashboard],
    authors = "JuliaGNSS",
    format = Documenter.HTML(
        prettyurls = get(ENV, "CI", nothing) == "true",
        canonical = "https://JuliaGNSS.github.io/GNSSReceiver.jl",
        # The API reference is one long page on purpose — it is read by search
        # and by `@ref` from the prose pages, and splitting it would scatter the
        # hardware-correlator contract's targets across files. It grew past
        # Documenter's 200 KiB default when the RF-band interface landed
        # (issue #134); raise the ceiling rather than shard the page. The warn
        # threshold stays low so the growth is still visible in the build log.
        size_threshold = 400 * 1024,
        size_threshold_warn = 200 * 1024,
    ),
    pages = [
        "Home" => "index.md",
        "Getting Started" => "getting_started.md",
        "Acquisition & Tracking Parameters" => "parameters.md",
        "Worked Example (Real Data)" => "example.md",
        "Signal Support & Evidence" => "signal_support.md",
        "Custom Receiver Output" => "custom_output.md",
        "Graphical User Interface" => "gui.md",
        "Hardware-Correlator Contract" => "hardware_contract.md",
        "API Reference" => "api.md",
    ],
    checkdocs = :exports,
    # `write_to_file` is re-exported from SignalChannels, so its docstring lives in
    # another module; tolerate that (and any other missing-docs) as a warning.
    warnonly = [:missing_docs],
)

deploydocs(
    repo = "github.com/JuliaGNSS/GNSSReceiver.jl.git",
    devbranch = "main",
    push_preview = true,
)
