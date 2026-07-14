using Documenter
using GNSSReceiver

makedocs(
    sitename = "GNSSReceiver.jl",
    modules = [GNSSReceiver],
    authors = "JuliaGNSS",
    format = Documenter.HTML(
        prettyurls = get(ENV, "CI", nothing) == "true",
        canonical = "https://JuliaGNSS.github.io/GNSSReceiver.jl",
    ),
    pages = [
        "Home" => "index.md",
        "Getting Started" => "getting_started.md",
        "Acquisition & Tracking Parameters" => "parameters.md",
        "Worked Example (Real Data)" => "example.md",
        "Graphical User Interface" => "gui.md",
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
