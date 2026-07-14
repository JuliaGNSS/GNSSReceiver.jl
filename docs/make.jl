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
