push!(LOAD_PATH, "../src/")

using Documenter

# Running `julia --project docs/make.jl` can be very slow locally.
# To speed it up during development, one can use make_local.jl instead.
# The code below checks wether its being called from make_local.jl or not.
const LOCAL = get(ENV, "LOCAL", "false") == "true"

if LOCAL
	include("../src/GNSSReceiver.jl")
	using .GNSSReceiver
else
	using GNSSReceiver
	ENV["GKSwstype"] = "100"
end

DocMeta.setdocmeta!(GNSSReceiver, :DocTestSetup, :(using GNSSReceiver); recursive = true)

makedocs(
	modules = [GNSSReceiver],
	format = Documenter.HTML(),
	sitename = "GNSSReceiver.jl",
	pages = Any[
		"index.md",
		"Examples"=>Any[
			"Examples/FileReading.md"
			"Examples/SoapyReceiver.md"
		],
	],
	doctest = true,
)

deploydocs(
	repo = "https://github.com/JuliaGNSS/GNSSReceiver.jl",
	push_preview = true,
)
