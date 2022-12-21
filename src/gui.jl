using UnicodePlots, Term
import REPL

struct GUIData{S<:SatelliteDataOfInterest}
    sat_data::Dict{Int,S}
    pvt::PVTSolution
    runtime::typeof(1.0u"s")
end

const PRNMARKERS = (
    '\U2780',
    '\U2781',
    '\U2782',
    '\U2783',
    '\U2784',
    '\U2785',
    '\U2786',
    '\U2787',
    '\U2788',
    '\U2789',
    '\U246A',
    '\U246B',
    '\U246C',
    '\U246D',
    '\U246E',
    '\U246F',
    '\U2470',
    '\U2471',
    '\U2472',
    '\U2473',
    '\U3251',
    '\U3252',
    '\U3253',
    '\U3254',
    '\U3255',
    '\U3256',
    '\U3257',
    '\U3258',
    '\U3259',
    '\U325A',
    '\U325B',
    '\U325C',
    '\U325D',
    '\U325E',
    '\U325F',
)

function get_gui_data_channel(
    data_channel::Channel{<:ReceiverDataOfInterest},
    push_gui_data_roughly_every = 500u"ms",
)
    gui_data_channel = Channel{GUIData}()
    last_gui_output = 0.0u"ms"
    first = true
    Base.errormonitor(
        Threads.@spawn begin
            consume_channel(data_channel) do data
                if (data.runtime - last_gui_output) > push_gui_data_roughly_every || first
                    push!(gui_data_channel, GUIData(data.sat_data, data.pvt, data.runtime))
                    last_gui_output = data.runtime
                    first = false
                end
            end
            close(gui_data_channel)
        end
    )
    gui_data_channel
end

_cursor_hide(io::IO) = print(io, "\x1b[?25l")
_cursor_show(io::IO) = print(io, "\x1b[?25h")
panel(plot; kw...) = Panel(string(plot; color = true); fit = true, kw...)

function gui(gui_data_channel, io::IO = stdout; construct_gui_panels = construct_gui_panels)
    terminal = REPL.Terminals.TTYTerminal("", stdin, io, stderr)
    num_dots = 0
    consume_channel(gui_data_channel) do gui_data
        panels = construct_gui_panels(gui_data, num_dots)
        num_dots = mod(num_dots + 1, 4)
        out = string(panels)
        REPL.Terminals.clear(terminal)
        _cursor_hide(io)
        println(io, out)
        _cursor_show(io)
    end
end

function construct_gui_panels(gui_data, num_dots)
    prn_strings = string.(keys(gui_data.sat_data))
    cn0s = map(x -> round(10 * log10(linear(x.cn0 == (Inf)u"Hz" ? 1u"Hz" : x.cn0) / u"Hz"); digits = 1), values(gui_data.sat_data))
    colors = map(x -> x.is_healthy ? :green : :red, values(gui_data.sat_data))
    pvt = gui_data.pvt
    sat_doa_panel_title = "Satellite Direction-of-Arrival (DOA)"
    position_panel_title = "User position"
    not_enough_sats_text = "Not enough satellites to calculate position."
    cn0_panel_title = "Carrier-to-Noise-Density-Ratio (CN0)"
    panels =
        !isempty(prn_strings) ?
        panel(
            barplot(
                prn_strings, cn0s;
                color = colors,
                ylabel = "Satellites",
                xlabel = "Carrier-to-Noise-Density-Ratio (CN0) [dBHz]",
            );
            fit = true,
            title = cn0_panel_title,
        ) :
        Panel(
            "Searching for satellites$(repeat(".", num_dots))";
            title = cn0_panel_title,
            width = length(cn0_panel_title) + 5,
        )
    if !isnothing(pvt.time)
        sat_enus = map(sat_pos -> get_sat_enu(pvt.position, sat_pos), pvt.sat_positions)
        azs = map(x -> x.θ, sat_enus)
        els = map(x -> x.ϕ, sat_enus)
        prn_markers = map(prn -> PRNMARKERS[prn], pvt.used_sats)
        panels *= panel(
            polarplot(
                azs,
                els * 180 / π;
                rlim = (0, 90),
                scatter = true,
                marker = prn_markers,
            );
            fit = true,
            title = sat_doa_panel_title,
        )
        lla = get_LLA(pvt)
        panels *= Panel(
            "Latitude: $(lla.lat)\nLongitude: $(lla.lon)\nAltitude: $(lla.alt)\nCompact:$(lla.lat),$(lla.lon)\nGoogle: https://www.google.com/maps/search/$(lla.lat),$(lla.lon)";
            fit = true,
            title = position_panel_title,
        )
    elseif length(prn_strings) < 4
        panels *= Panel(not_enough_sats_text; title = sat_doa_panel_title, fit = true)
        panels *= Panel(not_enough_sats_text; title = position_panel_title, fit = true)
    else
        decoding_text = "Decoding satellites$(repeat(".", num_dots))"
        panels *= Panel(
            decoding_text;
            title = sat_doa_panel_title,
            width = length(not_enough_sats_text) + 5,
        )
        panels *= Panel(
            decoding_text;
            title = position_panel_title,
            width = length(not_enough_sats_text) + 5,
        )
    end
    panels
end