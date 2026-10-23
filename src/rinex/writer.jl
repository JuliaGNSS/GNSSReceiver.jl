# Drain the record channel into the RINEX files. All file I/O — and all of RINEXParser's
# state — lives here, off the tracking loop.
function spawn_rinex_writer(channel::Channel{RinexRecord}, obs_writer, nav_writer)
    Base.errormonitor(
        Threads.@spawn try
            for record in channel
                if !isnothing(nav_writer)
                    # The header may still be replaced until the first ephemeris is written,
                    # which is how the ionosphere and UTC records — broadcast minutes after
                    # the first ephemeris — reach the file at all. An ephemeris that does
                    # not fit its fields is left out by the writer (`overflow = :skip`).
                    header_written(nav_writer) ||
                        set_header!(nav_writer, record.nav_header)
                    foreach(
                        ephemeris -> write_ephemeris!(nav_writer, ephemeris),
                        record.ephemerides,
                    )
                end
                (isnothing(obs_writer) || isnothing(record.epoch)) && continue
                # The receiver's own position and the broadcast leap seconds are known no
                # earlier than the first epoch — which is the epoch that writes the header,
                # so it can still take them.
                header_written(obs_writer) || set_header!(
                    obs_writer;
                    approx_position = record.approx_position,
                    leap_seconds = record.leap_seconds,
                )
                write_epoch!(obs_writer, record.epoch)
            end
        finally
            # Close the channel as well: on a write failure the producer would otherwise
            # block on a full channel forever, stalling the tracking loop.
            close(channel)
            foreach(close, filter(!isnothing, (obs_writer, nav_writer)))
        end
    )
end
