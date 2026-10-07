# Which forms of reading a file survive `--trim=safe`? The code tables
# GNSSSignals ships are files; the loop process has to read them somehow.
function read_whole_a(path::String)
    io = open(path, "r")
    n = filesize(io)
    buf = Vector{UInt8}(undef, n)
    read!(io, buf)
    close(io)
    buf
end

function read_whole_b(path::String)
    read(path)
end

function read_whole_c(path::String)
    fd = ccall(:open, Cint, (Cstring, Cint), path, 0)
    fd < 0 && return UInt8[]
    chunks = UInt8[]
    buf = Vector{UInt8}(undef, 1 << 16)
    while true
        n = ccall(:read, Cssize_t, (Cint, Ptr{UInt8}, Csize_t), fd, buf, length(buf))
        n <= 0 && break
        append!(chunks, view(buf, 1:Int(n)))
    end
    ccall(:close, Cint, (Cint,), fd)
    chunks
end

function (@main)(args::Vector{String})::Cint
    path = args[1]
    a = read_whole_a(path)
    Core.println(Core.stdout, length(a))
    b = read_whole_b(path)
    Core.println(Core.stdout, length(b))
    c = read_whole_c(path)
    Core.println(Core.stdout, length(c))
    # Printing: one value per call only.
    Core.println(Core.stdout, "done")
    return Cint(0)
end
