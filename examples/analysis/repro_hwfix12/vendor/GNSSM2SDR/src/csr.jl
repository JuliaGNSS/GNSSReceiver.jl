# LiteX CSR access over the litepcie character device.
#
# A port of gnss-m2sdr's `software/m2sdr_csr.py`: register addresses come from
# the gateware's own `csr.csv`, so a rebuild is picked up automatically and
# nothing has to be compiled against generated headers. The only kernel
# interface used is `LITEPCIE_IOCTL_REG`, which the litepcie driver has always
# exposed.

const _IOC_WRITE = UInt32(1)
const _IOC_READ = UInt32(2)

# struct litepcie_ioctl_reg { uint32_t addr; uint32_t val; uint8_t is_write; }
# — 12 bytes with the trailing padding the C struct gets.
const REG_STRUCT_SIZE = 12

_iowr(t::Char, nr::Integer, size::Integer) =
    ((_IOC_READ | _IOC_WRITE) << 30) | (UInt32(size) << 16) | (UInt32(t) << 8) | UInt32(nr)

const LITEPCIE_IOCTL_REG = _iowr('S', 0, REG_STRUCT_SIZE)

"""
    LiteXCSR(csr_csv; device = "/dev/m2sdr0")

Named access to the gateware's CSRs, resolved from `csr_csv` (the `csr.csv`
LiteX emits next to the bitstream). Close it with `close`.
"""
mutable struct LiteXCSR
    fd::RawFD
    regs::Dict{String,Tuple{UInt32,Int}}   # name → (address, width in subregisters)
    bases::Dict{String,UInt32}
    csr_data_width::Int
    const buffer::Vector{UInt8}
    # One ioctl scratch buffer per handle, so concurrent tasks (the dump poller,
    # the NCO writer, a foreground sweep) must serialise on it. Individual CSR
    # accesses are atomic in the driver; this lock only protects the buffer.
    const lock::ReentrantLock
    open::Bool
end

function LiteXCSR(csr_csv::AbstractString; device::AbstractString = "/dev/m2sdr0")
    regs = Dict{String,Tuple{UInt32,Int}}()
    bases = Dict{String,UInt32}()
    csr_data_width = 32
    for line in eachline(csr_csv)
        row = strip.(split(line, ','))
        isempty(row) && continue
        length(row) < 3 && continue
        if row[1] == "csr_register" && length(row) >= 4
            regs[row[2]] = (parse_number(row[3]), parse(Int, row[4]))
        elseif row[1] == "csr_base"
            bases[row[2]] = parse_number(row[3])
        elseif row[1] == "constant" && row[2] == "config_csr_data_width"
            csr_data_width = parse(Int, row[3])
        end
    end
    isempty(regs) && throw(ArgumentError("no csr_register rows found in $csr_csv"))
    fd = ccall(:open, Cint, (Cstring, Cint), device, 2 #= O_RDWR =#)
    fd < 0 && systemerror("open($device)", Libc.errno())
    LiteXCSR(
        RawFD(fd),
        regs,
        bases,
        csr_data_width,
        zeros(UInt8, REG_STRUCT_SIZE),
        ReentrantLock(),
        true,
    )
end

parse_number(s::AbstractString) =
    startswith(s, "0x") ? parse(UInt32, s[3:end]; base = 16) : parse(UInt32, s)

function Base.close(csr::LiteXCSR)
    csr.open || return csr
    ccall(:close, Cint, (Cint,), Base.cconvert(Cint, csr.fd))
    csr.open = false
    csr
end

function _ioctl_reg!(csr::LiteXCSR, addr::UInt32, value::UInt32, is_write::Bool)
    buf = csr.buffer
    @lock csr.lock GC.@preserve buf begin
        p = pointer(buf)
        unsafe_store!(Ptr{UInt32}(p), addr)
        unsafe_store!(Ptr{UInt32}(p + 4), value)
        unsafe_store!(Ptr{UInt8}(p + 8), is_write ? 0x01 : 0x00)
        rc = ccall(
            :ioctl,
            Cint,
            (Cint, Culong, Ptr{UInt8}),
            Base.cconvert(Cint, csr.fd),
            LITEPCIE_IOCTL_REG,
            p,
        )
        rc < 0 && systemerror("ioctl(LITEPCIE_IOCTL_REG)", Libc.errno())
        return unsafe_load(Ptr{UInt32}(p + 4))
    end
end

readl(csr::LiteXCSR, addr::Integer) = _ioctl_reg!(csr, UInt32(addr), UInt32(0), false)
writel(csr::LiteXCSR, addr::Integer, value::Integer) =
    (_ioctl_reg!(csr, UInt32(addr), UInt32(value & 0xFFFFFFFF), true); nothing)

"""
    read(csr, name) -> UInt64

Read a named CSR. A register wider than the CSR bus is split into subregisters,
most significant first, each occupying a full 32-bit MMIO slot but carrying only
`csr_data_width` bits — so the address stride is 4 while the shift is
`csr_data_width`. Assuming 32 there silently returns garbage on a
`csr_data_width = 8` build.
"""
function Base.read(csr::LiteXCSR, name::AbstractString)
    haskey(csr.regs, name) || throw(KeyError(name))
    addr, nwords = csr.regs[name]
    mask = (UInt64(1) << csr.csr_data_width) - 1
    value = UInt64(0)
    for i = 0:(nwords-1)
        value = (value << csr.csr_data_width) | (UInt64(readl(csr, addr + 4 * i)) & mask)
    end
    value
end

function Base.write(csr::LiteXCSR, name::AbstractString, value::Integer)
    haskey(csr.regs, name) || throw(KeyError(name))
    addr, nwords = csr.regs[name]
    v = UInt64(value)
    mask = (UInt64(1) << csr.csr_data_width) - 1
    for i = 0:(nwords-1)
        shift = csr.csr_data_width * (nwords - 1 - i)
        writel(csr, addr + 4 * i, (v >> shift) & mask)
    end
    nothing
end

"""
    read_signed(csr, name, bits = 32) -> Int64

Read a named CSR as a two's-complement signed value of `bits` width.
"""
function read_signed(csr::LiteXCSR, name::AbstractString, bits::Integer = 32)
    v = read(csr, name)
    v & (UInt64(1) << (bits - 1)) != 0 ? Int64(v) - (Int64(1) << bits) : Int64(v)
end

has_register(csr::LiteXCSR, name::AbstractString) = haskey(csr.regs, name)
