
-- Projects that include libfibre and also use tup can place a Tuprules.lua file
-- into their root directory with the line `no_libfibre = true` to prevent
-- libfibre from building.
if no_libfibre == true then
    return
end

tup.include('package.lua')

CFLAGS = {'-I./include -fPIC -std=c++11 -DFIBRE_COMPILE -DFIBRE_ENABLE_CLIENT'}
LDFLAGS = {'-static-libstdc++'}

-- Runs the specified shell command immediately (not as part of the dependency
-- graph).
-- Returns the values (return_code, stdout) where stdout has the trailing new
-- line removed.
function run_now(command)
    local handle
    handle = io.popen(command)
    local output = handle:read("*a")
    local rc = {handle:close()}
    if not rc[1] then
        error("failed to invoke "..command)
    end
    return string.sub(output, 0, -2)
end

if tup.getconfig("CC") == "" then
    CC = 'clang++'
    LINKER = 'clang++'
else
    CC = tup.getconfig("CC")
    LINKER = tup.getconfig("CC")
end

function get_bool_config(name, default)
    if tup.getconfig(name) == "" then
        return default
    elseif tup.getconfig(name) == "true" then
        return true
    elseif tup.getconfig(name) == "false" then
        return false
    else
        error(name.." ("..tup.getconfig(name).." must be 'true' or 'false'.")
    end
end

CFLAGS += tup.getconfig("CFLAGS")
LDFLAGS += tup.getconfig("LDFLAGS")
DEBUG = get_bool_config("DEBUG", true)
USE_PKGCONF = get_bool_config("USE_PKGCONF", true)
ENABLE_LIBUSB = get_bool_config("ENABLE_LIBUSB", true)

if USE_PKGCONF and ENABLE_LIBUSB then
    CFLAGS += run_now("pkgconf libusb-1.0 --cflags")
    LDFLAGS += run_now("pkgconf libusb-1.0 --libs")
end

machine = run_now(CC..' -dumpmachine') -- works with both clang and GCC

BUILD_TYPE='-shared'

if string.find(machine, "x86_64.*%-linux%-.*") then
    outname = 'libfibre-linux-amd64.so'
    LDFLAGS += '-lpthread -Wl,--version-script=libfibre.version -Wl,--gc-sections'
    STRIP = not DEBUG
elseif string.find(machine, "arm.*%-linux%-.*") then
	outname = 'libfibre-linux-armhf.so'
    LDFLAGS += '-lpthread -Wl,--version-script=libfibre.version -Wl,--gc-sections'
    STRIP = false
elseif string.find(machine, "x86_64.*-mingw.*") then
    outname = 'libfibre-windows-amd64.dll'
    LDFLAGS += '-lpthread -Wl,--version-script=libfibre.version'
    STRIP = not DEBUG
elseif string.find(machine, "x86_64.*-apple-.*") then
    outname = 'libfibre-macos-x86.dylib'
    STRIP = false
elseif string.find(machine, "wasm.*") then
    outname = 'libfibre-wasm.js'
    STRIP = false
    BUILD_TYPE = ''
else
    error('unknown machine identifier '..machine)
end

LDFLAGS += BUILD_TYPE

if DEBUG then
    CFLAGS += '-O1 -g'
else
    CFLAGS += '-O3' -- TODO: add back -lfto
end

function compile(src_file, obj_file)
    tup.frule{
        inputs={src_file},
        command='^co^ '..CC..' -c %f '..tostring(CFLAGS)..' -fdebug-prefix-map=/Data/Projects/fibre/cpp/build-local=/Data/Projects/fibre/cpp -o %o',
        outputs={obj_file}
    }
end

code_files = fibre_package.core_files

if ENABLE_LIBUSB then
    tup.append_table(code_files, fibre_package.features["LIBUSB"])
    CFLAGS += '-DFIBRE_ENABLE_LIBUSB=1'
end
if get_bool_config("ENABLE_LOGGING", true) then
    tup.append_table(code_files, fibre_package.features["LOGGING"])
end

for _, src_file in pairs(code_files) do
    obj_file = "build/"..src_file:gsub("/","_")..".o"
    object_files += obj_file
    compile(src_file, obj_file)
end

if not STRIP then
    compile_outname=outname
else
    compile_outname=outname..'.fat'
end

if tup.ext(outname) == 'js' then
    extra_outputs = {tup.base(compile_outname)..'.wasm'}
else
    extra_outputs = {}
end

tup.frule{
    inputs=object_files,
    command='^c^ '..LINKER..' %f '..tostring(CFLAGS)..' '..tostring(LDFLAGS)..' -o %o',
    outputs={compile_outname, extra_outputs=extra_outputs}
}

if STRIP then
    tup.frule{
        inputs={compile_outname},
        command='strip --strip-all --discard-all %f -o %o',
        outputs={outname}
    }
end
