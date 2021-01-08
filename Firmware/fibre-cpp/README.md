# fibre-cpp

This directory provides the C++ reference implementation of [Fibre](https://github.com/samuelsadok/fibre). Its home is located [here](https://github.com/samuelsadok/fibre/tree/master/cpp). There's also a standalone repository for this directory [here](https://github.com/samuelsadok/fibre-cpp).

## Overview

There are two approaches to include Fibre in your project:

  1. **Embedding fibre-cpp:** Your application's build process includes the source code files of fibre-cpp. Your application uses Fibre's C++ API to interact with Fibre. This is the recommended approach for embedded systems.
  2. **Linking to libfibre:** Your application links to a separately compiled library `libfibre` and uses Fibre's C API to interact with this library. You can obtain precompiled binaries on the main project's [release page](https://github.com/samuelsadok/fibre/releases). This is the recommended approach for desktop systems, where you want all backends enabled, because you can avoid the burden of collecting build dependencies. _Note:_ currently only the client role is supported with this approach. That means you can use it to discover and access remote objects but you cannot use it to expose local objects yet.

## Configuring fibre-cpp

Various preprocessor defines can be used to customize Fibre:

 - `FIBRE_ENABLE_SERVER={0|1}` (_default 0_): Enable support for exposing objects to remote peers.
 - `FIBRE_ENABLE_CLIENT={0|1}` (_default 0_): Enable support for discovering and using objects exposed by remote peers.
 - `FIBRE_ENABLE_EVENT_LOOP={0|1}` (_default 0_): Enable the builtin event loop implementation. Not supported on all platforms.
 - `FIBRE_ALLOW_HEAP={0|1}` (_default 0_): Allow Fibre to allocate memory on the heap using `malloc` and `free`. If this option is disabled only one Fibre instance can be opened. Currently `FIBRE_ENABLE_CLIENT` (and several other options) cannot be used together with this option.
 - `FIBRE_MAX_LOG_VERBOSITY={0...5}` (_default 5_): The maximum log verbosity that will be compiled into the binary. In embedded systems it's recommended to set this to 0 to reduce binary size. On platforms that support environment variables the actual run time log verbosity can be changed by setting the environment variable `FIBRE_LOG={0...5}`.
 - `FIBRE_DEFAULT_LOG_VERBOSITY={0...5}` (_default 5_): The default log verbosity that will be used unless overridden by other means (for instance through the environment variables).
 - `FIBRE_ENABLE_LIBUSB_BACKEND={0|1}` (_default 0_): Enable libusb backend for host side USB support. This requires `FIBRE_ALLOC_HEAP=1`.
 - `FIBRE_ENABLE_TCP_CLIENT_BACKEND={0|1}` (_default 0_): Enable TCP client backend. This requires `FIBRE_ALLOC_HEAP=1`.
 - `FIBRE_ENABLE_TCP_SERVER_BACKEND={0|1}` (_default 0_): Enable TCP server backend. This requires `FIBRE_ALLOC_HEAP=1`.

## Adding fibre-cpp to your application's build process

If your application uses [tup](http://gittup.org/tup/) as build system you can directly call the function `get_fibre_package()` in [package.lua](package.lua) as part of your build process. This function spits out a list of code files and compiler flags needed to compile fibre-cpp for a given configuration. Refer to [package.lua](package.lua) for more details.

If your application doesn't use tup, you have to manually check which code files you need.

## Using fibre-cpp

Currently there's no nice walkthrough for this but here are two applications that you can use as an example:

 - The [ODrive Firmware](https://github.com/madcowswe/ODrive/tree/devel/Firmware)
 - The [test server](https://github.com/samuelsadok/fibre/blob/devel/test/test_server.cpp)

## Configuring `libfibre`

A file called tup.config can be placed in this directory to customize the build. See [configs](configs/) for examples.

## Compiling `libfibre`

Before you compile libfibre yourself consider if the [official releases](https://github.com/samuelsadok/fibre/releases) may be suitable for you instead.

The recommended way for compiling libfibre is using Docker. You can use the same docker container to cross-compile for all supported targets.

However if you're actively developing fibre you may want to compile natively for faster compile times.

### Docker

The following example compiles libfibre for the `linux-amd64` target. Refer to the "configs/" folder for a list of supported targets. You can also add new targets there but you may need to modify the Dockerfile to include tooling for your new target.

```
docker build -t fibre-compiler .
docker run -it -v /tmp/build:/build/cpp/build --entrypoint bash fibre-compiler -c "rm -rd /build/cpp/build/*"
docker run -it -v "$(pwd)":/build -v /tmp/build:/build/build -w /build fibre-compiler configs/linux-amd64.config
```

The output file is now located under `/tmp/build/libfibre-linux-amd64.so` on your host system.

If something fails you can enter the container interactively with `docker run -it -v "$(pwd)":/build -v /tmp/build:/build/build -w /build --entrypoint bash fibre-compiler`.

### Windows
  1. Download MinGW from [here](https://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download) and install it.
  2. Add `C:\Program Files\mingw-w64\x86_64-8.1.0-posix-seh-rt_v6-rev0\mingw64\bin` (or similar) to your `PATH` environment variable.
  3. Download the libusb binaries from [here](`https://github.com/libusb/libusb/releases/download/v1.0.23/libusb-1.0.23.7z`) and unpack them to `third_party/libusb-windows` (such that the file `third_party/libusb-windows/libusb-1.0.23/MinGW64/static/libusb-1.0.a` exists).
  4. Navigate to this directory and run `make`

### Ubuntu
  1. `sudo apt-get install libusb-1.0-0-dev`
  2. Navigate to this directory and run `make`

### macOS
  1. `brew install libusb`
  2. Navigate to this directory and run `make`

## Using `libfibre`

The API is documented in [libfibre.h](include/fibre/libfibre.h).

To compile your application you need to link against the libfibre binary (`-L/path/to/libfibre.so`) and add "libfibre.h" to your include path under a folder named "fibre", e.g. `-I/path/to/fibre-cpp/include`.


## Notes for Contributors

 - Fibre currently targets C++11 to maximize compatibility with other projects
 - Notes on platform independent programming:
   - Don't use the keyword `interface` (defined as a macro on Windows in `rpc.h`)
