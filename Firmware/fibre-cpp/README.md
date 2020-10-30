# fibre-cpp

This directory provides the C/C++ reference implementation of [Fibre](https://github.com/samuelsadok/fibre). Its home is located [here](https://github.com/samuelsadok/fibre/tree/master/cpp). There's also a standalone repository for this directory [here](https://github.com/samuelsadok/fibre-cpp).

## How to use

The code files in this directory can be embedded directly into user applications by including the C and C++ files in the user application's compile and link process. This is the recommended approach when including Fibre in embedded systems. Currently there's no nice walkthrough for this but you can refer to the [ODrive Firmware](https://github.com/madcowswe/ODrive/tree/devel/Firmware) as an example.

If your application runs on a major Operating System it's recommended that you link to **libfibre** instead. This is essentially fibre-cpp packaged as a nice shared library (aka DLL) accompanied by a header file. We provide precompiled libfibre binaries for most platforms on the main project's [release page](https://github.com/samuelsadok/fibre/releases). The API is documented in [libfibre.h](include/fibre/libfibre.h).

_Note:_ Libfibre's API currently only exposes the client role, not the server role. That means you can use it to discover and access remote objects but you cannot use it to expose local objects yet. For this you have to embed the code files directly.

_Note:_ fibre-cpp makes use of dynamic memory if and only if the client role is used (this limitation might be lifted at some point).

## How to embed

Refer to

## `libfibre` Build Instructions

### Windows
  1. Download MinGW from [here](https://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download) and install it.
  2. Add `C:\Program Files\mingw-w64\x86_64-8.1.0-posix-seh-rt_v6-rev0\mingw64\bin` (or similar) to your `PATH` environment variable.
  3. Download the libusb binaries from [here](`https://github.com/libusb/libusb/releases/download/v1.0.23/libusb-1.0.23.7z`) and unpack them to `third_party/libusb-windows` (such that the file `third_party/libusb-windows/libusb-1.0.23/MinGW64/static/libusb-1.0.a` exists).
  4. Navigate to this directory and run `make`

### Ubuntu
  1. `sudo apt-get libusb-1.0-dev`
  2. Navigate to this directory and run `make`

### macOS
  1. `brew install libusb`
  2. Navigate to this directory and run `make`

### Cross-compile `libfibre` on Linux for all other platforms
The file `./compile_for_all_platforms.sh` cross-compiles libfibre for all supported platforms. This is mainly intended for CI to generate releases. It written to run on Arch Linux only. Check the script to see which packages need to be installed first.


## Notes for Contributors

 - Fibre currently targets C++11 to maximize compatibility with other projects
 - Notes on platform independent programming:
   - Don't use the keyword `interface` (defined as a macro on Windows in `rpc.h`)
