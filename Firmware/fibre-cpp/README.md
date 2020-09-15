
## Platform Compatibility

|     | Windows | macOS [1] | Linux |
|-----|---------|-----------|-------|
| USB | yes     | yes       | yes   |

 - [1] macOS 10.9 (Mavericks) or later

## `libfibre` API

Refer to [libfibre.h](include/fibre/libfibre.h) for documentation of the API.

## Build instructions

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
