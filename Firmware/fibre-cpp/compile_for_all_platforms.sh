#!/bin/bash
set -euo pipefail

# Prerequisites:
# Arch Linux:
#  gcc binutils
#  arm-linux-gnueabihf-gcc arm-linux-gnueabihf-binutils
#  mingw-w64-gcc mingw-w64-binutils
#  p7zip
#  apple-darwin-osxcross

mkdir -p third_party

# Usage: download_deb_pkg destination-dir url
function download_deb_pkg() {
    dir="$1"
    url="$2"
    file="$(sed 's|^.*/\([^/]*\)$|\1|' <<< "$url")"

    pushd third_party > /dev/null
    if ! [ -f "${file}" ]; then
        wget "${url}"
    fi
    if ! [ -d "${dir}/usr" ]; then
        ar x "${file}" "data.tar.xz"
        mkdir -p "${dir}"
        tar -xvf "data.tar.xz" -C "${dir}"
    fi
    popd > /dev/null
}

# Usage: compile_libusb arch-name arch
function compile_libusb() {
    arch_name="$1"
    arch="$2"
    libusb_version=1.0.23

    pushd third_party > /dev/null
    if ! [ -f "libusb-${libusb_version}.tar.bz2" ]; then
        wget "https://github.com/libusb/libusb/releases/download/v${libusb_version}/libusb-${libusb_version}.tar.bz2"
    fi
    if ! [ -d "libusb-${libusb_version}" ]; then
        tar -xvf "libusb-${libusb_version}.tar.bz2"
    fi

    mkdir -p "libusb-${libusb_version}/build-${arch_name}"
    pushd "libusb-${libusb_version}/build-${arch_name}" > /dev/null
    unset LDFLAGS
    if ! [ -f "libusb/.libs/libusb-1.0.a" ]; then
        ../configure --host="$arch" \
                    --enable-static \
                    --prefix=/opt/osxcross/ \
                    --disable-dependency-tracking
        # They broke parallel building in libusb 1.20
        make
    fi
    popd > /dev/null
    popd > /dev/null
}


### Download/compile prerequisites

download_deb_pkg libusb-dev-amd64 "http://mirrors.kernel.org/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0-dev_1.0.23-2build1_amd64.deb"
download_deb_pkg libusb-amd64 "http://mirrors.kernel.org/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0_1.0.23-2build1_amd64.deb"
download_deb_pkg libusb-i386 "http://mirrors.kernel.org/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0_1.0.23-2build1_i386.deb"
download_deb_pkg libusb-dev-i386 "http://mirrors.kernel.org/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0-dev_1.0.23-2build1_i386.deb"
download_deb_pkg libusb-armhf "http://mirrordirector.raspbian.org/raspbian/pool/main/libu/libusb-1.0/libusb-1.0-0_1.0.23-2_armhf.deb"
download_deb_pkg libusb-dev-armhf "http://mirrordirector.raspbian.org/raspbian/pool/main/libu/libusb-1.0/libusb-1.0-0-dev_1.0.23-2_armhf.deb"
download_deb_pkg libstdc++-linux-armhf "http://mirrors.kernel.org/ubuntu/pool/universe/g/gcc-10-cross/libstdc++-10-dev-armhf-cross_10-20200411-0ubuntu1cross1_all.deb"

mkdir -p "third_party/libusb-windows"
pushd "third_party/libusb-windows" > /dev/null
if [ ! -f libusb-1.0.23.7z ]; then
    wget "https://github.com/libusb/libusb/releases/download/v1.0.23/libusb-1.0.23.7z"
fi
if [ ! -f "libusb-1.0.23/libusb-1.0.def" ]; then
    7z x -o"libusb-1.0.23" "libusb-1.0.23.7z"
fi
popd > /dev/null


### compile libusb for macOS

# Link are broken:
# …ions/Current/Headers $ ls -l IOReturn.h 
# lrwxrwxrwx 1 root root 189 Dec 26  2019 IOReturn.h -> Users/phracker/Documents/Xcode-beta.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk/System/Library/Frameworks/Kernel.framework/Versions/A/Headers/IOKit/IOReturn.h
# Fix with:
# sudo ln -sf /opt/osxcross/SDK/MacOSX10.13.sdk/System/Library/Frameworks/Kernel.framework/Versions/A/Headers/IOKit/IOReturn.h IOReturn.h

oldprefix="Users/phracker/Documents/Xcode-beta.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk"
newprefix="/opt/osxcross/SDK/MacOSX10.13.sdk"
while IFS= read -r link; do
    destination="$(readlink "$link")"
    pruned_destination="${destination#"$oldprefix"}"
    if [ "${oldprefix}${pruned_destination}" == "${destination}" ]; then
        sudo mv -T "${newprefix}${pruned_destination}" "$link"
    fi
done <<< "$(find /opt/osxcross/SDK/MacOSX10.13.sdk/System/Library/Frameworks/IOKit.framework -xtype l)"

echo "building libusb for macOS..."

CC='/opt/osxcross/bin/o64-clang' \
LD_LIBRARY_PATH="/opt/osxcross/lib" \
PATH="/opt/osxcross/bin:$PATH" \
CFLAGS='-I/opt/osxcross/SDK/MacOSX10.13.sdk/usr/include -arch i386 -arch x86_64' \
MACOSX_DEPLOYMENT_TARGET='10.9' \
    compile_libusb 'macos-amd64' 'x86_64-apple-darwin17'




### Prepare tup.config files

mkdir -p build-linux-amd64
cat <<EOF > build-linux-amd64/tup.config
CONFIG_DEBUG=false
CONFIG_CC="clang++"
CONFIG_CFLAGS="-I./third_party/libusb-dev-armhf/usr/include/libusb-1.0"
CONFIG_LDFLAGS="./third_party/libusb-amd64/lib/x86_64-linux-gnu/libusb-1.0.so.0.2.0"
CONFIG_USE_PKGCONF=false
EOF

mkdir -p build-linux-armhf
cat <<EOF > build-linux-armhf/tup.config
CONFIG_DEBUG=false
CONFIG_CC="arm-linux-gnueabihf-g++"
CONFIG_CFLAGS="-I./third_party/libusb-dev-armhf/usr/include/libusb-1.0"
CONFIG_LDFLAGS="-L./third_party/libstdc++-linux-armhf/usr/lib/gcc-cross/arm-linux-gnueabihf/10 third_party/libusb-armhf/lib/arm-linux-gnueabihf/libusb-1.0.so.0.2.0"
CONFIG_USE_PKGCONF=false
EOF

mkdir -p build-windows-amd64
cat <<EOF > build-windows-amd64/tup.config
CONFIG_DEBUG=false
CONFIG_CC="x86_64-w64-mingw32-g++"
CONFIG_CFLAGS="-I./third_party/libusb-windows/libusb-1.0.23/include/libusb-1.0"
CONFIG_LDFLAGS="-static-libgcc ./third_party/libusb-windows/libusb-1.0.23/MinGW64/static/libusb-1.0.a"
CONFIG_USE_PKGCONF=false
EOF

mkdir -p build-macos-x86
cat <<EOF > build-macos-x86/tup.config
CONFIG_DEBUG=false
CONFIG_CC="LD_LIBRARY_PATH=/opt/osxcross/lib MACOSX_DEPLOYMENT_TARGET=10.9 /opt/osxcross/bin/o64-clang++"
CONFIG_CFLAGS="-I./third_party/libusb-1.0.23/libusb -arch i386 -arch x86_64"
CONFIG_LDFLAGS="./third_party/libusb-1.0.23/build-macos-amd64/libusb/.libs/libusb-1.0.a -framework CoreFoundation -framework IOKit"
CONFIG_USE_PKGCONF=false
EOF

# Uncomment this to generate the WebAssembly build target. If you do this you
# have to finish a compile without tup before tup works. This is because
# emscripten generates some cache files which tup is unhappy about.
#mkdir -p build-wasm
#cat <<EOF > build-wasm/tup.config
#CONFIG_DEBUG=true
#CONFIG_CC=/usr/lib/emscripten/em++
#CONFIG_CFLAGS=-include emscripten.h -DFIBRE_PUBLIC=EMSCRIPTEN_KEEPALIVE -s RESERVED_FUNCTION_POINTERS=1
#CONFIG_LDFLAGS=-s EXPORT_ES6=1 -s MODULARIZE=1 -s USE_ES6_IMPORT_META=0 -s 'EXTRA_EXPORTED_RUNTIME_METHODS=[addFunction, stringToUTF8Array, UTF8ArrayToString, ENV]'
#CONFIG_USE_PKGCONF=false
#CONFIG_ENABLE_LIBUSB=false
#EOF

mkdir -p build-local
echo "" > build-local/tup.config


### Invoke tup for all configs

tup --no-environ-check


### Copy to other locations

function copy_to() {
    cp build-linux-amd64/libfibre-linux-amd64.so "$1/"
    cp build-linux-armhf/libfibre-linux-armhf.so "$1/"
    cp build-windows-amd64/libfibre-windows-amd64.dll "$1/"
    cp /usr/x86_64-w64-mingw32/bin/libwinpthread-1.dll "$1/"
    cp build-macos-x86/libfibre-macos-x86.dylib "$1/"
}

[ -d ../python/fibre ] && copy_to ../python/fibre
[ -d ../js ] && cp build-wasm/libfibre-* ../js
