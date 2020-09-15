#!/bin/bash
set -euo pipefail

# Prerequisites:
# Arch Linux:
#  gcc binutils
#  arm-linux-gnueabihf-gcc arm-linux-gnueabihf-binutils
#  mingw-w64-gcc mingw-w64-binutils
#  p7zip
#  apple-darwin-osxcross

# TODO: support C++11

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

FILES=('libfibre.cpp'
	   'platform_support/libusb_transport.cpp'
	   'legacy_protocol.cpp'
	   'legacy_object_client.cpp'
	   'logging.cpp')

function build_for_linux() {
    arch_name="$1"
    "${CXX}" -shared -o libfibre-"$arch_name".so -fPIC -std=c++11 -I./include -DFIBRE_COMPILE -DFIBRE_ENABLE_CLIENT \
            ${CFLAGS} \
    		"${FILES[@]}" \
            ${LIBS} \
            -lpthread \
            -Wl,--unresolved-symbols=ignore-in-shared-libs -static-libstdc++
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



### Compile libfibre

echo "building libfibre for Linux (AMD64)..."
CFLAGS="-I./third_party/libusb-dev-amd64/usr/include/libusb-1.0" \
LIBS="third_party/libusb-amd64/lib/x86_64-linux-gnu/libusb-1.0.so.0.2.0" \
CXX="x86_64-pc-linux-gnu-g++" \
    build_for_linux 'linux-amd64'

echo "building libfibre for Linux (ARM)..."
CFLAGS="-I./third_party/libusb-dev-armhf/usr/include/libusb-1.0 -L./third_party/libstdc++-linux-armhf/usr/lib/gcc-cross/arm-linux-gnueabihf/10" \
LIBS="third_party/libusb-armhf/lib/arm-linux-gnueabihf/libusb-1.0.so.0.2.0" \
CXX="arm-linux-gnueabihf-g++" \
    build_for_linux 'linux-armhf'


### Windows

echo "building libfibre for Windows (AMD64)..."
x86_64-w64-mingw32-g++ -shared -o libfibre-windows-amd64.dll -fPIC -std=c++11 -I./include -DFIBRE_COMPILE -DFIBRE_ENABLE_CLIENT \
        -I./third_party/libusb-windows/libusb-1.0.23/include/libusb-1.0 \
        "${FILES[@]}" \
        -static-libgcc \
        -Wl,-Bstatic \
        -lstdc++ \
        ./third_party/libusb-windows/libusb-1.0.23/MinGW64/static/libusb-1.0.a \
        -Wl,-Bdynamic        
cp /usr/x86_64-w64-mingw32/bin/libwinpthread-1.dll .

### macOS

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

export PATH="/opt/osxcross/bin:$PATH"
export LD_LIBRARY_PATH="/opt/osxcross/lib"
export CFLAGS='-I/opt/osxcross/SDK/MacOSX10.13.sdk/usr/include -arch i386 -arch x86_64'
export MACOSX_DEPLOYMENT_TARGET='10.9'
CC='o64-clang' \
    compile_libusb 'macos-amd64' 'x86_64-apple-darwin17'

echo "building libfibre for macOS"

o64-clang++ -shared -o libfibre-macos-x86.dylib -fPIC -std=c++11 -I./include -DFIBRE_COMPILE -DFIBRE_ENABLE_CLIENT \
        -I./third_party/libusb-windows/libusb-1.0.23/include/libusb-1.0 \
        -arch x86_64 -arch i386 \
        "${FILES[@]}" \
        -static-libstdc++ \
        ./third_party/libusb-1.0.23/build-macos-amd64/libusb/.libs/libusb-1.0.a \
        -framework CoreFoundation -framework IOKit
