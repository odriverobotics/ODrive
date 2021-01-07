#!/bin/bash
set -euo pipefail

# Usage: download_deb_pkg destination-dir url
function download_deb_pkg() {
    mkdir -p third_party
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

function patch_macos_sdk() {
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
}

cmd="$1"
shift
case "$cmd" in
    download_deb_pkg) download_deb_pkg $@ ;;
    compile_libusb) compile_libusb $@ ;;
    patch_macos_sdk) patch_macos_sdk $@ ;;
    *) echo "unknown command" && false ;;
esac
