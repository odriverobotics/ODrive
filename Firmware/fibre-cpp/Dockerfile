FROM archlinux:base-devel

# Set up package manager
RUN echo "[custom]" >> /etc/pacman.conf && \
    echo "SigLevel = Required TrustedOnly" >> /etc/pacman.conf && \
    echo "Server = https://innovation-labs.appinstall.ch/archlinux/\$repo/os/\$arch" >> /etc/pacman.conf && \
    pacman-key --init && \
    pacman-key --recv-keys 0CB4116A1A3A789937D6DEFB506F27823D2B7B33 && \
    pacman-key --lsign-key 0CB4116A1A3A789937D6DEFB506F27823D2B7B33 && \
    pacman -Syu --noconfirm

# Install prerequisites for the following targets:
#  - Linux (AMD64)
#  - Linux (ARM)
#  - Windows (AMD64)
#  - macOS (x86_32/AMD64)
#  - WebAssembly
RUN pacman -S --noconfirm tup clang gcc binutils wget && \
    pacman -S --noconfirm arm-linux-gnueabihf-gcc arm-linux-gnueabihf-binutils && \
    pacman -S --noconfirm mingw-w64-gcc mingw-w64-binutils p7zip && \
    pacman -S --noconfirm apple-darwin-osxcross && \
    pacman -S --noconfirm emscripten

ENV PATH=${PATH}:/opt/osxcross/bin
ENV PATH=${PATH}:/usr/lib/emscripten

COPY get_dependencies.sh /get_dependencies.sh

# Download and compile dependencies
RUN /get_dependencies.sh download_deb_pkg libusb-dev-amd64 "http://mirrors.kernel.org/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0-dev_1.0.23-2build1_amd64.deb" && \
    /get_dependencies.sh download_deb_pkg libusb-amd64 "http://mirrors.kernel.org/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0_1.0.23-2build1_amd64.deb" && \
    /get_dependencies.sh download_deb_pkg libusb-i386 "http://mirrors.kernel.org/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0_1.0.23-2build1_i386.deb" && \
    /get_dependencies.sh download_deb_pkg libusb-dev-i386 "http://mirrors.kernel.org/ubuntu/pool/main/libu/libusb-1.0/libusb-1.0-0-dev_1.0.23-2build1_i386.deb" && \
    /get_dependencies.sh download_deb_pkg libusb-armhf "http://mirrordirector.raspbian.org/raspbian/pool/main/libu/libusb-1.0/libusb-1.0-0_1.0.24-2_armhf.deb" && \
    /get_dependencies.sh download_deb_pkg libusb-dev-armhf "http://mirrordirector.raspbian.org/raspbian/pool/main/libu/libusb-1.0/libusb-1.0-0-dev_1.0.24-2_armhf.deb" && \
    /get_dependencies.sh download_deb_pkg libstdc++-linux-armhf "http://mirrors.kernel.org/ubuntu/pool/universe/g/gcc-10-cross/libstdc++-10-dev-armhf-cross_10-20200411-0ubuntu1cross1_all.deb"


RUN /get_dependencies.sh patch_macos_sdk && \
    CC='/opt/osxcross/bin/o64-clang' LD_LIBRARY_PATH="/opt/osxcross/lib" CFLAGS='-I/opt/osxcross/SDK/MacOSX10.13.sdk/usr/include -arch i386 -arch x86_64' MACOSX_DEPLOYMENT_TARGET='10.9' /get_dependencies.sh compile_libusb 'macos-amd64' 'x86_64-apple-darwin17'

RUN mkdir -p "third_party/libusb-windows" && \
    pushd "third_party/libusb-windows" > /dev/null && \
    wget "https://github.com/libusb/libusb/releases/download/v1.0.23/libusb-1.0.23.7z" && \
    7z x -o"libusb-1.0.23" "libusb-1.0.23.7z"

# Make Emscripten build its standard libraries for the WebAssembly target
RUN echo "void test() {}" | em++ -x c - -o /tmp/a.out

# Install dependencies for interface_generator.py
RUN pacman -S --noconfirm python-yaml python-jinja python-jsonschema

ENV THIRD_PARTY=/

# Set up entrypoint
RUN echo "#!/bin/bash" > /entrypoint.sh && \
    echo "set -euo pipefail" >> /entrypoint.sh && \
    echo "rm -rdf build/*" >> /entrypoint.sh && \
    echo "echo building \$@" >> /entrypoint.sh && \
    echo "tup generate --config \$@ /tmp/build.sh" >> /entrypoint.sh && \
    echo "exec /usr/bin/bash -x -e /tmp/build.sh" >> /entrypoint.sh && \
    chmod +x /entrypoint.sh && \
    mkdir /build

WORKDIR /build
ENTRYPOINT ["/entrypoint.sh"]
