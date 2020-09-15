
# pkgconf --cflags libusb
# pkgconf --libs libusb

# Prerequisites:
#  Ubuntu: libusb-dev

ifeq ($(OS),Windows_NT)
	target = windows
    ifeq ($(PROCESSOR_ARCHITEW6432),AMD64)
    	outname = libfibre-windows-amd64.dll
    else
        $(error unsupported platform)
    endif
else ifeq ($(shell uname -s),Linux)
	target = linux
	ifeq ($(shell uname -m),x86_64)
		outname = libfibre-linux-amd64.so
	else ifeq (($(shell uname -m),armv7l)
		outname = libfibre-linux-armhf.so
	else
		$(error unsupported platform)
	endif
else ifeq ($(shell uname -s),Darwin)
	target = macos
	ifeq ($(shell uname -m),x86_64)
		outname = libfibre-macos-multiarch.dylib
	else
		$(error unsupported platform)
	endif
endif


FILES=libfibre.cpp \
	platform_support/libusb_transport.cpp \
	legacy_protocol.cpp \
	legacy_object_client.cpp \
	logging.cpp

linux:
	g++ -shared -o $(outname) -fPIC -std=c++11 -I/usr/include/libusb-1.0 -I./include -DFIBRE_COMPILE -DFIBRE_ENABLE_CLIENT \
		$(FILES) \
		-lusb-1.0

linux-cross:
	arm-linux-gnueabihf-g++ -march=armv7 -shared -o libfibre.so -fPIC -std=c++11 -I/usr/include/libusb-1.0 -I./include -DFIBRE_COMPILE -DFIBRE_ENABLE_CLIENT \
		$(FILES) \
		-lusb

windows:
	g++ -shared -o $(outname) -fPIC -std=c++11 -I./third_party/libusb-windows/libusb-1.0.23/include/libusb-1.0 -I./include -DFIBRE_COMPILE -DFIBRE_ENABLE_CLIENT \
		$(FILES) \
		-static-libgcc -Wl,-Bstatic -lstdc++ ./third_party/libusb-windows/libusb-1.0.23/MinGW64/static/libusb-1.0.a -Wl,-Bdynamic

# nm -D libfibre.so | grep T
