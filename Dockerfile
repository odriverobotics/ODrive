FROM ubuntu:bionic

# Prepare the build environment and dependencies
RUN apt-get update
RUN apt-get -y install software-properties-common
RUN add-apt-repository ppa:team-gcc-arm-embedded/ppa
RUN add-apt-repository ppa:jonathonf/tup
RUN apt-get update
RUN apt-get -y upgrade
RUN apt-get -y install gcc-arm-embedded openocd tup python3.7 build-essential git

# Build step below does not know about debian's python naming schemme
RUN ln -s /usr/bin/python3.7 /usr/bin/python

# Copy the firmware tree into the container
RUN mkdir ODrive
COPY . ODrive
WORKDIR ODrive/Firmware

# Hack around Tup's dependency on FUSE
RUN tup generate build.sh
RUN ./build.sh
