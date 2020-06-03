function cleanup {
    echo "Removing previous build artifacts"
    rm -rf build
    docker rm odrive-build-cont
}

function gc {
    cleanup
    docker rmi odrive-build-img
    docker image prune
}

function build {
    cleanup

    echo "Building the firmware"
    docker build -t odrive-build-img .

    echo "Create container"
    docker create --name odrive-build-cont odrive-build-img:latest

    echo "Extract build artifacts"
    docker cp odrive-build-cont:ODrive/Firmware/build .
}

function usage {
    echo "usage: $0 (build | cleanup | gc)"
    echo
    echo "build   -- build in docker and extract the artifacts."
    echo "cleanup -- remove build artifacts from previous build"
    echo "gc      -- remove all build images and containers"
}

case $1 in
    build)
	build
	;;
    cleanup)
	cleanup
	;;
    gc)
	gc
	;;
    *)
	usage
	;;
esac
