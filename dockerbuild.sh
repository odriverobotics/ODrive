function cleanup {
    echo "Removing previous build artifacts"
    rm -rf build
    docker rm build-cont
}

function gc {
    cleanup
    docker rmi build-img
    docker image prune
}

function build {
    cleanup

    echo "Building the firmware"
    docker build -t build-img .

    echo "Create container"
    docker create --name build-cont build-img:latest

    echo "Extract build artifacts"
    docker cp build-cont:ODrive/Firmware/build .
}

function usage {
    echo "usage: $0 build | cleanup | gc"
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
