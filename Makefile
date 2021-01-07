
all:
	tup --no-environ-check build-local
	tup --no-environ-check build-wasm
	cp build-local/libfibre-* ../python/fibre/
	cp build-wasm/libfibre-* ../js/
