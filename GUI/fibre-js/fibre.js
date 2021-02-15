
import wasm from './libfibre-wasm.js';

function assert(expression) {
    if (!expression) {
        throw Error("assert failed");
    }
}

const FIBRE_STATUS_OK = 0;
const FIBRE_STATUS_CANCELLED = 1;
const FIBRE_STATUS_CLOSED = 2;
const FIBRE_STATUS_INVALID_ARGUMENT = 3;
const FIBRE_STATUS_INTERNAL_ERROR = 4;

const ptrSize = 4;

function _getError(code) {
    if (code == FIBRE_CANCELLED) {
        return Error("libfibre: Operation Cancelled");
    } else if (code == FIBRE_STATUS_CLOSED) {
        return Error("libfibre: Closed");
    } else if (code == FIBRE_STATUS_INVALID_ARGUMENT) {
        return Error("libfibre: Invalid Argument");
    } else if (code == FIBRE_STATUS_INTERNAL_ERROR) {
        return Error("libfibre: Internal Error");
    } else {
        return Error("libfibre: Unknown Error " + code);
    }
}

class BasicCodec {
    constructor(typeName, byteLength, littleEndian) {
        this.getSize = () => byteLength;
        this.serialize = (libfibre, val) => {
            var myBuf = new ArrayBuffer(byteLength);
            new DataView(myBuf)['set' + typeName](0, val, littleEndian)
            return Array.from(new Uint8Array(myBuf));
        }
        this.deserialize = (libfibre, buf) => {
            var myBuf = new ArrayBuffer(buf.length);
            new Uint8Array(myBuf).set(buf);
            return new DataView(myBuf)['get' + typeName](0, littleEndian);
        }
    }
}

const codecs = {
    'int8': new BasicCodec('Int8', 1, true),
    'uint8': new BasicCodec('Uint8', 1, true),
    'int16': new BasicCodec('Int16', 2, true),
    'uint16': new BasicCodec('Uint16', 2, true),
    'int32': new BasicCodec('Int32', 4, true),
    'uint32': new BasicCodec('Uint32', 4, true),
    'int64': new BasicCodec('BigInt64', 8, true),
    'uint64': new BasicCodec('BigUint64', 8, true),
    'float': new BasicCodec('Float32', 4, true),
};

class EofError extends Error {
    constructor(msg) {
        super(msg);
    }
}

class TxStream {
    constructor(libfibre, handle) {
        this._libfibre = libfibre;
        this._handle = handle;
        this.isClosed = false;
        this._libfibre._txStreamMap[this._handle] = this;
    }

    write(data) {
        assert(this._handle);
        return new Promise((resolve, reject) => {
            let byteLength = Array.isArray(data) ? data.length : data.byteLength;
            if (!Array.isArray(data)) {
                data = new Uint8Array(data.buffer, data.byteOffset, data.byteLength)
            }
            this._buf = [this._libfibre.wasm.malloc(byteLength), byteLength];
            this._libfibre.wasm.Module.HEAPU8.set(data, this._buf[0]);
            this._writeResolve = resolve;
            this._writeReject = reject;
            this._libfibre.wasm.Module._libfibre_start_tx(this._handle, this._buf[0], this._buf[1], this._libfibre._onTxCompleted, this._handle);
        });
    }

    async writeAll(data) {
        while (true) {
            const nWritten = await this.write(data);
            data = data.slice(nWritten);
            if (data.length == 0) {
                break;
            }
            if (this.isClosed) {
                throw EofError(`the TX stream was closed but there are still ${data.length} bytes left to send`);
            }
            assert(nWritten > 0); // ensure progress
        }
    }

    close(status) {
        assert(this._handle);
        this._libfibre.wasm.Module._libfibre_close_tx(this._handle, status);
        delete this._handle;
    }

    _complete(status, txEnd) {
        const len = txEnd - this._buf[0];
        assert(len <= this._buf[1]);
        this._libfibre.wasm.free(this._buf[0]);
        delete this._buf;
        const resolve = this._writeResolve;
        const reject = this._writeReject;
        delete this._writeResolve;
        delete this._writeReject;

        this.isClosed = this.isClosed || (status == FIBRE_STATUS_CLOSED);

        if (status == FIBRE_STATUS_OK || status == FIBRE_STATUS_CLOSED) {
            resolve(len);
        } else {
            reject(_getError(status));
        }
    }
}

class RxStream {
    constructor(libfibre, handle) {
        this._libfibre = libfibre;
        this._handle = handle;
        this.isClosed = false;
        this._libfibre._rxStreamMap[this._handle] = this;
    }

    read(len) {
        assert(this._handle);
        return new Promise((resolve, reject) => {
            this._buf = [this._libfibre.wasm.malloc(len), len];
            this._readResolve = resolve;
            this._readReject = reject;
            this._libfibre.wasm.Module._libfibre_start_rx(this._handle, this._buf[0], this._buf[1], this._libfibre._onRxCompleted, this._handle);
        });
    }

    async readAll(len) {
        let data = [];
        while (true) {
            let chunk = await this.read(len - data.length);
            data.push(...chunk);
            if (data.length >= len) {
                break;
            }
            if (this.isClosed) {
                throw EofError(`the RX stream was closed but there are still ${len - data.length} bytes left to receive`);
            }
            assert(chunk.length > 0); // ensure progress
        }
        return data;
    }

    close(status) {
        assert(this._handle);
        this._libfibre.wasm.Module._libfibre_close_rx(this._handle, status);
        delete this._handle;
    }

    _complete(status, rxEnd) {
        const len = rxEnd - this._buf[0];
        assert(len <= this._buf[1]);
        const result = new Uint8Array(this._libfibre.wasm.Module.HEAPU8.buffer, this._buf[0], len).slice(0);
        this._libfibre.wasm.free(this._buf[0]);
        delete this._buf;
        const resolve = this._readResolve;
        const reject = this._readReject;
        delete this._readResolve;
        delete this._readReject;

        this.isClosed = this.isClosed || (status == FIBRE_STATUS_CLOSED);

        if (status == FIBRE_STATUS_OK || status == FIBRE_STATUS_CLOSED) {
            resolve(result);
        } else {
            reject(_getError(status));
        }
    }
}

class RemoteInterface {
    constructor(libfibre, intfName) {
        this._libfibre = libfibre;
        this.intfName = intfName;
        this._onLost = new Promise((resolve) => this._onLostResolve = resolve);
    }
}

const _staleObject = {};

class RemoteAttribute {
    constructor(libfibre, handle, subintf, subintfName) {
        this._libfibre = libfibre;
        this._handle = handle;
        this._subintf = subintf;
        this._subintfName = subintfName;
    }

    get(obj) {
        let objHandlePtr = this._libfibre.wasm.malloc(ptrSize);
        try {
            this._libfibre.wasm.Module._libfibre_get_attribute(obj._handle, this._handle, objHandlePtr);
            return this._libfibre._objMap[this._libfibre._derefIntPtr(objHandlePtr)];
        } finally {
            this._libfibre.wasm.free(objHandlePtr);
        }
    }
}

class RemoteFunction extends Function {
    constructor(libfibre, handle, inputArgs, outputArgs) {
        let closure = function(...args) { 
            return closure._call(this, ...args);
        }
        closure._libfibre = libfibre;
        closure._handle = handle;
        closure._inputArgs = inputArgs;
        closure._outputArgs = outputArgs;
        // Return closure instead of the original "this" object that was
        // associated with the constructor call.
        return Object.setPrototypeOf(closure, new.target.prototype);
    }
    
    /**
     * @brief Starts a function call on this function.
     * @return {TxStream, RxStream, Promise} A TX stream that can be used to
     * send data on this function call, an RX stream that can be used to receive
     * data on this function call and a promise that will be completed once the
     * function call completes.
     */
    startCall(obj) {
        let _callHandle, _txStreamHandle, _rxStreamHandle;
        let completionPromise = new Promise((resolve, reject) => {
            let completer = (status) => {
                if (status == FIBRE_STATUS_OK) {
                    resolve();
                } else {
                    reject(_getError(status));
                }
            };
            [_callHandle, _txStreamHandle, _rxStreamHandle] = this._libfibre._withOutputArg(
                (_callHandlePtr, _txStreamHandlePtr, _rxStreamHandlePtr) =>
                    this._libfibre.wasm.Module._libfibre_start_call(obj._handle, this._handle,
                        _callHandlePtr,
                        _txStreamHandlePtr, _rxStreamHandlePtr, this._libfibre._onCallCompleted,
                        this._libfibre._allocRef(completer)));
        });
        return [
            new TxStream(this._libfibre, _txStreamHandle),
            new RxStream(this._libfibre, _rxStreamHandle),
            completionPromise];
    }

    async _call(obj, ...args) {
        assert(args.length == this._inputArgs.length);

        let txBuf = [];
        for (let argSpec of this._inputArgs) {
            txBuf.push(...argSpec.codec.serialize());
        }
        
        const rxLen = this._outputArgs.reduce((a, b) => a + b.codec.getSize(), 0);
        assert(Number.isInteger(rxLen));

        const [txStream, rxStream, completionPromise] = this.startCall(obj);

        // RX must be started before TX
        const rxPromise = rxStream.readAll(rxLen);
        await txStream.writeAll(txBuf);
        let rxBuf = await rxPromise;
        await completionPromise;

        let outputs = [];
        for (let argSpec of this._outputArgs) {
            let argLength = argSpec.codec.getSize()
            outputs.push(argSpec.codec.deserialize(self._libfibre, rxBuf.slice(0, argLength)))
            rxBuf = rxBuf.slice(argLength);
        }

        if (outputs.length == 1) {
            return outputs[0];
        } else if (outputs.length > 1) {
            return outputs;
        }
    }
}

class LibFibre {
    constructor(wasm) {
        this.wasm = wasm;

        // The following functions are JavaScript callbacks that get invoked from C
        // code (WebAssembly). We convert each function to a pointer which can then
        // be passed to C code.
        // Function signature codes are documented here:
        // https://github.com/aheejin/emscripten/blob/master/site/source/docs/porting/connecting_cpp_and_javascript/Interacting-with-code.rst#calling-javascript-functions-as-function-pointers-from-c

        this._onPost = wasm.addFunction((cb, cbCtx) => {
            // TODO
            console.log("event loop post");
            return new Promise((resolve, reject) => {
                cb(cbCtx);
                resolve();
            });
        }, 'iii');

        this._onCallLater = wasm.addFunction((delay, cb, cbCtx) => {
            console.log("event loop call later");
            return setTimeout(() => cb(cbCtx), delay * 1000);
        }, 'ifii')

        this._onCancelTimer = wasm.addFunction((timer) => {
            return cancelTimeout(timer);
        }, 'ii')

        this._onConstructObject = wasm.addFunction((ctx, obj, intf, intfName, intfNameLength) => {
            let jsIntf = this._loadJsIntf(intf, this.wasm.UTF8ArrayToString(this.wasm.Module.HEAPU8, intfName, intfNameLength));
            this._objMap[obj] = new jsIntf(this, obj);
        }, 'viiiii')

        this._onDestroyObject = wasm.addFunction((ctx, obj) => {
            let jsObj = this._objMap[obj];
            delete this._objMap[obj];
            delete jsObj._handle;
            Object.setPrototypeOf(jsObj, _staleObject);
            console.log("destroyed remote object");
            jsObj._oldHandle = obj;
            jsObj._onLostResolve();
        }, 'vii')

        this._onStartDiscovery = wasm.addFunction((ctx, discCtx, specs, specsLength) => {
            console.log("start discovery");
            let discoverer = this._deref(ctx);
            discoverer.startChannelDiscovery(
                this.wasm.UTF8ArrayToString(this.wasm.Module.HEAPU8, specs, specsLength),
                discCtx);
        }, 'viiii')

        this._onStopDiscovery = wasm.addFunction((ctx, discCtx) => {
            console.log("stop discovery");
            this._freeRef(ctx);
        }, 'viiii')

        this._onFoundObject = wasm.addFunction((ctx, obj) => {
            let onFoundObject = this._deref(ctx);
            onFoundObject(this._objMap[obj]);
        }, 'vii')

        this._onAttributeAdded = wasm.addFunction((ctx, attr, name, nameNength, subintf, subintfName, subintfNameLength) => {
            let jsIntf = this._intfMap[ctx];
            let jsAttr = new RemoteAttribute(this, attr, subintf, this.wasm.UTF8ArrayToString(this.wasm.Module.HEAPU8, subintfName, subintfNameLength));
            Object.defineProperty(jsIntf.prototype, this.wasm.UTF8ArrayToString(this.wasm.Module.HEAPU8, name, nameNength), {
                get: function () { return jsAttr.get(this); }
            });
        }, 'viiiiiii')

        this._onAttributeRemoved = wasm.addFunction((ctx, attr) => {
            console.log("attribute removed"); // TODO
        }, 'vii')

        this._onFunctionAdded = wasm.addFunction((ctx, func, name, nameLength, inputNames, inputCodecs, outputNames, outputCodecs) => {
            const jsIntf = this._intfMap[ctx];
            const jsInputParams = [...this._decodeArgList(inputNames, inputCodecs)];
            const jsOutputParams = [...this._decodeArgList(outputNames, outputCodecs)];
            let jsFunc = new RemoteFunction(this, func, jsInputParams, jsOutputParams);
            jsIntf.prototype[this.wasm.UTF8ArrayToString(this.wasm.Module.HEAPU8, name, nameLength)] = jsFunc;
        }, 'viiiiiiii')

        this._onFunctionRemoved = wasm.addFunction((ctx, func) => {
            console.log("function removed"); // TODO
        }, 'vii')

        this._onCallCompleted = wasm.addFunction((ctx, status) => {
            let completer = this._freeRef(ctx);
            completer(status);
        }, 'vii');

        this._onTxCompleted = wasm.addFunction((ctx, txStream, status, txEnd) => {
            this._txStreamMap[txStream]._complete(status, txEnd);
        }, 'viiii')

        this._onRxCompleted = wasm.addFunction((ctx, rxStream, status, rxEnd) => {
            this._rxStreamMap[rxStream]._complete(status, rxEnd);
        }, 'viiii')

        const ptr = this.wasm.Module._libfibre_get_version();
        const version_array = (new Uint16Array(wasm.Module.HEAPU8.buffer, ptr, 3))
        this.version = {
            major: version_array[0],
            minor: version_array[1],
            patch: version_array[2]
        };

        this._intfMap = {};
        this._refMap = {};
        this._objMap = {};
        this._txStreamMap = {};
        this._rxStreamMap = {};

        this._handle = this.wasm.Module._libfibre_open(
            this._onPost,
            0,
            0,
            this._onCallLater,
            this._onCancelTimer,
            this._onConstructObject,
            this._onDestroyObject
        );

        this.usbDiscoverer = new WebUsbDiscoverer(this);

        let buf = [this.wasm.malloc(4), 4];
        try {
            let len = this.wasm.stringToUTF8Array("usb", this.wasm.Module.HEAPU8, buf[0], buf[1]);
            this.wasm.Module._libfibre_register_discoverer(this._handle, buf[0], len,
                this._onStartDiscovery, this._onStopDiscovery, this._allocRef(this.usbDiscoverer));
        } finally {
            this.wasm.free(buf[0]);
        }
    }

    addChannels(mtu, channelDiscoveryHandle) {
        console.log("add channel with mtu " + mtu);
        const [txChannelId, rxChannelId] = this._withOutputArg((txChannelIdPtr, rxChannelIdPtr) =>
            this.wasm.Module._libfibre_add_channels(this._handle, channelDiscoveryHandle, txChannelIdPtr, rxChannelIdPtr, mtu)
        );
        return [
            new RxStream(this, txChannelId),
            new TxStream(this, rxChannelId)
        ];
    }

    startDiscovery(filter, onFoundObject) {
        let buf = [this.wasm.malloc(filter.length + 1), filter.length + 1];
        try {
            let len = this.wasm.stringToUTF8Array(filter, this.wasm.Module.HEAPU8, buf[0], buf[1]);
            this._withOutputArg((discoveryHandlePtr) => 
                this.wasm.Module._libfibre_start_discovery(this._handle, buf[0], len,
                    discoveryHandlePtr,
                    this._onFoundObject, this._onStopped, this._allocRef(onFoundObject)));
        } finally {
            this.wasm.free(buf[0]);
        }
    }

    /**
     * @brief Allocates a unique integer key for the specified JavaScript object.
     * 
     * The unique key can later be used to look up the JavaScript object using
     * _freeRef().
     */
    _allocRef(obj) {
        let id = 0;
        while (++id in this._refMap) {}
        console.log("allocating id " + id, obj);
        this._refMap[id] = obj;
        return id;
    }

    _deref(id) {
        return this._refMap[id];
    }

    _derefIntPtr(ptr) {
        return new Uint32Array(this.wasm.Module.HEAPU8.buffer, ptr, 1)[0]
    }

    * _getStringList(ptr) {
        for (let i = 0; this._derefIntPtr(ptr + i * ptrSize); ++i) {
            yield this.wasm.UTF8ArrayToString(this.wasm.Module.HEAPU8, this._derefIntPtr(ptr + i * ptrSize));
        }
    }

    /**
     * @bried Deallocates a unique integer key that was previously obtained with
     * _allocRef.
     * 
     * @return The JavaScript object that was passed to the corresponding
     * _allocRef call.
     */
    _freeRef(id) {
        let obj = this._refMap[id];
        delete this._refMap[id];
        console.log("freeing id " + id, obj);
        return obj;
    }

    _withOutputArg(func) {
        let ptrs = [];
        try {
            for (let i = 0; i < func.length; ++i) {
                ptrs.push(this.wasm.malloc(ptrSize));
            }
            func(...ptrs);
            return ptrs.map((ptr) => this._derefIntPtr(ptr));
        } finally {
            for (let ptr of ptrs) {
                this.wasm.free(ptr);
            }
        }
    }

    * _decodeArgList(argNames, codecNames) {
        argNames = [...this._getStringList(argNames)];
        codecNames = [...this._getStringList(codecNames)];
        assert(argNames.length == codecNames.length);
        for (let i in argNames) {
            yield {
                name: argNames[i],
                codecName: codecNames[i],
                codec: codecs[codecNames[i]]
            };
        }
    }

    _loadJsIntf(intfHandle, intfName) {
        if (intfHandle in this._intfMap) {
            return this._intfMap[intfHandle];
        } else {
            class RemoteObject extends RemoteInterface {
                constructor(libfibre, handle) {
                    super(libfibre, intfName);
                    this._handle = handle;
                }
            };
            let jsIntf = RemoteObject;
            this._intfMap[intfHandle] = jsIntf;
            this.wasm.Module._libfibre_subscribe_to_interface(intfHandle,
                this._onAttributeAdded,
                this._onAttributeRemoved,
                this._onFunctionAdded,
                this._onFunctionRemoved,
                intfHandle);
            return jsIntf;
        }
    }

}

export function fibreOpen() {
    return new Promise(async (resolve) => {
        let Module = {
            preRun: [function() {Module.ENV.FIBRE_LOG = "4"}], // enable logging
            instantiateWasm: async (info, receiveInstance) => {
                const isWebpack = typeof __webpack_require__ === 'function';
                const wasmPath = isWebpack ? (await import("!!file-loader!./libfibre-wasm.wasm")).default
                    : "./libfibre-wasm.wasm";
                const response = fetch(wasmPath, { credentials: 'same-origin' });
                const result = await WebAssembly.instantiateStreaming(response, info);
                receiveInstance(result['instance']);
                return {};
            }
        };
        
        Module = await wasm(Module);
        await Module.ready;
        wasm.addFunction = Module.addFunction;
        wasm.stringToUTF8Array = Module.stringToUTF8Array;
        wasm.UTF8ArrayToString = Module.UTF8ArrayToString;
        wasm.malloc = Module._malloc;
        wasm.free = Module._free;
        wasm.Module = Module;
        
        resolve(new LibFibre(wasm));
    });
}

class WebUsbDiscoverer {
    constructor(libfibre) {
        this._libfibre = libfibre;
        this._discoveryProcesses = [];

        if (navigator.usb.onconnect != null) {
            console.warn("There was already a subscriber for usb.onconnect.");
        }
        navigator.usb.onconnect = (event) => {
            for (let discovery of this._discoveryProcesses) {
                this._consider(event.device, discovery.filter, discovery.handle);
            }
        }
        navigator.usb.ondisconnect = () => console.log("disconnect");

        this._filterKeys = {
            'idVendor': 'vendorId',
            'idProduct': 'productId',
            'bInterfaceClass': 'classCode',
            'bInterfaceSubClass': 'subclassCode',
            'bInterfaceProtocol': 'protocolCode',
        };

        this.showDialog = async () => {
            let filters = this._discoveryProcesses.map((d) => d.filter);
            let dev = await navigator.usb.requestDevice({filters: filters});
            for (let discovery of this._discoveryProcesses) {
                this._consider(dev, discovery.filter, discovery.handle);
            }
        };
    }

    async startChannelDiscovery(specs, discoveryHandle) {
        let filter = {}
        for (let item of specs.split(',')) {
            filter[this._filterKeys[item.split('=')[0]]] = item.split('=')[1]
        }

        this._discoveryProcesses.push({
            filter: filter,
            handle: discoveryHandle
        });

        for (let dev of await navigator.usb.getDevices({filters: [filter]})) {
            this._consider(dev, filter, discoveryHandle);
        }
    }

    async _consider(device, filter, channelDiscoveryHandle) {
        if ((filter.vendorId != undefined) && (filter.vendorId != device.vendorId)) {
            return;
        }
        if ((filter.productId != undefined) && (filter.productId != device.productId)) {
            return;
        }

        for (let config of device.configurations) {
            if (device.configuration !== null && device.configuration.configurationValue != config.configurationValue) {
                continue; // A configuration was already set and it's different from this one
            }

            for (let intf of config.interfaces) {
                for (let alternate of intf.alternates) {
                    const mismatch = ((filter.classCode != undefined) && (filter.classCode != alternate.interfaceClass))
                                  || ((filter.subclassCode != undefined) && (filter.subclassCode != alternate.interfaceSubclass))
                                  || ((filter.protocolCode != undefined) && (filter.protocolCode != alternate.interfaceProtocol));
                    if (mismatch) {
                        continue;
                    }

                    await device.open();
                    await device.selectConfiguration(config.configurationValue);
                    await device.claimInterface(intf.interfaceNumber);

                    let epOut = null, epIn = null;
                    for (let ep of alternate.endpoints) {
                        if (ep.type == "bulk" && ep.direction == "in")
                            epIn = ep;
                        else if (ep.type == "bulk" && ep.direction == "out")
                            epOut = ep;
                    }

                    device.knownInEndpoints = device.knownInEndpoints || [];
                    device.knownOutEndpoints = device.knownOutEndpoints || [];
                    console.log(device.knownOutEndpoints);
                    if (device.knownInEndpoints.indexOf(epIn.endpointNumber) >= 0) {
                        continue;
                    }
                    if (device.knownOutEndpoints.indexOf(epOut.endpointNumber) >= 0) {
                        continue;
                    }
                    device.knownInEndpoints.push(epIn.endpointNumber);
                    device.knownOutEndpoints.push(epOut.endpointNumber);

                    let mtu = Math.min(epIn.packetSize, epOut.packetSize);
                    const [txChannel, rxChannel] = this._libfibre.addChannels(mtu, channelDiscoveryHandle);
                    this._connectBulkInEp(device, epIn, rxChannel);
                    this._connectBulkOutEp(device, epOut, txChannel);
                }
            }
        }
    }

    async _connectBulkOutEp(dev, ep, stream) {
        while (true) {
            //console.log("waiting for data from libfibre...");
            const data = await stream.read(ep.packetSize);
            //console.log("forwarding " + data.length + " bytes to USB...");
            //console.log(data);
            let result;
            try {
                result = await dev.transferOut(ep.endpointNumber, data);
            } catch (e) { // TODO: propagate actual transfer errors
                result = {status: "stall"};
            }
            assert(result.bytesWritten == data.length);
            console.log(dev.opened);
            if (!dev.opened || result.status == "stall") {
                stream.close(FIBRE_STATUS_CLOSED);
                break;
            }
            assert(result.status == "ok")
        };
    }

    async _connectBulkInEp(dev, ep, stream) {
        while (true) {
            //console.log("waiting for up to " + epIn.packetSize + " bytes from USB...");
            let result;
            try {
                result = await dev.transferIn(ep.endpointNumber, ep.packetSize);
            } catch (e) { // TODO: propagate actual transfer errors
                result = {status: "stall"};
            }
            //console.log("got for data from USB...");
            console.log(dev.opened);
            console.log(dev);
            if (!dev.opened || result.status == "stall") {
                stream.close(FIBRE_STATUS_CLOSED);
                break;
            }
            assert(result.status == "ok");
            //console.log("forwarding " + result.data.byteLength + " bytes to libfibre...");
            const len = await stream.write(result.data);
            assert(len == result.data.byteLength);
        }
    }
}
