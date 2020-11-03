<template>
  <div class="home">
    <div class="logo">
      <img alt="Odrive Logo" src="../assets/odrive_logo.png">
    </div>
    <div class="home_text">
      To set up your ODrive, connect it and power it up.
    </div>
    <button class="connect-usb-button" @click="showUsbConnectDialog()">Connect to USB device...</button>
    <div id="hackyOutput">
      not connected
    </div>
    <button class="show-msg-button" @click="showServerMessages = !showServerMessages;">Having trouble? Click here for debug output</button>
    <div class="server-msgs" v-if="showServerMessages">
      <div class="server-msg" v-for="(msg, index) in serverMessages" :key="msg + index">{{msg}}</div>
    </div>
  </div>
</template>

<script>

//Object.getPrototypeOf(process).version = {
//  npm: '6.14.7',
//  ares: '1.16.1',
//  brotli: '1.0.9',
//  cldr: '37.0',
//  icu: '67.1',
//  llhttp: '2.1.3',
//  modules: '83',
//  napi: '7',
//  nghttp2: '1.41.0',
//  node: '14.14.0',
//  openssl: '1.1.1h',
//  tz: '2019c',
//  unicode: '13.0',
//  uv: '1.40.0',
//  v8: '8.4.371.19-node.17',
//  zlib: '1.2.11'
//};

//process.version.modules = 'v9.40';
//var process = require('process');
//console.log(process);
//console.log(process.version);
//console.log(process.versions);
//const mfs = require("fs");
//const mp = require("path");
//const m3 = require("file-uri-to-path");
//console.log("fs", mfs);
//console.log("path", mp);
//console.log("m3", m3);
const bi = require("bindings");
console.log(bi);
const usb = require('electron').remote.require('./usb_bindings.node')
//const usb = require("usb");
console.log(usb);

import { fibreOpen } from './../../fibre-js/fibre.js';

var theFibre = {};
fibreOpen().then(async (libfibre) => {
  theFibre = libfibre;
  const filter = 'usb:idVendor=0x1209,idProduct=0x0D32,bInterfaceClass=0,bInterfaceSubClass=1,bInterfaceProtocol=0';
  console.log("opened libfibre: ", libfibre);
  console.log(await navigator.usb.getDevices());
  let onFoundObject = async (obj) => {
    console.log("found an object!", obj);
    let isConnected = true;
    obj._onLost.then(() => isConnected = false);
    while (isConnected) {
       document.getElementById("hackyOutput").innerHTML = "vbus_voltage: " + (await obj.vbus_voltage.read());
       await new Promise((resolve) => setTimeout(resolve, 100));
    }
    document.getElementById("hackyOutput").innerHTML = "not connected";
 }
 libfibre.startDiscovery(filter, onFoundObject);
});

export default {
  name: 'Home',
  components: {
  },
  data() {
    return {
      showServerMessages: false,
    }
  },
  computed: {
    connected() {
      return this.$store.state.serverConnected == true;
    },
    notConnected() {
      return this.$store.state.serverConnected != true;
    },
    serverAddress() {
      return this.$store.state.odriveServerAddress;
    },
    serverMessages() {
      if (this.$store.state.serverOutput.length >= 10) {
        let len = this.$store.state.serverOutput.length;
        return this.$store.state.serverOutput.slice(len-10,len);
      }
      else {
        return this.$store.state.serverOutput;
      }
    }
  },
  methods: {
    setUrl(e) {
      console.log(e.target.value);
      this.$store.dispatch("setServerAddress", e.target.value);
    },
    showUsbConnectDialog() {
      theFibre.usbDiscoverer.showDialog();
    }
  }
}
</script>

<style>
.home {
  padding: 10% 0;
  margin: auto;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  background-color: var(--bg-color);
  height: 100vh;
}

img {
  margin: auto;
  display: block;
  max-width: 50%;
  width: auto;
  height: auto;
}

.home_text {
  font-size: 1.5rem;
  padding: 20px;
}

#hackyOutput {
  font-size: 1.5rem;
  padding: 20px;
  color: blue;
}

.connected, .notConnected {
  width: 40vw;
  margin: 0;
  padding: 10px;
  border: none;
  border-bottom: 1px solid lightgrey;
  font-size: 1.5rem;
  background-color: var(--bg-color);
  text-align: center;
}

input:focus {
  outline: none;
}

.connected {
  border-bottom: 1px solid lightgreen;
  background-color: lightgreen;
}

.notConnected {
  border-bottom: 1px solid red;
}

.show-msg-button {
  border: none;
  background-color: var(--bg-color);
  margin-top: 0.5rem;
  cursor: pointer;
}

.server-msg {
  font-family: "Roboto Mono", monospace;
}
</style>
