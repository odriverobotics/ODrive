<template>
  <div id="app">
    <!-- HEADER -->
    <div class="header" id="header">
      <button class="dash-button menu-button" @click="toggleMenu">Menu</button>
      <div class="card menu" id="menu" :style="{top: menuTop}" v-show="showMenu">
        <button class="dash-button menu-item" @click="exportDash">Export dash</button>
        <button class="dash-button menu-item" @click="importDashWrapper">
          Import dash
          <input
            type="file"
            id="inputDash"
            ref="fileInput"
            @change="
              importDashFile($event.target.files);
              $refs.fileInput.value = null;
            "
            value
            style="display: none"
          />
        </button>
        <button class="dash-button menu-item" id="importButton" @click="calcImportLeft();calcImportTop();toggleImport()">Import ODrive config</button>
        <button class="dash-button menu-item" id="exportButton" @click="calcExportLeft();calcExportTop();toggleExport()">Export ODrive config</button>
      </div>
      <div class="card import-menu" :style="{top: importTop, left: importLeft}" v-show="showImport">
        <button v-for="odrive in Object.keys(odrives)" :key="odrive" class="dash-button" @click="importConfigWrapper">
          {{odrive}}
          <input
            type="file"
            id="inputConfig"
            ref="fileInputConfig"
            @change="
              importConfigFile($event.target, odrive);
            "
            onclick="this.value=null"
            value
            style="display: none"
          />
          </button>
      </div>
      <div class="card export-menu" :style="{top: exportTop, left: exportLeft}" v-show="showExport">
        <button v-for="odrive in Object.keys(odrives)" :key="odrive" class="dash-button" @click="exportConfig(odrive)">{{odrive}}</button>
      </div>
      <button
        v-for="dash in dashboards"
        :key="dash.id"
        :class="['dash-button', { active: currentDash === dash.name }]"
        v-on:click.self="changeDash(dash.name)"
        v-on:dblclick="changeDashName(dash.id)"
      >
        <button
          v-if="
            dash.name !== 'Start' &&
            dash.name !== 'Tuning' &&
            dash.name !== 'Wizard'
          "
          class="close-button"
          v-on:click="deleteDash(dash.id)"
        >
          X
        </button>
        {{ dash.name }}
      </button>
      <button class="dash-button dash-add" @click="addDash">+</button>
      <button
        class="dash-button sample-button"
        :class="[{ active: sampling === true }]"
        @click="sampleButton"
      >
        {{ samplingText }}
      </button>
    </div>

    <!-- PAGE CONTENT -->
    <component
      :is="currentDashName"
      :odrives="odrives"
      :dash="dash"
    ></component>

    <!-- FOOTER -->
    <div class="footer">
      <Axis
        v-for="axis in axes"
        :key="axis.name"
        :axis="axis.name"
        :odrives="odrives"
      ></Axis>
    </div>

  </div>
</template>

<script>
import Start from "./views/Start.vue";
import Dashboard from "./views/Dashboard.vue";
import Axis from "./components/Axis.vue";
import Wizard from "./views/Wizard.vue";
import * as socketio from "./comms/socketio";
import { saveAs } from "file-saver";
import { v4 as uuidv4 } from "uuid";
import { pathsFromTree } from "./lib/utils.js";
import { getVal, putVal } from "./lib/odrive_utils.js";

export default {
  name: "App",
  components: {
    Start,
    Dashboard,
    Axis,
    Wizard,
  },
  data: function () {
    return {
      //currentDash: "Start",
      showMenu: false,
      showImport: false,
      showExport: false,
      menuTop: '0px',
      importTop: '0px',
      importLeft: '0px',
      exportTop: '0px',
      exportLeft: '0px',
    };
  },
  mounted() {
    var elem = document.getElementById('header');
    this.menuTop = elem.offsetHeight + 'px';
  },
  computed: {
    currentDashName: function () {
      //get the appropriate component name from the currentDash variable
      let comp = {};
      for (const dash of this.dashboards) {
        if (dash.name === this.$store.state.currentDash) {
          comp = dash.component;
        }
      }
      return comp;
    },
    dash: function () {
      let comp = {};
      for (const dash of this.dashboards) {
        if (dash.name === this.$store.state.currentDash) {
          comp = dash;
        }
      }
      return comp;
    },
    currentCtrlList: function () {
      let comp = {};
      for (const dash of this.dashboards) {
        if (dash.name === this.$store.state.currentDash) {
          comp = dash.controls;
        }
      }
      return comp;
    },
    axes: function () {
      return this.$store.state.axes;
    },
    odrives: function () {
      return this.$store.state.odrives;
    },
    dashboards: function () {
      return this.$store.state.dashboards;
    },
    sampling: function () {
      return this.$store.state.sampling;
    },
    currentDash: function () {
      return this.$store.state.currentDash;
    },
    samplingText: function () {
      let ret;
      if (this.$store.state.sampling) {
        ret = "stop sampling";
      } else {
        ret = "start sampling";
      }
      return ret;
    },
  },
  methods: {
    toggleMenu() {
      this.showMenu = !this.showMenu;
      if (!this.showMenu) {
        this.showImport = false;
        this.showExport = false;
      }
    },
    toggleImport() {
      this.showImport = !this.showImport;
      this.showExport = false;
    },
    toggleExport() {
      this.showExport = !this.showExport;
      this.showImport=  false;
    },
    importConfigWrapper() {
      const inputElem = document.getElementById("inputConfig");
      if (inputElem) {
        console.log("importing config");
        inputElem.click();
      }
    },
    importConfigFile(event, odrive) {
      console.log("Importing config to odrive " + odrive);
      let file = event.files[0];
      const reader = new FileReader();
      // this is ugly, but it gets around scoping problems the "load" callback
      let importConfig = (config) => {
        let paths = pathsFromTree(config);
        for (const path of paths) {
          let val = config;
          for(const key of path.split('.')) {
            val = val[key];
          }
          putVal(odrive + '.' + path, val);
        }
        console.log("Config imported to odrive " + odrive);
      };
      reader.addEventListener("load", function (e) {
        let importString = e.target.result;
        importString = importString.replace(/ Infinity/g, '" Infinity"');
        importString = importString.replace(/ -Infinity/g, '" -Infinity"');
        importConfig(JSON.parse(importString));
      });
      reader.readAsText(file);
    },
    exportConfig(odrive) {
      console.log("Exporting config from odrive " + odrive);
      let configPaths = [];
      let exportConfig = {};

      let paramTree = this.$store.state.odriveConfigs.params[odrive];
      for (const path of pathsFromTree(paramTree)) {
        if (path.includes('config.') && !path.includes('mapping')) {
          configPaths.push(path);
        }
      }
      for (const path of configPaths) {
        let keys = path.split('.');
        let val = String(getVal(odrive + '.' + path));
        // super ugly! Need a better way to do this and handle tuples in odrive_utils
        if (val.includes('True')) {
          val = true;
        }
        else if (val.includes('False')) {
          val = false;
        }
        else if (val.includes('.')) {
          val = parseFloat(val);
        }
        else if (!val.includes(',') && !val.includes('Infinity')){
          val = parseInt(val);
        }
        let obj = exportConfig;
        for (const key of keys.slice(0,-1)) {
          if (!Object.keys(obj).includes(key)) {
            obj[key] = {}
          }
          obj = obj[key];
        }
        obj[keys.pop()] = val;
      }
      let exportString = JSON.stringify(exportConfig, null, 2);
      if (exportString.includes('"Infinity"')) {
        console.log("found infinity!");
      }
      console.log(typeof exportString);
      exportString = exportString.replace(/"Infinity"/g, 'Infinity');
      exportString = exportString.replace(/"-Infinity"/g, '-Infinity');
      const blob = new Blob([exportString], {
        type: "application/json",
      });
      saveAs(blob, odrive + '-config.json');
    },
    calcImportLeft() {
      let elem = document.getElementById('menu');
      this.importLeft = elem.offsetWidth + 'px';
    },
    calcImportTop() {
      this.importTop = document.getElementById('importButton').getBoundingClientRect().top + 'px';
    },
    calcExportLeft() {
      let elem = document.getElementById('menu')
      this.exportLeft = elem.offsetWidth + 'px';
    },
    calcExportTop() {
      this.exportTop = document.getElementById('exportButton').getBoundingClientRect().top + 'px';
    },
    changeDash(dashName) {
      console.log(dashName);
      this.$store.commit("setDash", dashName);
    },
    addDash() {
      let dashname = "Dashboard " + (this.dashboards.length - 2);
      this.dashboards.push({
        component: "Dashboard",
        name: dashname,
        id: uuidv4(),
        controls: [],
        actions: [],
        plots: [],
      });
    },
    deleteDash(dashID) {
      this.$store.commit("setDash", "Start");
      console.log("Deleting dash " + dashID);
      for (const dash of this.dashboards) {
        if (dashID === dash.id) {
          this.dashboards.splice(this.dashboards.indexOf(dash), 1);
        }
      }
    },
    exportDash() {
      console.log("exporting dashboard");
      const blob = new Blob([JSON.stringify(this.dash, null, 2)], {
        type: "application/json",
      });
      saveAs(blob, this.dash.name);
    },
    importDashWrapper() {
      const inputElem = document.getElementById("inputDash");
      if (inputElem) {
        console.log("importing dashboard");
        inputElem.click();
      }
    },
    importDashFile(files) {
      console.log("file handler callback");
      let file = files[0];
      const reader = new FileReader();
      // this is ugly, but it gets around scoping problems the "load" callback
      let dashes = this.dashboards;
      let addImportedDash = (dash) => {
        console.log(dash);
        dashes.push(dash);
        // plots will have variables associated, add them to sampled variables list
        for (const plot of dash.plots) {
          console.log(plot);
          for (const plotVar of plot.vars) {
            console.log(plotVar);
            //addsampledprop(path);
            this.$store.commit("addSampledProperty", plotVar.path);
          }
        }
      };
      reader.addEventListener("load", function (e) {
        addImportedDash(JSON.parse(e.target.result));
      });
      reader.readAsText(file);
    },
    changeDashName(e) {
      console.log(e);
      console.log("double clicked dashboard name");
    },
    sampleButton() {
      if (this.$store.state.sampling) {
        // sampling acttive, stop
        socketio.sendEvent({
          type: "stopSampling",
        });
        this.$store.state.sampling = false;
      } else {
        // sampling inactive, start sampling
        socketio.sendEvent({
          type: "sampledVarNames",
          data: {
            paths: this.$store.state.sampledProperties,
          },
        });
        socketio.sendEvent({
          type: "enableSampling",
        });
        this.$store.state.timeSampleStart = Date.now();
        this.$store.state.sampling = true;
      }
    },
    startsample() {
      socketio.sendEvent({
        type: "sampledVarNames",
        data: {
          paths: this.$store.state.sampledProperties,
        },
      });
      socketio.sendEvent({
        type: "enableSampling",
      });
      this.$store.state.timeSampleStart = Date.now();
      this.$store.state.sampling = true;
    },
    stopsample() {
      socketio.sendEvent({
        type: "stopSampling",
      });
      this.$store.state.sampling = false;
    },
    estop() {
      // send stop command to odrives
      // behavior on reset?
    },
    emitFindODrives() {
      socketio.sendEvent({
        type: "findODrives",
        data: {},
      });
    },
  },
  created() {
    // on app creation, set the address to the default
    this.$store.dispatch("setServerAddress", "http://127.0.0.1:5000");

    // to allow running as web app
    if (window.ipcRenderer != undefined) {
      window.ipcRenderer.on("server-stdout", (event, arg) => {
        this.$store.commit("logServerMessage", arg);
      });
      window.ipcRenderer.on("server-stderr", (event, arg) => {
        this.$store.commit("logServerMessage", arg);
      });
      window.ipcRenderer.send("start-server");
    }
  },
};
</script>

<style>
@import "./assets/styles/vars.css";
@import "./assets/styles/style.css";

* {
  /* font-family: Arial, Helvetica, sans-serif; */
  font-family: "Roboto", sans-serif;
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

#app {
  height: 100vh;
}

.header {
  /* want this fixed and full width */
  position: fixed;
  top: 0px;
  left: 0px;
  width: 100vw;
  display: flex;
  background-color: var(--fg-color);
  box-shadow: 0 0px 8px 0 rgba(0, 0, 0, 0.4);
  z-index: 1;
}

button {
  cursor: pointer;
  font-size: 1rem;
  color: black;
  text-decoration: none;
  padding: 10px;
  background-color: var(--fg-color);
  border-style: none;
  outline: none;
}

.dash-button {
  cursor: pointer;
  font-size: 1rem;
  color: black;
  text-decoration: none;
  padding: 10px;
  background-color: var(--fg-color);
  border-style: none;
  outline: none;
}

.dash-button:active {
  background-color: var(--bg-color);
}

.dash-button:hover {
  background-color: var(--bg-color);
}

.menu-item {
  text-align: left;
}

.active {
  color: #000000;
  background-color: var(--bg-color);
}

.footer {
  position: fixed;
  width: 100vw;
  left: 0px;
  bottom: 0px;
  display: flex;
  background-color: var(--fg-color);
  box-shadow: 0 0px 8px 0 rgba(0, 0, 0, 0.4);
  z-index: 1;
}

.odrvSer,
.errorState {
  font-weight: bold;
  font-family: "Roboto Mono", monospace;
  margin: auto 5px;
  background-color: var(--fg-color);
}

.errorState {
  color: #13a100;
}

.dash-add {
  font-weight: bold;
}

.emergency-stop {
  margin-left: auto;
  padding-left: 2rem;
  padding-right: 2rem;
  background-color: red;
  font-weight: bold;
  color: white;
  display: none;
}

.sample-button {
  margin-left: auto;
}

.odrive-status {
  margin-left: auto;
  font-family: "Roboto Mono", monospace;
  padding: 5px 10px;
}

.menu-button {
  border-right: 1px solid grey;
}

.menu {
  position: absolute;
  margin: 0;
  padding: 0;
  left: 0;
  display: flex;
  flex-direction: column;
}

.import-menu {
  position: absolute;
  margin: 0;
  padding: 0;
  display: flex;
  flex-direction: column;
}

.export-menu {
  position: absolute;
  margin: 0;
  padding: 0;
  display: flex;
  flex-direction: column;
}
</style>
