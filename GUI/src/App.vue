<template>
  <div id="app">
    <!-- HEADER -->
    <div class="header">
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
      <button class="dash-button sample-button" :class="[{ active: sampling === true }]" @click="sampleButton">{{samplingText}}</button>
      <button class="dash-button" @click="exportDash">export dash</button>
      <button class="dash-button" @click="importDashWrapper">
        import dash
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
      <!--<div class="odrive-status">
        ODrive:{{ODriveConnected}}
      </div>-->
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
    };
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
    ODriveConnected: function () {
      // if server and odrive disconnected, disconnected
      // if server connected and odrive disco, connecting
      // if server and odrive connected, connected
      let ret;
      if (this.$store.state.serverConnected && this.$store.state.ODriveConnected) {
        ret = "connected";
      }
      else if (this.$store.state.serverConnected && !this.$store.state.ODriveConnected) {
        ret = "connecting...";
      }
      else {
        ret = "disconnected";
      }
      return ret;
    },
    samplingText: function () {
      let ret;
      if (this.$store.state.sampling) {
        ret = "stop sampling"
      }
      else {
        ret = "start sampling"
      }
      return ret;
    }
  },
  methods: {
    changeDash(dashName) {
      console.log(dashName);
      this.$store.commit("setDash", dashName);
    },
    //updateOdrives() {
    //  if (this.$store.state.serverConnected == true) {
        //} && this.sampling == false) {
    //    this.$store.dispatch("getOdrives");
    //  }
      //setTimeout(() => {
      //  this.updateOdrives();
      //}, 1000);
      //console.log("updating data...");
    //},
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
      }
      else {
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
      window.ipcRenderer.on('server-stdout', (event, arg) => {
        this.$store.commit('logServerMessage', arg);
      });
        window.ipcRenderer.on('server-stderr', (event, arg) => {
        this.$store.commit('logServerMessage', arg);
      });
      window.ipcRenderer.send('start-server');
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

</style>
