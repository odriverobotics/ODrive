<template>
  <div id="app">
    <!-- HEADER -->
    <div class="header">
      <button
        class="dash-button"
        @click="startsample"
        :class="[{active: sampling === true}]"
      >start sampling</button>
      <button class="dash-button" @click="stopsample">stop sampling</button>
      <button class="dash-button" @click="exportDash">export dash</button>
      <button class="dash-button" @click="importDashWrapper">
        import dash
        <input
          type="file"
          id="inputDash"
          ref="fileInput"
          @change="importDashFile($event.target.files);$refs.fileInput.value=null"
          value
          style="display:none"
        />
      </button>
      <button
        v-for="dash in dashboards"
        :key="dash.id"
        :class="['dash-button', { active: currentDash === dash.name}]"
        v-on:click.self="changeDash(dash.name)"
        v-on:dblclick="changeDashName(dash.id)"
      >
        <button
          v-if="dash.name !== 'Start' && dash.name !== 'Config' && dash.name !== 'Wizard'"
          class="close-button"
          v-on:click="deleteDash(dash.id)"
        >X</button>
        {{ dash.name }}
      </button>
      <button class="dash-button dash-add" @click="addDash">+</button>
      <button class="emergency-stop" @click="estop">STOP</button>
    </div>

    <!-- PAGE CONTENT -->
    <component
      :is="currentDashName"
      :odrives="odrives"
      :dash="dash"
    ></component>

    <!-- FOOTER -->
    <div class="footer">
      <Axis v-for="axis in axes" :key="axis.name" :axis="axis" :odrives="odrives"></Axis>
    </div>
  </div>
</template>

<script>
import Start from "./views/Start.vue";
import Dashboard from "./views/Dashboard.vue";
import Axis from "./components/Axis.vue";
import Wizard from "./views/Wizard.vue"
import * as socketio from "./comms/socketio";
import { saveAs } from "file-saver";
import { v4 as uuidv4 } from "uuid";

//let propSamplePeriod = 100; //sampling period for properties in ms

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
    }
  },
  methods: {
    changeDash(dashName){
      console.log(dashName);
      this.$store.commit("setDash", dashName);
    },
    updateOdrives() {
      if (this.$store.state.serverConnected == true) {
        //} && this.sampling == false) {
        this.$store.dispatch("getOdrives");
      }
      setTimeout(() => {
        this.updateOdrives();
      }, 1000);
      //console.log("updating data...");
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
  },
  created() {
    //grab full JSON
    //this.getOdrives();

    this.$store.dispatch("setServerAddress", "http://127.0.0.1:5000");
    // connect to socketio on server for sampled data
    this.updateOdrives();
  },
};
</script>

<style>
@import url("https://fonts.googleapis.com/css2?family=Roboto+Mono:wght@400;700&family=Roboto:wght@400;700&display=swap");
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
  font-size: 1rem;
  color: #2c3e50;
  text-decoration: none;
  padding: 10px;
  background-color: var(--fg-color);
  border-style: none;
  outline: none;
}

.dash-button:active {
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

.footer .left,
.right {
  /* flex-grow: 1; */
  display: flex;
  background-color: var(--fg-color);
  font-family: "Roboto Mono", monospace;
  margin: auto 5px;
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

.parameter-button {
  border-right: 1px solid lightgrey;
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

json-view {
  z-index: 2;
}
</style>
