<template>
  <div class="dashboard">
    <!-- PARAMETER DROPDOWN MENU -->
    <div v-show="paramsVisible" class="dropdown">
      <div class="card dropdown-content">
        <div>
          <button class="close-button" @click="hideTree">X</button>
          Parameters
        </div>
        <json-view
          v-bind:data="odriveConfigs"
          v-bind:rootKey="'odrives'"
          v-on:selected="addVarToElement"
        />
      </div>
    </div>
    <div class="dashboard_container">
      <div class="controls">
        <template v-for="(control, index) in dash.controls">
          <component
            :is="control.controlType"
            :key="index + '-control'"
            :path="control.path"
            :name="control.name"
            :odrives="odrives"
            :dashID="dash.id"
          />
        </template>
        <div class="control-buttons">
          <div class="add-button card" @click="addComponent('control')">Add Control</div>
          <div class="add-button card" @click="addComponent('slider')">Add Slider</div>
        </div>
        <template v-for="(action, index) in dash.actions">
          <action
            :id="action.id"
            :key="index + '-action'"
            :path="action.path"
            :odrives="odrives"
            :initVal="action.val"
            :dashID="dash.id"
          />
        </template>
        <div class="add-button card" @click="addComponent('action')">Add Action</div>
      </div>
      <div class="plots">
        <template v-for="(plot, index) in dash.plots">
          <plot
            :plot="plot"
            :key="index"
            :name="plot.name"
            :dashID="dash.id"
            v-on:add-var="currentPlot=plot.name;addComponent('plot')"
          />
        </template>
        <div class="add-button card" @click="addPlot">Add Plot</div>
      </div>
    </div>
  </div>
</template>

<script>
//this component will get passed a list of controls and plots
//display controls on the left and plots on the right?
//leave full names for plots
//collate controls into individual cards based on the deepest common level
import CtrlBoolean from "../components/controls/CtrlBoolean.vue";
import CtrlNumeric from "../components/controls/CtrlNumeric.vue";
import CtrlFunction from "../components/controls/CtrlFunction.vue";
import CtrlSlider from "../components/controls/CtrlSlider.vue";
import Plot from "../components/plots/Plot.vue";
import Action from "../components/actions/Action.vue";
import { JSONView } from "vue-json-component";
import { v4 as uuidv4 } from "uuid";

let plotColors = [
  "#195bd7", // blue
  "#d6941a", // orange
  "#1ad636", // green
  "#d61aba", // purple
  "#d5241a", // red
];

export default {
  name: "Dashboard",
  components: {
    CtrlBoolean,
    CtrlNumeric,
    CtrlFunction,
    CtrlSlider,
    Plot,
    Action,
    "json-view": JSONView,
  },
  props: ["dash", "odrives"],
  data() {
    return {
      paramsVisible: false,
      addCompType: undefined,
      currentPlot: undefined,
    };
  },
  computed: {
    odriveConfigs: function () {
      return this.$store.state.odriveConfigs;
    },
  },
  methods: {
    deleteAction(e) {
      this.$emit("delete-action", e);
      console.log(e);
    },
    deletePlot(e) {
      this.$emit("delete-plot", e);
    },
    addVar(e) {
      this.$emit("add-var", e);
    },
    showTree() {
      //show the parameter tree
      this.paramsVisible = true;
    },
    hideTree() {
      this.paramsVisible = false;
      this.addCompType = undefined;
    },
    addComponent(componentType) {
      this.addCompType = componentType;
      this.paramsVisible = true;
    },
    addVarToElement(e) {
      //when the parameter tree is open and a parameter is clicked,
      //add the clicked parameter to the list of controls for the
      //current dashboard
      switch (this.addCompType) {
        case "control":
          switch (typeof e.value) {
            case "boolean":
              this.dash.controls.push({
                controlType: "CtrlBoolean",
                path: e.path,
              });
              //this.$store.commit("addSampledProperty", e.path);
              break;
            case "number":
              this.dash.controls.push({
                controlType: "CtrlNumeric",
                path: e.path,
              });
              //this.$store.commit("addSampledProperty", e.path);
              break;
            case "string":
              this.dash.controls.push({
                controlType: "CtrlFunction",
                path: e.path,
              });
              break;
            default:
              break;
          }
          break;
        case "plot":
          // add the selected element to the plot var list
          // add the selected element to the sampling var list
          // find the plot, append path to plot.vars
          console.log(e);
          for (const plot of this.dash.plots) {
            if (plot.name == this.currentPlot) {
              plot.vars.push({
                path: e.path,
                color: plotColors[plot.vars.length % plotColors.length],
              });
              this.$store.commit("addSampledProperty", e.path);
              console.log(plot);
              break;
            }
          }
          break;
        case "action":
          {
            // add an action to the current dash
            let id = uuidv4();
            this.dash.actions.push({
              id: id,
              path: e.path,
              val: undefined,
            });
          }
          break;
        case "slider":
          // add a slider to the list of controls if the selected item is valid (numeric)
          switch (typeof e.value) {
            case "number":
              this.dash.controls.push({
                controlType: "CtrlSlider",
                path: e.path,
              });
              break;
            default:
              break;
          }
      }
    },
    addPlot() {
      let plotId = uuidv4();
      this.dash.plots.push({
        name: plotId,
        vars: [],
      });
    },
  },
};
</script>

<style scoped>
.dashboard {
  background-color: var(--bg-color);
  height: 100vh;
  max-height: 100vh;
  width: 100vw;
  padding-top: var(--top-height);
  padding-bottom: var(--bottom-height);
}

.dashboard_container {
  display: flex;
  flex-direction: row;
  height: 95vh;
}
.controls {
  flex-grow: 1;
  border-right: 1px solid lightgrey;
}

.add-button {
  background-color: lightcyan;
  width: 120px;
  margin: 10px auto;
  margin-left: 10px;
  margin-right: 10px;
  text-align: center;
  border-radius: 20px;
  cursor: pointer;
  user-select: none;
}

.add-button:active {
  background-color: lightblue;
}

.plots {
  flex-grow: 1;
  max-height: 94vh;
  overflow-y: scroll;
  display: flex;
  flex-direction: column;
}

.control-buttons {
  display: flex;
  flex-direction: row;
}

.dropdown {
  position: absolute;
  padding: var(--top-height) 0;
  display: inline-block;
}

.dropdown-content {
  position: absolute;
  z-index: 1;
  padding: var(--top-height) 0;
  max-height: 90vh;
  overflow-y: scroll;
}
</style>