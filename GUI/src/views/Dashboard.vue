<template>
  <div class="dashboard">
    <!-- PARAMETER DROPDOWN MENU -->
    <div v-show="paramsVisible" class="dropdown">
      <div class="card dropdown-content">
        <div>
          <button class="close-button" @click="hideTree">X</button>
          Parameters
        </div>
        <div class="param-tree">
        <json-view
          :data="treeParams"
          :rootKey="'odrives'"
          v-on:selected="addVarToElement"
        />
        </div>
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
            :options="control.options"
            :odrives="odrives"
            :dashID="dash.id"
          />
        </template>
        <div class="control-buttons">
          <div class="add-button card" @click="addComponent('control')">Add Control</div>
          <div class="add-button card" @click="addComponent('slider')">Add Slider</div>
          <div class="add-button card" @click="addComponent('function')">Add Function</div>
        </div>
        <template v-for="(action, index) in dash.actions">
          <component
            :is="action.actionType"
            :id="action.id"
            :key="index + '-action'"
            :path="action.path"
            :odrives="odrives"
            :initVal="action.val"
            :dashID="dash.id"
            :options="action.options"
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
import CtrlEnum from "../components/controls/CtrlEnum.vue";
import Plot from "../components/plots/Plot.vue";
import Action from "../components/actions/Action.vue";
import ActionEnum from "../components/actions/ActionEnum.vue"
import { JSONView } from "vue-json-component";
import { v4 as uuidv4 } from "uuid";

let plotColors = [
  "#1f77b4", // blue
  "#ff7f0e", // orange
  "#2ca02c", // green
  "#d62728", // red
  "#9467bd", // purple
  "#8c564b", // brown
  "#e377c2", // pink
  "#7f7f7f", // gray
  "#bcbd22", // olive
  "#17becf", // cyan
];

let odriveEnums = {
  // axis requested state
  requested_state: [
    {
      text: "Undefined",
      value: 0,
    },
    {
      text: "Idle",
      value: 1,
    },
    {
      text: "Startup Sequence",
      value: 2,
    },
    {
      text: "Full Calibration Sequence",
      value: 3,
    },
    {
      text: "Motor Calibration",
      value: 4,
    },
    {
      text: "Sensorless Control",
      value: 5,
    },
    {
      text: "Encoder Index Search",
      value: 6,
    },
    {
      text: "Encoder Offset Calibration",
      value: 7,
    },
    {
      text: "Closed Loop Control",
      value: 8,
    },
    {
      text: "Lockin Spin",
      value: 9,
    },
    {
      text: "Encoder Direction Find",
      value: 10,
    },
    {
      text: "Homing",
      value: 11,
    },
    {
      text: "Encoder Hall Polarity Calibration",
      value: 12,
    },
    {
      text: "Encoder Hall Phase Calibration",
      value: 13,
    }
  ],
  current_state: [
    {
      text: "Undefined",
      value: 0,
    },
    {
      text: "Idle",
      value: 1,
    },
    {
      text: "Startup Sequence",
      value: 2,
    },
    {
      text: "Full Calibration Sequence",
      value: 3,
    },
    {
      text: "Motor Calibration",
      value: 4,
    },
    {
      text: "Sensorless Control",
      value: 5,
    },
    {
      text: "Encoder Index Search",
      value: 6,
    },
    {
      text: "Encoder Offset Calibration",
      value: 7,
    },
    {
      text: "Closed Loop Control",
      value: 8,
    },
    {
      text: "Lockin Spin",
      value: 9,
    },
    {
      text: "Encoder Direction Find",
      value: 10,
    },
    {
      text: "Homing",
      value: 11,
    },
    {
      text: "Encoder Hall Polarity Calibration",
      value: 12,
    },
    {
      text: "Encoder Hall Phase Calibration",
      value: 13,
    }
  ],
  // encoder mode
  mode: [
    {
      text: "Incremental",
      value: 0,
    },
    {
      text: "Hall Effect",
      value: 1,
    },
    {
      text: "Sine Cosine",
      value:  2,
    },
    {
      text: "CUI Absolute",
      value: 256,
    },
    {
      text: "AMS Absolute",
      value: 257,
    },
    {
      text: "AEAT Absolute",
      value: 258,
    },
    {
      text: "RLS Absolute",
      value: 259,
    },
  ],
  // motor type
  motor_type: [
    {
      text: "High Current",
      value: 0,
    },
    {
      text: "Gimbal",
      value: 2,
    },
    {
      text: "Induction",
      value: 3,
    },
  ],
  // controller control mode
  control_mode: [
    {
      text: "Voltage Control",
      value: 0,
    },
    {
      text: "Torque Control",
      value: 1,
    },
    {
      text: "Velocity Control",
      value: 2,
    },
    {
      text: "Position Control",
      value: 3,
    },
  ],
  input_mode: [
    {
      text: "Inactive",
      value: 0
    },
    {
      text: "Passthrough",
      value: 1
    },
    {
      text: "Velocity Ramp",
      value: 2
    },
    {
      text: "Position Filter",
      value: 3
    },
    {
      text: "Mix Channels",
      value: 4
    },
    {
      text: "Trapezoidal Trajectory",
      value: 5
    },
    {
      text: "Torque Ramp",
      value: 6
    },
    {
      text: "Mirror",
      value: 7
    },
    {
      text: "Tuning",
      value: 8
    }
  ]
}

export default {
  name: "Dashboard",
  components: {
    CtrlBoolean,
    CtrlNumeric,
    CtrlFunction,
    CtrlSlider,
    CtrlEnum,
    Plot,
    Action,
    ActionEnum,
    "json-view": JSONView,
  },
  props: ["dash", "odrives"],
  data() {
    return {
      paramsVisible: false,
      addCompType: undefined,
      currentPlot: undefined,
      treeParams: undefined,
    };
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
      switch (this.addCompType) {
        case "control":
          this.treeParams = this.$store.state.odriveConfigs['params'];
          console.log(this.treeParams);
          break;
        case "plot":
          this.treeParams = this.$store.state.odriveConfigs['params'];
          break;
        case "slider":
          this.treeParams = this.$store.state.odriveConfigs['writeAbleNumeric'];
          break;
        case "action":
          this.treeParams = this.$store.state.odriveConfigs['writeAble'];
          break;
        case "function":
          this.treeParams = this.$store.state.odriveConfigs['functions'];
          break;
        default:
          break;
      }
      this.paramsVisible = true;
    },
    addVarToElement(e) {
      //when the parameter tree is open and a parameter is clicked,
      //add the clicked parameter to the list of controls for the
      //current dashboard
      console.log(e);
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
              if (Object.keys(odriveEnums).includes(e.key)){
                this.dash.controls.push({
                  controlType: "CtrlEnum",
                  path: e.path,
                  options: odriveEnums[e.key],
                })
              }
              else {
                this.dash.controls.push({
                  controlType: "CtrlNumeric",
                  path: e.path,
                });
              }
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
        case "function":
          this.dash.controls.push({
            controlType: "CtrlFunction",
            path: e.path,
          });
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
            console.log("action, key = " + e.key);
            if (Object.keys(odriveEnums).includes(e.key)){
              this.dash.actions.push({
                actionType: "ActionEnum",
                id: id,
                path: e.path,
                options: odriveEnums[e.key],
              })
            }
            else {
              this.dash.actions.push({
                actionType: "Action",
                id: id,
                path: e.path,
                val: undefined,
              });
            }
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
  overflow-y: scroll;
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
  padding: 0 0;
  display: inline-block;
}

.dropdown-content {
  position: absolute;
  z-index: 1;
}

.param-tree {
  overflow-y: scroll;
  max-height: 85vh;
}
</style>