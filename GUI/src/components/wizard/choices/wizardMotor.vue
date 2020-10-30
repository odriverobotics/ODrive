<template>
  <div class="card wizard-motor-custom wizard-choice" :class="{'choice-inactive': !allowed}">
    <div class="left">
      <span>KV =</span>
      <input type="number" v-on:change="setKV" placeholder="Change Me!" />
    </div>
    <div class="left">
      <span>Pole Pairs =</span>
      <input type="number" v-on:change="setPP" placeholder="Change Me!" />
    </div>
    <div class="left">
      <span>Phase Resistance =</span>
      <span>{{" " + resistance}}</span>
      <span class="unit">[Ohms]</span>
    </div>
    <div class="left">
      <span>Phase Inductance =</span>
      <span>{{" " + inductance}}</span>
      <span class="unit">[Henries]</span>
    </div>
    <span class="name">Other Motor</span>
  </div>
</template>

<script>
import odriveEnums from "../../../assets/odriveEnums.json";
import { getVal } from "../../../lib/odrive_utils.js"

export default {
  name: "wizardMotor",
  props: {
    data: Object,
    axis: String,
    allowed: Boolean,
  },
  data: function () {
    return {
      kv_set: false,
      pp_set: false,
      r_set: false,
      l_set: false,
      pole_pairs: undefined,
      phase_resistance: undefined,
      phase_inductance: undefined,
      torque_constant: undefined,
    };
  },
  computed: {
    resistance: function () {
      let path = "odrive0." + this.data.axis + ".motor.config.phase_resistance";
      return parseFloat(getVal(path)).toExponential(3);
    },
    inductance: function () {
      let path = "odrive0." + this.data.axis + ".motor.config.phase_inductance";
      return parseFloat(getVal(path)).toExponential(3);
    },
  },
  watch: {
    resistance: function (newVal) {
      console.log("from resistance watcher: " + newVal);
      let path = "odrive0." + this.data.axis + ".motor.config.phase_resistance";
      this.phase_resistance = parseFloat(getVal(path));
    },
    inductance: function (newVal) {
      console.log("from inductance watcher: " + newVal);
      let path = "odrive0." + this.data.axis + ".motor.config.phase_inductance";
      this.phase_inductance = parseFloat(getVal(path));
    },
    pp_set: function () {
      console.log("pp_set: " + this.pp_set);
      this.sendConfig();
    },
    kv_set: function () {
      console.log("kv_set: " + this.kv_set);
      this.sendConfig();
    },
    r_set: function () {
      console.log("r_set: " + this.r_set);
      this.sendConfig();
    },
    l_set: function () {
      console.log("l_set: " + this.l_set);
      this.sendConfig();
    },
  },
  methods: {
    sendConfig() {
      // if all of the set flags are true, emit an event
      if (this.phase_resistance != 0) {
        this.r_set = true;
      } else {
        this.r_set = false;
      }
      if (this.phase_inductance != 0) {
        this.l_set = true;
      } else {
        this.l_set = false;
      }
      if (
        this.r_set == true &&
        this.l_set == true &&
        this.pp_set == true &&
        this.kv_set == true
      ) {
        console.log("emitting choice event from other motor");
        let configStub = undefined;
        if (this.axis == "axis0") {
          configStub = {
            axis0: {
              motor: {
                config: {
                  pole_pairs: this.pole_pairs,
                  torque_constant: this.torque_constant,
                  phase_resistance: this.phase_resistance,
                  phase_inductance: this.phase_inductance,
                  type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT,
                },
              },
            },
          };
        } else if (this.axis == "axis1") {
          configStub = {
            axis1: {
              motor: {
                config: {
                  pole_pairs: this.pole_pairs,
                  torque_constant: this.torque_constant,
                  phase_resistance: this.phase_resistance,
                  phase_inductance: this.phase_inductance,
                  type: odriveEnums.MOTOR_TYPE_HIGH_CURRENT,
                },
              },
            },
          };
        }
        this.$emit("choice", {
          choice: "Other motor",
          configStub: configStub,
          hooks: [],
        });
      }
    },
    setKV(e) {
      this.torque_constant = 8.27 / parseFloat(e.target.value);
      this.kv_set = true;
      console.log(this.torque_constant);
    },
    setPP(e) {
      this.pole_pairs = parseFloat(e.target.value);
      this.pp_set = true;
      console.log(this.pole_pairs);
    },
  },
};
</script>

<style scoped>
.wizard-choice {
  display: flex;
  flex-direction: column;
  margin: 2rem;
}
.left {
  margin-right: auto;
}

.name {
  margin-top: auto;
}

input {
  width: 5rem;
  font-family: inherit;
  border-style: none;
  border-bottom: 1px solid grey;
  text-align: center;
  background-color: transparent;
}

input::-webkit-outer-spin-button,
input::-webkit-inner-spin-button {
  /* display: none; <- Crashes Chrome on hover */
  -webkit-appearance: none;
  margin: 0; /* <-- Apparently some margin are still there even though it's hidden */
}

input[type="number"] {
  -moz-appearance: textfield; /* Firefox */
}

.name {
  text-align: center;
}

.unit {
  font-family: "Roboto Mono", monospace;
}
</style>