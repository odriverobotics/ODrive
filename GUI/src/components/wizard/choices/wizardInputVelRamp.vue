<template>
  <div class="card wizard-motor-custom wizard-choice">
    <div class="left">
      <span>Velocity ramp rate =</span>
      <input type="number" v-on:change="setRampRate" :placeholder="vel_ramp_rate" />
      <span>turns / s^2</span>
    </div>
    <span class="name">Ramped Velocity Input Mode</span>
  </div>
</template>

<script>
import odriveEnums from "../../../assets/odriveEnums.json";

export default {
  name: "wizardInputVelRamp",
  props: {
    data: Object,
  },
  data: function () {
    return {
      vel_ramp_rate: undefined,
    };
  },
  created() {
    let keys = [
      "odrive0",
      this.data.axis,
      "controller",
      "config",
      "vel_ramp_rate",
    ];
    let odriveObj = this.$store.state.odrives;
    for (const key of keys) {
      odriveObj = odriveObj[key];
    }
    this.vel_ramp_rate = parseFloat(odriveObj["val"]);
  },
  methods: {
    sendConfig() {
      console.log("emitting choice event from limits page");
      let configStub = undefined;
      if (this.data.axis == "axis0") {
        configStub = {
          axis0: {
            controller: {
              config: {
                vel_ramp_rate: this.vel_ramp_rate,
                input_mode: odriveEnums.INPUT_MODE_VEL_RAMP,
              },
            },
          },
        };
      } else if (this.data.axis == "axis1") {
        configStub = {
          axis1: {
            controller: {
              config: {
                vel_ramp_rate: this.vel_ramp_rate,
                input_mode: odriveEnums.INPUT_MODE_VEL_RAMP,
              },
            },
          },
        };
      }
      this.$emit("choice", {
        choice: "Velocity ramp " + this.data.axis,
        configStub: configStub,
        hooks: [],
      });
    },
    setRampRate(e) {
      this.vel_ramp_rate = parseFloat(e.target.value);
      this.sendConfig();
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
  margin-left: auto;
}

input {
  width: 5rem;
  font-family: inherit;
  border-style: none;
  border-bottom: 1px solid grey;
  text-align: center;
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

.measure-button {
  margin-top: 2rem;
}

.measure-button:active {
  background-color: var(--bg-color);
}

.name {
  text-align: center;
}
</style>