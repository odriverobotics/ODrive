<template>
  <div class="card wizard-motor-custom wizard-choice" :class="{'choice-inactive': !allowed}">
    <div class="left">
      <span>Input filter bandwidth =</span>
      <input type="number" v-on:change="setBandwidth" :placeholder="bandwidth" />
      <span>Hz</span>
    </div>
    <span class="name">Filtered Position Input Mode</span>
  </div>
</template>

<script>
import odriveEnums from "../../../assets/odriveEnums.json";
import { getVal } from "../../../lib/odrive_utils.js"

export default {
  name: "wizardInputFiltered",
  props: {
    data: Object,
    allowed: Boolean,
  },
  data: function () {
    return {
      bandwidth: undefined,
    };
  },
  created() {
    let path = "odrive0." + this.data.axis + ".controller.config.input_filter_bandwidth";
    this.bandwidth = parseFloat(getVal(path));
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
                input_filter_bandwidth: this.bandwidth,
                input_mode: odriveEnums.INPUT_MODE_POS_FILTER,
              },
            },
          },
        };
      } else if (this.data.axis == "axis1") {
        configStub = {
          axis1: {
            controller: {
              config: {
                input_filter_bandwidth: this.bandwidth,
                input_mode: odriveEnums.INPUT_MODE_POS_FILTER,
              },
            },
          },
        };
      }
      this.$emit("choice", {
        choice: "Input Filter " + this.data.axis,
        configStub: configStub,
        hooks: [],
      });
    },
    setBandwidth(e) {
      this.bandwidth = parseFloat(e.target.value);
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