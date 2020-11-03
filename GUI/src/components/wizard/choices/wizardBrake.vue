<template>
  <div class="card wizard-choice" :class="{ 'choice-inactive': !allowed }">
    <div class="left">
      <span>Brake resistor value = </span>
      <input v-on:change="setBR" placeholder="Change Me!" :value="brake_resistance"/>
      <span class="unit"> [{{ unit }}] </span>
    </div>
  </div>
</template>

<script>
import { getUnit, parseMath, getVal } from "../../../lib/odrive_utils.js";

export default {
  name: "wizardBrake",
  props: {
    data: Object,
    allowed: Boolean,
  },
  data: function () {
    return {
      brake_resistance: undefined,
    };
  },
  computed: {
    unit() {
      // goal is to return "Ohms" from odriveUnits
      let path = "odrive0.config.brake_resistance";
      return getUnit(this.$store.state.odrives.odrive0, path);
    },
    value() {
      let path = "odrive0.config.brake_resistance";
      return getVal(path);
    }
  },
  methods: {
    setBR(e) {
      console.log("from setBR " + e.target.value);
      let val = parseMath(e.target.value);
      if (val !== false) {
        this.brake_resistance = val;
        let configStub = undefined;
        configStub = {
          config: {
            brake_resistance: this.brake_resistance,
          },
        };
        this.$emit("choice", {
          choice: "Brake Resistor",
          configStub: configStub,
          hooks: [],
        });
      }
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

.unit {
  font-family: "Roboto Mono", monospace;
}
</style>