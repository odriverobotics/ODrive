<template>
  <div class="card wizard-choice" :class="{ 'choice-inactive': !allowed }">
    <div class="left">
      <span>counts per revolution = </span>
      <input v-on:change="setCPR" placeholder="Change Me!" :value="cpr" />
    </div>
    <span class="name">Incremental Encoder with Index</span>
  </div>
</template>

<script>
import odriveEnums from "../../../assets/odriveEnums.json";
import { parseMath } from "../../../lib/odrive_utils.js";

export default {
  name: "wizardEncoderIncremental",
  props: {
    data: Object,
    allowed: Boolean,
  },
  data: function () {
    return {
      cpr_set: false,
      cpr: undefined,
    };
  },
  methods: {
    setCPR(e) {
      let val = parseMath(e.target.value);
      if (val !== false) {
        this.cpr = val;
        let configStub = undefined;
        if (this.data.axis == "axis0") {
          configStub = {
            axis0: {
              encoder: {
                config: {
                  cpr: this.cpr,
                  use_index: true,
                  mode: odriveEnums.ENCODER_MODE_INCREMENTAL,
                },
              },
            },
          };
        } else if (this.data.axis == "axis1") {
          configStub = {
            axis1: {
              encoder: {
                config: {
                  cpr: this.cpr,
                  use_index: true,
                  mode: odriveEnums.ENCODER_MODE_INCREMENTAL,
                },
              },
            },
          };
        }
        this.$emit("choice", {
          choice: "IncrementalIndex",
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

.name {
  margin-top: auto;
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
</style>