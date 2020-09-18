<template>
  <div class="card wizard-motor-custom wizard-choice" v-bind:class="{'choice-inactive': !allowed}">
    <div class="left">
      <span>Motor Velocity Limit =</span>
      <input type="number" v-on:change="setVelocityLimit" :placeholder="velocityLimit" />
    </div>
    <div class="left">
      <span>Motor Current Limit =</span>
      <input type="number" v-on:change="setCurrentLimit" :placeholder="currentLimit" />
    </div>
  </div>
</template>

<script>
export default {
  name: "wizardMisc",
  props: {
    data: Object,
    allowed: Boolean,
  },
  data: function () {
    return {
      current_lim: undefined,
      vel_limit: undefined,
      vel_set: false,
      current_set: false,
    };
  },
  computed: {
    velocityLimit: function () {
      let keys = [
        "odrive0",
        this.data.axis,
        "controller",
        "config",
        "vel_limit",
      ];
      let odriveObj = this.$store.state.odrives;
      for (const key of keys) {
        odriveObj = odriveObj[key];
      }
      return parseFloat(odriveObj["val"]);
    },
    currentLimit: function () {
      let keys = ["odrive0", this.data.axis, "motor", "config", "current_lim"];
      let odriveObj = this.$store.state.odrives;
      for (const key of keys) {
        odriveObj = odriveObj[key];
      }
      return parseFloat(odriveObj["val"]);
    },
  },
  methods: {
    sendConfig() {
      if (this.vel_set == true && this.current_set == true) {
        console.log("emitting choice event from limits page");
        let configStub = undefined;
        if (this.data.axis == "axis0") {
          configStub = {
            axis0: {
              controller: {
                config: {
                  vel_limit: this.vel_limit,
                }
              },
              motor: {
                config: {
                  current_lim: this.current_lim,
                }
              }
            },
          };
        } else if (this.data.axis == "axis1") {
          configStub = {
            axis1: {
              controller: {
                config: {
                  vel_limit: this.vel_limit,
                }
              },
              motor: {
                config: {
                  current_lim: this.current_lim,
                }
              }
            },
          };
        }
        this.$emit("choice", {
          choice: "Misc " + this.data.axis,
          configStub: configStub,
          hooks: [],
        });
      }
    },
    setVelocityLimit(e) {
      this.vel_limit = parseFloat(e.target.value);
      this.vel_set = true;
      this.sendConfig();
    },
    setCurrentLimit(e) {
      this.current_lim = parseFloat(e.target.value);
      this.current_set = true;
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