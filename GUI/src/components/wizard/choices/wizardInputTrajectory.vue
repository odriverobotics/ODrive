<template>
  <div class="card wizard-motor-custom wizard-choice">
    <div class="left">
      <span>Velocity limit =</span>
      <input type="number" v-on:change="setVelLimit" :placeholder="vel_limit" />
      <span>turns / s</span>
    </div>
    <div class="left">
      <span>Acceleration limit =</span>
      <input type="number" v-on:change="setAccelLimit" :placeholder="accel_limit" />
      <span>turns / s^2</span>
    </div>
    <div class="left">
      <span>Deceleration limit =</span>
      <input type="number" v-on:change="setDecelLimit" :placeholder="decel_limit" />
      <span>turns / s^2</span>
    </div>
    <div class="left">
      <span>Inertia =</span>
      <input type="number" v-on:change="setInertia" :placeholder="inertia" />
    </div>
    <span class="name">Trajectory Planner</span>
  </div>
</template>

<script>
export default {
  name: "wizardInputTrajectory",
  props: {
    data: Object,
  },
  data: function () {
    return {
      vel_limit: undefined,
      accel_limit: undefined,
      decel_limit: undefined,
      inertia: undefined,
    };
  },
  created() {
    // get pre-existing values for trap_traj vel limit, accel, devel, and config.inertia
    if (this.data.axis == "axis0") {
      this.vel_limit = this.getODriveVal(
        "odrive0.axis0.trap_traj.config.vel_limit"
      );
      this.accel_limit = this.getODriveVal(
        "odrive0.axis0.trap_traj.config.accel_limit"
      );
      this.decel_limit = this.getODriveVal(
        "odrive0.axis0.trap_traj.config.decel_limit"
      );
      this.inertia = this.getODriveVal(
        "odrive0.axis0.controller.config.inertia"
      );
    } else if (this.data.axis == "axis1") {
      this.vel_limit = this.getODriveVal(
        "odrive0.axis1.trap_traj.config.vel_limit"
      );
      this.accel_limit = this.getODriveVal(
        "odrive0.axis1.trap_traj.config.accel_limit"
      );
      this.decel_limit = this.getODriveVal(
        "odrive0.axis1.trap_traj.config.decel_limit"
      );
      this.inertia = this.getODriveVal(
        "odrive0.axis1.controller.config.inertia"
      );
    }
  },
  methods: {
    getODriveVal(path) {
      let odriveObj = this.$store.state.odrives;
      console.log("get val " + path);
      for (const key of path.split(".")) {
        odriveObj = odriveObj[key];
      }
      return parseFloat(odriveObj["val"]);
    },
    sendConfig() {
      console.log("emitting choice event from limits page");
      let configStub = undefined;
      if (this.data.axis == "axis0") {
        configStub = {
          axis0: {
            controller: {
              config: {
                inertia: this.inertia,
              },
            },
            trap_traj: {
              config: {
                vel_limit: this.vel_limit,
                accel_limit: this.accel_limit,
                decel_limit: this.decel_limit,
              },
            },
          },
        };
      } else if (this.data.axis == "axis1") {
        configStub = {
          axis1: {
            controller: {
              config: {
                inertia: this.inertia,
              },
            },
            trap_traj: {
              config: {
                vel_limit: this.vel_limit,
                accel_limit: this.accel_limit,
                decel_limit: this.decel_limit,
              },
            },
          },
        };
      }
      this.$emit("choice", {
        choice: "Trapezoidal trajectory " + this.data.axis,
        configStub: configStub,
        hooks: [],
      });
    },
    setVelLimit(e) {
      this.vel_limit = parseFloat(e.target.value);
      this.sendConfig();
    },
    setAccelLimit(e) {
      this.accel_limit = parseFloat(e.target.value);
      this.sendConfig();
    },
    setDecelLimit(e) {
      this.decel_limit = parseFloat(e.target.value);
      this.sendConfig();
    },
    setInertia(e) {
      this.inertia = parseFloat(e.target.value);
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