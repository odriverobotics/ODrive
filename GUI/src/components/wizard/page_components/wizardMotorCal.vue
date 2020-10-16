<template>
  <div>
    <button class="measure-button card" @click="calibrate">{{text}}</button>
  </div>
</template>

<script>
import odriveEnums from "../../../assets/odriveEnums.json";
import { putVal } from "../../../lib/odrive_utils.js"

export default {
  name: "wizardMotorCal",
  props: {
    data: Object,
    calibrating: Boolean,
  },
  computed: {
    text() {
      return this.calibrating ? "Calibrating..." : "Calibrate Motor";
    }
  },
  methods: {
    calibrate() {
      // ask ODrive to measure resistance and inductance
      let path = "odrive0." + this.data.axis + ".requested_state";
      putVal(path, odriveEnums.AXIS_STATE_MOTOR_CALIBRATION);
      this.$emit('page-comp-event', {data: "motor calibration", axis: this.data.axis});
    },
  },
};
</script>

<style scoped>

.measure-button:active {
  background-color: var(--bg-color);
}

</style>