<template>
  <div>
    <button class="measure-button card" @click="calibrate">{{text}}</button>
  </div>
</template>

<script>
const axios = require("axios");
import odriveEnums from "../../../assets/odriveEnums.json";

export default {
  name: "wizardMotorMeasure",
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
      var params = new URLSearchParams();
      let keys = ["odrive0", this.data.axis, "requested_state"];
      for (const key of keys) {
        params.append("key", key);
      }
      params.append("val", odriveEnums.AXIS_STATE_MOTOR_CALIBRATION);
      params.append("type", "number");
      console.log(params.toString());
      let request = {
        params: params,
      };
      console.log(request);
      axios.put(
        this.$store.state.odriveServerAddress + "/api/property",
        null,
        request
      );
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