<template>
  <div>
    <button class="measure-button card" @click="clear">Clear Errors</button>
  </div>
</template>

<script>
const axios = require("axios");
export default {
  name: "wizardClearErrors.vue",
  props: {
    data: Object,
  },
  methods: {
    clear() {
      // clear odrive errors
      // set them to 0?
      let paths = [
        "odrive0.axis0.error",
        "odrive0.axis0.motor.error",
        "odrive0.axis0.encoder.error",
        "odrive0.axis0.controller.error",
        "odrive0.axis1.error",
        "odrive0.axis1.motor.error",
        "odrive0.axis1.encoder.error",
        "odrive0.axis1.controller.error",
      ];
      for (const path of paths) {
        var params = new URLSearchParams();
        let keys = path.split(".");
        for (const key of keys) {
          params.append("key", key);
        }
        params.append("val", 0);
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
      }
    },
  },
};
</script>

<style scoped>
.measure-button:active {
  background-color: var(--bg-color);
}
</style>