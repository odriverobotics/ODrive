<template>
  <div>
    <button class="measure-button card" @click="clear">Clear Errors</button>
  </div>
</template>

<script>
const axios = require("axios");
export default {
  name: "clearErrors.vue",
  props: {
    data: Object,
  },
  methods: {
    clear() {
      // clear odrive errors
      // set them to 0?
      console.log(this.data);
      let paths = [
        "odrive0." + this.data.axis + ".error",
        "odrive0." + this.data.axis + ".motor.error",
        "odrive0." + this.data.axis + ".encoder.error",
        "odrive0." + this.data.axis + ".controller.error",
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