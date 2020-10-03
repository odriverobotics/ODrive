<template>
  <div>
    <button class="measure-button card" @click="clear">Clear Errors</button>
  </div>
</template>

<script>
import { putVal } from "../lib/odrive_utils.js";

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
        this.data.axis + ".error",
        this.data.axis + ".motor.error",
        this.data.axis + ".encoder.error",
        this.data.axis + ".controller.error",
      ];
      for (const path of paths) {
        putVal(path, 0);
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