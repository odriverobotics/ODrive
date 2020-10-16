<template>
  <div>
    <button class="measure-button card" @click="calibrate">{{text}}</button>
  </div>
</template>

<script>

export default {
  name: "wizardEncoderCal",
  props: {
    data: Object,
    calibrating: Boolean,
  },
  computed: {
    text() {
      return this.calibrating ? "Calibrating..." : "Calibrate Encoder";
    }
  },
  methods: {
    calibrate() {
      // ask ODrive to measure resistance and inductance
      this.$emit('page-comp-event', {data: "encoder calibration", axis: this.data.axis});
    },
  },
};
</script>

<style scoped>

.measure-button:active {
  background-color: var(--bg-color);
}

</style>