<template>
  <div>
    <button class="measure-button card" @click="calibrate">Calibrate Encoder</button>
  </div>
</template>

<script>

export default {
  name: "wizardEncoderCal",
  props: {
    data: Object,
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