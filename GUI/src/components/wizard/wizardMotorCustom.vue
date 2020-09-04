<template>
  <div class="card wizard-motor-custom wizard-choice">
      <span>KV = ?</span>
      <span>pole pairs = ?</span>
      <span>phase resistance = ? (measure)</span>
      <span>phase inductance = ? (measure)</span>
  </div>
</template>

<script>
export default {
    name: "wizardMotorCustom"
}
</script>

<style scoped>
.wizard-choice {
  text-align: center;
  display: flex;
  flex-direction: column;
  margin: 2rem;
}

</style>