<template>
  <!-- single choice for a wizard page -->
  <div class="wizard-choice card">
    <img v-bind:alt="text" :src="imageUrl" />
    {{ text }}
  </div>
</template>

<script>
export default {
  name: "wizardChoice",
  props: {
    imageUrl: String,
    text: String,
  },
};
</script>

<style scoped>
.wizard-choice {
  text-align: center;
  display: flex;
  flex-direction: column;
  margin: 2rem;
}

img {
  padding: 0px;
  margin: auto;
}
</style>