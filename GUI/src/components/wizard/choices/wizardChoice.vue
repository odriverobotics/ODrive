<template>
  <!-- single choice for a wizard page -->
  <div class="wizard-choice card">
    <img v-bind:alt="title" :src="data.imageURL" />
    {{ title }}
  </div>
</template>

<script>
export default {
  name: "wizardChoice",
  props: {
    data: Object,
    selected: Boolean,
    title: String,
    hooks: Array,
  },
  watch: {
    selected(newVal) {
      if (newVal == true) {
        // emit choice event
        this.$emit('choice', {choice: this.title, configStub: this.data.configStub, hooks: this.hooks});
      }
    }
  }
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