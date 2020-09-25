<template>
  <!-- single choice for a wizard page -->
  <div class="wizard-choice card" :class="{'choice-inactive': !allowed}">
    <img v-if="data.imageURL != null" :alt="title" :src="data.imageURL" />
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
    allowed: Boolean,
  },
  data() {
    return {
      oldSelected: undefined,
    }
  },
  watch: {
    selected(newVal) {
      if (newVal == true) {
        // emit choice event
        this.$emit('choice', {choice: this.title, configStub: this.data.configStub, hooks: this.hooks});
      }
      if (this.oldSelected == true && newVal == false){
        console.log("emitting undo-choice");
        this.$emit('undo-choice', {choice: this.title, configStub: this.data.configStub, hooks: this.hooks})
      }
      this.oldSelected = newVal;
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

.inactive {
  background-color: var(--bg-color);
}

img {
  padding: 0px;
  margin: auto;
}
</style>