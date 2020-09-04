<template>
  <!-- render a list of choices horizontally in cards -->
  <div class="wizard-page card">
    <div class="wizard-title">{{title}}</div>
    <div class="wizard-choices">
      <wizardChoice
        v-for="choice in choices"
        v-bind:class="{chosen: selectedChoice == choice.text, unchosen: selectedChoice != choice.text}"
        :key="choice.id"
        :imageUrl="choice.imageURL"
        :text="choice.text"
        v-on:click.native="selectedChoice=choice.text;$emit('choice', {title: title, choice: choice.text})"
      />
      <template v-for="customComponent in customComponents">
          <component v-bind:is="customComponent.compName" :key="customComponent.id" />
      </template>    
    </div>
  </div>
</template>

<script>
import wizardChoice from "./wizardChoice.vue";
import wizardLimits from "./wizardLimits.vue";
import wizardMotorCustom from "./wizardMotorCustom.vue";

export default {
  name: "wizardPage",
  props: {
    choices: Array,
    title: String,
    customComponents: Array,
  },
  components: {
    wizardChoice,
    wizardLimits,
    wizardMotorCustom,
  },
  data: function () {
    return {
      selectedChoice: undefined,
    };
  },
  methods: {},
};
</script>

<style scoped>
.wizard-page {
  display: flex;
  flex-direction: column;
  text-align: center;
}

.wizard-choices {
  display: flex;
  flex-direction: row;
}

.chosen {
  border: 2px solid black;
}

.unchosen {
  border: 2px solid transparent;
}
</style>